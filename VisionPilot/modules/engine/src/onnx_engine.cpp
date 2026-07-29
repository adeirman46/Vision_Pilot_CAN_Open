#include "engine/onnx_engine.hpp"

#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace visionpilot::engine {

OnnxEngine::OnnxEngine(const Config& cfg)
    : env_(ORT_LOGGING_LEVEL_WARNING, "VisionPilot")
    , cfg_(cfg)
{
    printf("[OnnxEngine] provider=%s", cfg_.provider.c_str());
    if (cfg_.provider == "tensorrt" || cfg_.provider == "cuda") {
        printf("  device=%d", cfg_.device_id);
    }
    if (cfg_.provider == "tensorrt") {
        printf("  precision=%s  workspace=%.1fGB  cache=%s",
               cfg_.precision.c_str(), cfg_.workspace_gb, cfg_.cache_dir.c_str());
    }
    printf("\n");
}

// ─── Public entry point ───────────────────────────────────────────────────────

std::unique_ptr<Ort::Session> OnnxEngine::create_session(
    const std::string& model_path,
    const std::string& cache_prefix,
    const std::string& precision_override) const
{
    const std::string precision =
        precision_override.empty() ? cfg_.precision : precision_override;

    if (cfg_.provider == "cpu") {
        return create_cpu_session(model_path);
    }
    if (cfg_.provider == "cuda") {
        return create_cuda_session(model_path);
    }
    if (cfg_.provider == "tensorrt") {
        return create_tensorrt_session(model_path, cache_prefix, precision);
    }
    throw std::runtime_error(
        "[OnnxEngine] Unknown provider '" + cfg_.provider +
        "'. Valid: cpu | cuda | tensorrt");
}

// ─── CPU ─────────────────────────────────────────────────────────────────────

std::unique_ptr<Ort::Session> OnnxEngine::create_cpu_session(
    const std::string& model_path) const
{
    Ort::SessionOptions opts;
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL);

    printf("[OnnxEngine] Creating CPU session → %s\n", model_path.c_str());
    return std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);
}

// ─── CUDA ────────────────────────────────────────────────────────────────────

std::unique_ptr<Ort::Session> OnnxEngine::create_cuda_session(
    const std::string& model_path) const
{
    Ort::SessionOptions opts;
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    OrtCUDAProviderOptions cuda_opts{};
    cuda_opts.device_id = cfg_.device_id;
    opts.AppendExecutionProvider_CUDA(cuda_opts);

    printf("[OnnxEngine] Creating CUDA session (device %d) → %s\n",
           cfg_.device_id, model_path.c_str());
    return std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);
}

// ─── TensorRT ────────────────────────────────────────────────────────────────

std::unique_ptr<Ort::Session> OnnxEngine::create_tensorrt_session(
    const std::string& model_path,
    const std::string& cache_prefix,
    const std::string& precision) const
{
    std::filesystem::create_directories(cfg_.cache_dir);

    Ort::SessionOptions opts;
    // ORT's own optimiser rewrites QDQ pairs (folding Quantize away, fusing into
    // com.microsoft QLinear ops) before the TensorRT EP sees the graph, which leaves
    // TensorRT with DequantizeLinear nodes fed by float tensors — it then rejects every
    // subgraph and silently falls back to CUDA, where int8 is slower than fp32. Hand the
    // quantised graph to TensorRT untouched and let it do the fusing.
    opts.SetGraphOptimizationLevel(precision == "int8"
                                       ? GraphOptimizationLevel::ORT_DISABLE_ALL
                                       : GraphOptimizationLevel::ORT_ENABLE_ALL);

    const auto& api = Ort::GetApi();
    OrtTensorRTProviderOptionsV2* trt_opts = nullptr;
    Ort::ThrowOnError(api.CreateTensorRTProviderOptions(&trt_opts));

    const std::string fp16_flag   = (precision == "fp16") ? "1" : "0";
    const std::string device_str  = std::to_string(cfg_.device_id);
    const std::string ws_str      = std::to_string(
        static_cast<size_t>(cfg_.workspace_gb * 1024.0 * 1024.0 * 1024.0));
    // Per-model prefix lets each model cache its own TRT engine file
    const std::string full_prefix = cache_prefix + precision + "_";

    // Builder tactic search: level 5 is exhaustive, and for an int8 QDQ graph it explores
    // enough int8/fp16 kernel combinations to peak past 13 GB of *host* RAM — enough to get
    // the process OOM-killed on a 16 GB machine. Level 3 (TensorRT's default) builds the
    // same int8 kernels within about 3 GB. Engines are cached, so this cost is one-off.
    const char* builder_level = (precision == "int8") ? "3" : "5";

    // Minimum nodes before a subgraph is worth handing to TensorRT. Quantised graphs get
    // partitioned around the nodes left in fp32, and the small leftover fragments are the
    // ones that fail to build — AutoDrive's head fragment ends up needing a shape-tensor
    // input that the TensorRT EP cannot supply an optimisation profile for, which aborts
    // the whole session. Leave those fragments to the CUDA fallback; the big backbone
    // subgraphs, where the time actually goes, still run as int8 TensorRT engines.
    const char* min_subgraph = (precision == "int8") ? "20" : "1";

    const std::vector<const char*> keys = {
        "device_id",
        "trt_max_workspace_size",
        "trt_fp16_enable",
        "trt_engine_cache_enable",
        "trt_engine_cache_path",
        "trt_engine_cache_prefix",
        "trt_timing_cache_enable",
        "trt_timing_cache_path",
        "trt_builder_optimization_level",
        "trt_min_subgraph_size",
    };
    const std::vector<const char*> vals = {
        device_str.c_str(),
        ws_str.c_str(),
        fp16_flag.c_str(),
        "1",
        cfg_.cache_dir.c_str(),
        full_prefix.c_str(),
        "1",
        cfg_.cache_dir.c_str(),
        builder_level,
        min_subgraph,
    };

    Ort::ThrowOnError(api.UpdateTensorRTProviderOptions(
        trt_opts, keys.data(), vals.data(), keys.size()));

    opts.AppendExecutionProvider_TensorRT_V2(*trt_opts);

    // CUDA fallback for any subgraph TRT cannot handle
    OrtCUDAProviderOptions cuda_opts{};
    cuda_opts.device_id = cfg_.device_id;
    opts.AppendExecutionProvider_CUDA(cuda_opts);

    api.ReleaseTensorRTProviderOptions(trt_opts);

    printf("[OnnxEngine] Creating TensorRT session (%s, device %d, prefix=%s) → %s\n",
           precision.c_str(), cfg_.device_id,
           full_prefix.c_str(), model_path.c_str());

    return std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);
}

}  // namespace visionpilot::engine
