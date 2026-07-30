#include "engine/onnx_engine.hpp"

#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace visionpilot::engine {

std::string inference_error_hint(const std::string& what)
{
    const bool out_of_memory =
        what.find("Failed to allocate memory") != std::string::npos ||
        what.find("out of memory")             != std::string::npos ||
        what.find("CUDA_ERROR_OUT_OF_MEMORY")  != std::string::npos ||
        what.find("cudaErrorMemoryAllocation") != std::string::npos;

    if (!out_of_memory) return "";
    return "\n  → the GPU is out of memory. VisionPilot needs roughly 0.6 GB; check what else"
           "\n    is using the card (nvidia-smi), or cap this process with"
           " engine.gpu_mem_limit_gb.";
}

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

// ─── Shared CUDA allocator settings ──────────────────────────────────────────
// Each of the three sessions grows its own CUDA arena. kSameAsRequested extends by what
// was actually asked for instead of rounding up to the next power of two, which keeps
// the three arenas predictable; measured peak here is ~550 MB either way, so this is
// steadiness rather than a saving. gpu_mem_limit_gb caps what VisionPilot will hold, so
// it can be told to leave room for other GPU applications.
//
// If inference fails mid-run with "Failed to allocate memory for requested buffer", the
// GPU is full — usually because something else on the machine (browser, IDE) is holding
// it. VisionPilot itself still runs with a 0.45 GB cap.
void OnnxEngine::configure_cuda(OrtCUDAProviderOptions& cuda_opts) const
{
    cuda_opts.device_id = cfg_.device_id;
    cuda_opts.arena_extend_strategy = 1;  // kSameAsRequested
    if (cfg_.gpu_mem_limit_gb > 0.0) {
        cuda_opts.gpu_mem_limit =
            static_cast<size_t>(cfg_.gpu_mem_limit_gb * 1024.0 * 1024.0 * 1024.0);
    }
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
    configure_cuda(cuda_opts);
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

    // Builder tactic search. Level 5 is exhaustive and explores enough kernel combinations
    // to peak past 13 GB of *host* RAM while building all three models, which is enough to
    // get the process OOM-killed on a 16 GB machine — it happened for both int8 and fp32.
    // Level 3 is TensorRT's own default and builds the same kernels in a fraction of that.
    // Engines are cached, so the tactic search is a one-off cost either way.
    const char* builder_level = "3";

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
    configure_cuda(cuda_opts);
    opts.AppendExecutionProvider_CUDA(cuda_opts);

    api.ReleaseTensorRTProviderOptions(trt_opts);

    printf("[OnnxEngine] Creating TensorRT session (%s, device %d, prefix=%s) → %s\n",
           precision.c_str(), cfg_.device_id,
           full_prefix.c_str(), model_path.c_str());

    return std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);
}

}  // namespace visionpilot::engine
