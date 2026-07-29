// Build the TensorRT engine for ONE model, then exit.
//
// VisionPilot itself creates all three sessions in a single process, so on the first
// TensorRT run three engine builds pile their peak allocations on top of each other —
// around 13 GB of host RAM for int8, enough to get the process OOM-killed on a 16 GB
// laptop. Building one model per process caps the peak at a single build, and the
// serialised engines land in the same cache VisionPilot reads afterwards.
//
// Usage (from the build directory, so config/ and weights resolve):
//     ./build_trt_engines autodrive
//     ./build_trt_engines all          # one child process per model
//
// scripts/warm_trt_cache.sh wraps the `all` case with a memory cap.

#include <config/vision_pilot_config.hpp>
#include <engine/onnx_engine.hpp>
#include <logging/logger.hpp>

#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

namespace {

constexpr std::array<const char*, 3> kModels{"autodrive", "autosteer", "autospeed"};

std::string weights_path(const std::string& model, const std::string& precision)
{
    const std::string local = "modules/models/weights/" + model + "_" + precision + ".onnx";
    if (std::filesystem::exists(local)) return local;

    const std::string system =
        "/usr/share/visionpilot/modules/models/weights/" + model + "_" + precision + ".onnx";
    if (std::filesystem::exists(system)) return system;

    throw std::runtime_error("weights not found: " + local);
}

std::string precision_for(const Config& cfg, const std::string& model)
{
    if (model == "autodrive") return cfg.inference.precision_autodrive();
    if (model == "autosteer") return cfg.inference.precision_autosteer();
    if (model == "autospeed") return cfg.inference.precision_autospeed();
    return cfg.inference.precision;
}

int build_one(const Config& cfg, const std::string& model)
{
    const std::string precision = precision_for(cfg, model);
    const std::string path = weights_path(model, precision);
    VP_INFO("[warmup] %s (%s) — building, this can take several minutes",
            model.c_str(), precision.c_str());

    const auto t0 = std::chrono::steady_clock::now();
    try {
        visionpilot::engine::OnnxEngine engine(cfg.engine);
        auto session = engine.create_session(path, model + "_", precision);

        // The TensorRT EP defers the engine build to the first inference, so creating the
        // session is not enough to fill the cache — run one zero-filled frame through it.
        Ort::AllocatorWithDefaultOptions alloc;
        auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        std::vector<std::string> in_names_str, out_names_str;
        std::vector<const char*> in_names, out_names;
        std::vector<std::vector<int64_t>> shapes;
        std::vector<std::vector<float>> buffers;
        std::vector<Ort::Value> inputs;

        for (size_t i = 0; i < session->GetInputCount(); ++i) {
            in_names_str.push_back(session->GetInputNameAllocated(i, alloc).get());
            auto shape = session->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
            size_t n = 1;
            for (auto& d : shape) { if (d < 0) d = 1; n *= static_cast<size_t>(d); }
            shapes.push_back(std::move(shape));
            buffers.emplace_back(n, 0.f);
        }
        for (size_t i = 0; i < in_names_str.size(); ++i) {
            in_names.push_back(in_names_str[i].c_str());
            inputs.push_back(Ort::Value::CreateTensor<float>(
                mem, buffers[i].data(), buffers[i].size(),
                shapes[i].data(), shapes[i].size()));
        }
        for (size_t i = 0; i < session->GetOutputCount(); ++i)
            out_names_str.push_back(session->GetOutputNameAllocated(i, alloc).get());
        for (const auto& s : out_names_str) out_names.push_back(s.c_str());

        session->Run(Ort::RunOptions{nullptr}, in_names.data(), inputs.data(), inputs.size(),
                     out_names.data(), out_names.size());
    } catch (const std::exception& e) {
        VP_ERROR("[warmup] %s failed: %s", model.c_str(), e.what());
        return 1;
    }
    const double secs =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    VP_INFO("[warmup] %s done in %.0f s", model.c_str(), secs);
    return 0;
}

}  // namespace

int main(int argc, char** argv)
{
    Config cfg;
    try { cfg = load_vision_pilot_config(); }
    catch (const std::exception& e) { VP_ERROR("Config: %s", e.what()); return 1; }

    if (cfg.engine.provider != "tensorrt")
        VP_WARN("[warmup] engine.provider = %s — only the tensorrt provider caches engines",
                cfg.engine.provider.c_str());

    const std::string arg = (argc > 1) ? argv[1] : "all";

    if (arg != "all") {
        return build_one(cfg, arg);
    }

    // One child process per model: the OS reclaims each build's memory on exit.
    int failures = 0;
    for (const char* model : kModels) {
        const pid_t pid = fork();
        if (pid < 0) { VP_ERROR("[warmup] fork failed"); return 1; }
        if (pid == 0) _exit(build_one(cfg, model));

        int status = 0;
        waitpid(pid, &status, 0);
        if (WIFSIGNALED(status)) {
            VP_ERROR("[warmup] %s was killed by signal %d — most likely the OOM killer. "
                     "Lower trt_builder_optimization_level or engine.workspace_gb.",
                     model, WTERMSIG(status));
            ++failures;
        } else if (WEXITSTATUS(status) != 0) {
            ++failures;
        }
    }

    VP_INFO("[warmup] %s", failures ? "finished with failures" : "all engines cached");
    return failures ? 1 : 0;
}
