#pragma once
#include <cstdio>

// ─── Simple printf-style logging macros ───────────────────────────────────────
//
//  VP_INFO("Loading model: %s", path.c_str());
//  VP_WARN("CUDA unavailable, falling back to CPU");
//  VP_ERROR("Failed to open video: %s", path.c_str());
//
// stdout for INFO, stderr for WARN/ERROR.

#define VP_INFO(fmt, ...)  std::printf( "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define VP_WARN(fmt, ...)  std::fprintf(stderr, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define VP_ERROR(fmt, ...) std::fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)
