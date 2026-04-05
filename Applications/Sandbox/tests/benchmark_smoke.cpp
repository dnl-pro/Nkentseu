#include <Unitest/Unitest.h>
#include <Unitest/TestMacro.h>
#include "NKLogger/NkLog.h"
#include <chrono>

TEST_CASE(BenchmarkSuite, BenchmarkColorPack) {
    constexpr int n = 1000000;
    volatile unsigned int r = 0;
    auto t0 = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < n; ++i)
        r ^= ((i & 0xF0u) << 24) | ((i & 0x0Fu) << 16) | (((i*2u) & 0xFFu) << 8) | 0xAAu;

    auto t1 = std::chrono::high_resolution_clock::now();
    double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();

    ASSERT_TRUE(ns > 0.0);
    logger.Info("[BenchmarkSuite] Color packing completed: {0} ns (r={1})", ns, r);
}