#pragma once

#include "../impl/integer.hpp"

namespace SaturnMath::Tests
{
    using namespace SaturnMath::Types;

    /**
     * @brief Static assertion tests for the Integer class
     *
     * This file contains compile-time tests for the Integer class,
     * focusing on FastSqrt and FastSqrt64 approximation algorithms.
     * All tests are performed using static_assert, ensuring that the
     * functionality is verified at compile time.
     */
    struct IntegerTests
    {
        static constexpr void TestFastSqrt32()
        {
            // FastSqrt is an approximation algorithm, not exact
            static_assert(Integer::FastSqrt(1u) == 1, "FastSqrt(1)");
            static_assert(Integer::FastSqrt(4u) == 2, "FastSqrt(4)");
            static_assert(Integer::FastSqrt(9u) == 3, "FastSqrt(9)");
            static_assert(Integer::FastSqrt(16u) == 4, "FastSqrt(16)");
            static_assert(Integer::FastSqrt(100u) == 11, "FastSqrt(100) approx");
            static_assert(Integer::FastSqrt(10000u) == 103, "FastSqrt(10000) approx");
            static_assert(Integer::FastSqrt(1000000u) == 1000, "FastSqrt(1000000)");
        }

        static constexpr void TestFastSqrt64()
        {
            // FastSqrt64 should match FastSqrt for 32-bit inputs (hi=0)
            static_assert(Integer::FastSqrt(0u, 0u) == 0, "FastSqrt64(0)");
            static_assert(Integer::FastSqrt(0u, 1u) == Integer::FastSqrt(1u), "FastSqrt64(1) matches FastSqrt");
            static_assert(Integer::FastSqrt(0u, 4u) == Integer::FastSqrt(4u), "FastSqrt64(4) matches FastSqrt");
            static_assert(Integer::FastSqrt(0u, 9u) == Integer::FastSqrt(9u), "FastSqrt64(9) matches FastSqrt");
            static_assert(Integer::FastSqrt(0u, 16u) == Integer::FastSqrt(16u), "FastSqrt64(16) matches FastSqrt");
            static_assert(Integer::FastSqrt(0u, 100u) == Integer::FastSqrt(100u), "FastSqrt64(100) matches FastSqrt");
            static_assert(Integer::FastSqrt(0u, 10000u) == Integer::FastSqrt(10000u), "FastSqrt64(10000) matches FastSqrt");
            static_assert(Integer::FastSqrt(0u, 1000000u) == Integer::FastSqrt(1000000u), "FastSqrt64(1000000) matches FastSqrt");
            static_assert(Integer::FastSqrt(0u, 0xFFFFFFFFu) == Integer::FastSqrt(0xFFFFFFFFu), "FastSqrt64(max32) matches FastSqrt");

            // 64-bit value: 4294967296 = 2^32, sqrt = 65536
            static_assert(Integer::FastSqrt(1u, 0u) == 65536, "FastSqrt64(2^32) should be 65536");
        }

        static constexpr void TestFastSqrt_EdgeCases()
        {
            // Zero: FastSqrt(0) returns 1 (baseEstimation starts at 1, estimation=0, loop doesn't run)
            static_assert(Integer::FastSqrt(0u) == 1, "FastSqrt(0) = 1 (approximation quirk)");
            // FastSqrt64(0,0) has an early return for zero
            static_assert(Integer::FastSqrt(0u, 0u) == 0, "FastSqrt64(0,0) = 0");

            // One
            static_assert(Integer::FastSqrt(1u) == 1, "FastSqrt(1) = 1");
            static_assert(Integer::FastSqrt(0u, 1u) == Integer::FastSqrt(1u), "FastSqrt64(0,1) matches FastSqrt(1)");

            // Max 32-bit value
            constexpr uint32_t max32 = 0xFFFFFFFFu;
            constexpr uint32_t sqrtMax32 = Integer::FastSqrt(max32);
            static_assert(sqrtMax32 > 65000 && sqrtMax32 < 66000, "FastSqrt(0xFFFFFFFF) ~65536");

            // Large 64-bit value: sqrt(2^48) = 2^24 = 16777216
            // (max 64-bit would overflow uint32_t result, so test a large safe value)
            constexpr uint32_t sqrtBig64 = Integer::FastSqrt(0x00010000u, 0x00000000u);
            static_assert(sqrtBig64 >= 16000000u && sqrtBig64 <= 17000000u, "FastSqrt64(2^48) ~16777216");

            // Power of 2: sqrt(256) = 16
            static_assert(Integer::FastSqrt(256u) == 16, "FastSqrt(256) = 16");
            static_assert(Integer::FastSqrt(0u, 256u) == Integer::FastSqrt(256u), "FastSqrt64(0,256) matches FastSqrt(256)");

            // 2^32: sqrt = 65536
            static_assert(Integer::FastSqrt(1u, 0u) == 65536, "FastSqrt64(2^32) = 65536");

            // 2^32 + 1: sqrt should be ~65536
            constexpr uint32_t sqrt2_32_plus1 = Integer::FastSqrt(1u, 1u);
            static_assert(sqrt2_32_plus1 >= 65536 && sqrt2_32_plus1 <= 65537, "FastSqrt64(2^32+1) ~65536");
        }

        static constexpr void RunAll()
        {
            TestFastSqrt32();
            TestFastSqrt64();
            TestFastSqrt_EdgeCases();
        }
    };

    // Execute all tests
    static_assert((IntegerTests::RunAll(), true), "Integer tests failed");
}
