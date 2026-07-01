#pragma once

#include "../impl/utils.hpp"
#include "../impl/fxp.hpp"

namespace SaturnMath::Tests
{
    using namespace SaturnMath::Types;

    /**
     * @brief Static assertion tests for the Utils functions
     *
     * This file contains compile-time tests for utility functions
     * such as Abs, Min, Max, and Clamp.
     * All tests are performed using static_assert, ensuring that the
     * functionality is verified at compile time.
     */
    struct UtilsTests
    {
        static constexpr void TestAbs()
        {
            static_assert(SaturnMath::Abs(5) == 5, "Abs(5)");
            static_assert(SaturnMath::Abs(-5) == 5, "Abs(-5)");
            static_assert(SaturnMath::Abs(0) == 0, "Abs(0)");
            static_assert(SaturnMath::Abs(-1) == 1, "Abs(-1)");

            constexpr Fxp a(3.5);
            constexpr Fxp b(-3.5);
            static_assert(SaturnMath::Abs(a) == a, "Abs(3.5)");
            static_assert(SaturnMath::Abs(b) == a, "Abs(-3.5)");
        }

        static constexpr void TestMinMax()
        {
            static_assert(SaturnMath::Max(3, 5) == 5, "Max(3,5)");
            static_assert(SaturnMath::Max(5, 3) == 5, "Max(5,3)");
            static_assert(SaturnMath::Min(3, 5) == 3, "Min(3,5)");
            static_assert(SaturnMath::Min(5, 3) == 3, "Min(5,3)");
            static_assert(SaturnMath::Min(1, 2, 3) == 1, "Min(1,2,3)");
            static_assert(SaturnMath::Min(3, 2, 1) == 1, "Min(3,2,1)");
            static_assert(SaturnMath::Min(2, 1, 3) == 1, "Min(2,1,3)");
        }

        static constexpr void TestClamp()
        {
            static_assert(SaturnMath::Clamp(5, 1, 10) == 5, "Clamp(5,1,10)");
            static_assert(SaturnMath::Clamp(0, 1, 10) == 1, "Clamp(0,1,10)");
            static_assert(SaturnMath::Clamp(15, 1, 10) == 10, "Clamp(15,1,10)");
            static_assert(SaturnMath::Clamp(-5, -10, 10) == -5, "Clamp(-5,-10,10)");
        }

        static constexpr void RunAll()
        {
            TestAbs();
            TestMinMax();
            TestClamp();
        }
    };

    // Execute all tests
    static_assert((UtilsTests::RunAll(), true), "Utils tests failed");
}
