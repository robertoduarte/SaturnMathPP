#pragma once

#include "../impl/constmath.hpp"

namespace SaturnMath::Tests
{
    using namespace SaturnMath::Types;

    /**
     * @brief Static assertion tests for the ConstexprMath class
     *
     * This file contains compile-time tests for the ConstexprMath class.
     * All tests are performed using static_assert, ensuring that the
     * functionality is verified at compile time.
     */
    struct ConstexprMathTests
    {
        static constexpr void TestSqrt()
        {
            static_assert(ConstexprMath::Sqrt(0.0) == 0.0, "Sqrt(0) should be 0");
            static_assert(ConstexprMath::Sqrt(1.0) == 1.0, "Sqrt(1) should be 1");
            static_assert(ConstexprMath::Sqrt(4.0) == 2.0, "Sqrt(4) should be 2");
            static_assert(ConstexprMath::Sqrt(9.0) == 3.0, "Sqrt(9) should be 3");
            static_assert(ConstexprMath::Sqrt(16.0) == 4.0, "Sqrt(16) should be 4");
            static_assert(ConstexprMath::Sqrt(100.0) == 10.0, "Sqrt(100) should be 10");

            constexpr double sqrt2 = ConstexprMath::Sqrt(2.0);
            static_assert(sqrt2 > 1.414 && sqrt2 < 1.415, "Sqrt(2) should be ~1.414");

            constexpr double sqrtHalf = ConstexprMath::Sqrt(0.5);
            static_assert(sqrtHalf > 0.707 && sqrtHalf < 0.708, "Sqrt(0.5) should be ~0.707");
        }

        static constexpr void TestSinCos()
        {
            constexpr double sin0 = ConstexprMath::Sin(0.0);
            static_assert(sin0 > -1e-10 && sin0 < 1e-10, "Sin(0) should be 0");

            constexpr double sinPi2 = ConstexprMath::Sin(3.14159265358979 / 2.0);
            static_assert(sinPi2 > 0.99999 && sinPi2 < 1.00001, "Sin(pi/2) should be 1");

            constexpr double cos0 = ConstexprMath::Cos(0.0);
            static_assert(cos0 > 0.99999 && cos0 < 1.00001, "Cos(0) should be 1");

            constexpr double cosPi = ConstexprMath::Cos(3.14159265358979);
            static_assert(cosPi > -1.00001 && cosPi < -0.99999, "Cos(pi) should be -1");

            constexpr double sinPi4 = ConstexprMath::Sin(3.14159265358979 / 4.0);
            static_assert(sinPi4 > 0.707 && sinPi4 < 0.708, "Sin(pi/4) should be ~0.707");
        }

        static constexpr void TestTan()
        {
            constexpr double tan0 = ConstexprMath::Tan(0.0);
            static_assert(tan0 > -1e-10 && tan0 < 1e-10, "Tan(0) should be 0");

            constexpr double tanPi4 = ConstexprMath::Tan(3.14159265358979 / 4.0);
            static_assert(tanPi4 > 0.999 && tanPi4 < 1.001, "Tan(pi/4) should be ~1");
        }

        static constexpr void TestAtan()
        {
            constexpr double atan0 = ConstexprMath::Atan(0.0);
            static_assert(atan0 > -1e-10 && atan0 < 1e-10, "Atan(0) should be 0");

            constexpr double atan1 = ConstexprMath::Atan(1.0);
            static_assert(atan1 > 0.785 && atan1 < 0.786, "Atan(1) should be ~pi/4");

            constexpr double atan2_1_1 = ConstexprMath::Atan2(1.0, 1.0);
            static_assert(atan2_1_1 > 0.785 && atan2_1_1 < 0.786, "Atan2(1,1) should be ~pi/4");

            constexpr double atan2_1_0 = ConstexprMath::Atan2(1.0, 0.0);
            static_assert(atan2_1_0 > 1.5707 && atan2_1_0 < 1.5708, "Atan2(1,0) should be ~pi/2");
        }

        static constexpr void RunAll()
        {
            TestSqrt();
            TestSinCos();
            TestTan();
            TestAtan();
        }
    };

    // Execute all tests
    static_assert((ConstexprMathTests::RunAll(), true), "ConstexprMath tests failed");
}
