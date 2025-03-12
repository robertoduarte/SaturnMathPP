#pragma once

#include "../impl/trigonometry.hpp"
#include "../impl/angle.hpp"

namespace SaturnMath::Tests
{
    using namespace SaturnMath::Types;

    /**
     * @brief Static assertion tests for the Trigonometry class
     *
     * This file contains compile-time tests for the Trigonometry class.
     * All tests are performed using static_assert, ensuring that the
     * functionality is verified at compile time.
     *
     * Note: The Angle class is designed to use turns as the primary unit of measure
     * for efficiency reasons. While degrees and radians are supported for convenience,
     * using turns directly is recommended for optimal performance.
     *
     * For readability in these tests, we primarily use the FromDegrees method to create
     * angles, as this makes the test cases more intuitive to understand. In production code,
     * using turns directly would be more efficient.
     *
     * 1 turn = 360 degrees = 2π radians
     */
    struct TrigonometryTests
    {
        /**
         * @brief Tests for the sine function using turns
         *
         * Verifies that:
         * - Sine function produces correct values at key points in the unit circle
         * - Values match expected mathematical results
         * - Function handles full range of inputs correctly
         */
        static constexpr void TestSine()
        {
            // Test sine of 0°
            constexpr Angle angle0 = Angle::FromDegrees(0);
            constexpr Fxp sin0 = Trigonometry::Sin(angle0);
            static_assert(sin0 == 0, "Sin(0°) should be 0");

            // Test sine of 30°
            constexpr Angle angle30 = Angle::FromDegrees(30);
            constexpr Fxp sin30 = Trigonometry::Sin(angle30);
            static_assert(sin30 > 0.49 && sin30 < 0.51, "Sin(30°) should be approximately 0.5");

            // Test sine of 45°
            constexpr Angle angle45 = Angle::FromDegrees(45);
            constexpr Fxp sin45 = Trigonometry::Sin(angle45);
            static_assert(sin45 > 0.7 && sin45 < 0.71, "Sin(45°) should be approximately 0.7071");

            // Test sine of 90°
            constexpr Angle angle90 = Angle::FromDegrees(90);
            constexpr Fxp sin90 = Trigonometry::Sin(angle90);
            static_assert(sin90 == 1, "Sin(90°) should be 1");

            // Test sine of 180°
            constexpr Angle angle180 = Angle::FromDegrees(180);
            constexpr Fxp sin180 = Trigonometry::Sin(angle180);
            static_assert(sin180 == 0, "Sin(180°) should be 0");

            // Test sine of 270°
            constexpr Angle angle270 = Angle::FromDegrees(270);
            constexpr Fxp sin270 = Trigonometry::Sin(angle270);
            static_assert(sin270 == -1, "Sin(270°) should be -1");

            // Test sine of 360°
            constexpr Angle angle360 = Angle::FromDegrees(360);
            constexpr Fxp sin360 = Trigonometry::Sin(angle360);
            static_assert(sin360 == 0, "Sin(360°) should be 0");

            // Test sine of negative angles
            constexpr Angle angleNeg90 = Angle::FromDegrees(-90);
            constexpr Fxp sinNeg90 = Trigonometry::Sin(angleNeg90);
            static_assert(sinNeg90 == -1, "Sin(-90°) should be -1");
        }

        /**
         * @brief Tests for the cosine function using turns
         *
         * Verifies that:
         * - Cosine function produces correct values at key points in the unit circle
         * - Values match expected mathematical results
         * - Function handles full range of inputs correctly
         */
        static constexpr void TestCosine()
        {
            // Test cosine of 0°
            constexpr Angle angle0 = Angle::FromDegrees(0);
            constexpr Fxp cos0 = Trigonometry::Cos(angle0);
            static_assert(cos0 == 1, "Cos(0°) should be 1");

            // Test cosine of 30°
            constexpr Angle angle30 = Angle::FromDegrees(30);
            constexpr Fxp cos30 = Trigonometry::Cos(angle30);
            static_assert(cos30 > 0.86 && cos30 < 0.87, "Cos(30°) should be approximately 0.866");

            // Test cosine of 45°
            constexpr Angle angle45 = Angle::FromDegrees(45);
            constexpr Fxp cos45 = Trigonometry::Cos(angle45);
            static_assert(cos45 > 0.7 && cos45 < 0.71, "Cos(45°) should be approximately 0.7071");

            // Test cosine of 90°
            constexpr Angle angle90 = Angle::FromDegrees(90);
            constexpr Fxp cos90 = Trigonometry::Cos(angle90);
            static_assert(cos90 == 0, "Cos(90°) should be 0");

            // Test cosine of 180°
            constexpr Angle angle180 = Angle::FromDegrees(180);
            constexpr Fxp cos180 = Trigonometry::Cos(angle180);
            static_assert(cos180 == -1, "Cos(180°) should be -1");

            // Test cosine of 270°
            constexpr Angle angle270 = Angle::FromDegrees(270);
            constexpr Fxp cos270 = Trigonometry::Cos(angle270);
            static_assert(cos270 == 0, "Cos(270°) should be 0");

            // Test cosine of 360°
            constexpr Angle angle360 = Angle::FromDegrees(360);
            constexpr Fxp cos360 = Trigonometry::Cos(angle360);
            static_assert(cos360 == 1, "Cos(360°) should be 1");

            // Test cosine of negative angles
            constexpr Angle angleNeg90 = Angle::FromDegrees(-90);
            constexpr Fxp cosNeg90 = Trigonometry::Cos(angleNeg90);
            static_assert(cosNeg90 == 0, "Cos(-90°) should be 0");
        }

        /**
         * @brief Tests for the tangent function using turns
         *
         * Verifies that:
         * - Tangent function produces correct values at key points in the unit circle
         * - Values match expected mathematical results
         * - Function handles full range of inputs correctly
         */
        static constexpr void TestTangent()
        {
            // Test tangent of 0°
            constexpr Angle angle0 = Angle::FromDegrees(0);
            constexpr Fxp tan0 = Trigonometry::Tan(angle0);
            static_assert(tan0 == 0, "Tan(0°) should be 0");

            // Test tangent of 45°
            constexpr Angle angle45 = Angle::FromDegrees(45);
            constexpr Fxp tan45 = Trigonometry::Tan(angle45);
            static_assert(tan45 > 0.99 && tan45 < 1.01, "Tan(45°) should be approximately 1");

            // Test tangent of 180°
            constexpr Angle angle180 = Angle::FromDegrees(180);
            constexpr Fxp tan180 = Trigonometry::Tan(angle180);
            static_assert(tan180 == 0, "Tan(180°) should be 0");

            // Test tangent of 360°
            constexpr Angle angle360 = Angle::FromDegrees(360);
            constexpr Fxp tan360 = Trigonometry::Tan(angle360);
            static_assert(tan360 == 0, "Tan(360°) should be 0");

            // Test tangent of negative angles
            constexpr Angle angleNeg45 = Angle::FromDegrees(-45);
            constexpr Fxp tanNeg45 = Trigonometry::Tan(angleNeg45);
            static_assert(tanNeg45 > -1.01 && tanNeg45 < -0.99, "Tan(-45°) should be approximately -1");
        }


        /**
         * @brief Tests for the Pythagorean identity using turns
         *
         * Verifies that:
         * - The fundamental identity sin²(θ) + cos²(θ) = 1 holds for all angles
         * - Tests across full range of the unit circle
         */
        static constexpr void TestPythagoreanIdentities()
        {
            // Test sin²(θ) + cos²(θ) = 1 for various angles
            constexpr auto TestIdentity = [](const Angle& angle)
            {
                Fxp sin = Trigonometry::Sin(angle);
                Fxp cos = Trigonometry::Cos(angle);
                Fxp sum = sin * sin + cos * cos;
                // Allow small epsilon for fixed-point rounding errors
                return sum > 0.99 && sum < 1.01;
            };

            static_assert(TestIdentity(Angle::FromDegrees(0)),
                "Pythagorean identity should hold for 0°");
            static_assert(TestIdentity(Angle::FromDegrees(30)),
                "Pythagorean identity should hold for 30°");
            static_assert(TestIdentity(Angle::FromDegrees(45)),
                "Pythagorean identity should hold for 45°");
            static_assert(TestIdentity(Angle::FromDegrees(60)),
                "Pythagorean identity should hold for 60°");
            static_assert(TestIdentity(Angle::FromDegrees(90)),
                "Pythagorean identity should hold for 90°");
            static_assert(TestIdentity(Angle::FromDegrees(180)),
                "Pythagorean identity should hold for 180°");
            static_assert(TestIdentity(Angle::FromDegrees(270)),
                "Pythagorean identity should hold for 270°");
            static_assert(TestIdentity(Angle::FromDegrees(360)),
                "Pythagorean identity should hold for 360°");
        }

        /**
         * @brief Tests for spherical linear interpolation (SLerp) using turns
         *
         * Verifies that:
         * - SLerp works correctly for various interpolation factors
         * - Edge cases (t=0 and t=1) produce expected results
         * - Interpolation is accurate at intermediate points
         * - Shortest path interpolation works correctly
         */
        static constexpr void TestInterpolation()
        {
            constexpr Angle angle0 = Angle::FromDegrees(0);
            constexpr Angle angle90 = Angle::FromDegrees(90);

            // Linear interpolation
            constexpr Angle lerp25 = Trigonometry::SLerp(angle0, angle90, 0.25);
            static_assert(lerp25.ToDegrees() == 22.5, "SLerp(0°, 90°, 0.25) should be 22.5°");

            constexpr Angle lerp50 = Trigonometry::SLerp(angle0, angle90, 0.5);
            static_assert(lerp50.ToDegrees() == 45, "SLerp(0°, 90°, 0.5) should be 45°");

            constexpr Angle lerp75 = Trigonometry::SLerp(angle0, angle90, 0.75);
            static_assert(lerp75.ToDegrees() == 67.5, "SLerp(0°, 90°, 0.75) should be 67.5°");

            // Edge cases
            constexpr Angle lerp0 = Trigonometry::SLerp(angle0, angle90, 0);
            static_assert(lerp0.ToDegrees() == 0, "SLerp(0°, 90°, 0) should be 0°");

            constexpr Angle lerp1 = Trigonometry::SLerp(angle0, angle90, 1);
            static_assert(lerp1.ToDegrees() == 90, "SLerp(0°, 90°, 1) should be 90°");

            // Test with angles spanning more than 90 degrees
            constexpr Angle angle180 = Angle::FromDegrees(180);
            constexpr Angle lerp180_50 = Trigonometry::SLerp(angle0, angle180, 0.5);
            static_assert(lerp180_50.ToDegrees() == 90, "SLerp(0°, 180°, 0.5) should be 90°");

            // Test with negative angles
            constexpr Angle angleNeg90 = Angle::FromDegrees(-90);
            constexpr Angle lerpNeg_50 = Trigonometry::SLerp(angle0, angleNeg90, 0.5);
            static_assert(lerpNeg_50.ToDegrees() == 135,
                "SLerp(0°, -90°, 0.5) should be 135° because -90° wraps to 270° and the shortest path is clockwise");
        }

        /**
         * @brief Run all tests
         *
         * Executes all test functions to verify the Trigonometry class functionality
         */
        static constexpr void RunAll()
        {
            TestSine();
            TestCosine();
            TestTangent();
            TestPythagoreanIdentities();
            TestInterpolation();
        }
    };

    // Execute all tests
    static_assert((TrigonometryTests::RunAll(), true), "Trigonometry tests failed");
}
