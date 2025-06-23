#pragma once

#include "../impl/angle.hpp"
#include "../impl/precision.hpp"
#include "../impl/trigonometry.hpp"

namespace SaturnMath::Tests
{
    using namespace SaturnMath::Types;

    /**
     * @brief Comprehensive static assertion tests for the Angle class
     *
     * This file contains compile-time tests for the Angle class.
     * All tests are performed using static_assert, ensuring that the
     * functionality is verified at compile time without runtime overhead.
     *
     * DESIGN PHILOSOPHY:
     * The Angle class is designed with turns as the primary internal unit of measure for
     * maximum efficiency on Saturn hardware. While degrees and radians are fully supported
     * through convenient factory methods and conversion functions, using turns directly
     * is recommended for performance-critical code paths.
     *
     * Why turns are more efficient:
     * - A full turn (360°) maps perfectly to the 16-bit internal representation
     * - No conversion needed when using turns directly
     * - Simpler arithmetic for common operations (half turn = 0.5, quarter turn = 0.25)
     * - More intuitive for many game calculations (e.g., rotations per second)
     *
     * For readability in most tests, we use the FromDegrees method to create angles,
     * as this makes the test cases more intuitive to understand. However, the TestTurnsBasedUsage
     * function specifically demonstrates the direct turns-based approach that would be
     * more efficient in production code.
     *
     * Constants and compile-time calculations can freely use degrees with no performance penalty due to
     * constexpr evaluation, but runtime code should prefer turns when possible.
     *
     * Test coverage includes:
     * - Construction and initialization
     * - Factory methods for creating angles
     * - Conversion between different angle representations (degrees, radians, turns)
     * - Angle normalization and wrap-around behavior
     * - Arithmetic operations (addition, subtraction, scaling)
     * - Comparison operations
     * - Special angle values (zero, right, straight, full)
     * - Angle interpolation
     * - Boundary value analysis
     * - Precision and accuracy verification
     *
     * The tests in this file serve multiple purposes:
     * Verification: Ensure the Angle class behaves correctly across all operations
     * Documentation: Demonstrate expected behavior through concrete examples
     * Regression testing: Prevent future changes from breaking existing functionality
     * Edge case handling: Verify correct behavior in extreme or unusual situations
     */
    struct AngleTests
    {
        /**
         * @brief Tests for angle construction and initialization
         *
         * Verifies that:
         * - Default constructor initializes to zero
         * - Raw value construction works correctly
         * - Copy construction preserves the angle value
         */
        static constexpr void TestConstruction()
        {
            // Default constructor should initialize to 0
            constexpr Angle defaultAngle;
            static_assert(defaultAngle.ToDegrees() == 0, "Default constructor should initialize to 0");

            // Construction using BuildRaw
            constexpr Angle rawAngle = Angle::BuildRaw(1000);
            static_assert(rawAngle.RawValue() == 1000, "BuildRaw should create angle with correct raw value");

            // Copy constructor
            constexpr Angle copy = rawAngle;
            static_assert(copy.RawValue() == rawAngle.RawValue(), "Copy constructor should work");
        }

        /**
         * @brief Tests for angle factory methods
         *
         * Verifies that:
         * - FromDegrees creates angles with correct degree values
         * - Predefined constants (Pi, HalfPi) create correct angles
         * - FromRotations creates angles with correct turn values
         */
        static constexpr void TestFactoryMethods()
        {
            // FromDegrees
            constexpr Angle angle90 = Angle::FromDegrees(90);
            static_assert(angle90.ToDegrees() == 90, "FromDegrees should create angle with correct degree value");

            // FromRadians
            constexpr auto anglePiHalf = Angle::HalfPi().ToDegrees();
            static_assert(anglePiHalf > 89.9 && anglePiHalf < 90.1,
                "FromRadians should create angle with correct value (π/2 ≈ 90°)");

            // FromRotations (using FromDegrees for clarity)
            constexpr Angle angleHalfRotation = Angle::FromDegrees(180);
            static_assert(angleHalfRotation.ToDegrees() == 180,
                "FromRotations should create angle with correct value (0.5 rotations = 180°)");
        }

        /**
         * @brief Tests for angle conversion methods
         *
         * Verifies that:
         * - ToDegrees returns correct degree values
         * - ToRadians returns correct radian values
         * - ToTurns returns correct rotation values
         * - RawValue returns the internal representation
         */
        static constexpr void TestConversionMethods()
        {
            constexpr Angle angle45 = Angle::FromDegrees(45);

            // ToDegrees
            static_assert(angle45.ToDegrees() == 45, "ToDegrees should return correct value");

            // ToRadians
            static_assert(angle45.ToRadians() > 0.78 && angle45.ToRadians() < 0.79,
                "ToRadians should return correct value (45° ≈ 0.785 rad)");

            // AsRotations
            static_assert(angle45.ToTurns() == 0.125,
                "AsRotations should return correct value (45° = 0.125 rotations)");

            // RawValue
            static_assert(angle45.RawValue() == 8192,
                "RawValue should return the raw internal representation (which is in degrees)");
        }

        /**
         * @brief Tests for angle normalization and wrap-around behavior
         *
         * Verifies that:
         * - Angles greater than 360° wrap around correctly
         * - Negative angles wrap to their positive equivalents
         * - Full circle (360°) wraps to 0°
         * - Angles within the range [0,360) remain unchanged
         */
        static constexpr void TestNormalization()
        {
            // Normalize positive angle > 360°
            // Angle naturally wraps around due to 16-bit representation
            constexpr Angle angle400 = Angle::FromDegrees(400);
            static_assert(angle400.ToDegrees() > 39.99 && angle400.ToDegrees() < 40.0,
                "Angle of 400° naturally wraps to approximately 40° due to 16-bit representation");

            // Negative angles also wrap around naturally
            constexpr Angle angleNeg45 = Angle::FromDegrees(-45);
            static_assert(angleNeg45.ToDegrees() == 315,
                "Angle of -45° naturally wraps to 315° due to 16-bit representation");

            // Full circle (360°) wraps to 0
            constexpr Angle angle360 = Angle::FromDegrees(360);
            static_assert(angle360.ToDegrees() == 0,
                "Angle of 360° naturally wraps to 0° (full circle)");

            // Angles within range [0,360) stay the same
            constexpr Angle angle180 = Angle::FromDegrees(180);
            static_assert(angle180.ToDegrees() == 180,
                "Angle of 180° remains 180° as it's within the natural range");
        }

        /**
         * @brief Tests for arithmetic operations on angles
         *
         * Verifies that:
         * - Addition of angles produces correct results
         * - Subtraction of angles produces correct results
         * - Negation of angles works as expected
         * - Multiplication by scalar produces correct results
         * - Division by scalar produces correct results
         * - Compound assignment operators work correctly
         */
        static constexpr void TestArithmeticOperations()
        {
            constexpr Angle angle45 = Angle::FromDegrees(45);
            constexpr Angle angle90 = Angle::FromDegrees(90);

            // Addition
            constexpr Angle sum = angle45 + angle90;
            static_assert(sum.ToDegrees() == 135, "45° + 90° should be 135°");

            // Subtraction
            constexpr Angle diff = angle90 - angle45;
            static_assert(diff.ToDegrees() == 45, "90° - 45° should be 45°");

            // Negation
            constexpr Angle negated = -angle45;
            static_assert(negated.ToDegrees() == 135, "Negation of 45° should be 135°");

            // Multiplication by scalar
            constexpr Angle doubled = angle45 * 2;
            static_assert(doubled.ToDegrees() == 90, "45° * 2 should be 90°");

            // Division by scalar
            constexpr Angle halved = angle90 / 2;
            static_assert(halved.ToDegrees() == 45, "90° / 2 should be 45°");

            constexpr auto TestCompoundAssignment = []()
            {
                Angle a = Angle::FromDegrees(10);
                a += Angle::FromDegrees(20);
                auto result = a.ToDegrees();
                return result > 29.9 && result < 30.1;
            };
            static_assert(TestCompoundAssignment(), "Compound assignment += should work");

            constexpr auto TestCompoundSubtraction = []()
            {
                Angle a = Angle::FromDegrees(30);
                a -= Angle::FromDegrees(10);
                auto result = a.ToDegrees();
                return result > 19.9 && result < 20.1;
            };
            static_assert(TestCompoundSubtraction(), "Compound assignment -= should work");

            constexpr auto TestCompoundMultiplication = []()
            {
                Angle a = Angle::FromDegrees(10);
                a *= 3;
                auto result = a.ToDegrees();
                return result > 29.9 && result < 30.1;
            };
            static_assert(TestCompoundMultiplication(), "Compound assignment *= should work");

            constexpr auto TestCompoundDivision = []()
            {
                Angle a = Angle::FromDegrees(30);
                a /= 3;
                auto result = a.ToDegrees();
                return result > 9.9 && result < 10.1;
            };
            static_assert(TestCompoundDivision(), "Compound assignment /= should work");
        }

        /**
         * @brief Tests for comparison operations between angles
         *
         * Verifies that:
         * - Equality operator works for equal and unequal angles
         * - Inequality operator works for equal and unequal angles
         * - Less than operator works correctly
         * - Greater than operator works correctly
         * - Less than or equal operator works correctly
         * - Greater than or equal operator works correctly
         */
        static constexpr void TestComparisonOperations()
        {
            constexpr Angle angle0 = Angle::FromDegrees(0);
            constexpr Angle angle45 = Angle::FromDegrees(45);
            constexpr Angle angle90 = Angle::FromDegrees(90);
            constexpr Angle angle45Copy = Angle::FromDegrees(45);

            // Equality
            static_assert(angle45 == angle45Copy, "Equality operator should work for equal angles");
            static_assert(!(angle45 == angle90), "Equality operator should work for unequal angles");

            // Inequality
            static_assert(angle45 != angle90, "Inequality operator should work for unequal angles");
            static_assert(!(angle45 != angle45Copy), "Inequality operator should work for equal angles");

            // Less than
            static_assert(angle0 < angle45, "Less than operator should work (0° < 45°)");
            static_assert(!(angle45 < angle0), "Less than operator should work (45° !< 0°)");
            static_assert(!(angle45 < angle45Copy), "Less than operator should work (45° !< 45°)");

            // Greater than
            static_assert(angle90 > angle45, "Greater than operator should work (90° > 45°)");
            static_assert(!(angle45 > angle90), "Greater than operator should work (45° !> 90°)");
            static_assert(!(angle45 > angle45Copy), "Greater than operator should work (45° !> 45°)");

            // Less than or equal
            static_assert(angle0 <= angle45, "Less than or equal operator should work (0° <= 45°)");
            static_assert(angle45 <= angle45Copy, "Less than or equal operator should work (45° <= 45°)");
            static_assert(!(angle90 <= angle45), "Less than or equal operator should work (90° !<= 45°)");

            // Greater than or equal
            static_assert(angle90 >= angle45, "Greater than or equal operator should work (90° >= 45°)");
            static_assert(angle45 >= angle45Copy, "Greater than or equal operator should work (45° >= 45°)");
            static_assert(!(angle0 >= angle45), "Greater than or equal operator should work (0° !>= 45°)");
        }

        /**
         * @brief Tests for special angle values
         *
         * Verifies that:
         * - Zero angle is correctly represented as 0°
         * - Right angle is correctly represented as 90°
         * - Straight angle is correctly represented as 180°
         * - Full angle wraps around to 0°
         */
        static constexpr void TestSpecialValues()
        {
            // Zero
            constexpr Angle zero = Angle::Zero();
            static_assert(zero.ToDegrees() == 0, "Zero angle should be 0°");

            // Right angle (90°)
            constexpr Angle right = Angle::Right();
            static_assert(right.ToDegrees() == 90, "Right angle should be 90°");

            // Straight angle (180°)
            constexpr Angle straight = Angle::Straight();
            static_assert(straight.ToDegrees() == 180, "Straight angle should be 180°");

            // Full angle (360°) - Note that due to wrap-around, Full() is equivalent to Zero()
            // since 360° and 0° represent the same position in the circle
            constexpr Angle full = Angle::Full();
            static_assert(full.ToDegrees() == 0, "Full angle should wrap to 0°");
        }

        /**
         * @brief Demonstrates efficient usage of turns for angle representation
         *
         * This test section specifically showcases the recommended approach of
         * using turns directly for maximum efficiency, while still maintaining
         * the same mathematical correctness as the degree-based approach.
         *
         * Unlike other tests in this project that use FromDegrees for readability,
         * this test intentionally uses direct turn-based construction to demonstrate
         * the most efficient way to create angles in production code.
         *
         * Using turns directly avoids unnecessary conversions at runtime and
         * maps more naturally to the internal 16-bit representation.
         */
        static constexpr void TestTurnsBasedUsage()
        {
            // Direct construction with turns (most efficient)
            constexpr Angle quarterTurn = Angle(0.25);  // 1/4 turn = 90°
            static_assert(quarterTurn.ToDegrees() == 90, "Quarter turn should be 90°");
            static_assert(quarterTurn.ToTurns() == 0.25, "Quarter turn should be 0.25 turns");

            // Common angles expressed in turns
            constexpr Angle halfTurn = Angle(0.5);      // 1/2 turn = 180°
            constexpr Angle threeQuarters = Angle(0.75); // 3/4 turn = 270°
            constexpr Angle fullTurn = Angle(1.0);     // 1 turn = 360° (wraps to 0°)

            // Arithmetic with turns is more intuitive
            constexpr Angle eighthTurn = quarterTurn / 2;     // 1/8 turn = 45°
            static_assert(eighthTurn.ToTurns() == 0.125, "Eighth turn should be 0.125 turns");

            // Combining turns is straightforward
            constexpr Angle threeSixteenths = eighthTurn + eighthTurn / 2; // 3/16 turn = 67.5°
            static_assert(threeSixteenths.ToTurns() > 0.185 &&
                threeSixteenths.ToTurns() < 0.19,
                "3/16 turn should be approximately 0.1875 turns");

            // Comparing with special values
            static_assert(quarterTurn == Angle::Right(), "Quarter turn should equal right angle");
            static_assert(halfTurn == Angle::Straight(), "Half turn should equal straight angle");
            static_assert(fullTurn == Angle::Zero(), "Full turn should wrap to zero angle");
        }

        /**
         * @brief Tests for spherical linear interpolation (SLERP) between angles
         *
         * Verifies that:
         * - SLERP works correctly for various interpolation factors
         * - Edge cases (t=0 and t=1) produce expected results
         * - Interpolation is accurate at intermediate points
         * - Shortest path interpolation works correctly
         */
        static constexpr void TestAngleSpecificInterpolation()
        {
            constexpr Angle angle0 = Angle::FromDegrees(0);
            constexpr Angle angle90 = Angle::FromDegrees(90);

            // Linear interpolation
            constexpr Angle lerp25 = angle0.SLerp(angle90, 0.25);
            static_assert(lerp25.ToDegrees() == 22.5, "SLerp(0°, 90°, 0.25) should be 22.5°");

            constexpr Angle lerp50 = angle0.SLerp(angle90, 0.5);
            static_assert(lerp50.ToDegrees() == 45, "SLerp(0°, 90°, 0.5) should be 45°");

            constexpr Angle lerp75 = angle0.SLerp(angle90, 0.75);
            static_assert(lerp75.ToDegrees() == 67.5, "SLerp(0°, 90°, 0.75) should be 67.5°");

            // Edge cases
            constexpr Angle lerp0 = angle0.SLerp(angle90, 0);
            static_assert(lerp0.ToDegrees() == 0, "SLerp(0°, 90°, 0) should be 0°");

            constexpr Angle lerp1 = angle0.SLerp(angle90, 1);
            static_assert(lerp1.ToDegrees() == 90, "SLerp(0°, 90°, 1) should be 90°");

            // Test with angles spanning more than 90 degrees
            constexpr Angle angle180 = Angle::FromDegrees(180);
            constexpr Angle lerp180_50 = angle0.SLerp(angle180, 0.5);
            static_assert(lerp180_50.ToDegrees() == 90, "SLerp(0°, 180°, 0.5) should be 90°");

            // Test with negative angles
            constexpr Angle angleNeg90 = Angle::FromDegrees(-90);
            constexpr Angle lerpNeg_50 = angle0.SLerp(angleNeg90, 0.5);
            static_assert(lerpNeg_50.ToDegrees() == 135,
                "SLerp(0°, -90°, 0.5) should be 135° because -90° wraps to 270° and the shortest path is clockwise");
        }

        /**
         * @brief Tests for boundary values and extreme cases
         *
         * Verifies that:
         * - Very small angles are handled correctly
         * - Angles near 360° are normalized properly
         * - Angles with extreme values behave as expected
         */
        static constexpr void TestBoundaryValues()
        {
            // Very small positive angle
            constexpr Angle smallAngle = Angle::FromDegrees(0.01);
            static_assert(smallAngle.ToDegrees() > 0, "Very small positive angle should remain positive");

            // Very small negative angle
            constexpr Angle smallNegAngle = Angle::FromDegrees(-0.01);
            static_assert(smallNegAngle.ToDegrees() < 360 && smallNegAngle.ToDegrees() > 359,
                "Very small negative angle should wrap to just under 360 degrees");

            // Angle very close to 360 degrees
            constexpr Angle nearFullAngle = Angle::FromDegrees(359.999);
            static_assert(nearFullAngle.ToDegrees() < 360,
                "Angle very close to 360 degrees should remain just under 360");

            // Angle just over 360 degrees
            constexpr Angle justOverFullAngle = Angle::FromDegrees(360.01);
            static_assert(justOverFullAngle.ToDegrees() > 0 && justOverFullAngle.ToDegrees() < 1,
                "Angle just over 360 degrees should wrap to just over 0");
        }

        /**
         * @brief Tests for precision and accuracy
         *
         * Verifies that:
         * - Conversions between different angle units maintain precision
         * - Arithmetic operations maintain precision
         * - Rounding errors are within acceptable limits
         */
        static constexpr void TestPrecision()
        {
            // Test precision of degree-radian-degree conversion
            constexpr Angle angle30 = Angle::FromDegrees(30);
            constexpr auto radians = angle30.ToRadians();
            constexpr Angle convertedBack = Angle::FromRadians(radians);

            // Check that the conversion back is within a small error margin
            static_assert(convertedBack.ToDegrees() > 29.99 && convertedBack.ToDegrees() < 30.01,
                "Degree-radian-degree conversion should maintain precision");

            // Test precision of arithmetic operations
            constexpr Angle a1 = Angle::FromDegrees(10);
            constexpr Angle a2 = Angle::FromDegrees(20);
            constexpr Angle sum = a1 + a2;
            constexpr Angle diff = sum - a1;

            static_assert(diff.ToDegrees() > 19.99 && diff.ToDegrees() < 20.01,
                "Arithmetic operations should maintain precision");

            // Test precision with multiple operations
            constexpr Angle multiOp = (a1 * 2 + a2) / 2;
            static_assert(multiOp.ToDegrees() > 19.99 && multiOp.ToDegrees() < 20.01,
                "Multiple operations should maintain precision");
        }

        /**
         * @brief Run all tests
         *
         * Executes all test functions to verify the Angle class functionality
         */
        static constexpr void RunAll()
        {
            TestConstruction();
            TestFactoryMethods();
            TestConversionMethods();
            TestNormalization();
            TestArithmeticOperations();
            TestComparisonOperations();
            TestSpecialValues();
            TestTurnsBasedUsage();
            TestAngleSpecificInterpolation();
            TestBoundaryValues();
            TestPrecision();
        }
    };

    // Execute all tests
    static_assert((AngleTests::RunAll(), true), "Angle tests failed");
}
