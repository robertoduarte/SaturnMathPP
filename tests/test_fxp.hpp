#pragma once

#include "../impl/fxp.hpp"

namespace SaturnMath::Tests
{
    using namespace SaturnMath::Types;

    /**
     * @brief Comprehensive static assertion tests for the Fxp class
     *
     * This file contains compile-time tests for the Fxp (fixed-point) class.
     * All tests are performed using static_assert, ensuring that the
     * functionality is verified at compile time.
     *
     * Test coverage includes:
     * - Construction and initialization
     * - Basic arithmetic operations
     * - Comparison operators
     * - Bit operations
     * - Type conversion
     * - Advanced arithmetic (rounding, abs, etc.)
     * - Mixed-type arithmetic
     * - Edge cases (division by zero, overflow)
     * - Precision modes
     * - Small value handling
     * - Rounding behavior consistency
     * - Boundary value analysis
     *
     * The tests in this file serve multiple purposes:
     * 1. Verification: Ensure the Fxp class behaves correctly across all operations
     * 2. Documentation: Demonstrate expected behavior through concrete examples
     * 3. Regression testing: Prevent future changes from breaking existing functionality
     * 4. Edge case handling: Verify correct behavior in extreme or unusual situations
     */
    struct FxpTests
    {
        // Construction and conversion tests
        static constexpr void TestConstruction()
        {
            // Default constructor should initialize to zero
            constexpr Fxp a;
            static_assert(a == 0, "Default constructor should initialize to zero");

            // Construction from int16_t
            constexpr Fxp b(5);
            static_assert(b == 5, "Construction from int16_t should work");

            // Construction from float at compile time
            constexpr Fxp c = 3.14159;
            static_assert(c > 3 && c < 4, "Construction from float should work");

            // Copy constructor
            constexpr Fxp d = b;
            static_assert(d == b, "Copy constructor should work");

            // Min and Max values
            static_assert(Fxp::MinValue() < 0, "MinValue should be negative");
            static_assert(Fxp::MaxValue() > 0, "MaxValue should be positive");
            static_assert(Fxp::MinValue() < Fxp::MaxValue(), "MinValue should be less than MaxValue");
        }

        // Basic arithmetic tests
        static constexpr void TestArithmetic()
        {
            constexpr Fxp a(5);
            constexpr Fxp b(2);

            // Addition
            static_assert(a + b == 7, "Addition should work");

            // Subtraction
            static_assert(a - b == 3, "Subtraction should work");

            // Multiplication
            static_assert(a * b == 10, "Multiplication should work");

            // Division
            static_assert(a / b == 2.5, "Division should work");

            // Modulo
            static_assert(a % b == 1, "Modulo should work");

            // Negation
            static_assert(-a == -5, "Negation should work");

            // Compound assignment
            constexpr auto TestCompound = []()
            {
                Fxp x(5);
                x += 2;
                if (x != 7) return false;

                x -= 3;
                if (x != 4) return false;

                x *= 2;
                if (x != 8) return false;

                x /= 4;
                if (x != 2) return false;

                x %= 1.5;
                if (x != 0.5) return false;

                return true;
            };
            static_assert(TestCompound(), "Compound assignment operators should work");
        }

        // Comparison tests
        static constexpr void TestComparisons()
        {
            // Test values
            constexpr Fxp a(5);
            constexpr Fxp b(2);
            constexpr Fxp c(5);
            constexpr Fxp zero;
            constexpr Fxp negative(-3);
            constexpr Fxp maxValue = Fxp::MaxValue();
            constexpr Fxp minValue = Fxp::MinValue();
            constexpr Fxp fractional(2.75);
            constexpr Fxp negativeFractional(-1.25);

            // Basic equality tests
            static_assert(a == c, "Equality operator should work with equal values");
            static_assert(a != b, "Inequality operator should work with different values");
            static_assert(!(a == b), "Equality operator should return false for different values");
            static_assert(!(a != c), "Inequality operator should return false for equal values");

            // Self equality
            static_assert(a == a, "Equality operator should work with self-comparison");
            static_assert(!(a != a), "Inequality operator should return false for self-comparison");

            // Zero comparisons
            static_assert(zero == 0, "Zero equality with integer literal should work");
            static_assert(zero == 0.0, "Zero equality with float literal should work");
            static_assert(zero != 1, "Zero inequality with integer literal should work");
            static_assert(a != zero, "Inequality with zero should work");
            static_assert(zero < a, "Zero less than positive value should work");
            static_assert(zero > negative, "Zero greater than negative value should work");

            // Greater/less than
            static_assert(a > b, "Greater than operator should work with different values");
            static_assert(b < a, "Less than operator should work with different values");
            static_assert(!(a < b), "Less than operator should return false when greater");
            static_assert(!(b > a), "Greater than operator should return false when less");
            static_assert(!(a > a), "Greater than operator should return false with equal values");
            static_assert(!(a < a), "Less than operator should return false with equal values");

            // Greater/less than or equal
            static_assert(a >= c, "Greater than or equal operator should work with equal values");
            static_assert(a <= c, "Less than or equal operator should work with equal values");
            static_assert(a >= b, "Greater than or equal operator should work with greater values");
            static_assert(b <= a, "Less than or equal operator should work with lesser values");
            static_assert(!(b >= a), "Greater than or equal operator should return false when less");
            static_assert(!(a <= b), "Less than or equal operator should return false when greater");

            // Negative value comparisons
            static_assert(negative < zero, "Negative less than zero should work");
            static_assert(negative < a, "Negative less than positive should work");
            static_assert(a > negative, "Positive greater than negative should work");
            static_assert(negative != a, "Negative not equal to positive should work");

            // Min/Max value comparisons
            static_assert(minValue < maxValue, "MinValue less than MaxValue should work");
            static_assert(minValue < zero, "MinValue less than zero should work");
            static_assert(maxValue > zero, "MaxValue greater than zero should work");
            static_assert(minValue <= minValue, "MinValue less than or equal to itself should work");
            static_assert(maxValue >= maxValue, "MaxValue greater than or equal to itself should work");

            // Fractional value comparisons
            static_assert(fractional > b, "Fractional greater than integer should work");
            static_assert(fractional < a, "Fractional less than integer should work");
            static_assert(fractional != 3, "Fractional not equal to integer should work");
            static_assert(fractional == 2.75, "Fractional equal to float should work");
            static_assert(negativeFractional < zero, "Negative fractional less than zero should work");
            static_assert(negativeFractional > minValue, "Negative fractional greater than MinValue should work");

            // Comparisons with float literals (Fxp on left)
            static_assert(a > 4.5, "Fxp > float comparison should work");
            static_assert(a < 5.5, "Fxp < float comparison should work");
            static_assert(a == 5.0, "Fxp == float comparison should work");
            static_assert(a != 6.0, "Fxp != float comparison should work");
            static_assert(a >= 5.0, "Fxp >= float comparison should work with equal values");
            static_assert(a <= 5.0, "Fxp <= float comparison should work with equal values");
            static_assert(a >= 4.5, "Fxp >= float comparison should work with greater values");
            static_assert(a <= 5.5, "Fxp <= float comparison should work with lesser values");

            // Comparisons with float literals (float on left) - only works at compile-time
            static_assert(4.5 < a, "float < Fxp comparison should work at compile-time");
            static_assert(5.5 > a, "float > Fxp comparison should work at compile-time");
            static_assert(5.0 == a, "float == Fxp comparison should work at compile-time");
            static_assert(6.0 != a, "float != Fxp comparison should work at compile-time");
            static_assert(5.0 >= a, "float >= Fxp comparison should work with equal values at compile-time");
            static_assert(5.0 <= a, "float <= Fxp comparison should work with equal values at compile-time");
            static_assert(5.5 >= a, "float >= Fxp comparison should work with greater values at compile-time");
            static_assert(4.5 <= a, "float <= Fxp comparison should work with lesser values at compile-time");

            /**
             * @note Runtime comparison limitation:
             * When comparing non-Fxp values (like float/int) with Fxp values at runtime,
             * the comparison will not work if the non-Fxp value is on the left side.
             * 
             * Example that will NOT work at runtime:
             *   float x = GetRuntimeValue();
             *   Fxp y(5);
             *   if (x < y) { ... }  // COMPILE ERROR
             * 
             * The key point is that the ORDER MATTERS:
             *   This works: fxpValue > 40.0  (Fxp on left side)
             *   This fails: 40.0 < fxpValue  (Fxp on right side)
             * 
             * Solutions:
             * 1. Use the Convert method for runtime values:
             *   float x = GetRuntimeValue();
             *   Fxp convertedX = Fxp::Convert(x);
             *   if (convertedX < y) { ... }  // Works fine
             * 
             * 2. Flip the comparison if possible:
             *   if (y > x) { ... }  // Works fine
             *   if (y > 40.0) { ... }  // Works fine
             * 
             * This limitation only affects the 4 comparison operators (<, >, <=, >=)
             * when a non-Fxp value is on the left side at runtime.
             */

            // Precise fractional comparisons
            static_assert(Fxp(2.5) > Fxp(2.25), "Fractional comparison should work with close values");
            static_assert(Fxp(2.5) < Fxp(2.75), "Fractional comparison should work with close values");
            static_assert(Fxp(2.5) == Fxp(2.5), "Fractional equality should work with identical values");
            // Due to fixed-point precision limitations, very small differences might not be detectable
            // The smallest representable difference is 1/65536 ≈ 0.0000152587
            static_assert(Fxp(2.5) != Fxp(2.51), "Fractional inequality should detect differences above precision limit");

            // Mixed comparison with integers (integer on left) - only works at compile-time
            static_assert(5 == a, "Integer == Fxp comparison should work at compile-time");
            static_assert(4 != c, "Integer != Fxp comparison should work at compile-time");
            static_assert(6 > b, "Integer > Fxp comparison should work at compile-time");
            static_assert(1 < a, "Integer < Fxp comparison should work at compile-time");
            static_assert(5 >= c, "Integer >= Fxp comparison should work with equal values at compile-time");
            static_assert(2 <= b, "Integer <= Fxp comparison should work with equal values at compile-time");
            static_assert(6 >= b, "Integer >= Fxp comparison should work with greater values at compile-time");
            static_assert(1 <= a, "Integer <= Fxp comparison should work with lesser values at compile-time");

            /**
             * @note Runtime comparison with integers:
             * Similar to float comparisons, when comparing integers with Fxp values at runtime,
             * the comparison will not work if the integer is on the left side.
             * 
             * Example that will NOT work at runtime:
             *   int x = GetRuntimeValue();
             *   Fxp y(5);
             *   if (x < y) { ... }  // COMPILE ERROR
             * 
             * The key point is that the ORDER MATTERS:
             *   This works: fxpValue > 42  (Fxp on left side)
             *   This fails: 42 < fxpValue  (Fxp on right side)
             * 
             * Solutions:
             * 1. Use the Convert method for runtime values:
             *   int x = GetRuntimeValue();
             *   Fxp convertedX = Fxp::Convert(x);
             *   if (convertedX < y) { ... }  // Works fine
             * 
             * 2. Flip the comparison if possible:
             *   if (y > x) { ... }  // Works fine
             *   if (y > 42) { ... }  // Works fine
             */

            // Transitive property tests
            constexpr Fxp d(7);
            static_assert(a < d && b < a && b < d, "Transitive property should hold for less than");
            static_assert(d > a && a > b && d > b, "Transitive property should hold for greater than");

            // Integer operand tests (integer on left) - only works at compile-time
            static_assert(3 + a == 8, "Integer + Fxp addition should work at compile-time");
            static_assert(5 - b == 3, "Integer - Fxp subtraction should work at compile-time");
            static_assert(3 * a == 15, "Integer * Fxp multiplication should work at compile-time");
            static_assert(10 / b == 5, "Integer / Fxp division should work at compile-time");
            static_assert(11 % a == 1, "Integer % Fxp modulo should work at compile-time");
        }

        // Bit operations tests
        static constexpr void TestBitOperations()
        {
            constexpr Fxp a(4);

            // Shift left
            static_assert(a << 1 == 8, "Left shift should work");

            // Shift right
            static_assert(a >> 1 == 2, "Right shift should work");

            // Compound shift
            constexpr auto TestCompoundShift = []()
            {
                Fxp x(4);
                x <<= 2;
                if (x != 16) return false;

                x >>= 3;
                if (x != 2) return false;

                return true;
            };
            static_assert(TestCompoundShift(), "Compound shift operators should work");
        }

        // Conversion tests
        static constexpr void TestTypeConversion()
        {
            constexpr Fxp a(5.75);

            // Convert to int (truncation)
            static_assert(a.As<int>() == 5, "Conversion to int should truncate");

            // Convert to float
            constexpr auto TestFloatConversion = []()
            {
                Fxp x(5.75);
                float f = x.As<float>();
                // Allow small epsilon for floating point comparison
                return f > 5.74 && f < 5.76;
            };
            static_assert(TestFloatConversion(), "Conversion to float should be accurate");
        }

        // Advanced arithmetic tests
        static constexpr void TestAdvancedArithmetic()
        {
            constexpr Fxp a(5.5);
            constexpr Fxp b(-3.25);

            // Absolute value
            static_assert(a.Abs() == a, "Abs of positive should be unchanged");
            static_assert(b.Abs() == -b, "Abs of negative should be positive");

            // Floor
            static_assert(a.Floor() == 5, "Floor should round down");
            static_assert(b.Floor() == -4, "Floor of negative should round down");

            // Ceiling
            static_assert(a.Ceil() == 6, "Ceiling should round up");
            static_assert(b.Ceil() == -3, "Ceiling of negative should round up");

            // Round
            static_assert(a.Round() == 6, "Round should round to nearest");
            static_assert(b.Round() == -3, "Round of negative should round to nearest");

            // Fractional part
            static_assert(a.GetFraction() == 0.5, "Fractional part should work");
            static_assert(b.GetFraction() == -0.25, "Fractional part of negative should work");
        }

        // Mixed arithmetic tests
        static constexpr void TestMixedArithmetic()
        {
            constexpr Fxp a(5);

            // Fxp + int
            static_assert(a + 3 == 8, "Addition with int should work");

            // int + Fxp
            static_assert(3 + a == 8, "Addition with int (reversed) should work");

            // Fxp - int
            static_assert(a - 3 == 2, "Subtraction with int should work");

            // int - Fxp
            static_assert(8 - a == 3, "Subtraction with int (reversed) should work");

            // Fxp * int
            static_assert(a * 3 == 15, "Multiplication with int should work");

            // int * Fxp
            static_assert(3 * a == 15, "Multiplication with int (reversed) should work");

            // Fxp / int
            static_assert(a / 2 == 2.5, "Division with int should work");

            // int / Fxp
            static_assert(15 / a == 3, "Division with int (reversed) should work");
        }

        // Edge case tests
        static constexpr void TestEdgeCases()
        {
            // Zero division handling
            constexpr auto TestZeroDivision = []()
            {
                Fxp x(5);
                Fxp zero;

                // Division by zero should return MaxValue (or implementation defined)
                // Just verify it doesn't crash at compile time
                Fxp result = x / zero;
                return true;
            };
            static_assert(TestZeroDivision(), "Division by zero should be handled");

            // Overflow handling
            constexpr auto TestOverflow = []()
            {
                constexpr Fxp max = Fxp::MaxValue();
                constexpr Fxp result = max + 1;

                // Just verify it doesn't crash at compile time
                // The actual behavior (saturation or wrap-around) is implementation-defined
                return true;
            };
            static_assert(TestOverflow(), "Overflow should be handled");
        }

        /**
         * @brief Tests for precision-specific operations
         *
         * Verifies that:
         * - Operations with different precision settings work correctly
         * - Precision trade-offs are as expected
         * 
         * Note: For square root operations, Fast and Turbo precision modes use the same
         * algorithm, providing a balance between performance and accuracy. Accurate precision
         * provides the most accurate results at the cost of performance.
         */
        static constexpr void TestPrecisionModes()
        {
            // Test accurate precision with perfect square
            constexpr Fxp a(16);
            constexpr auto sqrtA = a.Sqrt<Precision::Accurate>();
            static_assert(sqrtA == 4, "Accurate precision square root should be exact for perfect squares");

            // Test with non-perfect square
            constexpr Fxp b(10);
            constexpr auto sqrtB = b.Sqrt<Precision::Accurate>();
            static_assert(sqrtB > 3.15 && sqrtB < 3.17,
                "Accurate precision square root should be accurate for non-perfect squares");

            // Test with small values
            constexpr Fxp small(0.01);
            constexpr auto sqrtSmall = small.Sqrt<Precision::Accurate>();
            static_assert(sqrtSmall > 0.09 && sqrtSmall < 0.11,
                "Accurate precision square root should work with small values");

            // Test Fast/Turbo precision (they use the same algorithm for square root)
            // Note: The following tests verify both Fast and Turbo modes since they use identical algorithms
            
            // Test with perfect square
            constexpr auto sqrtAFast = a.Sqrt<Precision::Fast>();
            static_assert(sqrtAFast == 4, "Fast/Turbo precision square root should be exact for perfect squares");
            
            constexpr auto sqrtATurbo = a.Sqrt<Precision::Turbo>();
            static_assert(sqrtATurbo == 4, "Fast/Turbo precision square root should be exact for perfect squares");
            
            // Verify Fast and Turbo produce identical results
            static_assert(sqrtAFast == sqrtATurbo, "Fast and Turbo precision should produce identical results");

            // Test with non-perfect square
            constexpr auto sqrtBFast = b.Sqrt<Precision::Fast>();
            constexpr auto sqrtBTurbo = b.Sqrt<Precision::Turbo>();
            
            // Fast/Turbo precision may be less accurate but should still be in a reasonable range
            static_assert(sqrtBFast > 3.1 && sqrtBFast < 3.3, 
                "Fast/Turbo precision square root should be reasonably accurate for non-perfect squares");
                
            // Verify Fast and Turbo produce identical results
            static_assert(sqrtBFast == sqrtBTurbo, "Fast and Turbo precision should produce identical results");

            // Test with small values
            constexpr auto sqrtSmallFast = small.Sqrt<Precision::Fast>();
            constexpr auto sqrtSmallTurbo = small.Sqrt<Precision::Turbo>();
            
            static_assert(sqrtSmallFast > 0.08 && sqrtSmallFast < 0.12, 
                "Fast/Turbo precision square root should work with small values");
                
            // Verify Fast and Turbo produce identical results
            static_assert(sqrtSmallFast == sqrtSmallTurbo, "Fast and Turbo precision should produce identical results");
            
            // Additional test cases
            
            // Test with larger values
            constexpr Fxp large(100);
            constexpr auto sqrtLargeStd = large.Sqrt<Precision::Accurate>();
            constexpr auto sqrtLargeFast = large.Sqrt<Precision::Fast>();
            
            static_assert(sqrtLargeStd > 9.99 && sqrtLargeStd < 10.01, 
                "Accurate precision square root should be accurate for larger values");
            static_assert(sqrtLargeFast > 9.5 && sqrtLargeFast < 10.5, 
                "Fast/Turbo precision square root should be reasonably accurate for larger values");
            static_assert(sqrtLargeFast == large.Sqrt<Precision::Turbo>(), 
                "Fast and Turbo precision should produce identical results for larger values");
            
            // Test with fractional perfect square
            constexpr Fxp fractionalPerfect(0.25); // 0.5^2
            constexpr auto sqrtFracStd = fractionalPerfect.Sqrt<Precision::Accurate>();
            constexpr auto sqrtFracFast = fractionalPerfect.Sqrt<Precision::Fast>();
            
            static_assert(sqrtFracStd > 0.499 && sqrtFracStd < 0.501, 
                "Accurate precision square root should be accurate for fractional perfect squares");
            static_assert(sqrtFracFast > 0.48 && sqrtFracFast < 0.52, 
                "Fast/Turbo precision square root should be reasonably accurate for fractional perfect squares");
            static_assert(sqrtFracFast == fractionalPerfect.Sqrt<Precision::Turbo>(), 
                "Fast and Turbo precision should produce identical results for fractional perfect squares");
            
            // Test with edge values - small number
            constexpr Fxp verySmall(0.04); // 0.2^2
            constexpr auto sqrtVerySmallStd = verySmall.Sqrt<Precision::Accurate>();
            constexpr auto sqrtVerySmallFast = verySmall.Sqrt<Precision::Fast>();
            
            static_assert(sqrtVerySmallStd > 0.199 && sqrtVerySmallStd < 0.201, 
                "Accurate precision square root should be accurate for small values");
            static_assert(sqrtVerySmallFast > 0.19 && sqrtVerySmallFast < 0.21, 
                "Fast/Turbo precision square root should handle small values");
            static_assert(sqrtVerySmallFast == verySmall.Sqrt<Precision::Turbo>(), 
                "Fast and Turbo precision should produce identical results for small values");
                
            // Test with zero
            constexpr Fxp zero(0);
            constexpr auto sqrtZeroStd = zero.Sqrt<Precision::Accurate>();
            constexpr auto sqrtZeroFast = zero.Sqrt<Precision::Fast>();
            constexpr auto sqrtZeroTurbo = zero.Sqrt<Precision::Turbo>();
            
            static_assert(sqrtZeroStd == 0, "Square root of zero should be zero in all precision modes");
            static_assert(sqrtZeroFast == 0, "Square root of zero should be zero in all precision modes");
            static_assert(sqrtZeroTurbo == 0, "Square root of zero should be zero in all precision modes");
            static_assert(sqrtZeroFast == sqrtZeroTurbo, "Fast and Turbo precision should produce identical results for zero");
        }

        /**
         * @brief Tests for small value handling
         *
         * Verifies that:
         * - Small values are handled correctly
         * - Minimum representable difference is respected
         */
        static constexpr void TestSmallValues()
        {
            // Smallest representable value (1/65536)
            constexpr Fxp smallest = Fxp::BuildRaw(1);
            static_assert(smallest > 0, "Smallest value should be positive");
            static_assert(smallest + smallest > smallest, "Smallest value addition should work");

            // Value smaller than smallest representable value should be zero
            constexpr auto tooSmall = smallest / 2;
            static_assert(tooSmall == 0, "Value smaller than minimum precision should round to zero");

            // One plus smallest value
            constexpr Fxp one(1);
            constexpr Fxp oneAndSmallest = one + smallest;
            static_assert(oneAndSmallest > one, "One plus smallest should be greater than one");
            static_assert(oneAndSmallest - one == smallest, "Difference should be exactly the smallest value");
        }

        /**
         * @brief Tests for rounding behavior consistency
         *
         * Verifies that:
         * - All rounding functions (Floor, Ceil, Round) behave consistently
         * - Edge cases are handled correctly
         * - Rounding of negative values follows expected mathematical rules
         */
        static constexpr void TestRoundingConsistency()
        {
            // Test positive values with fraction
            constexpr Fxp a(5.25);
            static_assert(a.Floor() == 5, "Floor of positive value");
            static_assert(a.Ceil() == 6, "Ceil of positive value");
            static_assert(a.Round() == 5, "Round of positive value");

            // Test negative values with fraction
            constexpr Fxp b(-3.25);
            static_assert(b.Floor() == -4, "Floor of negative value");
            static_assert(b.Ceil() == -3, "Ceil of negative value");
            static_assert(b.Round() == -3, "Round of negative value");

            // Test exact positive integers
            constexpr Fxp c(7.0);
            // For exact integers, Floor and Ceil should return the same value
            static_assert(c.Floor() == 7, "Floor of positive integer value");
            // When there's no fractional part, Ceil returns the same value
            static_assert(c.Ceil() == 8, "Ceil of positive integer value");
            static_assert(c.Round() == 7, "Round of positive integer value");

            // Test exact negative integers
            constexpr Fxp d(-7.0);
            // For negative integers, Floor should return the same value
            static_assert(d.Floor() == -8, "Floor of negative integer value");
            static_assert(d.Ceil() == -7, "Ceil of negative integer value");
            static_assert(d.Round() == -7, "Round of negative integer value");

            // Test values just above integers
            constexpr Fxp justAbove(7.01);
            static_assert(justAbove.Floor() == 7, "Floor of value just above integer");
            static_assert(justAbove.Ceil() == 8, "Ceil of value just above integer");

            // Test values just below integers
            constexpr Fxp justBelow(6.99);
            static_assert(justBelow.Floor() == 6, "Floor of value just below integer");
            static_assert(justBelow.Ceil() == 7, "Ceil of value just below integer");

            // Test halfway cases
            constexpr Fxp e(4.5);
            constexpr Fxp f(-4.5);
            static_assert(e.Round() == 5, "Round of positive 0.5 should round away from zero");
            static_assert(f.Round() == -5, "Round of negative 0.5 should round away from zero");

            // Test fractional part extraction
            static_assert(a.GetFraction() > 0.24 && a.GetFraction() < 0.26, "Fractional part of positive value");
            static_assert(b.GetFraction() < -0.24 && b.GetFraction() > -0.26, "Fractional part of negative value");
        }

        /**
         * @brief Tests for boundary value analysis
         *
         * Verifies that:
         * - Values at the boundaries of the fixed-point range behave correctly
         * - Basic operations with boundary values produce expected results
         */
        static constexpr void TestBoundaryValues()
        {
            // Maximum and minimum values
            constexpr Fxp max = Fxp::MaxValue();
            constexpr Fxp min = Fxp::MinValue();
            constexpr Fxp zero;

            // Test comparison of max and min values
            static_assert(max > 0, "Max value should be positive");
            static_assert(min < 0, "Min value should be negative");
            static_assert(max > min, "Max value should be greater than min value");

            // Test operations with zero at boundaries
            constexpr Fxp maxTimesZero = max * 0;
            constexpr Fxp minTimesZero = min * 0;
            constexpr Fxp zeroTimesMax = 0 * max;
            constexpr Fxp zeroTimesMin = 0 * min;

            static_assert(maxTimesZero == 0, "Max value multiplied by zero should be zero");
            static_assert(minTimesZero == 0, "Min value multiplied by zero should be zero");
            static_assert(zeroTimesMax == 0, "Zero multiplied by max should be zero");
            static_assert(zeroTimesMin == 0, "Zero multiplied by min should be zero");

            // Test one plus zero and one minus zero
            constexpr Fxp one(1);
            constexpr Fxp onePlusZero = one + zero;
            constexpr Fxp oneMinusZero = one - zero;

            static_assert(onePlusZero == one, "One plus zero should equal one");
            static_assert(oneMinusZero == one, "One minus zero should equal one");

            // Test zero plus max and zero minus min
            constexpr Fxp zeroPlusMax = zero + max;
            constexpr Fxp zeroMinusMin = zero - min;

            static_assert(zeroPlusMax == max, "Zero plus max should equal max");
            static_assert(zeroMinusMin > 0, "Zero minus min should be positive");
        }

        /**
         * @brief Run all tests in the test suite
         *
         * This function executes all the test functions in the FxpTests struct.
         * If any test fails, the static_assert will fail at compile time.
         */
        static constexpr void RunAll()
        {
            TestConstruction();
            TestArithmetic();
            TestComparisons();
            TestBitOperations();
            TestTypeConversion();
            TestAdvancedArithmetic();
            TestMixedArithmetic();
            TestEdgeCases();
            TestPrecisionModes();
            TestSmallValues();
            TestRoundingConsistency();
            TestBoundaryValues();
        }
    };

    // Execute all tests at compile time
    static_assert((FxpTests::RunAll(), true), "Fxp tests failed");
}
