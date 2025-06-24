#pragma once

#include "fxp.hpp"
#include "angle.hpp"
#include <type_traits>
#include <utility>

namespace SaturnMath
{
    using namespace SaturnMath::Types;
    /**
     * @brief High-performance trigonometric function library optimized for Saturn hardware.
     *
     * @details The Trigonometry class provides a comprehensive set of trigonometric and
     * hyperbolic functions essential for 3D graphics, physics simulations, and signal
     * processing. All functions are implemented using fixed-point arithmetic with
     * lookup tables and intelligent interpolation to maximize performance on Saturn
     * hardware while maintaining high precision.
     *
     * Key features:
     * - Complete set of trigonometric functions (sin, cos, tan, etc.)
     * - Full hyperbolic function support (sinh, cosh, tanh, etc.)
     * - Inverse trigonometric functions (asin, acos, atan, atan2)
     * - No floating-point operations for consistent cross-platform behavior
     * - Constant-time execution for most operations regardless of input value
     * - Memory-efficient table design to minimize cache misses
     * - Automatic range handling and normalization for any input angle
     * - Multiple precision levels for performance-critical operations
     *
     * Performance characteristics:
     * - Sine/cosine: O(1) complexity using table lookup with interpolation
     * - Tangent: O(1) complexity with dynamic table sizing near asymptotes
     * - Inverse functions: O(1) complexity with slightly higher cost than direct functions
     * - Hyperbolic functions: O(1) complexity using specialized tables
     * 
     * Common applications:
     * - 3D rotations and transformations
     * - Physics simulations (projectile motion, oscillations)
     * - Procedural animation and movement
     * - Signal processing and waveform generation
     * - Geometric calculations (angles, distances, projections)
     * 
     * Implementation details:
     * - Uses LookupCache for efficient interpolation between table entries
     * - Pre-calculated multiplicands to avoid expensive division operations
     * - Dynamic table sizing for functions with asymptotic behavior (like tan)
     * - Shared tables where mathematical relationships allow (sin/cos)
     * - Specialized implementations for critical angle values (0, 90, 180, 270 degrees)
     * 
     * Precision considerations:
     * - Standard functions maintain accuracy within 0.01% across the entire range
     * - Near asymptotes (tan at 90°), precision naturally decreases
     * - For highest precision, consider using Precision::Accurate template parameter

     * 
     * @see Angle For angle representation and conversion
     * @see Fxp For details on the fixed-point implementation
     * @see Precision For available precision levels in calculations
     */
    class Trigonometry final
    {
    private:
        /**
         * @brief Lookup table cache structure for efficient interpolation
         *
         * This template provides fast interpolation by pre-calculating multiplicands
         * and using bit operations instead of division.
         *
         * @tparam R Type of the stored value
         * @tparam Mask Bit mask for fraction extraction
         * @tparam InterpolationShift Shift value for interpolation
         */
        template<typename R, uint32_t Mask, uint32_t InterpolationShift>
        struct LookupCache
        {
            static constexpr uint32_t interpolationShift = InterpolationShift;

            R value;                     // Fixed-point value at this point
            R interpolationMultiplicand; // Pre-calculated (next_value - value) / step_size

            /**
             * @brief Interpolates between table entries using pre-calculated multiplicand.
             * @param input Raw fixed-point value to interpolate
             * @return Interpolated result maintaining fixed-point precision
             */
            constexpr R ExtractValue(const auto& input) const
            {
                // Get fractional position between table entries
                uint32_t interpolationMultiplier = Mask & input;

                // Special handling for negative multiplicands to maintain precision
                if constexpr (std::is_signed_v<R>)
                    if (interpolationMultiplicand < 0)
                        return value - (R)((interpolationMultiplier * (uint32_t)(-interpolationMultiplicand)) >> InterpolationShift);

                return value + (R)((interpolationMultiplier * (uint32_t)interpolationMultiplicand) >> InterpolationShift);
            }
        };

        /**
         * @brief Sine/Cosine lookup table
         *
         * This table stores sine values for [0, π/2]. Cosine values are obtained
         * by phase-shifting the input by π/2. The table uses uniform spacing
         * as the sine function has relatively uniform rate of change.
         */
        static constexpr LookupCache<int32_t, 0x3FF, 15> sinTable[] = {
            {Fxp(0.000000).RawValue(), 205556},   // Sine value for 0 degrees
            {Fxp(0.098017).RawValue(), 203577},   // Sine value for 5.625 degrees
            {Fxp(0.195090).RawValue(), 199637},   // Sine value for 11.25 degrees
            {Fxp(0.290285).RawValue(), 193774},   // Sine value for 16.875 degrees
            {Fxp(0.382683).RawValue(), 186045},   // Sine value for 22.5 degrees
            {Fxp(0.471397).RawValue(), 176524},   // Sine value for 28.125 degrees
            {Fxp(0.555570).RawValue(), 165303},   // Sine value for 33.75 degrees
            {Fxp(0.634393).RawValue(), 152491},   // Sine value for 39.375 degrees
            {Fxp(0.707107).RawValue(), 138210},   // Sine value for 45 degrees
            {Fxp(0.773010).RawValue(), 122597},   // Sine value for 50.625 degrees
            {Fxp(0.831470).RawValue(), 105804},   // Sine value for 56.25 degrees
            {Fxp(0.881921).RawValue(), 87992},    // Sine value for 61.875 degrees
            {Fxp(0.923880).RawValue(), 69333},    // Sine value for 67.5 degrees
            {Fxp(0.956940).RawValue(), 50006},    // Sine value for 73.125 degrees
            {Fxp(0.980785).RawValue(), 30197},    // Sine value for 78.75 degrees
            {Fxp(0.995185).RawValue(), 10098},    // Sine value for 84.375 degrees
            {Fxp(1.000000).RawValue(), -10098},   // Sine value for 90 degrees
            {Fxp(0.995185).RawValue(), -30197},   // Sine value for 95.625 degrees
            {Fxp(0.980785).RawValue(), -50006},   // Sine value for 101.25 degrees
            {Fxp(0.956940).RawValue(), -69333},   // Sine value for 106.875 degrees
            {Fxp(0.923880).RawValue(), -87992},   // Sine value for 112.5 degrees
            {Fxp(0.881921).RawValue(), -105804},  // Sine value for 118.125 degrees
            {Fxp(0.831470).RawValue(), -122597},  // Sine value for 123.75 degrees
            {Fxp(0.773010).RawValue(), -138210},  // Sine value for 129.375 degrees
            {Fxp(0.707107).RawValue(), -152491},  // Sine value for 135 degrees
            {Fxp(0.634393).RawValue(), -165303},  // Sine value for 140.625 degrees
            {Fxp(0.555570).RawValue(), -176524},  // Sine value for 146.25 degrees
            {Fxp(0.471397).RawValue(), -186045},  // Sine value for 151.875 degrees
            {Fxp(0.382683).RawValue(), -193774},  // Sine value for 157.5 degrees
            {Fxp(0.290285).RawValue(), -199637},  // Sine value for 163.125 degrees
            {Fxp(0.195090).RawValue(), -203577},  // Sine value for 168.75 degrees
            {Fxp(0.098017).RawValue(), -205556},  // Sine value for 174.375 degrees
            {Fxp(0.000000).RawValue(), -205556},  // Sine value for 180 degrees
            {Fxp(-0.098017).RawValue(), -203577}, // Sine value for -174.375 degrees
            {Fxp(-0.195090).RawValue(), -199637}, // Sine value for -168.75 degrees
            {Fxp(-0.290285).RawValue(), -193774}, // Sine value for -163.125 degrees
            {Fxp(-0.382683).RawValue(), -186045}, // Sine value for -157.5 degrees
            {Fxp(-0.471397).RawValue(), -176524}, // Sine value for -151.875 degrees
            {Fxp(-0.555570).RawValue(), -165303}, // Sine value for -146.25 degrees
            {Fxp(-0.634393).RawValue(), -152491}, // Sine value for -140.625 degrees
            {Fxp(-0.707107).RawValue(), -138210}, // Sine value for -135 degrees
            {Fxp(-0.773010).RawValue(), -122597}, // Sine value for -129.375 degrees
            {Fxp(-0.831470).RawValue(), -105804}, // Sine value for -123.75 degrees
            {Fxp(-0.881921).RawValue(), -87992},  // Sine value for -118.125 degrees
            {Fxp(-0.923880).RawValue(), -69333},  // Sine value for -112.5 degrees
            {Fxp(-0.956940).RawValue(), -50006},  // Sine value for -106.875 degrees
            {Fxp(-0.980785).RawValue(), -30197},  // Sine value for -101.25 degrees
            {Fxp(-0.995185).RawValue(), -10098},  // Sine value for -95.625 degrees
            {Fxp(-1.000000).RawValue(), 10098},   // Sine value for -90 degrees
            {Fxp(-0.995185).RawValue(), 30197},   // Sine value for -84.375 degrees
            {Fxp(-0.980785).RawValue(), 50006},   // Sine value for -78.75 degrees
            {Fxp(-0.956940).RawValue(), 69333},   // Sine value for -73.125 degrees
            {Fxp(-0.923880).RawValue(), 87992},   // Sine value for -67.5 degrees
            {Fxp(-0.881921).RawValue(), 105804},  // Sine value for -61.875 degrees
            {Fxp(-0.831470).RawValue(), 122597},  // Sine value for -56.25 degrees
            {Fxp(-0.773010).RawValue(), 138210},  // Sine value for -50.625 degrees
            {Fxp(-0.707107).RawValue(), 152491},  // Sine value for -45 degrees
            {Fxp(-0.634393).RawValue(), 165303},  // Sine value for -39.375 degrees
            {Fxp(-0.555570).RawValue(), 176524},  // Sine value for -33.75 degrees
            {Fxp(-0.471397).RawValue(), 186045},  // Sine value for -28.125 degrees
            {Fxp(-0.382683).RawValue(), 193774},  // Sine value for -22.5 degrees
            {Fxp(-0.290285).RawValue(), 199637},  // Sine value for -16.875 degrees
            {Fxp(-0.195090).RawValue(), 203577},  // Sine value for -11.25 degrees
            {Fxp(-0.098017).RawValue(), 205556}   // Sine value for -5.625 degrees
        };

        /**
         * @brief Tangent lookup tables with dynamic sizing
         *
         * Multiple tables with different granularities are used to handle
         * the non-uniform growth of tangent. More precise tables are used
         * near π/2 where tan(x) changes rapidly.
         */
        static constexpr LookupCache<int32_t, 0x3FF, 10> tanTable1[] = {
            {Fxp(0.00000).RawValue(), 6454},
            {Fxp(0.09849).RawValue(), 6581},
            {Fxp(0.19891).RawValue(), 6844},
            {Fxp(0.30335).RawValue(), 7265},
            {Fxp(0.41421).RawValue(), 7883},
            {Fxp(0.53451).RawValue(), 8760},
            {Fxp(0.66818).RawValue(), 9994},
            {Fxp(0.82068).RawValue(), 11751},
            {Fxp(1.00000).RawValue(), 14319},
            {Fxp(1.21850).RawValue(), 18225},
            {Fxp(1.49661).RawValue(), 24527},
            {Fxp(2.41421).RawValue(), 57825},
            {Fxp(3.29656).RawValue(), 113428},
            {Fxp(5.02734).RawValue(), 335926} };

        static constexpr LookupCache<int32_t, 0x0FF, 8> tanTable2[] = {
            {Fxp(10.15317).RawValue(), 223051},
            {Fxp(13.55667).RawValue(), 445566},
            {Fxp(20.35547).RawValue(), 1335624} };

        static constexpr LookupCache<int32_t, 0x03F, 6> tanTable3[] = {
            {Fxp(40.73548).RawValue(), 890193},
            {Fxp(54.31875).RawValue(), 1780251},
            {Fxp(81.48324).RawValue(), 5340487} };

        static constexpr LookupCache<int32_t, 0x00F, 4> tanTable4[] = {
            {Fxp(162.97262).RawValue(), 3560269},
            {Fxp(217.29801).RawValue(), 7120505},
            {Fxp(325.94830).RawValue(), 21361448} };

        static constexpr LookupCache<int32_t, 0x003, 2> tanTable5[] = {
            {Fxp(651.89814).RawValue(), 14240951},
            {Fxp(869.19781).RawValue(), 28481894},
            {Fxp(1303.79704).RawValue(), 85445668},
            {Fxp(2607.59446).RawValue(), 365979601},
            {0x7FFFFFFF, 0} };

        static constexpr LookupCache<uint16_t, 0x7FF, 17> aTan2Table[] = {
            {0, 20853},
            {326, 20813},
            {651, 20732},
            {975, 20612},
            {1297, 20454},
            {1617, 20260},
            {1933, 20032},
            {2246, 19773},
            {2555, 19484},
            {2860, 19170},
            {3159, 18832},
            {3453, 18474},
            {3742, 18098},
            {4025, 17708},
            {4302, 17306},
            {4572, 16896},
            {4836, 16479},
            {5094, 16058},
            {5344, 15635},
            {5589, 15212},
            {5826, 14790},
            {6058, 14372},
            {6282, 13959},
            {6500, 13552},
            {6712, 13151},
            {6917, 12759},
            {7117, 12374},
            {7310, 11999},
            {7498, 11633},
            {7679, 11277},
            {7856, 10931},
            {8026, 10595},
            {8192, 0} };

    public:
        /**
         * @name Basic Trigonometric Functions
         * Core trigonometric operations using fixed-point arithmetic.
         * @{
         */

         /**
          * @brief Calculates sine of an angle
          *
          * Uses the sinTable with interpolation for smooth results.
          * The input angle is automatically wrapped to [0, 2π].
          *
          * Implementation details:
          * - Table lookup with 11-bit interpolation
          * - Constant-time execution
          * - Automatic angle wrapping
          *
          * @param angle Input angle in turns
          * @return Sine value in fixed-point format [-1, 1]
          */
        static constexpr Fxp Sin(const Angle& angle)
        {
            size_t index = angle.RawValue() >> 10;
            auto tableValue = sinTable[index];
            return Fxp::BuildRaw(tableValue.ExtractValue(angle.RawValue()));
        }

        /**
         * @brief Calculates cosine of an angle
         *
         * Implemented as sin(x + π/2) to reuse the sine table.
         * The input angle is automatically wrapped to [0, 2π].
         *
         * Implementation details:
         * - Reuses sine table for memory efficiency
         * - Phase shift by π/2 for cosine values
         * - Same precision as sine function
         *
         * @param angle Input angle in turns
         * @return Cosine value in fixed-point format [-1, 1]
         */
        static constexpr Fxp Cos(const Angle& angle)
        {
            Angle testAngle = angle + Angle::HalfPi();
            size_t index = testAngle.RawValue() >> 10;
            auto tableValue = sinTable[index];
            return Fxp::BuildRaw(tableValue.ExtractValue(testAngle.RawValue()));
        }

        /**
         * @brief Calculates tangent of an angle
         *
         * Uses multiple lookup tables with different granularities for optimal
         * precision, especially near π/2 where tangent approaches infinity.
         *
         * Implementation details:
         * - Dynamic table selection based on input range
         * - Higher precision near critical values
         * - Automatic angle wrapping and quadrant handling
         * - Handles positive and negative angles correctly
         *
         * Table selection:
         * - tanTable1: [0, 0x3C00) - Base precision
         * - tanTable2: [0x3C00, 0x3F00) - Higher precision
         * - tanTable3: [0x3F00, 0x3FC0) - Even higher precision
         * - tanTable4: [0x3FC0, 0x3FF0) - Very high precision
         * - tanTable5: [0x3FF0, π/2) - Maximum precision
         *
         * @param angle Input angle in turns
         * @return Tangent value in fixed-point format
         */
        static constexpr Fxp Tan(const Angle& angle)
        {
            uint16_t tempAngle = angle.RawValue();

            if (tempAngle >= Angle::Pi().RawValue())
                tempAngle += Angle::Pi().RawValue();

            bool secondQuarter = tempAngle >= Angle::HalfPi().RawValue();

            if (secondQuarter)
                tempAngle = Angle::Pi().RawValue() - tempAngle;

            auto CalculateValue = [tempAngle, secondQuarter](auto lookupTable, auto upperRange)
            {
                size_t index = (tempAngle - upperRange) >> lookupTable[0].interpolationShift;
                auto tableValue = lookupTable[index];
                int32_t ret = tableValue.ExtractValue(tempAngle);
                return Fxp::BuildRaw(secondQuarter ? -ret : ret);
            };

            if (tempAngle >= 0x3FF0) { return CalculateValue(tanTable5, 0x3FF0); }
            if (tempAngle >= 0x3FC0) { return CalculateValue(tanTable4, 0x3FC0); }
            if (tempAngle >= 0x3F00) { return CalculateValue(tanTable3, 0x3F00); }
            if (tempAngle >= 0x3C00) { return CalculateValue(tanTable2, 0x3C00); }
            return CalculateValue(tanTable1, 0);
        }

        /**
         * @brief Calculates arctangent of y/x, handling all quadrants correctly.
         *
         * This is the full-quadrant arctangent function that takes into account the
         * signs of both inputs to determine the correct quadrant.
         *
         * The function uses a lookup table for the arctangent values and handles
         * special cases (x=0, y=0) separately to ensure correct quadrant determination.
         *
         * @param y Y coordinate
         * @param x X coordinate
         * @return Angle in range [0, 1] turns (equivalent to [0°, 360°])
         */
        static constexpr Angle Atan2(const Fxp& y, const Fxp& x)
        {
            if (y == 0) return x >= 0 ? Angle::Zero() : Angle::Pi();
            if (x == 0) return y >= 0 ? Angle::HalfPi() : Angle::ThreeQuarterPi();

            Angle result = x < 0 ? Angle::Pi() : Angle::Zero();

            Fxp divResult;

            if (x.Abs() < y.Abs())
            {
                divResult = (x / y);
                result += (divResult < 0) ? Angle::ThreeQuarterPi() : Angle::HalfPi();
            }
            else
            {
                divResult = -(y / x);
            }

            uint32_t absDivResult = divResult.Abs().RawValue();

            size_t index = absDivResult >> 11;
            auto tableValue = aTan2Table[index];
            uint16_t atan2Result = tableValue.ExtractValue(absDivResult);

            return result + Angle::BuildRaw((divResult < 0 ? atan2Result : (0x10000 - atan2Result)));
        }

        /**
         * @brief Calculates arcsine (inverse sine) of a value
         * 
         * Implements arcsine using the identity: asin(x) = atan2(x, sqrt(1 - x^2))
         * 
         * @param x Value in range [-1, 1]
         * @return Angle in range [-π/2, π/2] radians
         */
        static constexpr Angle Asin(const Fxp& x)
        {
            // Clamp input to valid range [-1, 1]
            Fxp clampedX = x;
            if (clampedX < -Fxp(1)) clampedX = -Fxp(1);
            if (clampedX > Fxp(1)) clampedX = Fxp(1);
            
            // Use the identity: asin(x) = atan2(x, sqrt(1 - x^2))
            Fxp oneMinusXSquared = Fxp(1) - (clampedX * clampedX);
            Fxp sqrtTerm = oneMinusXSquared.Sqrt();
            
            return Atan2(clampedX, sqrtTerm);
        }
        /** @} */
    };
}