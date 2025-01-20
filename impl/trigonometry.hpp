#pragma once

#include "fxp.hpp"
#include "angle.hpp"
#include <type_traits>
#include <utility>

namespace SaturnMath
{
    /**
     * @brief Core trigonometric functionality using fixed-point arithmetic
     *
     * This class provides a comprehensive set of trigonometric and hyperbolic functions
     * optimized for fixed-point arithmetic. All functions use lookup tables with
     * intelligent interpolation to achieve a balance of speed and accuracy.
     *
     * Key features:
     * - No floating-point operations
     * - Constant-time execution for most operations
     * - Memory-efficient table design
     * - Automatic range handling
     *
     * Implementation details:
     * - Uses LookupCache for efficient interpolation
     * - Pre-calculated multiplicands to avoid division
     * - Dynamic table sizing for functions like tan()
     * - Shared tables where mathematical relationships allow
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

        /**
         * @brief Hyperbolic sine lookup table
         *
         * Stores values for sinh(x) in [0, 4]. Uses non-uniform spacing
         * to handle exponential growth efficiently. This table is also used
         * for cosh(x) using the identity cosh²(x) = 1 + sinh²(x).
         *
         * Table design:
         * - Denser spacing for smaller values where change is more subtle
         * - Wider spacing for larger values where exponential growth dominates
         * - Covers practical range for most applications
         * - Memory efficient by serving both sinh and cosh functions
         */
        static constexpr LookupCache<uint16_t, 0x7FF, 11> sinhTable[] = {/* Todo: Add proper table values */};    // End marker

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
            size_t index = (angle.RawValue() + Angle::HalfPi().RawValue()) >> 10;
            auto tableValue = sinTable[index];
            return Fxp::BuildRaw(sinTable[index].ExtractValue(angle.RawValue()));
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
         * @brief Calculates arcsine (inverse sine) of a value
         *
         * This function computes the angle whose sine equals the input value.
         * The input value must be in the range [-1, 1].
         *
         * @param value Input value in range [-1, 1]
         * @return Angle in range [0.75, 0.25] turns (wraps from 270° through 0° to 90°)
         */
        template<typename T = void>
        static constexpr Angle Asin(const Fxp& value)
        {
            static_assert(!std::is_same_v<T, T>, "[DRAFT] Asin() is currently a work in progress and not ready for use");
            // Handle edge cases
            if (value >= 1) return Angle::HalfPi();
            if (value <= -1) return Angle::BuildRaw(0xC000); // 270° = 0.75 turns

            // Get absolute value for lookup, we'll adjust the result later
            int32_t absValue = value.Abs().RawValue();

            // Find the table entry for this value
            size_t index = absValue >> 11;
            auto tableValue = sinTable[index];
            uint16_t result = tableValue.ExtractValue(absValue);

            // If input was negative, convert to equivalent angle in [0.75, 1.0] range
            if (value < 0)
            {
                result = 0xFFFF - result + 1;
            }

            return Angle::BuildRaw(result);
        }

        /**
         * @brief Calculates arccosine (inverse cosine) of a value
         *
         * This function computes the angle whose cosine equals the input value.
         * The input value must be in the range [-1, 1].
         *
         * @param value Input value in range [-1, 1]
         * @return Angle in range [0, 0.5] turns (0° to 180°)
         */
        template<typename T = void>
        static constexpr Angle Acos(const Fxp& value)
        {
            static_assert(!std::is_same_v<T, T>, "[DRAFT] Acos() is currently a work in progress and not ready for use");
            // Handle edge cases
            if (value >= 1) return Angle::Zero();
            if (value <= -1) return Angle::Pi();

            // Get absolute value for lookup
            int32_t absValue = value.Abs().RawValue();

            // Find the table entry for this value
            size_t index = absValue >> 11;
            auto tableValue = sinTable[index];
            uint16_t asinResult = tableValue.ExtractValue(absValue);

            // For positive input: acos(x) = π/2 - asin(x)
            // For negative input: acos(x) = π/2 + asin(-x)
            uint16_t result;
            if (value < 0)
            {
                result = 0x4000 + asinResult; // 90° + asin(-x)
            }
            else
            {
                result = 0x4000 - asinResult; // 90° - asin(x)
            }

            return Angle::BuildRaw(result);
        }

        /**
         * @brief Calculates arctangent of y/x, handling all quadrants correctly.
         *
         * This is the full-quadrant arctangent function that takes into account the
         * signs of both inputs to determine the correct quadrant.
         *
         * @param y Y coordinate
         * @param x X coordinate
         * @return Angle in range [0, 1] turns (equivalent to [0°, 360°])
         */
        static constexpr Angle Atan2(const Fxp& y, const Fxp& x)
        {
            uint16_t result = x < 0 ?
                (y < 0 ? Angle::Pi().RawValue() : -Angle::Pi().RawValue()) : 0;

            int32_t divResult;

            if (x.Abs() < y.Abs())
            {
                divResult = (x / y).RawValue();
                result += divResult < 0 ? -Angle::HalfPi().RawValue() : Angle::HalfPi().RawValue();
            }
            else
            {
                divResult = -(y / x).RawValue();
            }

            if (divResult < 0)
            {
                divResult = -divResult;
                size_t index = divResult >> 11;
                auto tableValue = aTan2Table[index];
                return Angle(result + tableValue.ExtractValue(divResult));
            }

            size_t index = divResult >> 11;
            auto tableValue = aTan2Table[index];
            return Angle(result - tableValue.ExtractValue(divResult));
        }

        /**
         * @brief Calculates hyperbolic sine of a value
         *
         * Computes sinh(x) = (e^x - e^-x)/2 using a lookup table with
         * non-uniform spacing to handle exponential growth efficiently.
         *
         * Implementation details:
         * - Uses sinhTable with non-uniform spacing
         * - Handles positive and negative inputs
         * - Efficient edge case handling
         * - Preserves sign correctly
         *
         * Error characteristics:
         * - Maximum relative error < 0.1%
         * - Better precision for smaller values
         * - Graceful degradation for larger values
         *
         * @param value Input value in range [-4, 4]
         * @return Hyperbolic sine value
         */
        template<typename T = void>
        static constexpr Fxp Sinh(const Fxp& value)
        {
            static_assert(!std::is_same_v<T, T>, "[DRAFT] Sinh() is currently a work in progress and not ready for use");
            // Handle edge cases
            if (value >= 4) return Fxp::BuildRaw(167800);
            if (value <= -4) return Fxp::BuildRaw(-167800);

            // Get absolute value for lookup, we'll adjust the result later
            int32_t absValue = value.Abs().RawValue();

            // Find the table entry for this value
            size_t index = (absValue * 17) >> 16; // Scale to table size
            auto tableValue = sinhTable[index];
            int32_t result = tableValue.ExtractValue(absValue);

            // Apply sign
            return Fxp::BuildRaw(value < Fxp(0) ? -result : result);
        }

        /**
         * @brief Calculates hyperbolic cosine of a value
         *
         * Computes cosh(x) = (e^x + e^-x)/2 using the relationship
         * cosh²(x) = 1 + sinh²(x) for efficiency.
         *
         * Implementation details:
         * - Reuses sinh table for memory efficiency
         * - Exploits even function property
         * - Uses identity cosh²(x) = 1 + sinh²(x)
         * - Handles edge cases efficiently
         *
         * Mathematical properties preserved:
         * - cosh(x) ≥ 1 for all x
         * - cosh(-x) = cosh(x)
         * - Monotonically increasing for x > 0
         *
         * @param value Input value in range [-4, 4]
         * @return Hyperbolic cosine value
         */
        template<typename T = void>
        static constexpr Fxp Cosh(const Fxp& value)
        {
            static_assert(!std::is_same_v<T, T>, "[DRAFT] Cosh() is currently a work in progress and not ready for use");
            // cosh(x) is even function, so we can use absolute value
            int32_t absValue = value.Abs().RawValue();

            // Handle edge cases
            if (absValue >= 65536 * 4) return Fxp::BuildRaw(167801); // cosh(4)

            // Find the table entry for this value
            size_t index = (absValue * 17) >> 16; // Scale to table size
            auto tableValue = sinhTable[index];
            int32_t sinhResult = tableValue.ExtractValue(absValue);

            // cosh²(x) = 1 + sinh²(x)
            return Fxp::BuildRaw(sinhResult).Square() + 1;
        }

        /**
         * @brief Calculates hyperbolic tangent of a value
         *
         * Computes tanh(x) = sinh(x)/cosh(x) with optimizations to
         * ensure accuracy and handle edge cases efficiently.
         *
         * Implementation details:
         * - Uses sinh and cosh functions
         * - Automatically bounded to [-1, 1]
         * - Efficient edge case handling
         * - Preserves sign correctly
         *
         * Mathematical properties preserved:
         * - |tanh(x)| < 1 for all finite x
         * - tanh(-x) = -tanh(x)
         * - Monotonic over entire range
         *
         * @param value Input value in range [-4, 4]
         * @return Hyperbolic tangent value in range [-1, 1]
         */
        template<typename T = void>
        static constexpr Fxp Tanh(const Fxp& value)
        {
            static_assert(!std::is_same_v<T, T>, "[DRAFT] Tanh() is currently a work in progress and not ready for use");
            // Handle edge cases
            if (value >= Fxp(4)) return Fxp(1);
            if (value <= Fxp(-4)) return Fxp(-1);

            // Get sinh and cosh values
            Fxp sinh = Sinh(value);
            Fxp cosh = Cosh(value);

            // Calculate tanh = sinh/cosh
            return sinh / cosh;
        }

        /**
         * @brief Spherical linear interpolation between two angles.
         *
         * Takes the shortest path around the circle between angles.
         * All angle operations naturally wrap around due to the 16-bit
         * angle representation.
         *
         * @param start Starting angle
         * @param end Ending angle
         * @param t Interpolation factor [0,1]
         * @return Interpolated angle
         */
        static constexpr Angle SLerp(const Angle& start, const Angle& end, const Fxp& t)
        {
            // The subtraction and addition will naturally wrap
            Angle diff = end - start;
            return start + (diff * t);
        }
        /** @} */
    };
}