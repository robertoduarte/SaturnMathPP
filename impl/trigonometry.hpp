#pragma once

#include "fxp.hpp"
#include "angle.hpp"
#include <type_traits>
#include <utility>

namespace SaturnMath
{
    /**
     * @brief High-performance trigonometric functions using fixed-point arithmetic.
     * 
     * Implements trigonometric functions using lookup tables and linear interpolation,
     * optimized for Saturn hardware. All functions operate on fixed-point numbers and
     * the Angle class to avoid floating-point operations.
     * 
     * Features:
     * - Efficient lookup table-based implementation
     * - Linear interpolation for accuracy
     * - All operations are constexpr-capable
     * - Zero runtime floating-point operations
     * 
     * @note All trigonometric functions use angles in turns where 1 = 360°
     */
    struct Trigonometry
    {
    private:
        /** @cond INTERNAL */
        /**
         * @brief LookupCache provides efficient fixed-point value interpolation.
         * 
         * How it works:
         * 1. Each table entry contains:
         *    - value: The actual fixed-point value at this point
         *    - interpolationMultiplicand: (next_value - value) / step_size, pre-calculated
         * 
         * 2. For input X between points A and B:
         *    - fraction = X & Mask (gets position between A and B)
         *    - result = A + (fraction * multiplicand) >> shift
         * 
         * Example for sine table (InterpolationShift=15, Mask=0x3FF):
         * - Points every 5.625° (360° / 64)
         * - Mask extracts 10 bits for smooth interpolation
         * - Shift of 15 maintains fixed-point precision
         * 
         * @tparam R Value type (int32_t for Saturn fixed-point)
         * @tparam Mask Fractional bits for interpolation
         * @tparam InterpolationShift Maintains fixed-point precision
         */
        template <typename R, size_t Mask, size_t InterpolationShift>
        struct LookupCache
        {
            static constexpr size_t interpolationShift = InterpolationShift;

            R value;                     // Fixed-point value at this point
            R interpolationMultiplicand; // Pre-calculated (next_value - value) / step_size

            /**
             * @brief Interpolates between table entries using pre-calculated multiplicand.
             * @param input Raw fixed-point value to interpolate
             * @return Interpolated result maintaining fixed-point precision
             */
            constexpr R ExtractValue(const auto &input) const
            {
                // Get fractional position between table entries
                size_t interpolationMultiplier = Mask & input;

                // Special handling for negative multiplicands to maintain precision
                if constexpr (std::is_signed_v<R>)
                    if (interpolationMultiplicand < 0)
                        return value - (R)((interpolationMultiplier * (size_t)(-interpolationMultiplicand)) >> InterpolationShift);

                return value + (R)((interpolationMultiplier * (size_t)interpolationMultiplicand) >> InterpolationShift);
            }
        };

        // Lookup tables with pre-calculated values
        static constexpr LookupCache<int32_t, 0x3FF, 15> sinTable[] = {
            {Fxp(0.000000).value, 205556},   // Sine value for 0 degrees
            {Fxp(0.098017).value, 203577},   // Sine value for 5.625 degrees
            {Fxp(0.195090).value, 199637},   // Sine value for 11.25 degrees
            {Fxp(0.290285).value, 193774},   // Sine value for 16.875 degrees
            {Fxp(0.382683).value, 186045},   // Sine value for 22.5 degrees
            {Fxp(0.471397).value, 176524},   // Sine value for 28.125 degrees
            {Fxp(0.555570).value, 165303},   // Sine value for 33.75 degrees
            {Fxp(0.634393).value, 152491},   // Sine value for 39.375 degrees
            {Fxp(0.707107).value, 138210},   // Sine value for 45 degrees
            {Fxp(0.773010).value, 122597},   // Sine value for 50.625 degrees
            {Fxp(0.831470).value, 105804},   // Sine value for 56.25 degrees
            {Fxp(0.881921).value, 87992},    // Sine value for 61.875 degrees
            {Fxp(0.923880).value, 69333},    // Sine value for 67.5 degrees
            {Fxp(0.956940).value, 50006},    // Sine value for 73.125 degrees
            {Fxp(0.980785).value, 30197},    // Sine value for 78.75 degrees
            {Fxp(0.995185).value, 10098},    // Sine value for 84.375 degrees
            {Fxp(1.000000).value, -10098},   // Sine value for 90 degrees
            {Fxp(0.995185).value, -30197},   // Sine value for 95.625 degrees
            {Fxp(0.980785).value, -50006},   // Sine value for 101.25 degrees
            {Fxp(0.956940).value, -69333},   // Sine value for 106.875 degrees
            {Fxp(0.923880).value, -87992},   // Sine value for 112.5 degrees
            {Fxp(0.881921).value, -105804},  // Sine value for 118.125 degrees
            {Fxp(0.831470).value, -122597},  // Sine value for 123.75 degrees
            {Fxp(0.773010).value, -138210},  // Sine value for 129.375 degrees
            {Fxp(0.707107).value, -152491},  // Sine value for 135 degrees
            {Fxp(0.634393).value, -165303},  // Sine value for 140.625 degrees
            {Fxp(0.555570).value, -176524},  // Sine value for 146.25 degrees
            {Fxp(0.471397).value, -186045},  // Sine value for 151.875 degrees
            {Fxp(0.382683).value, -193774},  // Sine value for 157.5 degrees
            {Fxp(0.290285).value, -199637},  // Sine value for 163.125 degrees
            {Fxp(0.195090).value, -203577},  // Sine value for 168.75 degrees
            {Fxp(0.098017).value, -205556},  // Sine value for 174.375 degrees
            {Fxp(0.000000).value, -205556},  // Sine value for 180 degrees
            {Fxp(-0.098017).value, -203577}, // Sine value for -174.375 degrees
            {Fxp(-0.195090).value, -199637}, // Sine value for -168.75 degrees
            {Fxp(-0.290285).value, -193774}, // Sine value for -163.125 degrees
            {Fxp(-0.382683).value, -186045}, // Sine value for -157.5 degrees
            {Fxp(-0.471397).value, -176524}, // Sine value for -151.875 degrees
            {Fxp(-0.555570).value, -165303}, // Sine value for -146.25 degrees
            {Fxp(-0.634393).value, -152491}, // Sine value for -140.625 degrees
            {Fxp(-0.707107).value, -138210}, // Sine value for -135 degrees
            {Fxp(-0.773010).value, -122597}, // Sine value for -129.375 degrees
            {Fxp(-0.831470).value, -105804}, // Sine value for -123.75 degrees
            {Fxp(-0.881921).value, -87992},  // Sine value for -118.125 degrees
            {Fxp(-0.923880).value, -69333},  // Sine value for -112.5 degrees
            {Fxp(-0.956940).value, -50006},  // Sine value for -106.875 degrees
            {Fxp(-0.980785).value, -30197},  // Sine value for -101.25 degrees
            {Fxp(-0.995185).value, -10098},  // Sine value for -95.625 degrees
            {Fxp(-1.000000).value, 10098},   // Sine value for -90 degrees
            {Fxp(-0.995185).value, 30197},   // Sine value for -84.375 degrees
            {Fxp(-0.980785).value, 50006},   // Sine value for -78.75 degrees
            {Fxp(-0.956940).value, 69333},   // Sine value for -73.125 degrees
            {Fxp(-0.923880).value, 87992},   // Sine value for -67.5 degrees
            {Fxp(-0.881921).value, 105804},  // Sine value for -61.875 degrees
            {Fxp(-0.831470).value, 122597},  // Sine value for -56.25 degrees
            {Fxp(-0.773010).value, 138210},  // Sine value for -50.625 degrees
            {Fxp(-0.707107).value, 152491},  // Sine value for -45 degrees
            {Fxp(-0.634393).value, 165303},  // Sine value for -39.375 degrees
            {Fxp(-0.555570).value, 176524},  // Sine value for -33.75 degrees
            {Fxp(-0.471397).value, 186045},  // Sine value for -28.125 degrees
            {Fxp(-0.382683).value, 193774},  // Sine value for -22.5 degrees
            {Fxp(-0.290285).value, 199637},  // Sine value for -16.875 degrees
            {Fxp(-0.195090).value, 203577},  // Sine value for -11.25 degrees
            {Fxp(-0.098017).value, 205556}   // Sine value for -5.625 degrees
        };

        static constexpr LookupCache<int32_t, 0x3FF, 10> tanTable1[] = {
            {Fxp(0.00000).value, 6454},
            {Fxp(0.09849).value, 6581},
            {Fxp(0.19891).value, 6844},
            {Fxp(0.30335).value, 7265},
            {Fxp(0.41421).value, 7883},
            {Fxp(0.53451).value, 8760},
            {Fxp(0.66818).value, 9994},
            {Fxp(0.82068).value, 11751},
            {Fxp(1.00000).value, 14319},
            {Fxp(1.21850).value, 18225},
            {Fxp(1.49661).value, 24527},
            {Fxp(2.41421).value, 57825},
            {Fxp(3.29656).value, 113428},
            {Fxp(5.02734).value, 335926}};

        static constexpr LookupCache<int32_t, 0x0FF, 8> tanTable2[] = {
            {Fxp(10.15317).value, 223051},
            {Fxp(13.55667).value, 445566},
            {Fxp(20.35547).value, 1335624}};

        static constexpr LookupCache<int32_t, 0x03F, 6> tanTable3[] = {
            {Fxp(40.73548).value, 890193},
            {Fxp(54.31875).value, 1780251},
            {Fxp(81.48324).value, 5340487}};

        static constexpr LookupCache<int32_t, 0x00F, 4> tanTable4[] = {
            {Fxp(162.97262).value, 3560269},
            {Fxp(217.29801).value, 7120505},
            {Fxp(325.94830).value, 21361448}};

        static constexpr LookupCache<int32_t, 0x003, 2> tanTable5[] = {
            {Fxp(51.89814).value, 14240951},
            {Fxp(69.19781).value, 28481894},
            {Fxp(303.79704).value, 85445668},
            {Fxp(607.59446).value, 365979601},
            {0x7FFFFFFF, 0}};

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
            {8192, 0}};

    public:
        /**
         * @name Basic Trigonometric Functions
         * Core trigonometric operations using fixed-point arithmetic.
         * @{
         */
        
        /**
         * @brief Calculates sine of an angle.
         * @param angle Input angle
         * @return Sine value in fixed-point format [-1, 1]
         */
        static constexpr Fxp Sin(const Angle& angle)
        {
            size_t index = angle.value >> 10;
            auto tableValue = sinTable[index];
            return tableValue.ExtractValue(angle.value);
        }

        /**
         * @brief Calculates cosine of an angle.
         * @param angle Input angle
         * @return Cosine value in fixed-point format [-1, 1]
         */
        static constexpr Fxp Cos(const Angle& angle)
        {
            size_t index = (angle.value + Angle::HalfPi().value) >> 10;
            auto tableValue = sinTable[index];
            return tableValue.ExtractValue(angle.value);
        }

        /**
         * @brief Calculates tangent of an angle.
         * 
         * Computes the tangent value for any input angle. The function automatically
         * handles angle wrap-around and maintains high precision even near π/2
         * where tangent approaches infinity.
         * 
         * @param angle Input angle
         * @return Tangent value in fixed-point format
         */
        static constexpr Fxp Tan(const Angle& angle)
        {
            uint16_t tempAngle = angle.value;

            if (tempAngle >= Angle::Pi().value)
                tempAngle += Angle::Pi().value;

            bool secondQuarter = tempAngle >= Angle::HalfPi().value;

            if (secondQuarter)
                tempAngle = Angle::Pi().value - tempAngle;

            auto CalculateValue = [tempAngle, secondQuarter](auto lookupTable, auto upperRange)
            {
                size_t index = (tempAngle - upperRange) >> lookupTable[0].interpolationShift;
                auto tableValue = lookupTable[index];
                int32_t ret = tableValue.ExtractValue(tempAngle);
                return secondQuarter ? -ret : ret;
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
         * @param y Y coordinate
         * @param x X coordinate
         * @return Angle in range [-0.5, 0.5] turns (equivalent to [-180°, 180°])
         */
        static constexpr Angle Atan2(const Fxp& y, const Fxp& x)
        {
            uint16_t result = x < 0 ? 
                (y < 0 ? Angle::Pi().value : -Angle::Pi().value) : 0;

            int32_t divResult;

            if (x.Abs() < y.Abs())
            {
                divResult = (x / y).value;
                result += divResult < 0 ? -Angle::HalfPi().value : Angle::HalfPi().value;
            }
            else
            {
                divResult = -(y / x).value;
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
         * @brief Calculates sine and cosine simultaneously.
         * 
         * More efficient than calculating sine and cosine separately when both
         * values are needed.
         * 
         * @param angle Input angle
         * @return Pair of {sine, cosine} values
         */
        static constexpr std::pair<Fxp, Fxp> SinCos(const Angle& angle)
        {
            size_t sinIndex = angle.value >> 10;
            size_t cosIndex = (angle.value + Angle::HalfPi().value) >> 10;
            
            auto sinTableValue = sinTable[sinIndex];
            auto cosTableValue = sinTable[cosIndex];
            
            return {
                sinTableValue.ExtractValue(angle.value),
                cosTableValue.ExtractValue(angle.value)
            };
        }
        /** @} */
    };

    /**
     * @name Free Function Wrappers
     * Convenient free function wrappers for trigonometric operations.
     * @{
     */
    
    /** @copydoc Trigonometry::Sin */
    inline constexpr Fxp Sin(const Angle& angle) { return Trigonometry::Sin(angle); }
    
    /** @copydoc Trigonometry::Cos */
    inline constexpr Fxp Cos(const Angle& angle) { return Trigonometry::Cos(angle); }
    
    /** @copydoc Trigonometry::Tan */
    inline constexpr Fxp Tan(const Angle& angle) { return Trigonometry::Tan(angle); }
    
    /** @copydoc Trigonometry::Atan2 */
    inline constexpr Angle Atan2(const Fxp& y, const Fxp& x) { return Trigonometry::Atan2(y, x); }
    
    /** @copydoc Trigonometry::SinCos */
    inline constexpr std::pair<Fxp, Fxp> SinCos(const Angle& angle) { return Trigonometry::SinCos(angle); }
    /** @} */
}