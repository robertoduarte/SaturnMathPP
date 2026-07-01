#pragma once

#include <stddef.h>
#include <stdint.h>
#include <concepts>
#include <numbers>
#include "precision.hpp"
#include "hardware.hpp"

namespace SaturnMath::Types
{
    // Forward declarations for friend declarations in FixedPoint
    template<int I, int F> struct Vector2;
    template<int I, int F> struct Vector3;

    /**
     * @brief Configurable fixed-point arithmetic optimized for Saturn hardware.
     *
     * @tparam I Number of integer bits (default: 16, minimum: 2)
     * @tparam F Number of fractional bits (default: 16, minimum: 8)
     *
     * @details Uses a 32-bit signed integer split into I integer bits and F fractional bits,
     * where I + F must equal 32. The decimal point position is configurable at compile-time
     * via template parameters, allowing different precision/range trade-offs.
     *
     * Performance features:
     * - Compile-time evaluation with constexpr/consteval
     * - Hardware-optimized multiplication using dmuls.l
     * - Efficient division through hardware divider unit
     * - Zero runtime floating-point operations
     *
     * Example:
     * @code
     * // Default 16.16 format for general use
     * constexpr Fxp16_16 a = 5;            // 5    (0x00050000)
     * constexpr Fxp16_16 b = 2.5;          // 2.5  (0x00028000)
     *
     * // Custom formats via aliases
     * constexpr Fxp24_8  largeWorld = 100000;      // 24.8 format
     * constexpr Fxp8_24 preciseMat = 0.123456789; // 8.24 format
     *
     * // Runtime conversions
     * Fxp16_16 c = Fxp16_16::Convert(someInt);    // With compile-time range checking
     * Fxp16_16 d = Fxp16_16::Convert(someFloat);  // Will trigger a performance warning!
     *
     * // Arithmetic operations (Zero runtime float conversions)
     * Fxp16_16 result = a * 3.14;                 // Evaluated safely at compile-time
     * int16_t i = result.As<int16_t>();        // Extracts integer part
     * @endcode
     *
     * @note Template constraints enforce that I + F == 32, I >= 2, and F >= 8 to ensure
     * a valid fixed-point representation with sufficient range and precision.
     *
     * @note All operations are designed for maximum efficiency on Saturn hardware.
     * Avoid runtime floating-point conversions in performance-critical code.
     *
     * @note The \#pragma GCC optimize("O2") directive is used to enable optimizations that improve performance,
     * specifically for fixed-point arithmetic operations. It ensures that the compiler generates efficient
     * code that takes full advantage of the hardware capabilities, particularly in performance-critical sections.
     *
     * @note Operations between different FixedPoint formats:
     * - Multiplication and division are supported between different formats (result uses left operand's format)
     * - Addition, subtraction, comparison, and modulo are NOT supported between different formats
     * - This is by design to prevent silent precision loss or overflow. Converting between formats
     *   can cause:
     *   - Precision loss: converting from more fractional bits to fewer (e.g., 8.24 → 16.16)
     *   - Overflow risk: converting from fewer integer bits to more (e.g., 24.8 → 16.16)
     * - To perform these operations, explicitly convert one operand to the other's format using Convert():
     *   @code
     *   FixedPoint<16, 16> a = 10;
     *   FixedPoint<24, 8> b = 5;
     *   // This will NOT compile:
     *   // auto sum = a + b;
     *   // Use explicit conversion instead (you are now aware of potential precision loss):
     *   auto sum = a + FixedPoint<16, 16>::Convert(b);
     *   @endcode
     */
#pragma GCC optimize("O2")
    template<int I = 16, int F = 16>
        requires (I + F == 32) && (I >= 2) && (F >= 8)
    class FixedPoint
    {
    private:
        int32_t value; /**< Raw fixed-point value in I.F format */

        // Allow access to private members between different FixedPoint instantiations
        template<int OI, int OF>
            requires (OI + OF == 32) && (OI >= 2) && (OF >= 8)
        friend class FixedPoint;

        // Allow vectors to use InternalSqrtFrom64 for Length() calculations
        template<int VI, int VF> friend struct Vector2;
        template<int VI, int VF> friend struct Vector3;

        /**
         * @brief Private constructor for raw fixed-point values
         * @param inValue Raw I.F fixed-point value (I integer bits, F fractional bits)
         * @param unused Boolean flag to differentiate from implicit constructors
         */
        [[gnu::always_inline]] constexpr FixedPoint(int32_t inValue, bool /*unused*/) : value(inValue) {}

        /**
         * @brief Internal silent injection of integral values (no warnings)
         * @tparam T Integral type
         * @param value The integral value to inject
         * @return Fixed-point value with bits directly injected
         * @details This is for internal use only - bypasses safety checks and warnings
         */
        template<std::integral T>
        [[gnu::always_inline]] static constexpr FixedPoint InternalInject(T value)
        {
            return BuildRaw(static_cast<int32_t>(static_cast<uint32_t>(value) << F));
        }

        /**
         * @brief Fixed-point square root from a 64-bit intermediate.
         * @param fxpHigh Upper 32 bits of a 64-bit value with 2F fractional bits
         *                (e.g. MAC register output from dot product).
         * @param fxpLow Lower 32 bits.
         * @return FixedPoint with F fractional bits.
         * @details Internal API. The 64-bit integer sqrt naturally halves the
         *          fractional bit count (2F -> F), making this format-agnostic.
         *          Unlike the 32-bit Sqrt(), no F/2 scaling trick is needed
         *          since 64 bits provide enough headroom. Used by Vector2D::Length()
         *          and Vector3D::Length() to process MAC register output.
         */
        [[gnu::always_inline]] static constexpr FixedPoint InternalSqrtFrom64(uint32_t fxpHigh, uint32_t fxpLow)
        {
            if ((fxpHigh | fxpLow) == 0)
                return BuildRaw(0);

            auto shiftRight64 = [](uint32_t& hi, uint32_t& lo)
            {
                if consteval {
                    lo = (lo >> 1) | (hi << 31);
                    hi >>= 1;
                } else {
                    Hardware::ShiftRight64(hi, lo);
                }
            };

            auto extractMid32 = [](const uint32_t& hi, uint32_t& lo)
            {
                if consteval {
                    lo = (hi << 16) | (lo >> 16);
                } else {
                    Hardware::ExtractMid32(hi, lo);
                }
            };

            uint32_t baseEstimation;
            uint32_t estimation;
            uint32_t iterationValue;

            if (fxpHigh >= 0x00010000)
            {
                baseEstimation = 1 << 23;
                iterationValue = fxpHigh >> 17;
                estimation = (fxpHigh << 8) | (fxpLow >> 24);
                fxpHigh >>= 24;

                while (iterationValue)
                {
                    shiftRight64(fxpHigh, estimation);
                    baseEstimation <<= 1;
                    iterationValue >>= 2;
                }

                estimation >>= 1;
            }
            else
            {
                // Small value fix: when fxpHigh==0 && fxpLow < 0x10000,
                // extractMid32 zeros out all significant bits. Use fxpLow
                // directly and >> 8 the result (sqrt(2^16) = 2^8).
                // Only needed when F < 20 (for F >= 20, the smallest non-zero
                // raw value squared is 1, and sqrt(1) >> 8 = 0 either way).
                if constexpr (F < 20)
                {
                    if (fxpHigh == 0 && fxpLow < 0x00010000)
                    {
                        estimation = fxpLow;
                        baseEstimation = 1 << 7;
                        iterationValue = estimation >> 1;

                        while (iterationValue)
                        {
                            estimation >>= 1;
                            baseEstimation <<= 1;
                            iterationValue >>= 2;
                        }

                        estimation <<= 7;
                        return BuildRaw(static_cast<int32_t>((baseEstimation + estimation) >> 8));
                    }
                }

                estimation = fxpLow;
                extractMid32(fxpHigh, estimation);
                baseEstimation = 1 << 7;
                iterationValue = estimation >> 1;

                if (estimation >= 0x00010000)
                {
                    baseEstimation <<= 8;
                    estimation >>= 8;
                    iterationValue >>= 16;
                }

                while (iterationValue)
                {
                    estimation >>= 1;
                    baseEstimation <<= 1;
                    iterationValue >>= 2;
                }

                estimation <<= 7;
            }
            return BuildRaw(static_cast<int32_t>(baseEstimation + estimation));
        }

    public:
        static constexpr int IntBits = I;
        static constexpr int FracBits = F;
        static constexpr int TotalBits = I + F;
        static constexpr int FractionScale = 1 << F;
        static constexpr double FractionScaleDouble = static_cast<double>(1 << F);

        /**
         * @brief Auxiliary wrapper to force strict compile-time conversion of float literals.
         *
         * @details Intercepts floating-point values in mathematical and comparison operators.
         * Because the constructor is consteval, any attempt to pass a runtime float variable
         * will cause an immediate compilation error, protecting Saturn's hardware performance.
         * For explicit runtime conversions, use FixedPoint::Convert().
         */
        struct CompileTimeFloat
        {
            FixedPoint fxp;

            template <std::floating_point T>
            consteval CompileTimeFloat(T v) : fxp(v) {}
        };

        /** @name Core Constants */
        ///@{
        /** * @brief Returns the value 0.0
         * @return Zero value in I.F format 
         */
        static consteval FixedPoint Zero() { return BuildRaw(0); }

        /** * @brief Returns a small epsilon value for fixed-point comparisons 
         * @return Minimum positive representable value
         */
        static consteval FixedPoint Epsilon() { return BuildRaw(0x00000001); }

        /** * @brief Returns a value just below 1.0 
         * @return The largest fractional value less than 1.0
         */
        static consteval FixedPoint NearOne() { return BuildRaw(FractionScale - 1); }

        /** * @brief Returns the value 1.0 
         * @return The value 1.0 in I.F format
         */
        static consteval FixedPoint One() { return BuildRaw(FractionScale); }

        /** * @brief Returns the mathematical constant Pi (π)
         * @return Pi as a fixed-point value
         */
        static consteval FixedPoint Pi() { return FixedPoint(std::numbers::pi); }

        /** * @brief Returns the minimum possible value 
         * @return The most negative representable value
         */
        static consteval FixedPoint MinValue() { return BuildRaw(0x80000000); }

        /** * @brief Returns the maximum possible value 
         * @return The most positive representable value
         */
        static consteval FixedPoint MaxValue() { return BuildRaw(0x7FFFFFFF); }
        ///@}

        /** @name Constructors & Conversions */
        ///@{
        /**
         * @brief Default constructor for FixedPoint class.
         * Initializes the fixed-point number to zero.
         */
        constexpr FixedPoint() : value(0) {}

        /**
         * @brief Copy constructor for FixedPoint class.
         * @details Defaulted so the type stays trivially copyable: a user-defined
         *          copy constructor would make FixedPoint non-trivially-copyable,
         *          forcing it to be passed/returned via memory (with copy-constructor
         *          calls) instead of in registers on the SH-2 ABI.
         */
        constexpr FixedPoint(const FixedPoint& fxp) = default;

        /**
         * @brief Constructor for FixedPoint class from small integral types.
         * @tparam T Integral type that fits within I integer bits
         * @param inValue The integer value to convert to fixed-point.
         *
         * @details This is the primary runtime constructor for small integer values.
         * Uses direct shift by F bits for efficient conversion (no floating-point operations).
         */
        template<std::integral T>
            requires (std::is_signed_v<T> ? (sizeof(T) * 8 <= I) : (sizeof(T) * 8 + 1 <= I))
        constexpr FixedPoint(T inValue) : value(static_cast<int32_t>(static_cast<uint32_t>(inValue) << F)) {}

        /**
         * @brief Compile-time constructor for floating-point and large integral types.
         * @tparam T Numeric type (float, double, or large integral types)
         * @param inValue The value to convert to fixed-point
         *
         * @details This constructor is only available at compile time for floating-point
         * types and integral types that don't fit within I integer bits.
         */
        template<typename T>
            requires (!(std::is_signed_v<T> ? (sizeof(T) * 8 <= I) : (sizeof(T) * 8 + 1 <= I)))
        consteval FixedPoint(T inValue) : value(static_cast<int32_t>(inValue * FractionScaleDouble)) {}

        /**
         * @brief Convert from integral type (safe conversion)
         * @tparam T Integral type that fits within I integer bits
         * @param value Integral value to convert
         * @return FixedPoint value
         */
        template<std::integral T>
            requires (std::is_signed_v<T> ? (sizeof(T) * 8 <= I) : (sizeof(T) * 8 + 1 <= I))
        [[gnu::always_inline]] static constexpr FixedPoint Convert(T value)
        {
            return BuildRaw(static_cast<int32_t>(static_cast<uint32_t>(value) << F));
        }

        /**
         * @brief Convert from integral type (may cause narrowing)
         * @tparam T Integral type that may not fit within I integer bits
         * @param value Integral value to convert
         * @return FixedPoint value
         * @deprecated Type does not fit within I integer bits - narrowing may occur
         */
        template<std::integral T>
            requires (! (std::is_signed_v<T> ? (sizeof(T) * 8 <= I) : (sizeof(T) * 8 + 1 <= I)))
        [[deprecated("Type does not fit within integer bits - narrowing may occur")]]
        static constexpr FixedPoint Convert(T value)
        {
            return BuildRaw(static_cast<int32_t>(static_cast<uint32_t>(value) << F));
        }

        /**
         * @brief Convert floating-point to fixed-point with performance warning.
         * @tparam T Floating-point type (float, double)
         * @param value The value to convert
         * @return Fixed-point value
         */
        template <std::floating_point T>
#ifndef DISABLE_PERFORMANCE_WARNINGS
        [[gnu::warning("Converting from floating-point is a heavy operation - avoid in performance-critical code")]]
#endif
        static constexpr FixedPoint Convert(const T& value) { return BuildRaw(static_cast<int32_t>(value * FractionScaleDouble)); }

        /**
         * @brief Convert from another FixedPoint format (safe conversion)
         * @tparam OtherI Other integer bits
         * @tparam OtherF Other fractional bits
         * @param other FixedPoint value to convert
         * @return Fixed-point value
         * @details This version is for conversions that do not risk precision loss or overflow.
         * Use when the destination format has equal or more integer bits and equal or more fractional bits.
         * Example: Converting from 24.8 to 16.16 is safe (more fractional bits, fewer integer bits).
         */
        template<int OtherI, int OtherF>
            requires (OtherI <= I && OtherF <= F)
        [[gnu::always_inline]] static constexpr FixedPoint Convert(const FixedPoint<OtherI, OtherF>& other)
        {
            if constexpr (OtherF > F)
            {
                if consteval {
                    return BuildRaw(other.value >> (OtherF - F));
                } else {
                    int32_t raw = other.value;
                    Hardware::ArithmeticShiftRight<OtherF - F>(raw);
                    return BuildRaw(raw);
                }
            }
            else if constexpr (F > OtherF)
                return BuildRaw(other.value << (F - OtherF));
            else
                return BuildRaw(other.value);
        }

        /**
         * @brief Convert from another FixedPoint format (may cause precision loss or overflow)
         * @tparam OtherI Other integer bits
         * @tparam OtherF Other fractional bits
         * @param other FixedPoint value to convert
         * @return Fixed-point value
         * @deprecated Conversion may cause precision loss (fewer fractional bits) or overflow (fewer integer bits)
         * @details This version is for conversions that may lose precision or overflow.
         * Be aware of the risks when using this conversion.
         * Example: Converting from 16.16 to 24.8 may lose fractional precision.
         * Example: Converting from 8.24 to 16.16 may overflow if the integer part is too large.
         */
        template<int OtherI, int OtherF>
            requires (!(OtherI <= I && OtherF <= F))
        [[deprecated("Conversion may cause precision loss (fewer fractional bits) or overflow (fewer integer bits)")]]
        [[gnu::always_inline]] static constexpr FixedPoint Convert(const FixedPoint<OtherI, OtherF>& other)
        {
            if constexpr (OtherF > F)
            {
                if consteval {
                    return BuildRaw(other.value >> (OtherF - F));
                } else {
                    int32_t raw = other.value;
                    Hardware::ArithmeticShiftRight<OtherF - F>(raw);
                    return BuildRaw(raw);
                }
            }
            else if constexpr (F > OtherF)
                return BuildRaw(other.value << (F - OtherF));
            else
                return BuildRaw(other.value);
        }

        /**
         * @brief Convert from integral type without narrowing warning.
         * @tparam T Integral type that may not fit within I integer bits
         * @param value Integral value to convert
         * @return FixedPoint value
         * @details Use this when you know the value fits even though the type
         * is wider than I bits. Performs the same operation as the deprecated
         * Convert(T) but without the compiler warning.
         */
        template<std::integral T>
            requires (! (std::is_signed_v<T> ? (sizeof(T) * 8 <= I) : (sizeof(T) * 8 + 1 <= I)))
        [[gnu::always_inline]] static constexpr FixedPoint ConvertUnchecked(T value)
        {
            return BuildRaw(static_cast<int32_t>(static_cast<uint32_t>(value) << F));
        }

        /**
         * @brief Convert from another FixedPoint format without warning.
         * @tparam OtherI Other integer bits
         * @tparam OtherF Other fractional bits
         * @param other FixedPoint value to convert
         * @return FixedPoint value
         * @details Use this when you know the conversion is safe (e.g. the
         * value fits in the destination format even though the type has more
         * bits). Performs the same operation as the deprecated Convert but
         * without the compiler warning.
         */
        template<int OtherI, int OtherF>
            requires (!(OtherI <= I && OtherF <= F))
        [[gnu::always_inline]] static constexpr FixedPoint ConvertUnchecked(const FixedPoint<OtherI, OtherF>& other)
        {
            if constexpr (OtherF > F)
            {
                if consteval {
                    return BuildRaw(other.value >> (OtherF - F));
                } else {
                    int32_t raw = other.value;
                    Hardware::ArithmeticShiftRight<OtherF - F>(raw);
                    return BuildRaw(raw);
                }
            }
            else if constexpr (F > OtherF)
                return BuildRaw(other.value << (F - OtherF));
            else
                return BuildRaw(other.value);
        }
        ///@}

        /** @name Static Constructors & Utilities */
        ///@{
        /**
         * @brief Returns the larger of two fixed-point values.
         * @param a First value
         * @param b Second value
         * @return The maximum of the two values
         */
        static constexpr FixedPoint Max(FixedPoint a, FixedPoint b) { return (a > b) ? a : b; }

        /**
         * @brief Returns the smaller of two fixed-point values.
         * @param a First value
         * @param b Second value
         * @return The minimum of the two values
         */
        static constexpr FixedPoint Min(FixedPoint a, FixedPoint b) { return (a < b) ? a : b; }

        /**
         * @brief Creates fixed-point from raw I.F value (unrestricted)
         * @param rawValue The raw 32-bit integer representation
         * @return Fixed-point value constructed from raw bits
         * @details This is the only unrestricted way to create a FixedPoint without any validation
         */
        [[gnu::always_inline]] static constexpr FixedPoint BuildRaw(int32_t rawValue) { return FixedPoint(rawValue, true); }
        ///@}

        /** @name Hardware Division (Saturn Divider Unit) */
        ///@{
        /**
         * @brief Sets up hardware division unit for fixed-point division.
         * @param dividend Numerator
         * @param divisor Denominator
         * @deprecated Use ParallelDiv() instead. AsyncDivSet/AsyncDivGetResult operate on
         *             raw DIVU registers and are error-prone when mixing FixedPoint
         *             formats (the result format depends on the dividend format).
         *             To be removed in a future version.
         */
        [[gnu::deprecated("Use ParallelDiv() instead. To be removed in a future version.")]]
        [[gnu::always_inline]] static void AsyncDivSet(FixedPoint dividend, FixedPoint divisor)
        {
            int32_t dividendHigh = dividend.value;
            if constexpr (32 - F > 0)
                Hardware::ArithmeticShiftRight<32 - F>(dividendHigh);
            Hardware::DivSet(divisor.value, dividendHigh,
                             static_cast<int32_t>(static_cast<uint32_t>(dividend.value) << F));
        }

        /**
         * @brief Retrieves result from hardware division unit.
         * @return Fixed-point result from previous AsyncDivSet()
         * @deprecated Use ParallelDiv() instead. To be removed in a future version.
         */
        [[gnu::deprecated("Use ParallelDiv() instead. To be removed in a future version.")]]
        [[gnu::always_inline]] static FixedPoint AsyncDivGetResult() { return BuildRaw(Hardware::DivGetResult()); }

        /**
         * @brief Retrieves remainder from hardware division unit.
         * @return Fixed-point remainder from previous AsyncDivSet()
         * @deprecated Use ParallelDiv() instead. To be removed in a future version.
         */
        [[gnu::deprecated("Use ParallelDiv() instead. To be removed in a future version.")]]
        [[gnu::always_inline]] static FixedPoint AsyncDivGetRemainder() { return BuildRaw(Hardware::DivGetRemainder()); }
        ///@}

        /** @name Mathematical Operations */
        ///@{
        /**
         * @brief Removes fractional part, keeping only integer portion.
         * @return Fixed-point value with zeroed F fractional bits
         */
        constexpr FixedPoint TruncateFraction() const
        {
            constexpr int32_t mask = ~((1 << F) - 1);
            if (value >= 0) return BuildRaw(mask & value);
            else return -BuildRaw(mask & (-value));
        }

        /**
         * @brief Extracts the fractional part of the fixed-point value.
         * @return Fixed-point value containing only the F fractional bits (0.xxxx)
         */
        constexpr FixedPoint GetFraction() const
        {
            constexpr int32_t mask = (1 << F) - 1;
            if (value >= 0) return BuildRaw(mask & value);
            else return -BuildRaw(mask & (-value));
        }

        /**
         * @brief Rounds down to the nearest integer.
         * @return Fixed-point value rounded down to nearest integer.
         */
        constexpr FixedPoint Floor() const
        {
            constexpr int32_t mask = ~((1 << F) - 1);
            return BuildRaw(value & mask);
        }

        /**
         * @brief Rounds up to the nearest integer.
         * @return Fixed-point value rounded up to nearest integer.
         */
        constexpr FixedPoint Ceil() const
        {
            constexpr int32_t mask = ~((1 << F) - 1);
            return BuildRaw((value + ((1 << F) - 1)) & mask);
        }

        /**
         * @brief Rounds to the nearest integer.
         * @return Fixed-point value rounded to nearest integer.
         */
        constexpr FixedPoint Round() const
        {
            constexpr int32_t half = 1 << (F - 1);
            constexpr int32_t fracMask = (1 << F) - 1;

            if (value >= 0)
                return BuildRaw((value + half) & ~fracMask);
            else
            {
                FixedPoint frac = GetFraction();
                int32_t fracValue = (frac.value >= 0) ? frac.value : -frac.value;
                if (fracValue < half) return BuildRaw(value + half).Floor();
                else return BuildRaw(value - half).Floor();
            }
        }

        /**
         * @brief Calculate square root
         * @return Square root of the value in I.F format
         * @details The F/2 shifts are not cosmetic — they fold the 2^(F/2)
         *          scaling into the binary search to avoid 64-bit intermediates
         *          while preserving precision. This is equivalent to
         *          isqrt(value * 2^F) but fits in 32 bits.
         */
        constexpr FixedPoint Sqrt() const
        {
            if (value <= 0) return BuildRaw(0);

            uint32_t baseEstimation = 1 << ((F / 2) - 1);
            uint32_t estimation = value;
            uint32_t iterationValue = value >> 1;

            if (value >= (1 << F))
            {
                baseEstimation <<= (F / 2);
                estimation >>= (F / 2);
                iterationValue >>= F;
            }

            while (iterationValue)
            {
                estimation >>= 1;
                baseEstimation <<= 1;
                iterationValue >>= 2;
            }
            estimation <<= ((F / 2) - 1);
            return BuildRaw(static_cast<int32_t>(baseEstimation + estimation));
        }

        /**
         * @brief Calculate square root with configurable precision (deprecated)
         * @tparam P Precision level for calculation (ignored)
         * @return Square root of the value
         */
        template<SaturnMath::Precision P = SaturnMath::Precision::Default>
        [[deprecated("Use Sqrt() instead - precision parameter is ignored")]]
        constexpr FixedPoint Sqrt() const { return Sqrt(); }

        /**
         * @brief Squares the value (x²).
         * @return x * x in fixed-point
         */
        constexpr FixedPoint Square() const { return *this * *this; }

        /**
         * @brief Absolute value (|x|).
         * @return |x| in fixed-point
         */
        [[gnu::always_inline]] constexpr FixedPoint Abs() const { return BuildRaw(value > 0 ? value : -value); }

        /**
         * @brief Returns the internal raw fixed-point value.
         * @return The internal 32-bit representation (by value: keeps the type's
         *         accessors trivially inlinable and register-passed on SH-2).
         */
        [[gnu::always_inline]] constexpr int32_t RawValue() const { return value; }

        /**
         * @brief Returns the internal raw fixed-point value (deprecated).
         * @return The internal 32-bit representation
         * @deprecated Use RawValue() instead
         */
        [[deprecated("Use RawValue() instead")]]
        constexpr int32_t Raw() const { return RawValue(); }

        /**
         * @brief Converts to the specified integer type.
         * @tparam T The target integer type
         * @return Value as the specified type
         */
        template<typename T> requires std::integral<T>
        constexpr T As() const
        {
            if consteval {
                return static_cast<T>(value >> F);
            } else {
                int32_t raw = value;
                Hardware::ArithmeticShiftRight<F>(raw);
                return static_cast<T>(raw);
            }
        }

        /**
         * @brief Converts to the specified floating-point type.
         * @tparam T The target floating-point type (float or double)
         * @return Value as the specified type
         */
        template<typename T> requires std::floating_point<T>
#ifndef DISABLE_PERFORMANCE_WARNINGS
        [[gnu::warning("Converting to floating-point is a heavy operation - avoid in performance-critical code")]]
#endif
        constexpr T As() const { return value / T{ FractionScaleDouble }; }

        /**
         * @brief Clears the MAC (Multiply-and-Accumulate) registers.
         */
        [[gnu::always_inline]] static void ClearMac() { Hardware::MacClear(); }

        /**
         * @brief Extracts the result from MAC registers into a fixed-point value.
         * @return Fixed-point value containing the MAC operation result
         */
        [[gnu::always_inline]] static FixedPoint ExtractMac()
        {
            return BuildRaw(Hardware::MacExtract());
        }
        ///@}

        /** @name Arithmetic Operators */
        ///@{
        /**
         * @brief Float literal operations assignment.
         */
        constexpr FixedPoint& operator*=(CompileTimeFloat other) { return *this *= other.fxp; }

        /**
         * @brief Divide this object by a compile-time float literal.
         * @param other The compile-time float literal.
         * @return Reference to this object.
         */
        constexpr FixedPoint& operator/=(CompileTimeFloat other) { return *this /= other.fxp; }

        /**
         * @brief Fixed-point addition (a += b).
         * @param other Value to add
         * @return Reference to this
         */
        [[gnu::always_inline]] constexpr FixedPoint& operator+=(FixedPoint other) { value += other.value; return *this; }

        /**
         * @brief Fixed-point subtraction (a -= b).
         * @param other Value to subtract
         * @return Reference to this
         */
        [[gnu::always_inline]] constexpr FixedPoint& operator-=(FixedPoint other) { value -= other.value; return *this; }

        /**
         * @brief Multiplies the current fixed-point value by another fixed-point value (a *= b).
         * @param other The fixed-point value to multiply with.
         * @return A reference to the current instance.
         */
        template<int OI, int OF>
        [[gnu::always_inline]] constexpr FixedPoint& operator*=(const FixedPoint<OI, OF>& other)
        {
            constexpr int shift = OF;
            static_assert(shift >= 0 && shift < 32, "Shift out of bounds");

            if consteval
            {
                value = static_cast<int32_t>(
                    (static_cast<int64_t>(value) * static_cast<int64_t>(other.value)) >> shift
                );
                return *this;
            }

            int32_t mach, macl;
            Hardware::Mul64(value, other.value, mach, macl);
            Hardware::Extract32<shift>(mach, macl, value);
            return *this;
        }

        /**
         * @brief Multiplies the current fixed-point value by an integer (a *= b).
         * @tparam T The type of the integer.
         * @param value The integer value to multiply with.
         * @return A reference to this object.
         */
        template<typename T> requires std::is_integral_v<T>
        [[gnu::always_inline]] constexpr FixedPoint& operator*=(const T& value) { this->value *= value; return *this; }

        /**
         * @brief Fixed-point multiplication (a * b).
         * @param other Value to multiply by.
         * @return Product as FixedPoint.
         */
        template<int OI, int OF>
        [[gnu::always_inline]] constexpr FixedPoint operator*(FixedPoint<OI, OF> other) const { return FixedPoint(*this) *= other; }

        /**
         * @brief Float literal operations right side.
         */
        constexpr FixedPoint operator*(CompileTimeFloat other) const { return *this * other.fxp; }
        
        /**
         * @brief Divides this object by a compile-time float literal.
         * @param other The compile-time float literal.
         * @return The quotient as a new FixedPoint object.
         */
        constexpr FixedPoint operator/(CompileTimeFloat other) const { return *this / other.fxp; }

        /**
         * @brief Multiplies the current fixed-point value by an integer (a * b).
         * @tparam T The type of the integer.
         * @param value The integer value to multiply with.
         * @return The product as a new FixedPoint object.
         */
        template<typename T> requires std::is_integral_v<T>
        [[gnu::always_inline]] constexpr FixedPoint operator*(const T& value) const { return BuildRaw(value * this->value); }

        /**
         * @brief Adds an object to a compile-time float literal.
         * @param lhs The compile-time float literal.
         * @param rhs The FixedPoint object.
         * @return The sum as a new FixedPoint object.
         */
        constexpr friend FixedPoint operator+(CompileTimeFloat lhs, const FixedPoint& rhs) { return lhs.fxp + rhs; }
        
        /**
         * @brief Subtracts an object from a compile-time float literal.
         * @param lhs The compile-time float literal.
         * @param rhs The FixedPoint object.
         * @return The difference as a new FixedPoint object.
         */
        constexpr friend FixedPoint operator-(CompileTimeFloat lhs, const FixedPoint& rhs) { return lhs.fxp - rhs; }
        
        /**
         * @brief Multiplies a compile-time float literal by an object.
         * @param lhs The compile-time float literal.
         * @param rhs The FixedPoint object.
         * @return The product as a new FixedPoint object.
         */
        constexpr friend FixedPoint operator*(CompileTimeFloat lhs, const FixedPoint& rhs) { return lhs.fxp * rhs; }
        
        /**
         * @brief Divides a compile-time float literal by an object.
         * @param lhs The compile-time float literal.
         * @param rhs The FixedPoint object.
         * @return The quotient as a new FixedPoint object.
         */
        constexpr friend FixedPoint operator/(CompileTimeFloat lhs, const FixedPoint& rhs) { return lhs.fxp / rhs; }

        /**
         * @brief Multiplies an integral value by a fixed-point value (lhs * rhs).
         * @tparam T The type of the integer.
         * @param lhs The value to multiply.
         * @param rhs The fixed-point value.
         * @return The product as a new FixedPoint object.
         */
        template<typename T> requires std::is_integral_v<T>
        [[gnu::always_inline]] constexpr friend FixedPoint operator*(T lhs, const FixedPoint& rhs) { return rhs * lhs; }

        /**
         * @brief Divides the current fixed-point value by another fixed-point value (a /= b).
         * @param other Value to divide by.
         * @return Reference to this object.
         */
        template<int OI, int OF>
        [[gnu::always_inline]] constexpr FixedPoint& operator/=(FixedPoint<OI, OF> other)
        {
            if consteval
            {
                double a = static_cast<double>(value) / static_cast<double>(1 << F);
                double b = static_cast<double>(other.value) / static_cast<double>(1 << OF);
                if (b == 0.0) this->value = (a >= 0.0) ? MaxValue().value : MinValue().value;
                else this->value = static_cast<int32_t>((a / b) * static_cast<double>(1 << F));
            }
            else
            {
                int32_t dividendHigh = value;
                if constexpr (32 - OF > 0)
                    Hardware::ArithmeticShiftRight<32 - OF>(dividendHigh);
                Hardware::DivSet(other.value, dividendHigh,
                                 static_cast<int32_t>(static_cast<uint32_t>(value) << OF));
                this->value = Hardware::DivGetResult();
            }
            return *this;
        }

        /**
         * @brief Divides the current fixed-point value by another fixed-point value (a / b).
         * @param other Value to divide by.
         * @return Quotient as a new FixedPoint object.
         */
        template<int OI, int OF>
        [[gnu::always_inline]] constexpr FixedPoint operator/(FixedPoint<OI, OF> other) const { return FixedPoint(*this) /= other; }

        /**
         * @brief Divides the current fixed-point value by an integer (a /= b).
         * @tparam T The type of the integer.
         * @param value The integer value to divide by.
         * @return A reference to this object.
         */
        template<typename T> requires std::is_integral_v<T>
        [[gnu::always_inline]] constexpr FixedPoint& operator/=(const T& value)
        {
            if consteval
            {
                double a = this->value / FractionScaleDouble;
                if (value == 0) this->value = (this->value >= 0) ? INT32_MAX : INT32_MIN;
                else this->value = static_cast<int32_t>((a / static_cast<double>(value)) * FractionScaleDouble);
            }
            else
            {
                this->value /= value;
            }
            return *this;
        }

        /**
         * @brief Divides the current fixed-point value by an integer (a / b).
         * @tparam T The type of the integer.
         * @param value The integer value to divide by.
         * @return The quotient as a new FixedPoint object.
         */
        template<typename T> requires std::is_integral_v<T>
        [[gnu::always_inline]] constexpr FixedPoint operator/(const T& value) const { return BuildRaw(this->value / value); }

        /**
         * @brief Divides an integral value by a fixed-point value (lhs / rhs).
         * @tparam T The type of the integer.
         * @param lhs The integer value to divide.
         * @param rhs The fixed-point value.
         * @return The quotient as a new FixedPoint object.
         */
        template<typename T> requires std::is_integral_v<T>
        [[gnu::always_inline]] constexpr friend FixedPoint operator/(T lhs, const FixedPoint& rhs) { return InternalInject(lhs) / rhs; }

        /**
         * @brief Computes the modulo of the current fixed-point value with another fixed-point value (a % b).
         * @param other The fixed-point value to use as the modulus.
         * @return The result of the modulo operation.
         */
        [[gnu::always_inline]] constexpr FixedPoint operator%(FixedPoint other) const { return BuildRaw(value % other.value); }

        /**
         * @brief Computes the modulo of an integer with a fixed-point value (lhs % rhs).
         * @tparam T The type of the integer.
         * @param lhs The integer value.
         * @param rhs The fixed-point value to use as the modulus.
         * @return The result of the modulo operation.
         */
        template<typename T> requires std::is_integral_v<T>
        [[gnu::always_inline]] constexpr friend FixedPoint operator%(T lhs, FixedPoint rhs) { return InternalInject(lhs) %= rhs; }

        /**
         * @brief Computes the modulo of the current fixed-point value with another fixed-point value (a %= b).
         * @param other The fixed-point value to use as the modulus.
         * @return A reference to this object.
         */
        [[gnu::always_inline]] constexpr FixedPoint& operator%=(FixedPoint other) { this->value %= other.value; return *this; }

        /**
         * @brief Copy assignment operator.
         * @return A reference to this object.
         */
        constexpr FixedPoint& operator=(const FixedPoint&) = default;

        /**
         * @brief Negate the value.
         * @return The negated value as an FixedPoint object.
         */
        [[gnu::always_inline]] constexpr FixedPoint operator-() const { return BuildRaw(-value); }

        /**
         * @brief Add another FixedPoint object to this object.
         * @param other The FixedPoint object to add.
         * @return The sum as a new FixedPoint object.
         */
        [[gnu::always_inline]] constexpr FixedPoint operator+(FixedPoint other) const
        {
            if consteval
            {
                int64_t result = static_cast<int64_t>(value) + static_cast<int64_t>(other.value);
                if (result > INT32_MAX) return MaxValue();
                if (result < INT32_MIN) return MinValue();
                return BuildRaw(static_cast<int32_t>(result));
            }
            else
            {
                return BuildRaw(value + other.value);
            }
        }

        /**
         * @brief Add a fixed-point value to an integer (lhs + rhs).
         * @tparam T The type of the integer.
         * @param lhs The integer value to add.
         * @param rhs The fixed-point value to add.
         * @return The sum as a new FixedPoint object.
         */
        template<typename T> requires std::is_integral_v<T>
        [[gnu::always_inline]] constexpr friend FixedPoint operator+(const T& lhs, const FixedPoint& rhs) { return InternalInject(lhs) + rhs; }

        /**
         * @brief Subtract another FixedPoint object from this object.
         * @param other The FixedPoint object to subtract.
         * @return The difference as a new FixedPoint object.
         */
        [[gnu::always_inline]] constexpr FixedPoint operator-(FixedPoint other) const
        {
            if consteval
            {
                int64_t result = static_cast<int64_t>(value) - static_cast<int64_t>(other.value);
                if (result > INT32_MAX) return MaxValue();
                if (result < INT32_MIN) return MinValue();
                return BuildRaw(static_cast<int32_t>(result));
            }
            else
            {
                return BuildRaw(value - other.value);
            }
        }

        /**
         * @brief Subtract a fixed-point value from an integer (lhs - rhs).
         * @tparam T The type of the integer.
         * @param lhs The integer value.
         * @param rhs The fixed-point value to subtract.
         * @return The difference as a new FixedPoint object.
         */
        template<typename T> requires std::is_integral_v<T>
        [[gnu::always_inline]] constexpr friend FixedPoint operator-(T lhs, FixedPoint rhs) { return InternalInject(lhs) - rhs; }

        /**
         * @brief Right shift operator for logical right shift.
         * @param shiftAmount The number of bits to shift.
         * @return The result of the logical right shift as an FixedPoint object.
         */
        [[gnu::always_inline]] constexpr FixedPoint operator>>(const size_t& shiftAmount) const
        {
            if consteval {
                return BuildRaw(value >> shiftAmount);
            } else {
                int32_t raw = value;
                Hardware::ArithmeticShiftRight(raw, shiftAmount);
                return BuildRaw(raw);
            }
        }

        /**
         * @brief Right shift and assign operator for logical right shift.
         * @param shiftAmount The number of bits to shift.
         * @return A reference to this object after the logical right shift.
         */
        [[gnu::always_inline]] constexpr FixedPoint& operator>>=(const size_t& shiftAmount)
        {
            if consteval {
                value >>= shiftAmount;
            } else {
                Hardware::ArithmeticShiftRight(value, shiftAmount);
            }
            return *this;
        }

        /**
         * @brief Left shift operator for shifting the internal value by a specified number of bits.
         * @param shiftAmount The number of bits to shift the internal value to the left.
         * @return A new FixedPoint object with the internal value left-shifted by the specified amount.
         */
        [[gnu::always_inline]] constexpr FixedPoint operator<<(const size_t& shiftAmount) const { return BuildRaw(value << shiftAmount); }

        /**
         * @brief In-place left shift operator for shifting the internal value by a specified number of bits.
         * @param shiftAmount The number of bits to shift the internal value to the left.
         * @return A reference to this FixedPoint object after left-shifting the internal value in place.
         */
        [[gnu::always_inline]] constexpr FixedPoint& operator<<=(const size_t& shiftAmount) { value <<= shiftAmount; return *this; }
        ///@}

        /** @name Comparison Operators */
        ///@{
        /**
         * @brief Compare two FixedPoint objects for greater than.
         * @param other The FixedPoint object to compare with.
         * @return `true` if this object is greater than the other; otherwise, `false`.
         */
        [[gnu::always_inline]] constexpr bool operator>(FixedPoint other) const { return value > other.value; }

        /**
         * @brief Compare two FixedPoint objects for less than.
         * @param other The FixedPoint object to compare with.
         * @return `true` if this object is less than the other; otherwise, `false`.
         */
        [[gnu::always_inline]] constexpr bool operator<(FixedPoint other) const { return value < other.value; }

        /**
         * @brief Compare two FixedPoint objects for greater than or equal to.
         * @param other The FixedPoint object to compare with.
         * @return `true` if this object is greater than or equal to the other; otherwise, `false`.
         */
        [[gnu::always_inline]] constexpr bool operator>=(FixedPoint other) const { return value >= other.value; }

        /**
         * @brief Compare two FixedPoint objects for less than or equal to.
         * @param other The FixedPoint object to compare with.
         * @return `true` if this object is less than or equal to the other; otherwise, `false`.
         */
        [[gnu::always_inline]] constexpr bool operator<=(FixedPoint other) const { return value <= other.value; }

        /**
         * @brief Compare two FixedPoint objects for equality.
         * @param other The FixedPoint object to compare with.
         * @return `true` if this object is equal to the other; otherwise, `false`.
         */
        [[gnu::always_inline]] constexpr bool operator==(FixedPoint other) const { return value == other.value; }

        /**
         * @brief Compare two FixedPoint objects for inequality.
         * @param other The FixedPoint object to compare with.
         * @return `true` if this object is not equal to the other; otherwise, `false`.
         */
        [[gnu::always_inline]] constexpr bool operator!=(FixedPoint other) const { return value != other.value; }


        /**
         * @brief Compare a compile-time float literal with a FixedPoint object for greater than.
         * @param lhs The compile-time float literal.
         * @param rhs The FixedPoint object.
         * @return `true` if the float literal is greater than the object; otherwise, `false`.
         */
        constexpr friend bool operator>(CompileTimeFloat lhs, const FixedPoint& rhs) { return lhs.fxp.value > rhs.value; }

        /**
         * @brief Compare a compile-time float literal with a FixedPoint object for less than.
         * @param lhs The compile-time float literal.
         * @param rhs The FixedPoint object.
         * @return `true` if the float literal is less than the object; otherwise, `false`.
         */
        constexpr friend bool operator<(CompileTimeFloat lhs, const FixedPoint& rhs) { return lhs.fxp.value < rhs.value; }

        /**
         * @brief Compare a compile-time float literal with a FixedPoint object for greater than or equal to.
         * @param lhs The compile-time float literal.
         * @param rhs The FixedPoint object.
         * @return `true` if the float literal is greater than or equal to the object; otherwise, `false`.
         */
        constexpr friend bool operator>=(CompileTimeFloat lhs, const FixedPoint& rhs) { return lhs.fxp.value >= rhs.value; }

        /**
         * @brief Compare a compile-time float literal with a FixedPoint object for less than or equal to.
         * @param lhs The compile-time float literal.
         * @param rhs The FixedPoint object.
         * @return `true` if the float literal is less than or equal to the object; otherwise, `false`.
         */
        constexpr friend bool operator<=(CompileTimeFloat lhs, const FixedPoint& rhs) { return lhs.fxp.value <= rhs.value; }

        /**
         * @brief Compare a compile-time float literal with a FixedPoint object for equality.
         * @param lhs The compile-time float literal.
         * @param rhs The FixedPoint object.
         * @return `true` if the float literal is equal to the object; otherwise, `false`.
         */
        constexpr friend bool operator==(CompileTimeFloat lhs, const FixedPoint& rhs) { return lhs.fxp.value == rhs.value; }

        /**
         * @brief Compare a compile-time float literal with a FixedPoint object for inequality.
         * @param lhs The compile-time float literal.
         * @param rhs The FixedPoint object.
         * @return `true` if the float literal is not equal to the object; otherwise, `false`.
         */
        constexpr friend bool operator!=(CompileTimeFloat lhs, const FixedPoint& rhs) { return lhs.fxp.value != rhs.value; }

        /**
         * @brief Compare a FixedPoint object with an integer for greater than.
         * @tparam T The type of the integer.
         * @param other The integer to compare with.
         * @return `true` if this object is greater than the integer; otherwise, `false`.
         */
        template<std::integral T> [[gnu::always_inline]] constexpr bool operator>(const T& other) const { return *this > InternalInject(other); }

        /**
         * @brief Compare a FixedPoint object with an integer for less than.
         * @tparam T The type of the integer.
         * @param other The integer to compare with.
         * @return `true` if this object is less than the integer; otherwise, `false`.
         */
        template<std::integral T> [[gnu::always_inline]] constexpr bool operator<(const T& other) const { return *this < InternalInject(other); }

        /**
         * @brief Compare a FixedPoint object with an integer for greater than or equal to.
         * @tparam T The type of the integer.
         * @param other The integer to compare with.
         * @return `true` if this object is greater than or equal to the integer; otherwise, `false`.
         */
        template<std::integral T> [[gnu::always_inline]] constexpr bool operator>=(const T& other) const { return *this >= InternalInject(other); }

        /**
         * @brief Compare a FixedPoint object with an integer for less than or equal to.
         * @tparam T The type of the integer.
         * @param other The integer to compare with.
         * @return `true` if this object is less than or equal to the integer; otherwise, `false`.
         */
        template<std::integral T> [[gnu::always_inline]] constexpr bool operator<=(const T& other) const { return *this <= InternalInject(other); }

        /**
         * @brief Compare a FixedPoint object with an integer for equality.
         * @tparam T The type of the integer.
         * @param other The integer to compare with.
         * @return `true` if this object is equal to the integer; otherwise, `false`.
         */
        template<std::integral T> [[gnu::always_inline]] constexpr bool operator==(const T& other) const { return *this == InternalInject(other); }

        /**
         * @brief Compare a FixedPoint object with an integer for inequality.
         * @tparam T The type of the integer.
         * @param other The integer to compare with.
         * @return `true` if this object is not equal to the integer; otherwise, `false`.
         */
        template<std::integral T> [[gnu::always_inline]] constexpr bool operator!=(const T& other) const { return *this != InternalInject(other); }

        /**
         * @brief Compare an integer with a FixedPoint object for greater than.
         * @tparam T The type of the integer.
         * @param lhs The integer.
         * @param rhs The FixedPoint object.
         * @return `true` if the integer is greater than the object; otherwise, `false`.
         */
        template<std::integral T> [[gnu::always_inline]] constexpr friend bool operator>(T lhs, const FixedPoint& rhs) { return InternalInject(lhs) > rhs; }

        /**
         * @brief Compare an integer with a FixedPoint object for less than.
         * @tparam T The type of the integer.
         * @param lhs The integer.
         * @param rhs The FixedPoint object.
         * @return `true` if the integer is less than the object; otherwise, `false`.
         */
        template<std::integral T> [[gnu::always_inline]] constexpr friend bool operator<(T lhs, const FixedPoint& rhs) { return InternalInject(lhs) < rhs; }

        /**
         * @brief Compare an integer with a FixedPoint object for greater than or equal to.
         * @tparam T The type of the integer.
         * @param lhs The integer.
         * @param rhs The FixedPoint object.
         * @return `true` if the integer is greater than or equal to the object; otherwise, `false`.
         */
        template<std::integral T> [[gnu::always_inline]] constexpr friend bool operator>=(T lhs, const FixedPoint& rhs) { return InternalInject(lhs) >= rhs; }

        /**
         * @brief Compare an integer with a FixedPoint object for less than or equal to.
         * @tparam T The type of the integer.
         * @param lhs The integer.
         * @param rhs The FixedPoint object.
         * @return `true` if the integer is less than or equal to the object; otherwise, `false`.
         */
        template<std::integral T> [[gnu::always_inline]] constexpr friend bool operator<=(T lhs, const FixedPoint& rhs) { return InternalInject(lhs) <= rhs; }

        /**
         * @brief Compare an integer with a FixedPoint object for equality.
         * @tparam T The type of the integer.
         * @param lhs The integer.
         * @param rhs The FixedPoint object.
         * @return `true` if the integer is equal to the object; otherwise, `false`.
         */
        template<std::integral T> [[gnu::always_inline]] constexpr friend bool operator==(T lhs, const FixedPoint& rhs) { return InternalInject(lhs) == rhs; }

        /**
         * @brief Compare an integer with a FixedPoint object for inequality.
         * @tparam T The type of the integer.
         * @param lhs The integer.
         * @param rhs The FixedPoint object.
         * @return `true` if the integer is not equal to the object; otherwise, `false`.
         */
        template<std::integral T> [[gnu::always_inline]] constexpr friend bool operator!=(T lhs, const FixedPoint& rhs) { return InternalInject(lhs) != rhs; }
        ///@}

        /** @name Interpolation & Easing Functions */
        ///@{
        /**
         * @brief Power function for fixed-point numbers.
         *
         * Calculates this value raised to the power of exponent.
         * Uses repeated multiplication for integer exponents.
         *
         * @param exponent The power to raise this value to (as I.F fixed-point)
         * @return The result of this^exponent in I.F format
         *
         * @note Only supports non-negative integer exponents for efficiency.
         * The exponent is converted to integer by shifting right by F bits.
         */
        constexpr FixedPoint Pow(FixedPoint exponent) const
        {
            if (exponent == 0) return One();
            if (exponent == One()) return *this;

            int32_t intExp;
            if consteval {
                intExp = exponent.RawValue() >> F;
            } else {
                intExp = exponent.RawValue();
                Hardware::ArithmeticShiftRight<F>(intExp);
            }
            FixedPoint result = One();
            FixedPoint base = *this;

            while (intExp > 0)
            {
                if (intExp & 1) result = result * base;
                base = base * base;
                intExp >>= 1;
            }
            return result;
        }

        /**
         * @brief Clamps a value between minimum and maximum bounds.
         *
         * Ensures a value stays within specified bounds using the formula:
         * result = min(max(value, min), max)
         *
         * @details Clamping behavior:
         * - If value < min, returns min
         * - If value > max, returns max
         * - Otherwise, returns value unchanged
         *
         * Common uses:
         * - Constraining player movement
         * - Limiting camera angles
         * - Bounding UI element positions
         * - Normalizing input values
         *
         * @param value Value to be clamped
         * @param min Minimum allowed value
         * @param max Maximum allowed value
         * @return Clamped value
         *
         * @note Assumes min <= max
         */
        static constexpr FixedPoint Clamp(FixedPoint value, FixedPoint min, FixedPoint max) { return (value < min) ? min : ((value > max) ? max : value); }
        
        /**
         * @brief Linear interpolation between two fixed-point values.
         *
         * Performs linear interpolation (lerp) between start and end values using the formula:
         * result = start + (end - start) * t
         *
         * @details The interpolation follows this pattern:
         * - When t = 0, returns start
         * - When t = 1, returns end
         * - When t = 0.5, returns the midpoint
         *
         * Common uses:
         * - Camera movement
         * - UI element transitions
         * - Color blending
         * - Position interpolation
         *
         * @param start Starting value of the interpolation (I.F format)
         * @param end Ending value of the interpolation (I.F format)
         * @param t Interpolation factor in range [0,1] (I.F format)
         * @return Interpolated value
         *
         * @note For best performance, ensure t is pre-clamped to [0,1]
         */
        static constexpr FixedPoint Lerp(FixedPoint start, FixedPoint end, FixedPoint t) { return start + (end - start) * t; }
        
        /**
         * @brief Smoothstep interpolation for smooth acceleration and deceleration.
         *
         * Implements Ken Perlin's smoothstep function using the formula:
         * 3t² - 2t³
         *
         * @details The smoothstep produces this behavior:
         * - Smooth acceleration from start
         * - Constant velocity at midpoint
         * - Smooth deceleration to end
         *
         * Common uses:
         * - Camera transitions
         * - Smooth UI animations
         * - Particle system parameters
         * - Fade effects
         *
         * @param start Starting value of the interpolation
         * @param end Ending value of the interpolation
         * @param t Interpolation factor, automatically clamped to [0,1]
         * @return Smoothly interpolated value
         *
         * @note Automatically clamps input t to [0,1] for safety
         */
        static constexpr FixedPoint Smoothstep(FixedPoint start, FixedPoint end, FixedPoint t)
        {
            FixedPoint clampedT = Clamp(t, Zero(), One());
            FixedPoint factor = clampedT * clampedT * (FixedPoint(3) - FixedPoint(2) * clampedT);
            return Lerp(start, end, factor);
        }

        /**
         * @brief Smootherstep interpolation for even smoother acceleration and deceleration.
         *
         * Implements Ken Perlin's smootherstep function using a 5th-degree polynomial:
         * 6t⁵ - 15t⁴ + 10t³
         *
         * @details The smootherstep produces this behavior:
         * - Even smoother acceleration from start than smoothstep
         * - Zero first and second derivatives at t=0 and t=1
         * - More natural-looking motion than smoothstep
         *
         * Common uses:
         * - High-quality camera transitions
         * - Premium UI animations
         * - When smoothstep isn't smooth enough
         * - When continuity in second derivatives is desired
         *
         * @param start Starting value of the interpolation
         * @param end Ending value of the interpolation
         * @param t Interpolation factor, automatically clamped to [0,1]
         * @return Even more smoothly interpolated value
         *
         * @note Automatically clamps input t to [0,1] for safety
         * @note More computationally expensive than smoothstep
         */
        static constexpr FixedPoint Smootherstep(const FixedPoint& start, const FixedPoint& end, const FixedPoint& t)
        {
            FixedPoint x = (t < 0) ? 0 : ((t > 1) ? 1 : t);
            FixedPoint x2 = x * x;
            FixedPoint x3 = x2 * x;
            FixedPoint factor = x3 * (x * (FixedPoint(6) * x - FixedPoint(15)) + FixedPoint(10));
            return Lerp(start, end, factor);
        }

        /**
         * @brief Quadratic ease-in interpolation for accelerating motion.
         *
         * Implements quadratic easing using the formula:
         * t²
         *
         * @param start Starting value (I.F format)
         * @param end Ending value (I.F format)
         * @param t Interpolation factor in range [0,1] (I.F format)
         * @return Eased value in I.F format
         *
         * @details The easing produces this motion:
         * - Starts slow (zero velocity)
         * - Continuously accelerates
         * - Reaches full velocity at end
         *
         * Common uses:
         * - Character movement startup
         * - UI element entrance
         * - Zoom-in effects
         * - Power-up animations
         *
         * @note For symmetric animation, pair with EaseOut
         */
        static constexpr FixedPoint EaseIn(FixedPoint start, FixedPoint end, FixedPoint t) { return Lerp(start, end, t * t); }
        
        /**
         * @brief Quadratic ease-out interpolation for decelerating motion.
         *
         * Implements quadratic easing using the formula:
         * -t * (t - 2)
         *
         * @param start Starting value (I.F format)
         * @param end Ending value (I.F format)
         * @param t Interpolation factor in range [0,1] (I.F format)
         * @return Eased value in I.F format
         *
         * @details The easing produces this motion:
         * - Starts at full velocity
         * - Continuously decelerates
         * - Stops smoothly (zero velocity)
         *
         * Common uses:
         * - Character movement stop
         * - UI element exit
         * - Zoom-out effects
         * - Landing animations
         *
         * @note For symmetric animation, pair with EaseIn
         */
        static constexpr FixedPoint EaseOut(FixedPoint start, FixedPoint end, FixedPoint t) { return Lerp(start, end, -t * (t - FixedPoint(2))); }
        
        /**
         * @brief Cubic ease-in interpolation for stronger acceleration.
         *
         * Implements cubic easing using the formula:
         * t³
         *
         * @param start Starting value (I.F format)
         * @param end Ending value (I.F format)
         * @param t Interpolation factor in range [0,1] (I.F format)
         * @return Eased value in I.F format
         *
         * @details The easing produces this motion:
         * - Very slow start
         * - Rapid acceleration
         * - Maximum velocity at end
         *
         * Common uses:
         * - Dramatic entrances
         * - Power-up effects
         * - Explosion start
         * - Heavy object movement
         *
         * @note More pronounced than quadratic EaseIn
         */
        static constexpr FixedPoint CubicEaseIn(FixedPoint start, FixedPoint end, FixedPoint t) { return Lerp(start, end, t * t * t); }
        
        /**
         * @brief Cubic ease-out interpolation for stronger deceleration.
         *
         * Implements cubic easing using the formula:
         * (t - 1)³ + 1
         *
         * @param start Starting value (I.F format)
         * @param end Ending value (I.F format)
         * @param t Interpolation factor in range [0,1] (I.F format)
         * @return Eased value in I.F format
         *
         * @details The easing produces this motion:
         * - Maximum velocity at start
         * - Rapid deceleration
         * - Very slow end
         *
         * Common uses:
         * - Dramatic exits
         * - Impact effects
         * - Explosion end
         * - Heavy object stopping
         *
         * @note More pronounced than quadratic EaseOut
         */
        static constexpr FixedPoint CubicEaseOut(FixedPoint start, FixedPoint end, FixedPoint t)
        {
            FixedPoint tmp = t - One();
            return Lerp(start, end, tmp * tmp * tmp + One());
        }

        /**
         * @brief Elastic ease-in interpolation for spring-like motion.
         *
         * Implements elastic easing with configurable period and amplitude.
         * Uses quadratic approximation for efficiency.
         *
         * @param start Starting value (I.F format)
         * @param end Ending value (I.F format)
         * @param t Interpolation factor in range [0,1] (I.F format)
         * @return Eased value in I.F format
         *
         * @details The easing produces this motion:
         * - Multiple overshoots
         * - Decreasing amplitude
         * - Final snap to position
         *
         * Common uses:
         * - Menu bounces
         * - Character stretching
         * - Projectile charge-up
         * - Spring animations
         *
         * @note Uses approximated sine to avoid trigonometric tables
         */
        static constexpr FixedPoint ElasticEaseIn(FixedPoint start, FixedPoint end, FixedPoint t)
        {
            if (t <= Zero()) return start;
            if (t >= One()) return end;
            FixedPoint overshoot = t * (FixedPoint(2) - t) * FixedPoint(2);
            return Lerp(start, end, overshoot);
        }

        /**
         * @brief Bounce ease-out interpolation for bouncing ball effect.
         *
         * Implements bouncing using piecewise quadratic functions.
         * Simulates diminishing bounces of an elastic ball.
         *
         * @param start Starting value (I.F format)
         * @param end Ending value (I.F format)
         * @param t Interpolation factor in range [0,1] (I.F format)
         * @return Eased value in I.F format
         *
         * @details The motion consists of four phases:
         * - Initial fall (36% of time)
         * - First bounce (36% of time)
         * - Second bounce (18% of time)
         * - Final small bounces (10% of time)
         *
         * Common uses:
         * - Dropping objects
         * - UI element landings
         * - Character jump landing
         * - Button clicks
         *
         * @note Each bounce is approximately 75% the height of previous
         */
        static constexpr FixedPoint BounceEaseOut(FixedPoint start, FixedPoint end, FixedPoint t)
        {
            FixedPoint factor;
            if (t < FixedPoint(0.36363636)) { factor = FixedPoint(7.5625) * t * t; }
            else if (t < FixedPoint(0.72727272)) { FixedPoint tmp = t - FixedPoint(0.54545454); factor = FixedPoint(7.5625) * tmp * tmp + FixedPoint(0.75); }
            else if (t < FixedPoint(0.90909090)) { FixedPoint tmp = t - FixedPoint(0.81818181); factor = FixedPoint(7.5625) * tmp * tmp + FixedPoint(0.9375); }
            else { FixedPoint tmp = t - FixedPoint(0.95454545); factor = FixedPoint(7.5625) * tmp * tmp + FixedPoint(0.984375); }
            return Lerp(start, end, factor);
        }

        /**
         * @brief Bounce ease-in interpolation for reverse bouncing ball effect.
         *
         * Implements bouncing by reversing BounceEaseOut.
         * Creates a series of bounces that converge to the start.
         *
         * @param start Starting value (I.F format)
         * @param end Ending value (I.F format)
         * @param t Interpolation factor in range [0,1] (I.F format)
         * @return Eased value in I.F format
         *
         * @details The motion consists of four phases in reverse:
         * - Small initial bounces (10% of time)
         * - Medium bounce (18% of time)
         * - Large bounce (36% of time)
         * - Final launch (36% of time)
         *
         * Common uses:
         * - Object launches
         * - UI element takeoffs
         * - Character jump startup
         * - Power-up activation
         *
         * @note Reverses BounceEaseOut for symmetric animations
         */
        static constexpr FixedPoint BounceEaseIn(FixedPoint start, FixedPoint end, FixedPoint t)
        {
            return Lerp(end, start, BounceEaseOut(Zero(), One(), One() - t));
        }

        /**
         * @brief Cubic Bezier curve interpolation.
         *
         * Implements cubic Bezier curve interpolation using the formula:
         * (1-t)³·P0 + 3(1-t)²·t·P1 + 3(1-t)·t²·P2 + t³·P3
         *
         * @details The cubic Bezier curve is defined by four points:
         * - P0: Start point (t=0)
         * - P1: First control point (influences the start of the curve)
         * - P2: Second control point (influences the end of the curve)
         * - P3: End point (t=1)
         *
         * Common uses:
         * - Custom easing functions
         * - Smooth path generation
         * - Animation timing functions
         * - UI element motion paths
         *
         * @param p0 Start point
         * @param p1 First control point
         * @param p2 Second control point
         * @param p3 End point
         * @param t Interpolation factor in range [0,1]
         * @return Interpolated value along the curve
         *
         * @note For t outside [0,1], the curve will be extrapolated
         * @note More computationally expensive than simpler interpolation methods
         */
        static constexpr FixedPoint CubicBezier(FixedPoint p0, FixedPoint p1, FixedPoint p2, FixedPoint p3, FixedPoint t)
        {
            const FixedPoint oneMinusT = One() - t;
            const FixedPoint oneMinusT2 = oneMinusT * oneMinusT;
            const FixedPoint oneMinusT3 = oneMinusT2 * oneMinusT;
            const FixedPoint t2 = t * t;
            const FixedPoint t3 = t2 * t;
            return oneMinusT3 * p0 + FixedPoint(3) * oneMinusT2 * t * p1 + FixedPoint(3) * oneMinusT * t2 * p2 + t3 * p3;
        }

        /**
         * @brief Calculates the reciprocal (1/x) with target format control.
         *
         * Computes the reciprocal of the current fixed-point value, converting
         * the result to the specified output format. Uses hardware division
         * for optimal performance on Saturn hardware.
         *
         * @tparam OI Output integer bits (default: same as current)
         * @tparam OF Output fractional bits (default: same as current)
         * @return FixedPoint<OI, OF> The reciprocal value in the target format
         *
         * @details The operation is performed as:
         * result = (1 << (OF + F)) / value
         *
         * For compile-time evaluation (consteval), uses floating-point
         * arithmetic for maximum precision. At runtime, leverages the
         * Saturn's hardware divider unit for zero-cost division.
         *
         * @warning Callers MUST validate that the input is non-zero before calling.
         *          Returns MaxValue() (saturated +infinity) for input of 0 to avoid
         *          division by zero. This is a sentinel value, not an error indicator.
         *          Do NOT use the returned value to detect zero input.
         * @note The output format can differ from the input format
         */
        template<int OI = I, int OF = F>
            requires (OI + OF == 32) && (OI >= 2) && (OF >= 8)
        [[gnu::always_inline]] constexpr FixedPoint<OI, OF> Reciprocal() const
        {
            if consteval
            {
                double a = value / FractionScaleDouble;
                if (a == 0.0) return FixedPoint<OI, OF>::MaxValue();
                else return FixedPoint<OI, OF>::BuildRaw(static_cast<int32_t>((1.0 / a) * (1 << OF)));
            }
            else
            {
                if (value == 0) return FixedPoint<OI, OF>::MaxValue();
                if constexpr (OF + F >= 32) { Hardware::DivSet(value, 1 << ((OF + F) - 32), 0); }
                else { Hardware::DivSet(value, 0, 1 << (OF + F)); }
                return FixedPoint<OI, OF>::BuildRaw(Hardware::DivGetResult());
            }
        }
        ///@}
    };

    // ========================================================================
    // FIXED POINT CONCEPT
    // ========================================================================

    template<typename T> struct is_fixed_point : std::false_type {};
    template<int I, int F> struct is_fixed_point<FixedPoint<I, F>> : std::true_type {};

    template<typename T>
    concept FixedPointType = is_fixed_point<std::remove_cvref_t<T>>::value;

    // ========================================================================
    // ALIASES
    // ========================================================================

    /**
     * @brief Standard 16.16 fixed-point type (Legacy alias).
     * @details This is the original alias for the 16.16 fixed-point format. 
     * It is completely identical and 100% interoperable with Fxp16_16. It is kept 
     * without deprecation warnings to maintain backwards compatibility with existing 
     * codebase. Provides a balanced range [-32768, 32767.999] and resolution (1/65536).
     */
    using Fxp = FixedPoint<16, 16>;

    /**
     * @brief Standard 16.16 fixed-point type.
     * @details The recommended default format for general-purpose math, physics, 
     * velocities, and game logic. It offers the most optimal balance between range 
     * and precision. Multiplication results alignment costs exactly 1 cycle on SH-2 
     * hardware using the `xtrct` instruction.
     */
    using Fxp16_16 = FixedPoint<16, 16>;

    /**
     * @brief Large-world 24.8 fixed-point type.
     * @details Prioritizes range over precision. Ideal for global world coordinates 
     * and macro object placement where sub-millimeter precision is not required.
     * - Range: [-8388608, 8388607.99]
     * - Resolution: ~0.003906 (1/256)
     * Multiplication is highly optimized on SH-2 hardware due to byte-aligned shifts.
     */
    using Fxp24_8 = FixedPoint<24, 8>;

    /**
     * @brief High-precision 8.24 fixed-point type.
     * @details Prioritizes precision over range. Primarily intended for normalized 
     * values (-1.0 to 1.0), rotation matrices, vector normals, and trigonometric 
     * calculations to prevent accumulated distortion.
     * - Range: [-128, 127.99]
     * - Resolution: ~0.0000000596 (1/16777216)
     * Multiplication is highly optimized on SH-2 hardware due to byte-aligned shifts.
     */
    using Fxp8_24 = FixedPoint<8, 24>;

    // ========================================================================
    // PARALLEL DIVISION API
    // ========================================================================

    /**
     * @brief Proxy for parallel division with overlapping CPU work.
     * @tparam DivT The FixedPoint type of the divisor
     * @tparam Fn The callable type (lambda)
     * @details Created by the free function ParallelDiv(), this proxy bundles a
     *          divisor with a lambda to execute while the hardware DIVU processes
     *          the division. Used with operator/:
     *          result = a / ParallelDiv(b, [&]{ ... });
     *
     * @note The lambda must NOT use the hardware DIVU (operator/, AsyncDivSet, etc.)
     *       as that would corrupt the in-flight division.
     */
    template<typename DivT, typename Fn>
    struct ParallelDivisor
    {
        const DivT& divisor;
        Fn fn;
    };

    /**
     * @brief Creates a parallel division proxy for use with operator/.
     * @tparam I Integer bits of divisor
     * @tparam F Fractional bits of divisor
     * @tparam Fn Callable type (lambda)
     * @param divisor The divisor (passed by reference, must outlive the expression)
     * @param fn Lambda to execute while the hardware divider processes a / divisor
     * @return ParallelDivisor proxy
     *
     * @code
     * Fxp cd;
     * Fxp r = (a / ParallelDiv(b, [&]{ cd = c * d; })) * e;
     * // a/b runs on DIVU hardware, lambda runs on CPU in parallel
     * @endcode
     */
    template<int I, int F, typename Fn>
    [[gnu::always_inline]] inline ParallelDivisor<FixedPoint<I, F>, Fn>
    ParallelDiv(const FixedPoint<I, F>& divisor, Fn fn)
    {
        return ParallelDivisor<FixedPoint<I, F>, Fn>{divisor, fn};
    }

    /**
     * @brief Parallel division operator: divides lhs by the divisor in op while
     *        executing op's lambda in parallel on the CPU.
     * @details Starts the hardware DIVU division (lhs / op.divisor), executes
     *          the lambda (for parallel CPU work), then
     *          collects the hardware result. This overlaps the DIVU latency
     *          with useful CPU computation.
     */
    template<int I, int F, typename Fn>
    [[gnu::always_inline]] inline FixedPoint<I, F>
    operator/(const FixedPoint<I, F>& lhs, ParallelDivisor<FixedPoint<I, F>, Fn>&& op)
    {
        int32_t dividendHigh = lhs.RawValue();
        if constexpr (32 - F > 0)
            Hardware::ArithmeticShiftRight<32 - F>(dividendHigh);
        Hardware::DivSet(op.divisor.RawValue(), dividendHigh,
                         static_cast<int32_t>(static_cast<uint32_t>(lhs.RawValue()) << F));
        op.fn();
        return FixedPoint<I, F>::BuildRaw(Hardware::DivGetResult());
    }
}
