#pragma once

#include <stddef.h>
#include <stdint.h>
#include <concepts>
#include "precision.hpp"

namespace SaturnMath::Types
{
    /**
     * @brief Fixed-point arithmetic optimized for Saturn hardware.
     *
     * Uses a 16.16 fixed-point representation where:
     * - Upper 16 bits: Integer part
     * - Lower 16 bits: Fractional part (1/65536 units)
     *
     * Value range:
     * - Minimum: -32768 (0x80000000)
     * - Maximum: 32767.99998474 (0x7FFFFFFF)
     * - Resolution: ~0000152587 (1/65536)
     *
     * Performance features:
     * - Compile-time evaluation with constexpr/consteval
     * - Hardware-optimized multiplication using dmuls.l
     * - Efficient division through hardware divider unit
     * - Zero runtime floating-point operations
     *
     * Example:
     * @code
     * // Compile-time conversion (preferred)
     * constexpr Fxp a = 5;              // 5    (0x00050000)
     * constexpr Fxp b = 2.5f;           // 2.5  (0x00028000)
     * 
     * // Runtime conversion
     * Fxp c = Fxp::Convert(someInt);    // With range checking
     * Fxp d = Fxp::Convert(someFloat);  // With performance warning
     * 
     * // Arithmetic operations
     * Fxp result = a * b;               // 12.5 (0x000C8000)
     * int16_t i = result.As<int16_t>(); // 12
     * @endcode
     *
     * @note All operations are designed for maximum efficiency on Saturn hardware.
     *       Avoid runtime floating-point conversions in performance-critical code.
     *
     * @note The \#pragma GCC optimize("O2") directive is used to enable optimizations that improve performance,
     *       specifically for fixed-point arithmetic operations. It ensures that the compiler generates efficient
     *       code that takes full advantage of the hardware capabilities, particularly in performance-critical sections.
     *       Without this optimization, the generated code may not perform as expected, leading to slower execution.
     *       This optimization is crucial for achieving optimal performance in the Fxp class, as it allows the compiler
     *       to apply various optimizations, such as dead code elimination, register allocation, and instruction scheduling.
     *       These optimizations can significantly improve the execution speed of the Fxp class, making it suitable for
     *       performance-critical applications.
     */
#pragma GCC optimize("O2")
    class Fxp
    {
    private:
        int32_t value; /**< Raw fixed-point value in 16.16 format */

        /**
         * @brief Private constructor for raw fixed-point values.
         * @param inValue Raw 16.16 fixed-point value
         */
        constexpr Fxp(const int32_t& inValue, const bool& /*unused*/) : value(inValue) {}

        /* Hardware division unit registers */
        static inline constexpr size_t cpuAddress = 0xFFFFF000UL;
        static inline auto& dvsr = *reinterpret_cast<volatile uint32_t*>(cpuAddress + 0x0F00UL);   /**< Divisor register */
        static inline auto& dvdnth = *reinterpret_cast<volatile uint32_t*>(cpuAddress + 0x0F10UL); /**< Dividend high register */
        static inline auto& dvdntl = *reinterpret_cast<volatile uint32_t*>(cpuAddress + 0x0F14UL); /**< Dividend low register */

    public:
        /** @return Minimum possible value (-32768.0) */
        static consteval Fxp MinValue() { return BuildRaw(0x80000000); }

        /** @return Maximum possible value (32767.99998474) */
        static consteval Fxp MaxValue() { return BuildRaw(0x7FFFFFFF); }

        /**
         * @brief Default constructor for Fxp class.
         * Initializes the fixed-point number to zero.
         */
        constexpr Fxp() : value(0) {}

        /**
         * @brief Copy constructor for Fxp class.
         * @param fxp The Fxp object to copy.
         */
        constexpr Fxp(const Fxp& fxp) : value(fxp.value) {}

        /**
         * @brief Constructor for Fxp class from a 16-bit signed integer.
         * @param value The integer value to convert to fixed-point.
         *
         * @details This is the primary runtime constructor for integer values.
         * For runtime conversion from other numeric types, use Convert():
         * @code
         * // Convert with compile-time range validation
         * Fxp a = Fxp::Convert(someValue);     // Warns at compile-time if value exceeds int16_t range
         * @endcode
         *
         * @note Converts to 16.16 fixed-point format (e.g., 10 becomes 10.0).
         *
         * @warning For advanced users who understand the risks of precision loss or overflow,
         * manual casting is allowed. However, use the [Convert](cci:1://file:///c:/Git/SaturnRingLib/modules/SaturnMathPP/impl/fxp.hpp:133:8-164:111) function for safer conversions.
         *
         * @code
         * // Manual casting (advanced users only)
         * Fxp b = static_cast<int32_t>(someValue << 16); // No warnings, but risks overflow
         * @endcode
         */
        constexpr Fxp(const int16_t& value) : value(static_cast<int32_t>(value) << 16) {}

        /**
         * @brief Compile-time constructor for numeric types other than int16_t.
         * @tparam T Numeric type (e.g., float, double, int32_t)
         * @param value The value to convert to fixed-point
         *
         * @details This constructor is only available at compile time. For runtime
         * conversions, use Convert():
         * @code
         * // Compile-time conversion (preferred when possible)
         * constexpr Fxp a = 3.14159;     // Exact conversion at compile-time
         * 
         * // Runtime conversion
         * Fxp b = Fxp::Convert(someFloat);    // Will warn about floating-point performance
         * @endcode
         *
         * @note Only available at compile time (consteval). 
         * @warning Values outside the valid fixed-point range may cause overflow.
         */
        template<typename T> requires (!std::is_same_v<T, int16_t>)
            consteval Fxp(const T& value) : value(value * 65536.0) {}

        /**
         * @brief Convert integral type to fixed-point with compile-time range validation.
         * @tparam T Integral type (e.g., int, int32_t)
         * @param value The value to convert
         * @return Fixed-point value
         * 
         * @details Converts an integral value to 16.16 fixed-point format with compile-time validation.
         * Using int16_t{value} ensures the value fits within the valid range (-32768 to 32767).
         * The compiler will warn if the value is outside this range.
         * 
         * This is a RUNTIME conversion method. For compile-time conversion, prefer using
         * the Fxp constructor directly with constexpr when possible.
         * 
         * Example:
         * @code
         * auto a = Fxp::Convert(5);      // OK: 5 fits in int16_t
         * auto b = Fxp::Convert(50000);  // Warning: value exceeds int16_t range
         * @endcode
         */
        template <std::integral T>
        static Fxp Convert(const T &value) { return BuildRaw(static_cast<int32_t>(int16_t{value}) << 16); }

        /**
         * @brief Convert floating-point to fixed-point with performance warning.
         * @tparam T Floating-point type (float, double)
         * @param value The value to convert
         * @return Fixed-point value
         * 
         * @details Converts a floating-point value to 16.16 fixed-point format.
         * This operation involves floating-point multiplication which is relatively expensive
         * on Saturn hardware. The compiler will emit a warning when this function is used
         * to help identify potential performance bottlenecks.
         * 
         * This is a RUNTIME conversion method. For compile-time conversion, prefer using
         * the Fxp constructor directly with constexpr when possible.
         * 
         * For better performance:
         * - Use integral types when possible
         * - Perform conversions at compile time with constexpr
         * - Cache converted values instead of converting in tight loops
         * 
         * Example:
         * @code
         * // Preferred: Compile-time conversion
         * constexpr Fxp a = 3.14159;  // Conversion done at compile time
         * 
         * // Runtime conversion (will trigger warning)
         * float f = get_value();
         * Fxp b = Fxp::Convert(f);    // Warning: heavy operation
         * @endcode
         * 
         * @note The performance warning can be disabled by defining DISABLE_PERFORMANCE_WARNINGS
         *       before including this header. This is useful for code sections where you have
         *       already considered and accepted the performance implications.
         * 
         * @warning Converting from floating-point is a heavy operation.
         * Avoid in performance-critical code paths.
         */
        template <std::floating_point T>
#ifndef DISABLE_PERFORMANCE_WARNINGS
        [[gnu::warning("Converting from floating-point is a heavy operation - avoid in performance-critical code")]]
#endif
        static Fxp Convert(const T &value){ return BuildRaw(static_cast<int32_t>(value * 65536.0)); }

        /***********Static Functions************/

        /**
         * @brief Returns the larger of two fixed-point values.
         * @param a First value
         * @param b Second value
         * @return max(a, b)
         */
        static constexpr Fxp Max(const Fxp& a, const Fxp& b)
        {
            return (a > b) ? a : b;
        }

        /**
         * @brief Returns the smaller of two fixed-point values.
         * @param a First value
         * @param b Second value
         * @return min(a, b)
         */
        static constexpr Fxp Min(const Fxp& a, const Fxp& b)
        {
            return (a < b) ? a : b;
        }

        /**
         * @brief Creates fixed-point from raw 16.16 value.
         * @param rawValue Raw fixed-point bits
         * @return Fixed-point value
         */
        static constexpr Fxp BuildRaw(const int32_t& rawValue) { return Fxp(rawValue, true); }

        /**
         * @brief Sets up hardware division unit for fixed-point division.
         *
         * Prepares the Saturn's hardware divider unit for fixed-point division:
         * result = (dividend << 16) / divisor
         *
         * @param dividend Numerator
         * @param divisor Denominator
         * @note Division result must be retrieved with AsyncDivGetResult()
         */
        static void AsyncDivSet(const Fxp& dividend, const Fxp& divisor)
        {
            dvsr = static_cast<uint32_t>(divisor.value);
            dvdnth = static_cast<uint32_t>(dividend.value) >> 16;
            dvdntl = static_cast<uint32_t>(dividend.value) << 16;
        }

        /**
         * @brief Retrieves result from hardware division unit.
         * @return Fixed-point result from previous AsyncDivSet()
         * @note Must be called after AsyncDivSet()
         */
        static Fxp AsyncDivGetResult()
        {
            return BuildRaw(static_cast<int32_t>(dvdntl));
        }

        /**
         * @brief Retrieves remainder from hardware division unit.
         * @return Fixed-point remainder from previous AsyncDivSet()
         * @note Must be called after AsyncDivSet()
         */
        static Fxp AsyncDivGetRemainder()
        {
            return BuildRaw(static_cast<int32_t>(dvdnth));
        }

        /**
         * @brief Removes fractional part, keeping only integer portion.
         * @return Fixed-point value with zeroed fractional bits
         */
        constexpr Fxp TruncateFraction() const
        {
            return BuildRaw(static_cast<int32_t>(0xFFFF0000) & value);
        }

        /**
         * @brief Calculate square root with configurable precision
         * @tparam P Precision level for calculation
         * @return Square root with specified precision
         */
        template<Precision P = Precision::Standard>
        constexpr Fxp Sqrt() const
        {
            if constexpr (P == Precision::Standard)
            {
                uint32_t remainder = static_cast<uint32_t>(value);
                uint32_t root = 0;
                uint32_t bit = 0x40000000;

                while (bit > 0x40)
                {
                    uint32_t trial = root + bit;
                    if (remainder >= trial)
                    {
                        remainder -= trial;
                        root = trial + bit;
                    }
                    remainder <<= 1;
                    bit >>= 1;
                }

                root >>= 8;
                return BuildRaw(root);
            }
            else // Precision::Fast or Precision::Turbo
            {
                int32_t baseEstimation = 0;
                int32_t estimation = value;

                if (estimation > 0)
                {
                    if (estimation < 65536)
                    {
                        baseEstimation = 1 << 7;
                        estimation <<= 7;

                        uint32_t iterationValue = value >> 1;
                        while (iterationValue)
                        {
                            estimation >>= 1;
                            baseEstimation <<= 1;
                            iterationValue >>= 2;
                        }
                    }
                    else
                    {
                        baseEstimation = (1 << 14);

                        while (baseEstimation < estimation)
                        {
                            estimation >>= 1;
                            baseEstimation <<= 1;
                        }
                    }
                }

                return baseEstimation + estimation;
            }
        }

        /**
         * @brief Squares the value (x²).
         * @return x * x in fixed-point
         * @note Can be evaluated at compile time
         */
        constexpr Fxp Square() const { return *this * *this; }

        /**
         * @brief Absolute value (|x|).
         * @return |x| in fixed-point
         * @note Can be evaluated at compile time
         */
        constexpr Fxp Abs() const { return BuildRaw(value > 0 ? value : -value); }

        /**
         * @brief Returns a const reference to the internal raw fixed-point value.
         * @return const reference to the internal 32-bit representation
         * @note Can be evaluated at compile time
         */
        constexpr const int32_t& RawValue() const { return value; }

        /**
         * @brief Converts to the specified integer type.
         * @tparam T The target integer type
         * @return Value as the specified type
         * 
         * Example:
         * @code
         * Fxp x = 3.14_fxp;
         * auto i = x.As<int16_t>();  // Convert to int
         * @endcode
         */
        template<typename T> requires std::integral<T>
        constexpr T As() const {
            return static_cast<T>(value >> 16);
        }

        /**
         * @brief Converts to the specified floating-point type.
         * @tparam T The target floating-point type (float or double)
         * @return Value as the specified type
         * 
         * Example:
         * @code
         * Fxp x = 3.14_fxp;
         * auto f = x.As<float>();    // Convert to float (heavy operation)
         * auto d = x.As<double>();   // Convert to double (heavy operation)
         * @endcode
         * 
         * @note The performance warning can be disabled by defining DISABLE_PERFORMANCE_WARNINGS
         *       before including this header. This is useful for code sections where you have
         *       already considered and accepted the performance implications.
         * 
         * @warning Converting to floating-point is a heavy operation.
         * Avoid in performance-critical code paths.
         */
        template<typename T> requires std::floating_point<T>
#ifndef DISABLE_PERFORMANCE_WARNINGS
        [[gnu::warning("Converting to floating-point is a heavy operation - avoid in performance-critical code")]]
#endif
        constexpr T As() const {
            return value / T{65536.0};
        }

        /**
         * @brief Clears the MAC (Multiply-and-Accumulate) registers.
         *
         * This static function generates inline assembly to clear both MACH and MACL registers
         * before starting a new MAC operation sequence.
         */
        static void ClearMac() { __asm__ volatile("\tclrmac\n" ::: "mach", "macl"); }

        /**
         * @brief Extracts the result from MAC registers into a fixed-point value.
         *
         * This static function generates inline assembly to:
         * 1. Store MACH and MACL values into temporary registers
         * 2. Extract the final result using xtrct instruction
         * 3. Return the result as a fixed-point value
         *
         * @return Fixed-point value containing the MAC operation result
         */
        static Fxp ExtractMac()
        {
            int32_t aux0, aux1;
            __asm__ volatile(
                "\tsts mach, %[aux0]\n"
                "\tsts macl, %[aux1]\n"
                "\txtrct %[aux0], %[aux1]\n"
                : [aux0] "=&r"(aux0),
                [aux1] "=&r"(aux1)
                :
                : "memory"
                );
            return BuildRaw(aux1);
        }

        /**
         * @brief Power function for fixed-point numbers.
         *
         * Calculates this value raised to the power of exponent.
         * Uses repeated multiplication for integer exponents.
         *
         * @param exponent The power to raise this value to
         * @return The result of this^exponent
         *
         * @note Only supports non-negative integer exponents for efficiency
         */
        constexpr Fxp Pow(const Fxp& exponent) const
        {
            // Handle special cases
            if (exponent == 0) return 1;
            if (exponent == 1) return *this;

            // Convert to integer for efficient calculation
            int32_t intExp = exponent.RawValue() >> 16;
            Fxp result(1);
            Fxp base = *this;

            while (intExp > 0)
            {
                if (intExp & 1)
                    result = result * base;
                base = base * base;
                intExp >>= 1;
            }

            return result;
        }

        /**
         * @brief Clamps this value between minimum and maximum bounds.
         *
         * @param min Minimum allowed value
         * @param max Maximum allowed value
         * @return Clamped value
         */
        constexpr Fxp Clamp(const Fxp& min, const Fxp& max) const
        {
            return (*this < min) ? min : ((*this > max) ? max : *this);
        }

        /**************Operators****************/

        /**
         * @brief Fixed-point addition (a += b).
         * @param fxp Value to add
         * @return Reference to this
         */
        constexpr Fxp& operator+=(const Fxp& fxp) { value += fxp.value; return *this; }

        /**
         * @brief Fixed-point subtraction (a -= b).
         * @param fxp Value to subtract
         * @return Reference to this
         */
        constexpr Fxp& operator-=(const Fxp& fxp) { value -= fxp.value; return *this; }

        /**
         * @brief Multiplies the current fixed-point value by another fixed-point value (a *= b).
         *
         * This operator performs a 64-bit multiplication of the current fixed-point value and the provided
         * fixed-point value, followed by a right shift of 16 bits. This operation effectively scales the
         * result to fit within the 16.16 fixed-point format, where the first 16 bits represent the integer
         * part and the last 16 bits represent the fractional part.
         *
         * @param fxp The fixed-point value to multiply with.
         * @return A reference to the current instance after performing the multiplication, allowing for
         *         chaining of operations.
         *
         * @note This operation modifies the current instance in place. Ensure that the input values are
         *       within the valid range to avoid overflow.
         */
        constexpr Fxp& operator*=(const Fxp& fxp)
        {
            value = (static_cast<uint64_t>(value) * static_cast<uint64_t>(fxp.value)) >> 16;
            return *this;
        }

        /**
         * @brief Fixed-point multiplication (a * b).
         * @param fxp Value to multiply by
         * @return Product as Fxp
         */
        constexpr Fxp operator*(const Fxp& fxp) const
        {
            return Fxp(*this) *= fxp;
        }

        /**
         * @brief Multiplies the current fixed-point value by an integer (a *= b).
         * @tparam T The type of the integer (e.g., int, int32_t).
         * @param value The integer value to multiply with.
         * @return A reference to this object after performing the multiplication,
         *         allowing for chaining of operations.
         *
         * @note This operation modifies the current instance in place. Ensure that
         *       the input value is within the valid range to avoid overflow.
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr Fxp& operator*=(const T& value)
        {
            value *= value;
            return *this;
        }

        /**
         * @brief Multiplies the current fixed-point value by an integer (a * b).
         * @tparam T The type of the integer (e.g., int, int32_t).
         * @param value The integer value to multiply with.
         * @return The product as a new Fxp object.
         *
         * @note This operation does not modify the current instance. It returns a new
         *       Fxp object representing the result of the multiplication.
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr Fxp operator*(const T& value) const
        {
            return BuildRaw(value * this->value);
        }

        /**
         * @brief Multiplies an integer by a fixed-point value (lhs * rhs).
         * @tparam T The type of the integer (e.g., int, int32_t).
         * @param lhs The integer value to multiply.
         * @param rhs The fixed-point value to multiply with.
         * @return The product as a new Fxp object.
         *
         * @note This operation does not modify the current instance. It returns a new
         *       Fxp object representing the result of the multiplication.
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr friend Fxp operator*(T lhs, const Fxp& rhs)
        {
            return rhs * lhs;
        }

        /**
         * @brief Divides the current fixed-point value by another fixed-point value (a /= b).
         *
         * Uses Saturn's hardware divider unit at runtime for optimal performance.
         * Falls back to double math at compile time.
         *
         * @param fxp Value to divide by
         * @return Reference to this
         */
        constexpr Fxp& operator/=(const Fxp& fxp)
        {
            if consteval
            {
                double a = value / 65536.0;
                double b = fxp.value / 65536.0;
                value = (a / b) * 65536.0;
            }
            else
            {
                AsyncDivSet(*this, fxp);
                value = static_cast<int32_t>(dvdntl);
            }
            return *this;
        }

        /**
         * @brief Divides the current fixed-point value by another fixed-point value (a / b).
         * @param fxp Value to divide by
         * @return Quotient as Fxp
         */
        constexpr Fxp operator/(const Fxp& fxp) const
        {
            return Fxp(*this) /= fxp;
        }

        /**
         * @brief Divides the current fixed-point value by an integer (a /= b).
         * @tparam T The type of the integer (e.g., int, int32_t).
         * @param value The integer value to divide by.
         * @return A reference to this object after performing the division,
         *         allowing for chaining of operations.
         *
         * @note This operation modifies the current instance in place. Ensure that
         *       the input value is non-zero to avoid division by zero errors.
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr Fxp& operator/=(const T& value)
        {
            value /= value;
            return *this;
        }

        /**
         * @brief Divides the current fixed-point value by an integer (a / b).
         * @tparam T The type of the integer (e.g., int, int32_t).
         * @param value The integer value to divide by.
         * @return The quotient as a new Fxp object.
         *
         * @note This operation does not modify the current instance. It returns a new
         *       Fxp object representing the result of the division.
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr Fxp operator/(const T& value) const
        {
            return BuildRaw(this->value / value);
        }

        /**
         * @brief Divides an integer by a fixed-point value (lhs / rhs).
         * @param lhs The integer value to divide.
         * @param rhs The fixed-point value to divide by.
         * @return The quotient as a new Fxp object.
         *
         * @note This operation does not modify the current instance. It returns a new
         *       Fxp object representing the result of the division.
         */
        constexpr friend Fxp operator/(const int16_t& lhs, const Fxp& rhs)
        {
            return Fxp(lhs) / rhs;
        }
        /**
         * @brief Computes the modulo of the current fixed-point value with another fixed-point value (a % b).
         * @param fxp The fixed-point value to use as the modulus.
         * @return The result of the modulo operation as a new Fxp object.
         *
         * @note This operation does not modify the current instance. It returns a new
         *       Fxp object representing the result of the modulo operation.
         */
        constexpr Fxp operator%(const Fxp& fxp) const
        {
            if consteval
            {
                double a = value / 65536.0;
                double b = fxp.value / 65536.0;
                double div = a / b;
                double floorDiv = div >= 0 ? static_cast<int64_t>(div) : static_cast<int64_t>(div - 1);
                return BuildRaw(static_cast<int32_t>((a - (b * floorDiv)) * 65536.0));
            }
            else
            {
                return Fxp(*this) %= fxp;
            }
        }

        /**
         * @brief Computes the modulo of the current fixed-point value with an integer (a % b).
         * @tparam T The type of the integer (e.g., int, int32_t).
         * @param value The integer value to use as the modulus.
         * @return The result of the modulo operation as a new Fxp object.
         *
         * @note This operation does not modify the current instance. It returns a new
         *       Fxp object representing the result of the modulo operation.
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr Fxp operator%(const T& value) const
        {
            if consteval
            {
                double a = this->value / 65536.0;
                double b = static_cast<double>(value);
                double div = a / b;
                double floorDiv = div >= 0 ? static_cast<int64_t>(div) : static_cast<int64_t>(div - 1);
                return BuildRaw(static_cast<int32_t>((a - (b * floorDiv)) * 65536.0));
            }
            else
            {
                return Fxp(*this) %= value;
            }
        }

        /**
         * @brief Computes the modulo of an integer with a fixed-point value (lhs % rhs).
         * @tparam T The type of the integer (e.g., int, int32_t).
         * @param lhs The integer value.
         * @param rhs The fixed-point value to use as the modulus.
         * @return The result of the modulo operation as a new Fxp object.
         *
         * @note This operation does not modify the current instance. It returns a new
         *       Fxp object representing the result of the modulo operation.
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr friend Fxp operator%(const T& lhs, const Fxp& rhs)
        {
            if consteval
            {
                double a = static_cast<double>(lhs);
                double b = rhs.value / 65536.0;
                double div = a / b;
                double floorDiv = div >= 0 ? static_cast<int64_t>(div) : static_cast<int64_t>(div - 1);
                return BuildRaw(static_cast<int32_t>((a - (b * floorDiv)) * 65536.0));
            }
            else
            {
                return Fxp(lhs) %= rhs;
            }
        }

        /**
         * @brief Computes the modulo of the current fixed-point value with another fixed-point value (a %= b).
         * @param fxp The fixed-point value to use as the modulus.
         * @return A reference to this object after performing the modulo operation.
         *
         * @note This operation modifies the current instance in place.
         */
        constexpr Fxp& operator%=(const Fxp& fxp)
        {
            if consteval
            {
                double a = value / 65536.0;
                double b = fxp.value / 65536.0;
                double div = a / b;
                double floorDiv = div >= 0 ? static_cast<int64_t>(div) : static_cast<int64_t>(div - 1);
                value = static_cast<int32_t>((a - (b * floorDiv)) * 65536.0);
            }
            else
            {
                AsyncDivSet(*this, fxp);
                value = static_cast<int32_t>(dvdnth);
            }
            return *this;
        }

        /**
         * @brief Computes the modulo of the current fixed-point value with an integer (a %= b).
         * @tparam T The type of the integer (e.g., int, int32_t).
         * @param value The integer value to use as the modulus.
         * @return A reference to this object after performing the modulo operation.
         *
         * @note This operation modifies the current instance in place.
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr Fxp& operator%=(const T& value)
        {
            this->value %= (value << 16);
            return *this;
        }

        /**
         * @brief Copy assignment operator.
         * @return A reference to this object.
         */
        constexpr Fxp& operator=(const Fxp&) = default;

        /**
         * @brief Negate the value.
         * @return The negated value as an Fxp object.
         */
        constexpr Fxp operator-() const { return BuildRaw(-value); }

        /**
         * @brief Add another Fxp object to this object.
         * @param fxp The Fxp object to add.
         * @return The sum as an Fxp object.
         */
        constexpr Fxp operator+(const Fxp& fxp) const { return BuildRaw(value + fxp.value); }

        /**
         * @brief Subtract another Fxp object from this object.
         * @param fxp The Fxp object to subtract.
         * @return The difference as an Fxp object.
         */
        constexpr Fxp operator-(const Fxp& fxp) const { return BuildRaw(value - fxp.value); }

        /**
         * @brief Compare two Fxp objects for greater than.
         * @param fxp The Fxp object to compare with.
         * @return `true` if this object is greater than the other; otherwise, `false`.
         */
        constexpr bool operator>(const Fxp& fxp) const { return value > fxp.value; }

        /**
         * @brief Compare two Fxp objects for less than.
         * @param fxp The Fxp object to compare with.
         * @return `true` if this object is less than the other; otherwise, `false`.
         */
        constexpr bool operator<(const Fxp& fxp) const { return value < fxp.value; }

        /**
         * @brief Compare two Fxp objects for greater than or equal to.
         * @param fxp The Fxp object to compare with.
         * @return `true` if this object is greater than or equal to the other; otherwise, `false`.
         */
        constexpr bool operator>=(const Fxp& fxp) const { return value >= fxp.value; }

        /**
         * @brief Compare two Fxp objects for less than or equal to.
         * @param fxp The Fxp object to compare with.
         * @return `true` if this object is less than or equal to the other; otherwise, `false`.
         */
        constexpr bool operator<=(const Fxp& fxp) const { return value <= fxp.value; }

        /**
         * @brief Compare two Fxp objects for equality.
         * @param fxp The Fxp object to compare with.
         * @return `true` if this object is equal to the other; otherwise, `false`.
         */
        constexpr bool operator==(const Fxp& fxp) const { return value == fxp.value; }

        /**
         * @brief Compare two Fxp objects for inequality.
         * @param fxp The Fxp object to compare with.
         * @return `true` if this object is not equal to the other; otherwise, `false`.
         */
        constexpr bool operator!=(const Fxp& fxp) const { return value != fxp.value; }

        /**
         * @brief Right shift operator for logical right shift.
         * @param shiftAmount The number of bits to shift.
         * @return The result of the logical right shift as an Fxp object.
         */
        constexpr Fxp operator>>(const size_t& shiftAmount) const { return BuildRaw(value >> shiftAmount); }

        /**
         * @brief Right shift and assign operator for logical right shift.
         * @param shiftAmount The number of bits to shift.
         * @return A reference to this object after the logical right shift.
         */
        constexpr Fxp& operator>>=(const size_t& shiftAmount) { value >>= shiftAmount; return *this; }

        /**
         * @brief Left shift operator for shifting the internal value by a specified number of bits.
         * @param shiftAmount The number of bits to shift the internal value to the left.
         * @return A new Fxp object with the internal value left-shifted by the specified amount.
         */
        constexpr Fxp operator<<(const size_t& shiftAmount) const { return BuildRaw(value << shiftAmount); }

        /**
         * @brief In-place left shift operator for shifting the internal value by a specified number of bits.
         * @param shiftAmount The number of bits to shift the internal value to the left.
         * @return A reference to this Fxp object after left-shifting the internal value in place.
         */
        constexpr Fxp& operator<<=(const size_t& shiftAmount) { value <<= shiftAmount; return *this; }
    };
}
#pragma GCC reset_options