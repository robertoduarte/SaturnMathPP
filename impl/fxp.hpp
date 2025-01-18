#pragma once

#include <stddef.h>
#include <stdint.h>
#include <concepts>
#include "precision.hpp"

namespace SaturnMath
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
     * Fxp a = 5;                   // 5    (0x00050000)
     * Fxp b(2.5);                  // 2.5  (0x00028000)
     * Fxp c = a * b;               // 12.5 (0x000C8000)
     * int16_t i = c.ToInt();       // 12
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
         * @details Accepts only int16_t values. Other types MUST be explicitly cast to int16_t.
         * This explicit casting requirement ensures you are aware of potential performance impacts
         * from type conversions
         * Valid range: -32768 to 32767
         *
         * Runtime casting examples:
         * @code
         * // From char (requires upcasting)
         * int16_t charValue = static_cast<int16_t>('A');
         * Fxp a = charValue;
         *
         * // From int (requires downcasting, check value range)
         * int16_t intValue = static_cast<int16_t>(100);
         * Fxp b = intValue;
         *
         * // From int32_t (requires downcasting, check value range)
         * int32_t largeValue = 30000;
         * Fxp c = static_cast<int16_t>(largeValue);
         * @endcode
         *
         * @warning Both upcasting and downcasting operations can impact runtime performance.
         * @note Converts to 16.16 fixed-point format (e.g., 10 becomes 10.0).
         */
        constexpr Fxp(const int16_t& value) : value(static_cast<int32_t>(value) << 16) {}

        /**
         * @brief Compile-time constructor for numeric types other than int16_t.
         * @tparam T Numeric type (e.g., float, double, int32_t)
         * @param value The value to convert to fixed-point
         *
         * @details This constructor converts any numeric type to 16.16 fixed-point format
         * at compile time. The conversion is done by multiplying the input by 65536 (2^16)
         * to properly align the decimal point.
         *
         * Example usage:
         * @code
         * constexpr Fxp a = 3.14159;     // From double
         * constexpr Fxp b = 42.0f;       // From float
         * constexpr Fxp c = 100000L;     // From long
         * @endcode
         *
         * @note Only available at compile time (consteval). For runtime conversions of
         * integer types, use explicit casting to int16_t with the runtime constructor.
         *
         * @warning Values outside the valid fixed-point range may cause overflow.
         */
        template<typename T> requires (!std::is_same_v<T, int16_t>)
            consteval Fxp(const T& value) : value(value * 65536.0) {}

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
         * @note Division result must be retrieved with AsyncDivGet()
         */
        static void AsyncDivSet(const Fxp& dividend, const Fxp& divisor)
        {
            dvsr = static_cast<uint32_t>(divisor.value);
            dvdnth = static_cast<uint32_t>(dividend.value) >> 16;
            dvdntl = static_cast<uint32_t>(dividend.value) << 16;
        }

        /**
         * @brief Retrieves result from hardware division unit.
         * @return Fixed-point quotient from previous AsyncDivSet()
         * @note Must be called after AsyncDivSet()
         */
        static Fxp AsyncDivGet() { return BuildRaw(static_cast<int32_t>(dvdntl)); }

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
                return static_cast<int32_t>(root);
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
         * @brief Extracts integer part.
         * @return Integer portion of value
         */
        constexpr int16_t ToInt() const { return static_cast<int16_t>(value >> 16); }

        /**
         * @brief Converts to float.
         * @return Float value
         * @note Only available at compile time due to consteval
         */
        consteval float ToFloat() const { return value / 65536.0f; }

        /**
         * @brief Converts to double.
         * @return Double value
         * @note Only available at compile time due to consteval
         */
        consteval double ToDouble() const { return value / 65536.0; }

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
         * @brief Fixed-point division (a /= b).
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
         * @brief Fixed-point division (a / b).
         * @param fxp Value to divide by
         * @return Quotient as Fxp
         */
        constexpr Fxp operator/(const Fxp& fxp) const
        {
            return Fxp(*this) /= fxp;
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
