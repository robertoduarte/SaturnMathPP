#pragma once

#include <stddef.h>
#include <stdint.h>

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
     * - Minimum: -32768.0 (0x80000000)
     * - Maximum: 32767.99998474 (0x7FFFFFFF)
     * - Resolution: ~0.0000152587 (1/65536)
     * 
     * Performance features:
     * - Compile-time evaluation with constexpr/consteval
     * - Hardware-optimized multiplication using dmuls.l
     * - Efficient division through hardware divider unit
     * - Zero runtime floating-point operations
     * 
     * Example:
     * @code
     * Fxp a = Fxp::FromInt(5);     // 5.0 (0x00050000)
     * Fxp b(2.5);                  // 2.5 (0x00028000)
     * Fxp c = a * b;               // 12.5 (0x000C8000)
     * int16_t i = c.ToInt();       // 12
     * @endcode
     * 
     * @note All operations are designed for maximum efficiency on Saturn hardware.
     *       Avoid runtime floating-point conversions in performance-critical code.
     */
    class Fxp
    {
    private:
        int32_t value; /**< Raw fixed-point value in 16.16 format */

        /* Hardware division unit registers */
        static inline constexpr size_t cpuAddress = 0xFFFFF000UL;
        static inline auto& dvsr = *reinterpret_cast<volatile uint32_t*>(cpuAddress + 0x0F00UL);   /**< Divisor register */
        static inline auto& dvdnth = *reinterpret_cast<volatile uint32_t*>(cpuAddress + 0x0F10UL); /**< Dividend high register */
        static inline auto& dvdntl = *reinterpret_cast<volatile uint32_t*>(cpuAddress + 0x0F14UL); /**< Dividend low register */

        friend class Vector2D;
        friend class Vector3D;
        friend class Trigonometry;

        /**
         * @brief Private constructor for raw fixed-point values.
         * @param inValue Raw 16.16 fixed-point value
         */
        constexpr Fxp(const int32_t& inValue) : value(inValue) {}

    public:
        /** @brief Default constructor. Initializes to 0.0 */
        constexpr Fxp() : value(0) {}

        /** @brief Copy constructor */
        constexpr Fxp(const Fxp& fxp) : value(fxp.value) {}

        /**
         * @brief Creates fixed-point from double at compile time.
         * @param f Double value to convert
         * @warning Only available at compile time to avoid runtime floating-point
         */
        consteval Fxp(const double& f) : value(f * 65536.0) {}

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
         * @brief Creates fixed-point from integer.
         * @param integerValue Integer value to convert
         * @return Fixed-point value (integerValue.0)
         */
        static constexpr Fxp FromInt(const int16_t& integerValue) { return Fxp(static_cast<int32_t>(integerValue) << 16); }

        /**
         * @brief Creates fixed-point from raw 16.16 value.
         * @param rawValue Raw fixed-point bits
         * @return Fixed-point value
         */
        static constexpr Fxp BuildRaw(const int32_t& rawValue) { return Fxp(rawValue); }

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
            uint32_t dividendh;
            __asm__ volatile("swap.w %[in], %[out]\n"
                : [out] "=&r"(dividendh)
                : [in] "r"(dividend.value));
            __asm__ volatile("exts.w %[in], %[out]"
                : [out] "=&r"(dividendh)
                : [in] "r"(dividendh));

            dvdnth = dividendh;
            dvsr = divisor.value;
            dvdntl = dividend.value << 16;
        }

        /**
         * @brief Retrieves result from hardware division unit.
         * @return Fixed-point quotient from previous AsyncDivSet()
         * @note Must be called after AsyncDivSet()
         */
        static Fxp AsyncDivGet() { return static_cast<int32_t>(dvdntl); }

        /**
         * @brief Removes fractional part, keeping only integer portion.
         * @return Fixed-point value with zeroed fractional bits
         */
        constexpr Fxp TruncateFraction() const
        {
            return (int32_t)0xFFFF0000 & value;
        }

        /**
         * @brief Precise square root calculation.
         * 
         * Uses bit-by-bit calculation for maximum precision:
         * - 31 iterations for full range
         * - No approximation errors (beyond fixed-point limits)
         * - Handles full 16.16 range correctly
         * 
         * @return √x in fixed-point format
         * @note Slower but more accurate than FastSqrt()
         */
        constexpr Fxp Sqrt() const
        {
            uint32_t remainder = static_cast<uint32_t>(value);
            uint32_t root = 0;
            uint32_t bit = 0x40000000; // Set the initial bit to the highest bit for 32-bit integers

            while (bit > 0x40)
            {
                // Try the current bit in the root calculation
                uint32_t trial = root + bit;

                if (remainder >= trial)
                {
                    remainder -= trial;
                    root = trial + bit;
                }

                // Shift the remainder and decrease the bit for the next iteration
                remainder <<= 1;
                bit >>= 1;
            }

            root >>= 8; // Adjust the result

            return static_cast<int32_t>(root);
        }

        /**
         * @brief Fast approximate square root.
         * 
         * Uses binary search approximation:
         * - ~6% maximum error
         * - Special handling for x < 1.0
         * - Much faster than precise Sqrt()
         * 
         * Best for:
         * - Games (physics, vectors)
         * - Graphics (distance calculations)
         * - Any case where speed > accuracy
         * 
         * @return Approximate √x in fixed-point
         */
        constexpr Fxp FastSqrt() const
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
        constexpr Fxp Abs() const { return value > 0 ? value : -value; }

        /**
         * @brief Gets raw 16.16 fixed-point bits.
         * @return Internal 32-bit representation
         * @note Can be evaluated at compile time
         */
        constexpr const int32_t& Value() const { return value; }

        /**
         * @brief Extracts integer part.
         * @return Integer portion of value
         */
        constexpr int16_t ToInt() { return static_cast<int16_t>(value >> 16); }

        /**
         * @brief Converts to double.
         * @return Double value
         * @note Only available at compile time due to consteval
         */
        consteval double ToFloat() { return value / 65536.0f; }

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
         * @brief Fixed-point multiplication (a *= b).
         * 
         * Uses Saturn's dmuls.l instruction at runtime.
         * For compile-time evaluation, uses double math.
         * 
         * @param fxp Value to multiply by
         * @return Reference to this
         * @note Can be evaluated at compile time
         */
        constexpr Fxp& operator*=(const Fxp& fxp)
        {
            if consteval
            {
                double a = value / 65536.0;
                double b = fxp.value / 65536.0;
                value = (a * b) * 65536.0;;
            }
            else
            {
                uint32_t mach;
                __asm__ volatile(
                    "\tdmuls.l %[a], %[b]\n"
                    "\tsts mach, %[mach]\n"
                    "\tsts macl, %[out]\n"
                    "\txtrct %[mach], %[out]\n"
                    /* Output */
                    : [mach] "=&r" (mach),
                    [out] "=&r" (value)
                    /* Input */
                    : [a] "r" (value),
                    [b] "r" (fxp.value)
                    : "mach", "macl");
            }

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
                value = (a / b) * 65536.0;;
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
         * @param fxp The Fxp object to copy.
         * @return A reference to this object.
         */
        constexpr Fxp& operator=(const Fxp&) = default;

        /**
         * @brief Negate the value.
         * @return The negated value as an Fxp object.
         */
        constexpr Fxp operator-() const { return -value; }

        /**
         * @brief Add another Fxp object to this object.
         * @param fxp The Fxp object to add.
         * @return The sum as an Fxp object.
         */
        constexpr Fxp operator+(const Fxp& fxp) const { return value + fxp.value; }

        /**
         * @brief Subtract another Fxp object from this object.
         * @param fxp The Fxp object to subtract.
         * @return The difference as an Fxp object.
         */
        constexpr Fxp operator-(const Fxp& fxp) const { return value - fxp.value; }

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
        constexpr Fxp operator>>(const size_t& shiftAmount) const { return value >> shiftAmount; }

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
        constexpr Fxp operator<<(const size_t& shiftAmount) const { return value << shiftAmount; }

        /**
         * @brief In-place left shift operator for shifting the internal value by a specified number of bits.
         * @param shiftAmount The number of bits to shift the internal value to the left.
         * @return A reference to this Fxp object after left-shifting the internal value in place.
         */
        constexpr Fxp& operator<<=(const size_t& shiftAmount) { value <<= shiftAmount; return *this; }
    };
}