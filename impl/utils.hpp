#pragma once
#include <stdint.h>
#include <concepts>

namespace SaturnMath
{
    /**
     * @brief Generic utility functions and specialized math operations
     */
    class Utils
    {
    public:
        /**
         * @brief Get absolute value
         * 
         * @details Works with any type that supports comparison operators.
         *
		 * @tparam ValueType Type of the value
		 * @param value Numeric value
		 * @return Absolute value
         */
		template<typename ValueType>
		constexpr static ValueType Abs(const ValueType& value)
		{
			return value < 0 ? -value : value;
		}

		/**
         * @brief Get maximum value of two values
		 * 
         * @details Works with any type that supports comparison operators.
         *
         * @tparam ValueType Type of the value
		 * 
         * @param first First value
		 * @param second Second value
         * @return Maximum value
		 */
		template<typename ValueType>
		constexpr static ValueType Max(const ValueType& first, const ValueType& second)
		{
			return first > second ? first : second;
		}

        /**
		 * @brief Get minimum value of two values
         * 
         * @details Works with any type that supports comparison operators.
         *
		 * @tparam ValueType Type of the value
         * 
		 * @param first First value
		 * @param second Second value
		 * @return Minimum value
         */
		template<typename ValueType>
		constexpr static ValueType Min(const ValueType& first, const ValueType& second)
		{
			return first < second ? first : second;
		}

        /**
         * @brief Clamps a value between min and max bounds.
         *
         * @details Works with any type that supports comparison operators.
         *
         * @tparam T Type of the values
         * @param value Value to clamp
         * @param min Minimum allowed value
         * @param max Maximum allowed value
         * @return Clamped value
         */
        template<typename T>
        static constexpr T Clamp(const T& value, const T& min, const T& max)
        {
            return value < min ? min : (value > max ? max : value);
        }

        /**
         * @brief Integer-specific utility functions optimized for performance
         */
        class Integer
        {
        public:
            /**
             * @brief Fast integer square root approximation with ~6% error.
             *
             * Binary search approximation supporting full uint32_t range.
             * Maximum 15 iterations after initial right shift by 2.
             * Ideal for games where integer precision is sufficient
             * and performance matters more than perfect accuracy.
             *
             * @param src The 32-bit integer value.
             * @return Approximate square root as whole number.
             */
            static constexpr uint32_t FastSqrt(uint32_t src)
            {
                uint32_t baseEstimation = 1;
                uint32_t estimation = src >> 2;

                while (baseEstimation < estimation)
                {
                    estimation >>= 1;
                    baseEstimation <<= 1;
                }

                return baseEstimation + estimation;
            }
        };
    };
}
