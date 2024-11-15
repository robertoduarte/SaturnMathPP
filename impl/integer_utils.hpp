#pragma once

#include <stdint.h>

namespace SaturnMath
{
    /**
     * @brief Utility class containing integer-specific mathematical functions.
     */
    class IntegerUtils
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
}
