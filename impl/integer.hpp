#pragma once
#include <stdint.h>
#include "hardware.hpp"

namespace SaturnMath
{
    /**
     * @brief Integer-specific utility functions optimized for performance
     */
    class Integer final
    {
    public:
        /**
         * @brief Fast square root approximation using binary search
         *
         * Binary search approximation supporting full uint32_t range.
         * Uses fast-path bit-size checks to skip several iterations up
         * front when the input is large, then falls back to the classic
         * @c while(base<est) binary search for the remaining bits.
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

            // Fast-path: skip 8 iterations at once for large inputs.
            if (estimation >= 0x00010000)
            {
                baseEstimation <<= 8;
                estimation >>= 8;
            }

            while (baseEstimation < estimation)
            {
                estimation >>= 1;
                baseEstimation <<= 1;
            }

            return baseEstimation + estimation;
        }

        /**
         * @brief Fast square root approximation for a 64-bit unsigned integer.
         *
         * Same algorithm as FastSqrt(uint32_t) but extended to 64-bit input.
         * Accepts the value split into a high and low 32-bit word, where the
         * full value is @c v = (static_cast<uint64_t>(high) << 32) | low.
         *
         * @param high Upper 32 bits of the source value.
         * @param low  Lower 32 bits of the source value.
         * @return Approximate square root as whole number.
         */
        static constexpr uint32_t FastSqrt(uint32_t hi, uint32_t lo)
        {
            if ((hi | lo) == 0)
                return 0;

            auto shiftRight64 = [](uint32_t& hi, uint32_t& lo)
            {
                if consteval {
                    lo = (lo >> 1) | (hi << 31);
                    hi >>= 1;
                } else {
                    Hardware::ShiftRight64(hi, lo);
                }
            };

            uint32_t baseEstimation = 1;

            // estimation = src >> 2 (same as FastSqrt 32-bit)
            shiftRight64(hi, lo);
            shiftRight64(hi, lo);

            // Fast-path: reduce 64-bit estimation until hi becomes 0.
            // Each shift corresponds to one iteration of the binary search.
            while (hi != 0)
            {
                shiftRight64(hi, lo);
                baseEstimation <<= 1;
            }

            // Now hi == 0, estimation is in lo (32-bit).
            // Fast-path: skip 8 iterations for large remaining values.
            if (lo >= 0x00010000)
            {
                baseEstimation <<= 8;
                lo >>= 8;
            }

            // Binary search phase (same as FastSqrt 32-bit)
            while (baseEstimation < lo)
            {
                lo >>= 1;
                baseEstimation <<= 1;
            }

            return baseEstimation + lo;
        }
    };
}
