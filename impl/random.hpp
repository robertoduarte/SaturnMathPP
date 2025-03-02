#pragma once
#include <stdint.h>
#include <type_traits>
#include <cmath>

namespace SaturnMath
{
    /** @brief Pseudo-Random number generator
     * @note This generator uses Xorshift (see https://en.wikipedia.org/wiki/Xorshift)
     * @tparam T Integral type for random number generation
     */
    template<typename T>
    class Random
    {
    private:
        static_assert(std::is_integral_v<T>, "T must be an integral type");

        /** @brief Pseudo-Random number generator seed
         */
        T startState;

        /** @brief Get next pseudo-random number
         * @return Generated number
         */
        T GetNextPeriod()
        {
            // Adjust shift values based on the bit width of T
            constexpr unsigned int bits = sizeof(T) * 8;
            
            // Recommended shift values for different bit widths
            // These are based on the Xorshift research paper recommendations
            unsigned int shift1, shift2, shift3;
            
            if constexpr (bits <= 16) {
                // For 8-bit and 16-bit types
                shift1 = 7;
                shift2 = 9;
                shift3 = 8;
            } else if constexpr (bits <= 32) {
                // For 32-bit types (original values)
                shift1 = 13;
                shift2 = 17;
                shift3 = 5;
            } else if constexpr (bits <= 64) {
                // For 64-bit types
                shift1 = 13;
                shift2 = 7;
                shift3 = 17;
            } else {
                // Fallback for larger types (though not recommended)
                shift1 = bits / 5;
                shift2 = bits / 3;
                shift3 = bits / 10;
            }
            
            this->startState ^= this->startState << shift1;
            this->startState ^= this->startState >> shift2;
            this->startState ^= this->startState << shift3;
            return this->startState;
        }

    public:
    
        /** @brief Construct a new pseudo-random number generator
         * @param seed Starting seed
         */
        Random(T seed)
        {
            this->startState = seed;
        }

        /** @brief Destroy the pseudo-random number generator object
         */
        ~Random() { }

        /** @brief Get next pseudo-random number
         * @return Generated number in a full range
         */
        T GetNumber()
        {
            return this->GetNextPeriod();
        }
        
        /** @brief Get next pseudo-random number
         * @param from Inclusive start of the range
         * @param to Inclusive end of the range
         * @return Generated number in a range
         */
        T GetNumber(T from, T to)
        {
            // Handle the case where from > to by swapping them
            if (from > to) {
                T temp = from;
                from = to;
                to = temp;
            }
            
            // Get a raw random number
            T number = this->GetNumber();
            
            // Calculate range size (inclusive)
            T range = to - from + 1;
            
            // For signed types, ensure positive modulo result
            if constexpr (std::is_signed_v<T>) {
                // Make number positive for modulo operation
                // This avoids implementation-defined behavior with negative numbers
                number = number < 0 ? -number : number;
            }
            
            // Apply modulo and add offset
            return from + (number % range);
        }
    };
}