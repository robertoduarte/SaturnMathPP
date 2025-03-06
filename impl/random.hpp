#pragma once
#include <stdint.h>
#include <type_traits>
#include <cmath>

namespace SaturnMath
{
    /** 
     * @brief High-performance pseudo-random number generator optimized for Saturn hardware.
     * 
     * @details The Random class implements a fast, lightweight Xorshift pseudo-random
     * number generator (PRNG) that provides high-quality random numbers with excellent
     * statistical properties while maintaining minimal computational overhead. This
     * implementation is specifically optimized for Saturn hardware and game development
     * scenarios where performance is critical.
     * 
     * Key features:
     * - Fast generation with minimal state (single variable)
     * - Long period relative to state size
     * - Good statistical distribution properties
     * - Deterministic sequence for reproducible results
     * - Configurable for different integral types
     * - No floating-point operations in core algorithm
     * 
     * Implementation details:
     * - Uses Xorshift algorithm (https://en.wikipedia.org/wiki/Xorshift)
     * - Shift parameters are optimized based on bit width of the template type
     * - Automatically adjusts algorithm for different integral types
     * - Period length depends on the bit width of type T
     * 
     * Statistical properties:
     * - Passes most common randomness tests (Die-Hard, TestU01)
     * - Even distribution across the entire range of type T
     * - Low correlation between consecutive outputs
     * - Suitable for most game development and simulation needs
     * 
     * Common applications:
     * - Procedural content generation
     * - AI decision making
     * - Particle effects
     * - Gameplay mechanics (damage ranges, critical hits)
     * - Simulation of natural phenomena
     * 
     * Performance considerations:
     * - Extremely fast (few CPU operations per number)
     * - Small memory footprint (single state variable)
     * - No branching in core algorithm
     * - No division operations
     * 
     * @note While this PRNG is suitable for games and simulations, it is NOT
     * cryptographically secure and should not be used for security-related
     * applications or where unpredictability is critical.
     * 
     * @tparam T Integral type for random number generation (determines range and period)
     * 
     * @see Fxp For fixed-point representation of random values
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