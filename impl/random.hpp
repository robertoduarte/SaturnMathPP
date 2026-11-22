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
     * - The state is unsigned (logical shifts, as xorshift requires) and at least 16 bits: 8-bit and
     *   16-bit T share a 16-bit xorshift (7, 9, 8); numbers are the state cast to T
     * - Period length depends on the bit width of type T (2^16 - 1 for 8-bit and 16-bit T)
     * - A seed of 0 is replaced by a non-zero constant (xorshift never leaves state 0)
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

        /** @brief Unsigned type of T, used for ranges and offsets (no signed overflow)
         */
        using Unsigned = std::make_unsigned_t<T>;

        /** @brief Bit width of T
         */
        static constexpr unsigned int Bits = sizeof(T) * 8;

        /** @brief State type: unsigned (logical shifts) and at least 16 bits, since xorshift has no 8-bit form
         */
        using State = std::conditional_t<(Bits <= 16), uint16_t,
            std::conditional_t<(Bits <= 32), uint32_t, std::conditional_t<(Bits <= 64), uint64_t, Unsigned>>>;

        /** @brief Non-zero state used for a seed of 0, which xorshift never leaves
         */
        static constexpr State ZeroSeedState = static_cast<State>(0x9E3779B9u);

        /** @brief Pseudo-Random number generator state
         */
        State startState;

        /** @brief Get next pseudo-random number
         * @return Next state
         */
        State GetNextPeriod()
        {
            // Adjust shift values based on the bit width of T
            constexpr unsigned int bits = Bits;
            
            // Recommended shift values for different bit widths
            // These are based on the Xorshift research paper recommendations
            unsigned int shift1, shift2, shift3;
            
            if constexpr (bits <= 16) {
                // For 8-bit and 16-bit types (16-bit state)
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
            
            this->startState ^= static_cast<State>(this->startState << shift1);
            this->startState ^= static_cast<State>(this->startState >> shift2);
            this->startState ^= static_cast<State>(this->startState << shift3);
            return this->startState;
        }

    public:
    
        /** @brief Construct a new pseudo-random number generator
         * @param seed Starting seed
         */
        Random(T seed) :
            startState(seed != 0 ? static_cast<State>(static_cast<Unsigned>(seed)) : ZeroSeedState)
        {
        }

        /** @brief Destroy the pseudo-random number generator object
         */
        ~Random() { }

        /** @brief Get next pseudo-random number
         * @return Generated number in a full range
         */
        T GetNumber()
        {
            return static_cast<T>(this->GetNextPeriod());
        }
        
        /** @brief Get next pseudo-random number
         * @param from Inclusive start of the range
         * @param to Inclusive end of the range (from and to may be given in either order)
         * @return Generated number in a range; the full range of T returns the raw number, and a raw number already
         *         inside [from, to] is returned unchanged
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
            const Unsigned number = static_cast<Unsigned>(this->GetNumber());

            // Range size (inclusive) in the unsigned type: it cannot overflow, and 0 means the full range of T
            const Unsigned range = static_cast<Unsigned>(static_cast<Unsigned>(to) - static_cast<Unsigned>(from) + 1u);

            if (range == 0)
            {
                return static_cast<T>(number);
            }

            // Offset from 'from', so a number already inside [from, to] is returned unchanged
            const Unsigned offset = static_cast<Unsigned>(number - static_cast<Unsigned>(from));
            return static_cast<T>(static_cast<Unsigned>(static_cast<Unsigned>(from) + static_cast<Unsigned>(offset % range)));
        }
    };
}