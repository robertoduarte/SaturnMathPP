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
     * - The state is an unsigned 32-bit xorshift32 (13, 17, 5) for every T up to 32 bits, and an
     *   unsigned 64-bit xorshift64 (13, 7, 17) for 64-bit T; numbers are the state cast to T
     * - Period: 2^32 - 1 (2^64 - 1 for 64-bit T), whatever the size of T
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

        /** @brief State type: xorshift needs at least 32 bits, and logical (unsigned) shifts
         */
        using State = std::conditional_t<(sizeof(T) > 4), uint64_t, uint32_t>;

        /** @brief Non-zero state used for a seed of 0, which xorshift never leaves
         */
        static constexpr State ZeroSeedState = 0x9E3779B9;

        /** @brief Pseudo-Random number generator state
         */
        State startState;

        /** @brief Get next pseudo-random number
         * @return Next state
         */
        State GetNextPeriod()
        {
            if constexpr (sizeof(State) == 4)
            {
                this->startState ^= this->startState << 13;
                this->startState ^= this->startState >> 17;
                this->startState ^= this->startState << 5;
            }
            else
            {
                this->startState ^= this->startState << 13;
                this->startState ^= this->startState >> 7;
                this->startState ^= this->startState << 17;
            }

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