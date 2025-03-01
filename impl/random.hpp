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
            this->startState ^= this->startState << 13;
            this->startState ^= this->startState >> 17;
            this->startState ^= this->startState << 5;
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
            T number = this->GetNumber();
            return from + (number % std::abs(to - from));
        }
    };
}