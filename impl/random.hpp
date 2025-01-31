#pragma once
#include <stdint.h>

namespace SaturnMath
{
    /** @brief Pseudo-Random number generator
     * @note This generator uses Xorshift (see https://en.wikipedia.org/wiki/Xorshift)
     */
    class Random
    {
    private:

        /** @brief Pseudo-Random number generator seed
         */
        uint32_t startState;

        /** @brief Get next pseudo-random number
         * @return Generated number
         */
        uint32_t GetNextPeriod()
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
        Random(uint32_t seed)
        {
            this->startState = seed;
        }

        /** @brief Destroy the pseudo-random number generator object
         */
        ~Random() { }

        /** @brief Get next pseudo-random number
         * @return Generated number in a full range
         */
        uint32_t GetNumber()
        {
            return this->GetNextPeriod();
        }
        
        /** @brief Get next pseudo-random number
         * @param from Inclusive start of the range
         * @param to Inclusive end of the range
         * @return Generated number in a range
         */
        int32_t GetNumber(int32_t from, int32_t to)
        {
            uint32_t number = this->GetNumber();
            return from + (number % Math::Abs(to - from));
        }
    };
}