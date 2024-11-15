#pragma once

#include "fxp.hpp"
#include <numbers>

namespace SaturnMath
{
    /**
     * @brief Efficient 16-bit angle representation optimized for Saturn hardware.
     * 
     * Represents angles using a 16-bit integer where the full circle (360°)
     * maps from 0x0000 to 0xFFFF. This provides an angle resolution of
     * approximately 0055 degrees (360° / 65536).
     * 
     * Common angle values:
     * - 0x0000 = 0° = 0 turns
     * - 0x4000 = 90° = 0.25 turns
     * - 0x8000 = 180° = 0.5 turns
     * - 0xC000 = 270° = 0.75 turns
     * - 0xFFFF ≈ 359.994° ≈ 0.99998 turns
     * 
     * @note All operations naturally handle wrap-around due to 16-bit arithmetic.
     */
    class Angle
    {
    private:
        // 16-bit angle where full rotation wraps to 0
        static constexpr uint16_t pi = 0x8000;       // π radians = 180° = 0.5 turns
        static constexpr uint16_t halfPi = 0x4000;   // π/2 radians = 90° = 0.25 turns
        static constexpr uint16_t quarterPi = 0x2000; // π/4 radians = 45° = 0.125 turns
        static constexpr uint16_t twoPi = 0;         // 2π radians = 360° = wraps to 0
        static constexpr double RadPi = std::numbers::pi;   // π constant for conversions
        
        uint16_t value; // Raw 16-bit angle where 0x0000-0xFFFF maps to 0-1 turns

    public:
        /**
         * @name Constant Angles
         * Common angle constants available at compile-time.
         * @{
         */
        static consteval Angle Zero() { return Angle(0); }         //!< 0° (0x0000)
        static consteval Angle Pi() { return Angle(pi); }          //!< 180° (0x8000)
        static consteval Angle HalfPi() { return Angle(halfPi); }  //!< 90° (0x4000)
        static consteval Angle QuarterPi() { return Angle(quarterPi); } //!< 45° (0x2000)
        static consteval Angle TwoPi() { return Angle(twoPi); }    //!< 360° (wraps to 0x0000)
        /** @} */

        /**
         * @name Constructors
         * @{
         */
        /** @brief Default constructor. Initializes angle to 0. */
        constexpr Angle() : value(0) {}

        /**
         * @brief Constructs angle from raw 16-bit value.
         * @param rawAngle Raw angle value where 0x0000-0xFFFF maps to 0-1 turns
         */
        constexpr Angle(uint16_t rawAngle) : value(rawAngle) {}
        /** @} */

        /**
         * @name Angle Conversions
         * Functions for converting between different angle representations.
         * @{
         */
        /**
         * @brief Creates angle from radians at compile time.
         * @param radians Angle in radians as floating-point value
         * @return Angle object
         */
        static consteval Angle FromRadians(double radians)
        {
            return Angle(static_cast<uint16_t>(Fxp(radians / (2 * RadPi)).Value));
        }

        /**
         * @brief Creates angle from radians.
         * @param radianTurns Angle in radians as fixed-point value
         * @return Angle object
         */
        static constexpr Angle FromRadians(const Fxp& radianTurns)
        {
            return Angle(static_cast<uint16_t>((radianTurns / (2 * RadPi)).Value));
        }

        /**
         * @brief Creates angle from degrees at compile time.
         * @param degrees Angle in degrees as floating-point value
         * @return Angle object
         */
        static consteval Angle FromDegrees(double degrees)
        {
            return Angle(static_cast<uint16_t>(Fxp(degrees / 360).Value));
        }

        /**
         * @brief Creates angle from degrees.
         * @param degreeTurns Angle in degrees as fixed-point value
         * @return Angle object
         */
        static constexpr Angle FromDegrees(const Fxp& degreeTurns)
        {
            return Angle(static_cast<uint16_t>((degreeTurns / 360).Value));
        }
        /** @} */

        /**
         * @brief Converts to fixed-point representation.
         * @return Angle value as fixed-point number
         */
        constexpr Fxp ToFxp() const
        {
            return Fxp::BuildRaw(value);
        }

        /**
         * @name Arithmetic Operations
         * Basic angle arithmetic with automatic wrap-around.
         * @{
         */
        /**
         * @brief Adds two angles.
         * @param other Angle to add
         * @return Sum of angles (automatically wraps around)
         */
        constexpr Angle operator+(const Angle& other) const
        {
            return Angle(value + other.value); // Natural 16-bit wrap-around
        }

        /**
         * @brief Subtracts two angles.
         * @param other Angle to subtract
         * @return Difference of angles (automatically wraps around)
         */
        constexpr Angle operator-(const Angle& other) const
        {
            return Angle(value - other.value); // Natural 16-bit wrap-around
        }
        /** @} */

        friend class Trigonometry;
    };
}