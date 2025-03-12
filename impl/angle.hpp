#pragma once

#include "fxp.hpp"
#include <numbers>
#include <concepts>

namespace SaturnMath::Types
{
    /**
     * @class Angle
     * @brief Efficient 16-bit angle representation optimized for Saturn hardware.
     *
     * @details Represents angles using a 16-bit integer where the full circle (360°)
     * maps from 0x0000 to 0xFFFF. This provides an angle resolution of
     * approximately 0.0055 degrees (360° / 65536).
     *
     * This representation is designed for optimal performance on Saturn hardware,
     * allowing for fast angle calculations with automatic wrap-around behavior.
     *
     * Common angle values:
     * - 0x0000 = 0° = 0 turns
     * - 0x4000 = 90° = 0.25 turns
     * - 0x8000 = 180° = 0.5 turns
     * - 0xC000 = 270° = 0.75 turns
     * - 0xFFFF ≈ 359.994° ≈ 0.99998 turns
     *
     * @note All operations naturally handle wrap-around due to 16-bit arithmetic.
     * @see Fxp For the fixed-point number representation used with this class
     */
    class Angle
    {
    private:
        /// @brief 16-bit angle where full rotation wraps to 0
        static constexpr uint16_t pi = 0x8000;       ///< π radians = 180° = 0.5 turns
        static constexpr uint16_t halfPi = 0x4000;   ///< π/2 radians = 90° = 0.25 turns
        static constexpr uint16_t quarterPi = 0x2000; ///< π/4 radians = 45° = 0.125 turns
        static constexpr uint16_t twoPi = 0;         ///< 2π radians = 360° = wraps to 0
        static constexpr double RadPi = std::numbers::pi;   ///< π constant for conversions

        uint16_t value; ///< Raw 16-bit angle where 0x0000-0xFFFF maps to 0-1 turns

        /**
         * @brief Constructs angle from raw 16-bit value.
         *
         * @param rawAngle Raw angle value where 0x0000-0xFFFF maps to 0-1 turns
         * @param dummy Dummy parameter to differentiate from other constructors
         *
         * @note This constructor allows direct initialization from a raw 16-bit value
         * without any conversions.
         */
        constexpr Angle(uint16_t rawAngle, bool dummy) : value(rawAngle) {}

    public:
        /**
         * @name Constant Angles
         * @{
         * @brief Common angle constants available at compile-time.
         */
        static consteval Angle Zero() { return BuildRaw(0); }         ///< 0° (0x0000)
        static consteval Angle Pi() { return BuildRaw(pi); }          ///< 180° (0x8000)
        static consteval Angle HalfPi() { return BuildRaw(halfPi); }  ///< 90° (0x4000)
        static consteval Angle QuarterPi() { return BuildRaw(quarterPi); } ///< 45° (0x2000)
        static consteval Angle TwoPi() { return BuildRaw(twoPi); }    ///< 360° (wraps to 0x0000)
        static consteval Angle Right() { return BuildRaw(halfPi); }   ///< 90° (0x4000)
        static consteval Angle Straight() { return BuildRaw(pi); }    ///< 180° (0x8000)
        static consteval Angle Full() { return BuildRaw(0); }         ///< 360° (wraps to 0x0000)
        /** @} */

        /**
         * @brief Creates an angle from a raw 16-bit value.
         * 
         * @param rawValue Raw 16-bit angle value where 0x0000-0xFFFF maps to 0-1 turns
         * @return Angle object initialized with the raw value
         * 
         * @note This is useful when you already have a raw angle value and want to
         * create an Angle object without any conversions.
         */
        static constexpr Angle BuildRaw(uint16_t rawValue)
        {
            return Angle(rawValue, true);
        }

        /**
         * @name Constructors
         * @{
         */
         /** 
          * @brief Default constructor. Initializes angle to 0.
          * 
          * Creates an Angle object with a value of 0, representing 0 degrees or 0 turns.
          */
        constexpr Angle() : value(0) {}

        /**
         * @brief Constructs angle from fixed-point turns.
         * 
         * @param turns Angle in turns as fixed-point value
         * @param isDummy Dummy parameter to differentiate from raw value constructor
         * 
         * @note This constructor allows conversion from a fixed-point representation
         * of turns (where 1.0 represents a full 360° rotation) to an Angle object.
         */
        constexpr Angle(const Fxp& turns, bool isDummy = true) : value(static_cast<uint16_t>(turns.RawValue())) {}
        /** @} */

        /**
         * @name Angle Conversions
         * @{
         * @brief Functions for converting between different angle representations.
         * 
         * These functions allow conversion between radians, degrees, and the internal
         * 16-bit representation. Performance warnings are included to encourage using
         * the native angle representation when possible.
         */
        #ifdef DISABLE_PERFORMANCE_WARNINGS
        #define ANGLE_CONVERSION_WARNING
        #else
        #define ANGLE_CONVERSION_WARNING [[gnu::warning("Angle conversion is computationally expensive. Consider using native angle representation (turns) for better performance.")]]
        #endif

        /**
         * @brief Creates angle from radians at compile time.
         * 
         * @param radians Angle in radians as floating-point value
         * @return Angle object
         * 
         * @note This is a compile-time function that converts from radians to
         * the internal 16-bit representation.
         */
        static consteval Angle FromRadians(double radians)
        {
            return BuildRaw(static_cast<uint16_t>(Fxp(radians / (2 * RadPi)).RawValue()));
        }

        /**
         * @brief Creates angle from radians.
         * 
         * @param radianTurns Angle in radians as fixed-point value
         * @return Angle object
         * 
         * @warning This conversion is computationally expensive. Consider using
         * the native angle representation (turns) for better performance.
         */
        ANGLE_CONVERSION_WARNING
        static constexpr Angle FromRadians(const Fxp& radianTurns)
        {
            return BuildRaw(static_cast<uint16_t>((radianTurns / (2 * RadPi)).RawValue()));
        }

        /**
         * @brief Converts angle to radians.
         * 
         * @return Angle in radians as fixed-point value
         * 
         * @warning This conversion is computationally expensive. Consider using
         * the native angle representation (turns) for better performance.
         */
        ANGLE_CONVERSION_WARNING
        constexpr Fxp ToRadians() const
        {
            return Fxp::BuildRaw(static_cast<uint32_t>(value) * 2 * RadPi);
        }

        /**
         * @brief Creates angle from degrees at compile time.
         * 
         * @param degrees Angle in degrees as floating-point value
         * @return Angle object
         * 
         * @note This is a compile-time function that converts from degrees to
         * the internal 16-bit representation.
         */
        static consteval Angle FromDegrees(double degrees)
        {
            return BuildRaw(static_cast<uint16_t>(Fxp(degrees / 360).RawValue()));
        }

        /**
         * @brief Converts angle to degrees.
         * 
         * @return Angle in degrees as fixed-point value
         * 
         * @warning This conversion is computationally expensive. Consider using
         * the native angle representation (turns) for better performance.
         */
        ANGLE_CONVERSION_WARNING
        constexpr Fxp ToDegrees() const
        {
            return Fxp::BuildRaw(static_cast<uint32_t>(value) * 360);
        }

        /**
         * @brief Converts angle to turns.
         * 
         * @return Angle in turns as fixed-point value
         * 
         * @note This conversion is efficient as it directly uses the internal
         * representation, where one full turn (360 degrees) is represented by 0xFFFF.
         */
        constexpr Fxp ToTurns() const
        {
            return Fxp::BuildRaw(value);
        }


        #undef ANGLE_CONVERSION_WARNING

        /**
         * @brief Creates angle from degrees.
         * 
         * @param degreeTurns Angle in degrees as fixed-point value
         * @return Angle object
         * 
         * @note Converts from a fixed-point representation of degrees to an Angle object.
         */
        static constexpr Angle FromDegrees(const Fxp& degreeTurns)
        {
            return BuildRaw(static_cast<uint16_t>((degreeTurns / 360).RawValue()));
        }

        /**
         * @brief Creates an angle from a floating-point value in turns.
         * 
         * @param turns Angle in turns as floating-point value
         * @return Angle object
         * 
         * @note This is a convenience method for creating angles from floating-point values.
         * It is less efficient than using the fixed-point version but more readable in code.
         */
        static consteval Angle FromTurns(float turns)
        {
            return BuildRaw(static_cast<uint16_t>(static_cast<int32_t>(turns * 65536.0)));
        }


        /** @} */

        /**
         * @brief Converts to fixed-point representation.
         * 
         * @return Angle value as fixed-point number
         * 
         * @note This provides the angle in turns as a fixed-point value,
         * where 1.0 represents a full 360° rotation.
         */
        constexpr Fxp ToFxp() const
        {
            return Fxp::BuildRaw(value);
        }

        /**
         * @brief Returns a const reference to the internal raw angle value.
         * 
         * @return const reference to the internal value
         * 
         * @note This provides direct access to the raw 16-bit angle value
         * for performance-critical operations.
         */
        constexpr const uint16_t& RawValue() const { return value; }

        /**
         * @name Arithmetic Operations
         * @{
         * @brief Basic angle arithmetic with automatic wrap-around.
         * 
         * These operations handle angle arithmetic with automatic wrap-around
         * behavior due to the 16-bit representation.
         */
         /**
          * @brief Adds two angles.
          * 
          * @param other Angle to add
          * @return Sum of angles (automatically wraps around)
          * 
          * @note The addition naturally wraps around due to 16-bit arithmetic.
          */
        constexpr Angle operator+(const Angle& other) const
        {
            return BuildRaw(value + other.value); // Natural 16-bit wrap-around
        }

        /**
         * @brief Subtracts two angles.
         * 
         * @param other Angle to subtract
         * @return Difference of angles (automatically wraps around)
         * 
         * @note The subtraction naturally wraps around due to 16-bit arithmetic.
         */
        constexpr Angle operator-(const Angle& other) const
        {
            return BuildRaw(value - other.value); // Natural 16-bit wrap-around
        }

        /**
         * @brief Multiplies an angle by a fixed-point scalar value.
         * 
         * @param fxp The fixed-point scalar value to multiply by
         * @return New Angle object containing the result
         * 
         * @note Automatically wraps around the result due to 16-bit arithmetic.
         */
        constexpr Angle operator*(const Fxp& fxp) const
        {
            return BuildRaw((ToFxp() * fxp).RawValue());
        }

        /**
         * @brief Multiplies an angle by an integer scalar value.
         * 
         * @tparam T Integer type (deduced from parameter)
         * @param scalar The integer scalar value to multiply by
         * @return New Angle object containing the result
         * 
         * @note This operates directly on the raw value without fixed-point conversion,
         * making it more efficient for integer multiplication.
         * @note Automatically wraps around the result due to 16-bit arithmetic.
         */
        template <typename T>
        constexpr Angle operator*(T scalar) const requires std::integral<T>
        {
            return BuildRaw(value * scalar);
        }

        /**
         * @brief Divides an angle by a fixed-point scalar value.
         * 
         * @param fxp The fixed-point scalar value to divide by
         * @return New Angle object containing the result
         * 
         * @note Automatically wraps around the result due to 16-bit arithmetic.
         * @warning Division by zero will cause undefined behavior.
         */
        constexpr Angle operator/(const Fxp& fxp) const
        {
            return BuildRaw((ToFxp() / fxp).RawValue());
        }

        /**
         * @brief Divides an angle by an integer scalar value.
         * 
         * @tparam T Integer type (deduced from parameter)
         * @param scalar The integer scalar value to divide by
         * @return New Angle object containing the result
         * 
         * @note This operates directly on the raw value without fixed-point conversion,
         * making it more efficient for integer division.
         * @note Automatically wraps around the result due to 16-bit arithmetic.
         * @warning Division by zero will cause undefined behavior.
         */
        template <typename T>
        constexpr Angle operator/(T scalar) const requires std::integral<T>
        {
            return BuildRaw(value / scalar);
        }
        
        /**
         * @brief Multiplies an angle by a fixed-point scalar value.
         * 
         * @param fxp The fixed-point scalar value to multiply by
         * @return Reference to this Angle object
         * @note Automatically wraps around the result
         */
        constexpr Angle& operator*=(const Fxp& fxp)
        {
            value = (ToFxp() * fxp).RawValue();
            return *this;
        }

        /**
         * @brief Multiplies an angle by an integer scalar value.
         * 
         * @tparam T Integer type (deduced from parameter)
         * @param scalar The integer scalar value to multiply by
         * @return Reference to this Angle object
         * 
         * @note This operates directly on the raw value without fixed-point conversion,
         * making it more efficient for integer multiplication.
         * @note Automatically wraps around the result due to 16-bit arithmetic.
         */
        template <typename T>
        constexpr Angle& operator*=(T scalar) requires std::integral<T>
        {
            value *= scalar;
            return *this;
        }

        /**
         * @brief Divides an angle by a fixed-point scalar value.
         * 
         * @param fxp The fixed-point scalar value to divide by
         * @return Reference to this Angle object
         * @note Automatically wraps around the result
         * @warning Division by zero will cause undefined behavior.
         */
        constexpr Angle& operator/=(const Fxp& fxp)
        {
            value = (ToFxp() / fxp).RawValue();
            return *this;
        }

        /**
         * @brief Divides an angle by an integer scalar value.
         * 
         * @tparam T Integer type (deduced from parameter)
         * @param scalar The integer scalar value to divide by
         * @return Reference to this Angle object
         * 
         * @note This operates directly on the raw value without fixed-point conversion,
         * making it more efficient for integer division.
         * @note Automatically wraps around the result due to 16-bit arithmetic.
         * @warning Division by zero will cause undefined behavior.
         */
        template <typename T>
        constexpr Angle& operator/=(T scalar) requires std::integral<T>
        {
            value /= scalar;
            return *this;
        }

        /**
         * @brief Adds two angles.
         * 
         * @param other Angle to add
         * @return Reference to the modified Angle object
         * 
         * @note Automatically wraps around the result due to 16-bit arithmetic.
         */
        constexpr Angle& operator+=(const Angle& other)
        {
            value += other.value; // Natural 16-bit wrap-around
            return *this;
        }

        /**
         * @brief Subtracts two angles.
         * 
         * @param other Angle to subtract
         * @return Reference to the modified Angle object
         * 
         * @note Automatically wraps around the result due to 16-bit arithmetic.
         */
        constexpr Angle& operator-=(const Angle& other)
        {
            value -= other.value; // Natural 16-bit wrap-around
            return *this;
        }

        /**
         * @brief Negates the angle, effectively adding half a turn (180 degrees).
         *
         * @return New Angle object representing the negated angle.
         * 
         * @details This operator allows for intuitive manipulation of angles by providing 
         * a way to obtain the angle that is directly opposite to the current angle.
         * It adds half a turn (180°) to the current angle.
         */
        constexpr Angle operator-() const
        {
            return BuildRaw(value + halfPi); // Add half a turn
        }

        /**
         * @brief Compares two angles for equality.
         * 
         * @param other The angle to compare with
         * @return true if the angles are equal, false otherwise
         * 
         * @note This compares the raw 16-bit values for exact equality.
         */
        constexpr bool operator==(const Angle& other) const
        {
            return value == other.value;
        }

        /**
         * @brief Compares two angles for inequality.
         * 
         * @param other The angle to compare with
         * @return true if the angles are not equal, false otherwise
         * 
         * @note This compares the raw 16-bit values for exact inequality.
         */
        constexpr bool operator!=(const Angle& other) const
        {
            return value != other.value;
        }

        /**
         * @brief Compares if this angle is less than another angle.
         * 
         * @param other The angle to compare with
         * @return true if this angle is less than the other angle, false otherwise
         * 
         * @warning Due to the wrap-around nature of angles, this comparison may lead to 
         * unexpected results if angles are near the wrap-around point (0/360 degrees).
         * For example, 359° would be considered less than 1° in the raw representation,
         * even though they are only 2° apart across the wrap-around boundary.
         * 
         * @note This is most useful when comparing angles within the same quadrant or
         * when the angles are known to be within 180 degrees of each other.
         */
        constexpr bool operator<(const Angle& other) const
        {
            return value < other.value;
        }

        /**
         * @brief Compares if this angle is greater than another angle.
         * 
         * @param other The angle to compare with
         * @return true if this angle is greater than the other angle, false otherwise
         * 
         * @warning Due to the wrap-around nature of angles, this comparison may lead to 
         * unexpected results if angles are near the wrap-around point (0/360 degrees).
         * For example, 1° would be considered greater than 359° in the raw representation,
         * even though they are only 2° apart across the wrap-around boundary.
         * 
         * @note This is most useful when comparing angles within the same quadrant or
         * when the angles are known to be within 180 degrees of each other.
         */
        constexpr bool operator>(const Angle& other) const
        {
            return value > other.value;
        }

        /**
         * @brief Compares if this angle is less than or equal to another angle.
         * 
         * @param other The angle to compare with
         * @return true if this angle is less than or equal to the other angle, false otherwise
         * 
         * @warning Due to the wrap-around nature of angles, this comparison may lead to 
         * unexpected results if angles are near the wrap-around point (0/360 degrees).
         * Always consider the context of your comparison and whether the wrap-around
         * behavior might affect your logic.
         * 
         * @note This is most useful when comparing angles within the same quadrant or
         * when the angles are known to be within 180 degrees of each other.
         */
        constexpr bool operator<=(const Angle& other) const
        {
            return value <= other.value;
        }

        /**
         * @brief Compares if this angle is greater than or equal to another angle.
         * 
         * @param other The angle to compare with
         * @return true if this angle is greater than or equal to the other angle, false otherwise
         * 
         * @warning Due to the wrap-around nature of angles, this comparison may lead to 
         * unexpected results if angles are near the wrap-around point (0/360 degrees).
         * Always consider the context of your comparison and whether the wrap-around
         * behavior might affect your logic.
         * 
         * @note This is most useful when comparing angles within the same quadrant or
         * when the angles are known to be within 180 degrees of each other.
         */
        constexpr bool operator>=(const Angle& other) const
        {
            return value >= other.value;
        }
        /** @} */
    };

    /**
     * @struct EulerAngles
     * @brief Represents three rotation angles in Euler angle format.
     * 
     * @details Uses the intrinsic Tait-Bryan angles with the order X-Y-Z (pitch-yaw-roll).
     * This representation is commonly used in 3D graphics and physics simulations.
     * 
     * The rotation order is:
     * 1. First rotate around X axis (pitch)
     * 2. Then rotate around Y axis (yaw)
     * 3. Finally rotate around Z axis (roll)
     * 
     * @see Angle The underlying angle representation used for each component
     */
    struct EulerAngles {
        /**
         * @brief Rotation around the X axis (up/down).
         * 
         * @details Represents the vertical rotation that causes looking up or down.
         * In the intrinsic Tait-Bryan X-Y-Z rotation sequence, pitch is applied first.
         * 
         * Mathematical properties:
         * - Range: Typically constrained to [-90°, 90°] to avoid gimbal lock
         * - Singularities occur at ±90° when used in conjunction with yaw and roll
         * - Positive pitch rotates upward (counter-clockwise around X-axis when viewed from positive X)
         * - Negative pitch rotates downward (clockwise around X-axis when viewed from positive X)
         * 
         * Common applications:
         * - Camera control systems (looking up/down)
         * - Flight simulators (aircraft nose up/down)
         * - Character head movement
         * 
         * @note When approaching ±90°, the system may experience gimbal lock where
         * yaw and roll rotations become indistinguishable. Consider using quaternions
         * for applications requiring full 360° rotation freedom.
         */
        Angle pitch;
        
        /**
         * @brief Rotation around the Y axis (left/right).
         * 
         * @details Represents the horizontal rotation that causes looking left or right.
         * In the intrinsic Tait-Bryan X-Y-Z rotation sequence, yaw is applied second,
         * after pitch but before roll.
         * 
         * Mathematical properties:
         * - Range: Full 360° rotation is possible without inherent limitations
         * - Represents rotation in the horizontal plane (around world Y-axis)
         * - Positive yaw rotates rightward (counter-clockwise around Y-axis when viewed from positive Y)
         * - Negative yaw rotates leftward (clockwise around Y-axis when viewed from positive Y)
         * 
         * Common applications:
         * - Camera control systems (panning left/right)
         * - Vehicle steering and navigation
         * - Character orientation in 3D space
         * 
         * @note Yaw is often the most frequently used rotation in navigation and
         * camera control systems. For performance-critical applications, consider
         * optimizing yaw-only rotations as special cases.
         */
        Angle yaw;
        
        /**
         * @brief Rotation around the Z axis (tilt).
         * 
         * @details Represents the tilting rotation that causes leaning to either side.
         * In the intrinsic Tait-Bryan X-Y-Z rotation sequence, roll is applied last,
         * after both pitch and yaw.
         * 
         * Mathematical properties:
         * - Range: Full 360° rotation is possible without inherent limitations
         * - Represents rotation around the local forward axis after pitch and yaw
         * - Positive roll rotates clockwise when viewed from behind
         * - Negative roll rotates counter-clockwise when viewed from behind
         * 
         * Common applications:
         * - Aircraft roll control (banking)
         * - Camera dutch angle effects
         * - Balancing simulations
         * - Special effects for disorientation
         * 
         * @note Roll is often less used in standard camera control systems but is
         * essential for flight simulators and certain special effects. The effect
         * of roll depends on the current pitch and yaw values due to the sequential
         * nature of Euler angle rotations.
         */
        Angle roll;

        /**
         * @brief Default constructor initializing all angles to zero.
         * 
         * Creates an EulerAngles object with all rotation angles set to zero.
         */
        constexpr EulerAngles() : pitch(Angle::Zero()), yaw(Angle::Zero()), roll(Angle::Zero()) {}

        /**
         * @brief Constructor with explicit angles.
         * 
         * @param pitchAngle Rotation around X axis (up/down)
         * @param yawAngle Rotation around Y axis (left/right)
         * @param rollAngle Rotation around Z axis (tilt)
         * 
         * @note Creates an EulerAngles object with the specified rotation angles.
         */
        constexpr EulerAngles(const Angle& pitchAngle, const Angle& yawAngle, const Angle& rollAngle)
            : pitch(pitchAngle), yaw(yawAngle), roll(rollAngle) {}
    };

} // namespace SaturnMath::Types