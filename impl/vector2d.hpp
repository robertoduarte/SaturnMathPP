// impl/vector2d.hpp
#pragma once

#include "fxp.hpp"

namespace SaturnMath
{
    /**
     * @brief A struct for two-dimensional fixed-point vector arithmetic operations.
     */
    struct Vector2D
    {
        Fxp x; /**< The X-coordinate. */
        Fxp y; /**< The Y-coordinate. */

        // Constructors
        /**
         * @brief Default constructor, initializes all coordinates to 0.
         */
        constexpr Vector2D() : x(), y() {}

        /**
         * @brief Constructor to initialize all coordinates with the same value.
         * @param fxp The value to initialize all coordinates with.
         */
        constexpr Vector2D(const Fxp& fxp) : x(fxp), y(fxp) {}

        /**
         * @brief Copy constructor.
         * @param vec The Vec2 object to copy.
         */
        constexpr Vector2D(const Vector2D& vec) : x(vec.x), y(vec.y) {}

        /**
         * @brief Constructor to initialize coordinates with specific values.
         * @param valueX The X-coordinate.
         * @param valueY The Y-coordinate.
         */
        constexpr Vector2D(const Fxp& valueX, const Fxp& valueY) : x(valueX), y(valueY) {}

        // Assignment operator
        /**
         * @brief Assignment operator.
         * @param vec The Vec2 object to assign.
         * @return Reference to the modified Vec2 object.
         */
        constexpr Vector2D& operator=(const Vector2D& vec)
        {
            x = vec.x;
            y = vec.y;
            return *this;
        }

        // Other member functions
        /**
         * @brief Calculate the absolute values of each coordinate.
         * @return A new Vec2 object with absolute values.
         */
        constexpr Vector2D Abs() const
        {
            return Vector2D(x.Abs(), y.Abs());
        }

        /**
         * @brief Sort coordinates in ascending order.
         * @return A new Vec2 object with sorted coordinates.
         */
        constexpr Vector2D Sorted() const
        {
            return Vector2D(x < y ? x : y, x < y ? y : x);
        }

        /**
         * @brief Calculate the dot product of two vectors.
         * @param vec The vector to calculate the dot product with.
         * @return The dot product of the two vectors.
         */
        constexpr Fxp Dot(const Vector2D& vec) const
        {
            if consteval
            {
                return x * vec.x + y * vec.y;
            }
            else
            {
                int32_t aux0;
                int32_t aux1;
                auto a = reinterpret_cast<const int32_t*>(this);
                auto b = reinterpret_cast<const int32_t*>(&vec);

                __asm__ volatile("\tclrmac\n"
                    "\tmac.l @%[a]+, @%[b]+\n"
                    "\tmac.l @%[a]+, @%[b]+\n"
                    "\tsts mach, %[aux0]\n"
                    "\tsts macl, %[aux1]\n"
                    "\txtrct %[aux0], %[aux1]\n"
                    : [a] "+r"(a),
                    [b] "+r"(b),
                    [aux0] "=&r"(aux0),
                    [aux1] "=&r"(aux1)
                    : "m"(*a),
                    "m"(*b)
                    : "mach", "macl", "memory");
                return aux1;
            }
        }
        /**
         * @brief Calculate the magnitude (length) of the vector.
         * @return The magnitude of the vector.
         */
        constexpr Fxp Magnitude() const
        {
            return (x * x + y * y).Sqrt();
        }

        /**
         * @brief Normalize the vector to have a length of 1.
         * @return A new Vec2 object with the normalized vector.
         */
        constexpr Vector2D Normalize() const
        {
            Fxp magnitude = Magnitude();
            return Vector2D(x / magnitude, y / magnitude);
        }

        // Arithmetic operators
        /**
         * @brief Addition operator.
         * @param vec The vector to add.
         * @return The result of the addition.
         */
        constexpr Vector2D operator+(const Vector2D& vec) const
        {
            return Vector2D(x + vec.x, y + vec.y);
        }

        /**
         * @brief Subtraction operator.
         * @param vec The vector to subtract.
         * @return The result of the subtraction.
         */
        constexpr Vector2D operator-(const Vector2D& vec) const
        {
            return Vector2D(x - vec.x, y - vec.y);
        }

        /**
         * @brief Scaling operator.
         * @param scalar The scalar to multiply with.
         * @return The result of the scaling.
         */
        constexpr Vector2D operator*(const Fxp& scalar) const
        {
            return Vector2D(x * scalar, y * scalar);
        }

        /**
         * @brief Division operator.
         * @param scalar The scalar to divide by.
         * @return The result of the division.
         */
        constexpr Vector2D operator/(const Fxp& scalar) const
        {
            return Vector2D(x / scalar, y / scalar);
        }
    };
}