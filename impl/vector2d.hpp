#pragma once

#include "fxp.hpp"
#include "precision.hpp"
#include "sort_order.hpp"

namespace SaturnMath
{
    /**
     * @brief A struct for two-dimensional fixed-point vector arithmetic operations.
     */
    struct Vector2D
    {
        Fxp X; /**< The X-coordinate. */
        Fxp Y; /**< The Y-coordinate. */

        // Constructors
        /**
         * @brief Default constructor, initializes all coordinates to 0.
         */
        constexpr Vector2D() : X(), Y() {}

        /**
         * @brief Constructor to initialize all coordinates with the same value.
         * @param fxp The value to initialize all coordinates with.
         */
        constexpr Vector2D(const Fxp& fxp) : X(fxp), Y(fxp) {}

        /**
         * @brief Copy constructor.
         * @param vec The Vec2 object to copy.
         */
        constexpr Vector2D(const Vector2D& vec) : X(vec.X), Y(vec.Y) {}

        /**
         * @brief Constructor to initialize coordinates with specific values.
         * @param valueX The X-coordinate.
         * @param valueY The Y-coordinate.
         */
        constexpr Vector2D(const Fxp& valueX, const Fxp& valueY) : X(valueX), Y(valueY) {}

        // Assignment operator
        /**
         * @brief Assignment operator.
         * @param vec The Vec2 object to assign.
         * @return Reference to the modified Vec2 object.
         */
        constexpr Vector2D& operator=(const Vector2D& vec)
        {
            X = vec.X;
            Y = vec.Y;
            return *this;
        }

        // Other member functions
        /**
         * @brief Calculate the absolute values of each coordinate.
         * @return A new Vec2 object with absolute values.
         */
        constexpr Vector2D Abs() const
        {
            return Vector2D(X.Abs(), Y.Abs());
        }

        /**
         * @brief Sort coordinates in specified order
         * @tparam O Sort order (Ascending or Descending)
         * @return A new Vec2 object with sorted coordinates
         */
        template <SortOrder O = SortOrder::Ascending>
        constexpr Vector2D Sort() const
        {
            Vector2D result(*this);
            if (O == SortOrder::Ascending ? result.X > result.Y : result.X < result.Y)
            {
                Fxp temp = result.X;
                result.X = result.Y;
                result.Y = temp;
            }
            return result;
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
         * @brief Calculate the length of the vector
         * @tparam P Precision level for calculation
         * @return Length of the vector
         */
        template<Precision P = Precision::Standard>
        constexpr Fxp Length() const
        {
            if constexpr (P == Precision::Turbo)
            {
                // Use alpha-beta coefficients for fastest approximation
                constexpr Vector2D alphaBeta(
                    0.96043387010342,    // Alpha
                    0.39782473475533     // Beta
                );
                return Abs().Sort<SortOrder::Descending>().Dot(alphaBeta);
            }
            else
            {
                return Dot(*this).Sqrt<P>();
            }
        }

        /**
         * @brief Normalize the vector
         * @tparam P Precision level for calculation
         * @return Normalized vector
         */
        template<Precision P = Precision::Standard>
        constexpr Vector2D Normalize() const
        {
            Fxp length = Length<P>();
            if (length != 0)
                return Vector2D(X / length, Y / length);
            return Vector2D();
        }

        // Unit vectors and directional methods
        /**
         * @brief Get a unit vector pointing along the X axis (1,0).
         * @return Unit vector along X axis.
         */
        static consteval Vector2D UnitX()
        {
            return Vector2D(1, 0);
        }

        /**
         * @brief Get a unit vector pointing along the Y axis (0,1).
         * @return Unit vector along Y axis.
         */
        static consteval Vector2D UnitY()
        {
            return Vector2D(0, 1);
        }

        /**
         * @brief Get a zero vector (0,0).
         * @return Zero vector.
         */
        static consteval Vector2D Zero()
        {
            return Vector2D(0);
        }

        /**
         * @brief Get a vector with all components set to one (1,1).
         * @return Vector with all ones.
         */
        static consteval Vector2D One()
        {
            return Vector2D(1);
        }

        /**
         * @brief Get a vector pointing right (1,0).
         * Same as UnitX(), provided for semantic clarity.
         * @return Right-pointing unit vector.
         */
        static consteval Vector2D Right()
        {
            return UnitX();
        }

        /**
         * @brief Get a vector pointing left (-1,0).
         * @return Left-pointing unit vector.
         */
        static consteval Vector2D Left()
        {
            return Vector2D(-1, 0);
        }

        /**
         * @brief Get a vector pointing up (0,1).
         * Same as UnitY(), provided for semantic clarity.
         * @return Upward-pointing unit vector.
         */
        static consteval Vector2D Up()
        {
            return UnitY();
        }

        /**
         * @brief Get a vector pointing down (0,-1).
         * @return Downward-pointing unit vector.
         */
        static consteval Vector2D Down()
        {
            return Vector2D(0, -1);
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