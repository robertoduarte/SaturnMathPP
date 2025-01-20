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
         * @brief Helper function to perform assembly-level dot product calculation and accumulation
         * @param first First vector
         * @param second Second vector
         * @warning This function MUST be used together with Fxp::ClearMac() and Fxp::ExtractMac().
         * Failing to clear the MAC registers before the first DotAccumulate or extract after the
         * last DotAccumulate will result in incorrect calculations.
         *
         * @note Hardware: This function uses the Saturn's MAC (Multiply and Accumulate) registers
         * for efficient SIMD-like operations. The MAC registers are shared hardware resources,
         * so their state must be properly managed.
         *
         * Required usage pattern:
         * @code
         * // Step 1: Always clear MAC registers before first DotAccumulate
         * Fxp::ClearMac();
         *
         * // Step 2: Call DotAccumulate one or more times
         * DotAccumulate(v1, v2);              // First dot product
         * DotAccumulate(v3, v4);              // Optional: accumulate more dot products
         *
         * // Step 3: Always extract result after last DotAccumulate
         * Fxp result = Fxp::ExtractMac();
         * @endcode
         */
        static void DotAccumulate(const Vector2D& first, const Vector2D& second)
        {
            auto a = reinterpret_cast<const int32_t*>(&first);
            auto b = reinterpret_cast<const int32_t*>(&second);
            __asm__ volatile(
                "\tmac.l @%[a]+, @%[b]+\n" // X * X
                "\tmac.l @%[a]+, @%[b]+\n" // Y * Y
                : [a] "+r"(a), [b] "+r"(b)
                : "m"(*a), "m"(*b)
                : "mach", "macl", "memory");
        }

        /**
         * @brief Calculate the dot product of this object and another Vec2 object.
         * @param vec The Vec2 object to calculate the dot product with.
         * @return The dot product as an Fxp value.
         * @details Calculates a single dot product between two vectors. For runtime calculations,
         * this uses the DotAccumulate helper with proper MAC register management.
         *
         * Example usage:
         * @code
         * Vector2D v1(1, 2);
         * Vector2D v2(4, 5);
         * Fxp result = v1.Dot(v2);    // Computes 1*4 + 2*5
         * @endcode
         */
        constexpr Fxp Dot(const Vector2D& vec) const
        {
            if consteval
            {
                return X * vec.X + Y * vec.Y;
            }
            else
            {
                Fxp::ClearMac();
                DotAccumulate(*this, vec);
                return Fxp::ExtractMac();
            }
        }

        /**
         * @brief Calculate and accumulate the dot products of multiple 2D vector pairs.
         * @param pairs Variadic parameter pack of vector pairs (e.g., {v1, v2}, {v3, v4}, ...).
         * @return The accumulated dot product of all pairs.
         * @details This function efficiently computes multiple dot products and accumulates their results
         * using the Saturn's MAC registers. It's more efficient than calculating individual dot products
         * and adding them, as it minimizes the number of MAC register operations and takes advantage
         * of the hardware's parallel multiply-accumulate capability.
         *
         * Example usage:
         * @code
         * Vector2D v1(1, 0), v2(1, 0);  // Unit vectors along X
         * Vector2D v3(0, 1), v4(0, 1);  // Unit vectors along Y
         *
         * // Computes (v1·v2) + (v3·v4) = 1 + 1 = 2
         * // All calculations done in parallel using Saturn's MAC registers
         * Fxp result = Vector2D::MultiDotAccumulate(
         *     std::pair{v1, v2},
         *     std::pair{v3, v4}
         * );
         * @endcode
         */
        template <typename... Pairs>
        static constexpr Fxp MultiDotAccumulate(const Pairs&... pairs)
        {
            if consteval
            {
                // Compile-time computation using fold expression
                return (... + pairs.first.Dot(pairs.second));
            }
            else
            {
                Fxp::ClearMac();

                // Loop through pairs and accumulate dot products
                ([&](const auto& pair)
                {
                    DotAccumulate(pair.first, pair.second);
                }(pairs), ...); // Unpack the variadic arguments

                return Fxp::ExtractMac();
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
         * @brief Compound multiplication assignment operator.
         * @param scalar The scalar value to multiply by.
         * @return Reference to the modified Vec2 object.
         */
        constexpr Vector2D& operator*=(const Fxp& scalar)
        {
            X *= scalar;
            Y *= scalar;
            return *this;
        }

        /**
         * @brief Division operator.
         * @param scalar The scalar to divide by.
         * @return The result of the division.
         */
        constexpr Vector2D operator/=(const Fxp& scalar)
        {
            X /= scalar;
            Y /= scalar;
            return *this;
        }

        /**
         * @brief Addition operator.
         * @param vec The vector to add.
         * @return The result of the addition.
         */
        constexpr Vector2D operator+=(const Vector2D& vec)
        {
            X += vec.X;
            Y += vec.Y;
            return *this;
        }

        /**
         * @brief Subtraction operator.
         * @param vec The vector to subtract.
         * @return The result of the subtraction.
         */
        constexpr Vector2D operator-=(const Vector2D& vec)
        {
            X -= vec.X;
            Y -= vec.Y;
            return *this;
        }

        /**
         * @brief Scaling operator.
         * @param scalar The scalar to multiply with.
         * @return The result of the scaling.
         */
        constexpr Vector2D operator*(const Fxp& scalar) const
        {
            Vector2D result(*this);
            result *= scalar;
            return result;
        }

        /**
         * @brief Addition operator.
         * @param vec The vector to add.
         * @return The result of the addition.
         */
        constexpr Vector2D operator+(const Vector2D& vec) const
        {
            return Vector2D(X + vec.X, Y + vec.Y);
        }

        /**
         * @brief Subtraction operator.
         * @param vec The vector to subtract.
         * @return The result of the subtraction.
         */
        constexpr Vector2D operator-(const Vector2D& vec) const
        {
            return Vector2D(X - vec.X, Y - vec.Y);
        }

        /**
         * @brief Division operator.
         * @param scalar The scalar to divide by.
         * @return The result of the division.
         */
        constexpr Vector2D operator/(const Fxp& scalar) const
        {
            return Vector2D(X / scalar, Y / scalar);
        }
        
        // Unary operators
        /**
         * @brief Unary negation operator.
         * @return A new Vec3 object with negated coordinates.
         */
        constexpr Vector2D operator-() const
        {
            return Vector2D(-X, -Y);
        }

        // Comparison operators
        /**
         * @brief Check if two Vec3 objects are not equal.
         * @param vec The Vec3 object to compare.
         * @return True if not equal, false otherwise.
         */
        constexpr bool operator!=(const Vector2D& vec) const
        {
            return X != vec.X && Y != vec.Y;
        }

        /**
         * @brief Check if two Vec3 objects are not equal.
         * @param vec The Vec3 object to compare.
         * @return True if not equal, false otherwise.
         */
        constexpr bool operator==(const Vector2D& vec) const
        {
            return X == vec.X && Y == vec.Y;
        }

        // Bitwise shift operators
        /**
         * @brief Bitwise right shift operator.
         * @param shiftAmount The number of positions to shift.
         * @return The resulting Vec3 object.
         */
        constexpr Vector2D operator>>(const size_t& shiftAmount)
        {
            return Vector2D(X >> shiftAmount, Y >> shiftAmount);
        }

        /**
         * @brief Bitwise right shift assignment operator.
         * @param shiftAmount The number of positions to shift.
         * @return Reference to the modified Vec3 object.
         */
        constexpr Vector2D& operator>>=(const size_t& shiftAmount)
        {
            X >>= shiftAmount;
            Y >>= shiftAmount;
            return *this;
        }

        /**
         * @brief Bitwise left shift operator.
         * @param shiftAmount The number of positions to shift.
         * @return The resulting Vec3 object.
         */
        constexpr Vector2D operator<<(const size_t& shiftAmount)
        {
            return Vector2D(X << shiftAmount, Y << shiftAmount);
        }

        /**
         * @brief Bitwise left shift assignment operator.
         * @param shiftAmount The number of positions to shift.
         * @return Reference to the modified Vec3 object.
         */
        constexpr Vector2D& operator<<=(const size_t& shiftAmount)
        {
            X <<= shiftAmount;
            Y <<= shiftAmount;
            return *this;
        }
    };
}