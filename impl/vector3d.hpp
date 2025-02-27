#pragma once

#include <utility>
#include "vector2d.hpp"
#include "precision.hpp"
#include "sort_order.hpp"

namespace SaturnMath::Types
{
    /**
     * @brief A struct for three-dimensional fixed-point vector arithmetic operations.
     */
    struct Vector3D : public Vector2D
    {
        Fxp Z; /**< The Z-coordinate. */

        // Constructors
        /**
         * @brief Default constructor, initializes all coordinates to 0.
         */
        constexpr Vector3D() : Vector2D(), Z() {}

        /**
         * @brief Constructor to initialize all coordinates with the same value.
         * @param fxp The value to initialize all coordinates with.
         */
        constexpr Vector3D(const Fxp& fxp) : Vector2D(fxp), Z(fxp) {}

        /**
         * @brief Copy constructor.
         * @param vec The Vec3 object to copy.
         */
        constexpr Vector3D(const Vector3D& vec) : Vector2D(vec), Z(vec.Z) {}

        /**
         * @brief Constructor to initialize coordinates with specific values.
         * @param valueX The X-coordinate.
         * @param valueY The Y-coordinate.
         * @param valueZ The Z-coordinate.
         */
        constexpr Vector3D(const Fxp& valueX, const Fxp& valueY, const Fxp& valueZ) : Vector2D(valueX, valueY), Z(valueZ) {}

        /**
         * @brief Constructor to initialize from a Vector2D and a Z coordinate.
         * @param vec2d The Vector2D to copy X and Y from.
         * @param valueZ The Z-coordinate.
         */
        constexpr Vector3D(const Vector2D& vec2d, const Fxp& valueZ) : Vector2D(vec2d), Z(valueZ) {}

        // Assignment operator
        /**
         * @brief Assignment operator.
         * @param vec The Vec3 object to assign.
         * @return Reference to the modified Vec3 object.
         */
        constexpr Vector3D& operator=(const Vector3D& vec)
        {
            Vector2D::operator=(vec);
            Z = vec.Z;
            return *this;
        }

        /**
         * @brief Calculate the absolute values of each coordinate.
         * @return A new Vec3 object with absolute values.
         */
        constexpr Vector3D Abs() const
        {
            return Vector3D(Vector2D::Abs(), Z.Abs());
        }

        /**
         * @brief Sort coordinates in ascending order.
         * @tparam Ascending If true, sort in ascending order; otherwise, in descending order.
         * @return A new Vec3 object with sorted coordinates.
         */
        template <SortOrder O = SortOrder::Ascending>
        constexpr Vector3D Sort()
        {
            Vector3D result(*this);
            Fxp temp;

            if (O == SortOrder::Ascending ? result.X > result.Y : result.X < result.Y)
            {
                temp = result.X;
                result.X = result.Y;
                result.Y = temp;
            }

            if (O == SortOrder::Ascending ? result.X > result.Z : result.X < result.Z)
            {
                temp = result.X;
                result.X = result.Z;
                result.Z = temp;
            }

            if (O == SortOrder::Ascending ? result.Y > result.Z : result.Y < result.Z)
            {
                temp = result.Y;
                result.Y = result.Z;
                result.Z = temp;
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
        static void DotAccumulate(const Vector3D& first, const Vector3D& second)
        {
            auto a = reinterpret_cast<const int32_t*>(&first);
            auto b = reinterpret_cast<const int32_t*>(&second);
            __asm__ volatile(
                "\tmac.l @%[a]+, @%[b]+\n" // X * X
                "\tmac.l @%[a]+, @%[b]+\n" // Y * Y
                "\tmac.l @%[a]+, @%[b]+\n" // Z * Z
                : [a] "+r"(a), [b] "+r"(b)
                : "m"(*a), "m"(*b)
                : "mach", "macl", "memory");
        }

        /**
         * @brief Calculate the dot product of this object and another Vec3 object.
         * @param vec The Vec3 object to calculate the dot product with.
         * @return The dot product as an Fxp value.
         * @details Calculates a single dot product between two vectors. For runtime calculations,
         * this uses the DotAccumulate helper with proper MAC register management.
         *
         * Example usage:
         * @code
         * Vector3D v1(1, 2, 3);
         * Vector3D v2(4, 5, 6);
         * Fxp result = v1.Dot(v2);    // Computes 1*4 + 2*5 + 3*6
         * @endcode
         */
        constexpr Fxp Dot(const Vector3D& vec) const
        {
            if consteval
            {
                return X * vec.X + Y * vec.Y + Z * vec.Z;
            }
            else
            {
                Fxp::ClearMac();
                DotAccumulate(*this, vec);
                return Fxp::ExtractMac();
            }
        }

        /**
         * @brief Calculate and accumulate the dot products of multiple 3D vector pairs.
         * @param pairs Variadic parameter pack of vector pairs (e.g., {v1, v2}, {v3, v4}, ...).
         * @return The accumulated dot product of all pairs.
         * @details This function efficiently computes multiple dot products and accumulates their results
         * using the Saturn's MAC registers. It's more efficient than calculating individual dot products
         * and adding them, as it minimizes the number of MAC register operations and takes advantage
         * of the hardware's parallel multiply-accumulate capability.
         *
         * Example usage:
         * @code
         * Vector3D v1(1, 0, 0), v2(1, 0, 0);  // Unit vectors along X
         * Vector3D v3(0, 1, 0), v4(0, 1, 0);  // Unit vectors along Y
         * Vector3D v5(0, 0, 1), v6(0, 0, 1);  // Unit vectors along Z
         *
         * // Computes (v1·v2) + (v3·v4) + (v5·v6) = 1 + 1 + 1 = 3
         * // All calculations done in parallel using Saturn's MAC registers
         * Fxp result = Vector3D::MultiDotAccumulate(
         *     std::pair{v1, v2},
         *     std::pair{v3, v4},
         *     std::pair{v5, v6}
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
         * @brief Calculate the cross product of this object and another Vec3 object.
         * @param vec The Vec3 object to calculate the cross product with.
         * @return The cross product as an Vec3 object.
         */
        constexpr Vector3D Cross(const Vector3D& vec) const
        {
            return Vector3D(
                Z * vec.Y - Y * vec.Z,
                X * vec.Z - Z * vec.X,
                Y * vec.X - X * vec.Y
            );
        }

        /**
         * @brief Calculate the squared length of the vector.
         * @return The squared length as an Fxp value.
         * @details Useful for comparisons where the actual length is not needed.
         * 
         * Example usage:
         * @code
         * Vector3D v1(1, 2, 3);
         * Vector3D v2(4, 6, 8);
         * if (v1.LengthSquared() < v2.LengthSquared()) {
         *     // v1 is shorter than v2
         * }
         * @endcode
         */
        constexpr Fxp LengthSquared() const {
            return Dot(*this);
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
                // Use alpha-beta-gamma coefficients for fastest approximation
                constexpr Vector3D alphaBetaGamma(
                    0.9398086351723256,  // Alpha
                    0.38928148272372454, // Beta
                    0.2987061876143797   // Gamma
                );
                return alphaBetaGamma.Dot(Abs().Sort<SortOrder::Descending>());
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
        constexpr Vector3D Normalize() const
        {
            Fxp length = Length<P>();
            if (length != 0)
                return Vector3D(X / length, Y / length, Z / length);
            return Vector3D();
        }

        /**
         * @brief Calculate normal vector for a triangle
         * @tparam P Precision level for calculation
         * @param vertexA First vertex of the triangle
         * @param vertexB Second vertex of the triangle
         * @param vertexC Third vertex of the triangle
         * @return Normalized normal vector
         */
        template<Precision P = Precision::Standard>
        static Vector3D CalcNormal(
            const Vector3D& vertexA,
            const Vector3D& vertexB,
            const Vector3D& vertexC)
        {
            const Vector3D edge1 = vertexB - vertexA;
            const Vector3D edge2 = vertexC - vertexA;
            return edge1.Cross(edge2).Normalize<P>();
        }

        /**
         * @brief Calculate the Euclidean distance from this vector to another vector.
         * @param other The other vector to calculate the distance to.
         * @return The distance as an Fxp value.
         * @details Computes the distance using the formula: sqrt((X - other.X)^2 + (Y - other.Y)^2 + (Z - other.Z)^2).
         * 
         * Example usage:
         * @code
         * Vector3D v1(1, 2, 3);
         * Vector3D v2(4, 6, 8);
         * Fxp distance = v1.DistanceTo(v2); // Computes distance between (1, 2, 3) and (4, 6, 8)
         * @endcode
         */
        constexpr Fxp DistanceTo(const Vector3D& other) const {
            return (*this - other).Length();
        }

        // Scalar multiplication and division
        /**
         * @brief Compound multiplication assignment operator.
         * @param scalar The scalar value to multiply by.
         * @return Reference to the modified Vec3 object.
         */
        constexpr Vector3D& operator*=(const Fxp& scalar)
        {
            Vector2D::operator*=(scalar);
            Z *= scalar;
            return *this;
        }

        /**
         * @brief Divide each coordinate by a scalar value.
         * @param fxp The scalar value to divide by.
         * @return The resulting Vec3 object.
         */
        constexpr Vector3D operator/=(const Fxp& fxp)
        {
            Vector2D::operator*=(fxp);
            Z *= fxp;
            return *this;
        }

        /**
         * @brief Multiply each coordinate by a scalar value.
         * @param scalar The scalar value to multiply by.
         * @return The resulting Vec3 object.
         */
        constexpr Vector3D operator*(const Fxp& scalar) const
        {
            Vector3D result(*this);
            result *= scalar;
            return result;
        }

        /**
         * @brief Divide each coordinate by a scalar value.
         * @param fxp The scalar value to divide by.
         * @return The resulting Vec3 object.
         */
        constexpr Vector3D operator/(const Fxp& fxp) const
        {
            return Vector3D(Vector2D::operator/(fxp), Z / fxp);
        }

        // Unit vector and common vector constant methods
        /**
         * @brief Get a unit vector pointing along the X axis (1,0,0).
         * @return Unit vector along X axis.
         */
        static consteval Vector3D UnitX()
        {
            return Vector3D(1, 0, 0);
        }

        /**
         * @brief Get a unit vector pointing along the Y axis (0,1,0).
         * @return Unit vector along Y axis.
         */
        static consteval Vector3D UnitY()
        {
            return Vector3D(0, 1, 0);
        }

        /**
         * @brief Get a unit vector pointing along the Z axis (0,0,1).
         * @return Unit vector along Z axis.
         */
        static consteval Vector3D UnitZ()
        {
            return Vector3D(0, 0, 1);
        }

        /**
         * @brief Get a zero vector (0,0,0).
         * @return Zero vector.
         */
        static consteval Vector3D Zero()
        {
            return Vector3D(0);
        }

        /**
         * @brief Get a vector with all components set to one (1,1,1).
         * @return Vector with all ones.
         */
        static consteval Vector3D One()
        {
            return Vector3D(1);
        }

        // Comparison operators
        /**
         * @brief Check if two Vec3 objects are not equal.
         * @param vec The Vec3 object to compare.
         * @return True if not equal, false otherwise.
         */
        constexpr bool operator!=(const Vector3D& vec) const
        {
            return X != vec.X && Y != vec.Y && Z != vec.Z;
        }

        /**
         * @brief Check if two Vec3 objects are not equal.
         * @param vec The Vec3 object to compare.
         * @return True if not equal, false otherwise.
         */
        constexpr bool operator==(const Vector3D& vec) const
        {
            return X == vec.X && Y == vec.Y && Z == vec.Z;
        }

        // Bitwise shift operators
        /**
         * @brief Bitwise right shift operator.
         * @param shiftAmount The number of positions to shift.
         * @return The resulting Vec3 object.
         */
        constexpr Vector3D operator>>(const size_t& shiftAmount)
        {
            return Vector3D(X >> shiftAmount, Y >> shiftAmount, Z >> shiftAmount);
        }

        /**
         * @brief Bitwise right shift assignment operator.
         * @param shiftAmount The number of positions to shift.
         * @return Reference to the modified Vec3 object.
         */
        constexpr Vector3D& operator>>=(const size_t& shiftAmount)
        {
            X >>= shiftAmount;
            Y >>= shiftAmount;
            Z >>= shiftAmount;
            return *this;
        }

        /**
         * @brief Bitwise left shift operator.
         * @param shiftAmount The number of positions to shift.
         * @return The resulting Vec3 object.
         */
        constexpr Vector3D operator<<(const size_t& shiftAmount)
        {
            return Vector3D(X << shiftAmount, Y << shiftAmount, Z << shiftAmount);
        }

        /**
         * @brief Bitwise left shift assignment operator.
         * @param shiftAmount The number of positions to shift.
         * @return Reference to the modified Vec3 object.
         */
        constexpr Vector3D& operator<<=(const size_t& shiftAmount)
        {
            X <<= shiftAmount;
            Y <<= shiftAmount;
            Z <<= shiftAmount;
            return *this;
        }

        // Unary operators
        /**
         * @brief Unary negation operator.
         * @return A new Vec3 object with negated coordinates.
         */
        constexpr Vector3D operator-() const
        {
            return Vector3D(-X, -Y, -Z);
        }

        // Binary operators
        /**
         * @brief Binary addition operator.
         * @param vec The Vec3 object to add.
         * @return The sum as an Vec3 object.
         */
        constexpr Vector3D operator+(const Vector3D& vec) const
        {
            return Vector3D(Vector2D::operator+(vec), Z + vec.Z);
        }

        /**
         * @brief Binary subtraction operator.
         * @param vec The Vec3 object to subtract.
         * @return The difference as an Vec3 object.
         */
        constexpr Vector3D operator-(const Vector3D& vec) const
        {
            return Vector3D(Vector2D::operator-(vec), Z - vec.Z);
        }

        /**
         * @brief Compound addition assignment operator.
         * @param vec The Vec3 object to add.
         * @return Reference to the modified Vec3 object.
         */
        constexpr Vector3D operator+=(const Vector3D& vec)
        {
            X += vec.X;
            Y += vec.Y;
            Z += vec.Z;
            return *this;
        }

        /**
         * @brief Compound subtraction assignment operator.
         * @param vec The Vec3 object to subtract.
         * @return Reference to the modified Vec3 object.
         */
        constexpr Vector3D operator-=(const Vector3D& vec)
        {
            X -= vec.X;
            Y -= vec.Y;
            Z -= vec.Z;
            return *this;
        }
    };
}