#pragma once

#include "vector2d.hpp"
#include "precision.hpp"
#include "sort_order.hpp"

namespace SaturnMath
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
         * @brief Calculate the dot product of this object and another Vec3 object.
         * @param vec The Vec3 object to calculate the dot product with.
         * @return The dot product as an Fxp value.
         */
        constexpr Fxp Dot(const Vector3D& vec) const
        {
            if consteval
            {
                return x * vec.x + y * vec.y + z * vec.z;
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
                return Abs().Sort<SortOrder::Descending>().Dot(alphaBetaGamma);
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

        // Scalar multiplication and division
        /**
         * @brief Multiply each coordinate by a scalar value.
         * @param fxp The scalar value to multiply by.
         * @return The resulting Vec3 object.
         */
        constexpr Vector3D operator*(const Fxp& fxp) const
        {
            return Vector3D(Vector2D::operator*(fxp), Z * fxp);
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
            return x != vec.x && y != vec.y && z != vec.z;
        }

        // Bitwise shift operators
        /**
         * @brief Bitwise right shift operator.
         * @param shiftAmount The number of positions to shift.
         * @return The resulting Vec3 object.
         */
        constexpr Vector3D operator>>(const size_t& shiftAmount)
        {
            return Vector3D(x >> shiftAmount, y >> shiftAmount, z >> shiftAmount);
        }

        /**
         * @brief Bitwise right shift assignment operator.
         * @param shiftAmount The number of positions to shift.
         * @return Reference to the modified Vec3 object.
         */
        constexpr Vector3D& operator>>=(const size_t& shiftAmount)
        {
            x >>= shiftAmount;
            y >>= shiftAmount;
            z >>= shiftAmount;
            return *this;
        }

        /**
         * @brief Bitwise left shift operator.
         * @param shiftAmount The number of positions to shift.
         * @return The resulting Vec3 object.
         */
        constexpr Vector3D operator<<(const size_t& shiftAmount)
        {
            return Vector3D(x << shiftAmount, y << shiftAmount, z << shiftAmount);
        }

        /**
         * @brief Bitwise left shift assignment operator.
         * @param shiftAmount The number of positions to shift.
         * @return Reference to the modified Vec3 object.
         */
        constexpr Vector3D& operator<<=(const size_t& shiftAmount)
        {
            x <<= shiftAmount;
            y <<= shiftAmount;
            z <<= shiftAmount;
            return *this;
        }

        // Unary operators
        /**
         * @brief Unary negation operator.
         * @return A new Vec3 object with negated coordinates.
         */
        constexpr Vector3D operator-() const
        {
            return Vector3D(-x, -y, -z);
        }

        // Binary operators
        /**
         * @brief Binary addition operator.
         * @param vec The Vec3 object to add.
         * @return The sum as an Vec3 object.
         */
        constexpr Vector3D operator+(const Vector3D& vec) const
        {
            return Vector3D(Vector2D::operator+(vec), z + vec.z);
        }

        /**
         * @brief Binary subtraction operator.
         * @param vec The Vec3 object to subtract.
         * @return The difference as an Vec3 object.
         */
        constexpr Vector3D operator-(const Vector3D& vec) const
        {
            return Vector3D(Vector2D::operator-(vec), z - vec.z);
        }

        /**
         * @brief Compound addition assignment operator.
         * @param vec The Vec3 object to add.
         * @return Reference to the modified Vec3 object.
         */
        constexpr Vector3D operator+=(const Vector3D& vec)
        {
            x += vec.x;
            y += vec.y;
            z += vec.z;
            return *this;
        }

        /**
         * @brief Compound subtraction assignment operator.
         * @param vec The Vec3 object to subtract.
         * @return Reference to the modified Vec3 object.
         */
        constexpr Vector3D operator-=(const Vector3D& vec)
        {
            x -= vec.x;
            y -= vec.y;
            z -= vec.z;
            return *this;
        }
    };
}