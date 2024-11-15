#pragma once

#include "vector2d.hpp"

namespace SaturnMath
{
    /**
     * @brief A struct for three-dimensional fixed-point vector arithmetic operations.
     */
    struct Vector3D : public Vector2D
    {
        Fxp z; /**< The Z-coordinate. */

        // Constructors
        /**
         * @brief Default constructor, initializes all coordinates to 0.
         */
        constexpr Vector3D() : Vector2D(), z() {}

        /**
         * @brief Constructor to initialize all coordinates with the same value.
         * @param fxp The value to initialize all coordinates with.
         */
        constexpr Vector3D(const Fxp& fxp) : Vector2D(fxp), z(fxp) {}

        /**
         * @brief Copy constructor.
         * @param vec The Vec3 object to copy.
         */
        constexpr Vector3D(const Vector3D& vec) : Vector2D(vec), z(vec.z) {}

        /**
         * @brief Constructor to initialize coordinates with specific values.
         * @param valueX The X-coordinate.
         * @param valueY The Y-coordinate.
         * @param valueZ The Z-coordinate.
         */
        constexpr Vector3D(const Fxp& valueX, const Fxp& valueY, const Fxp& valueZ) : Vector2D(valueX, valueY), z(valueZ) {}

        // Assignment operator
        /**
         * @brief Assignment operator.
         * @param vec The Vec3 object to assign.
         * @return Reference to the modified Vec3 object.
         */
        constexpr Vector3D& operator=(const Vector3D& vec)
        {
            Vector2D::operator=(vec);
            z = vec.z;
            return *this;
        }

        // Other member functions
        /**
         * @brief Calculate the absolute values of each coordinate.
         * @return A new Vec3 object with absolute values.
         */
        constexpr Vector3D Abs() const
        {
            return Vector3D(Vector2D::Abs(), z.Abs());
        }

        /**
         * @brief Sort coordinates in ascending order.
         * @tparam Ascending If true, sort in ascending order; otherwise, in descending order.
         * @return A new Vec3 object with sorted coordinates.
         */
        template <bool Ascending = true>
        constexpr Vector3D Sort()
        {
            Vector3D result(*this);
            Fxp temp;

            if (Ascending ? result.x > result.y : result.x < result.y)
            {
                temp = result.x;
                result.x = result.y;
                result.y = temp;
            }

            if (Ascending ? result.x > result.z : result.x < result.z)
            {
                temp = result.x;
                result.x = result.z;
                result.z = temp;
            }

            if (Ascending ? result.y > result.z : result.y < result.z)
            {
                temp = result.y;
                result.y = result.z;
                result.z = temp;
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
                z * vec.y - y * vec.z,
                x * vec.z - z * vec.x,
                y * vec.x - x * vec.y
            );
        }

        // Scalar multiplication and division
        /**
         * @brief Multiply each coordinate by a scalar value.
         * @param fxp The scalar value to multiply by.
         * @return The resulting Vec3 object.
         */
        constexpr Vector3D operator*(const Fxp& fxp) const
        {
            return Vector3D(Vector2D::operator*(fxp), z * fxp);
        }

        /**
         * @brief Divide each coordinate by a scalar value.
         * @param fxp The scalar value to divide by.
         * @return The resulting Vec3 object.
         */
        constexpr Vector3D operator/(const Fxp& fxp) const
        {
            return Vector3D(Vector2D::operator/(fxp), z / fxp);
        }

        // Vector length calculation
        /**
         * @brief Calculate the length of the vector represented by this Vec3 object.
         * @return The length as an Fxp value.
         */
        constexpr Fxp CalcLength() const
        {
            return Dot(*this).Sqrt();
        }

        /**
         * @brief Calculate the length of the vector using fast approximation.
         * Good for real-time calculations where precision isn't critical.
         * @return The approximate length as an Fxp value.
         */
        constexpr Fxp FastCalcLength() const
        {
            return Dot(*this).FastSqrt();
        }

        /**
         * @brief Calculate the length of the vector using fastest approximation.
         * Best for particle effects and non-critical calculations.
         * @return The approximate length as an Fxp value.
         */
        constexpr Fxp TurboCalcLength() const
        {
            constexpr Vector3D alphaBetaGama =
                Vector3D
                (
                    0.9398086351723256, // Alpha
                    0.38928148272372454, // Beta
                    0.2987061876143797   // Gamma
                );

            return Abs().Sort<false>().Dot(alphaBetaGama);
        }

        // Vector normalization
        /**
         * @brief Normalize the vector using standard precision.
         * Ideal for important calculations requiring accuracy.
         * @return A normalized Vec3 object.
         */
        constexpr Vector3D Normalize() const
        {
            Fxp length = CalcLength();
            if (length != 0.0)
                return Vector3D(x / length, y / length, z / length);
            else
                return Vector3D();
        }

        /**
         * @brief Normalize the vector using fast approximation.
         * Uses reciprocal approximation to avoid division.
         * Good for real-time updates where slight imprecision is acceptable.
         * @return A normalized Vec3 object.
         */
        constexpr Vector3D FastNormalize() const
        {
            Fxp length = FastCalcLength();
            if (length != 0.0F)
                return Vector3D(length / x, length / y, length / z);
            else
                return Vector3D();
        }

        /**
         * @brief Normalize the vector using fastest approximation.
         * Uses alpha/beta/gamma coefficients and no division.
         * Best for particle effects and non-critical calculations.
         * @return A normalized Vec3 object.
         */
        constexpr Vector3D TurboNormalize() const
        {
            Fxp length = TurboCalcLength();
            if (length != 0.0F)
                return Vector3D(length / x, length / y, length / z);
            else
                return Vector3D();
        }

        /**
         * @brief Calculate normal vector with standard precision.
         * Ideal for static geometry and important surface calculations.
         * @param vertexA First vertex of the triangle.
         * @param vertexB Second vertex of the triangle.
         * @param vertexC Third vertex of the triangle.
         * @return Normalized normal vector.
         */
        static Vector3D CalcNormal(
            const Vector3D& vertexA,
            const Vector3D& vertexB,
            const Vector3D& vertexC)
        {
            const Vector3D edge1 = vertexB - vertexA;
            const Vector3D edge2 = vertexC - vertexA;
            return edge1.Cross(edge2).Normalize();
        }

        /**
         * @brief Calculate normal vector with fast approximation.
         * Good for dynamic geometry and real-time mesh deformation.
         * @param vertexA First vertex of the triangle.
         * @param vertexB Second vertex of the triangle.
         * @param vertexC Third vertex of the triangle.
         * @return Fast-normalized normal vector.
         */
        static Vector3D FastCalcNormal(
            const Vector3D& vertexA,
            const Vector3D& vertexB,
            const Vector3D& vertexC)
        {
            const Vector3D edge1 = vertexB - vertexA;
            const Vector3D edge2 = vertexC - vertexA;
            return edge1.Cross(edge2).FastNormalize();
        }

        /**
         * @brief Calculate normal vector with fastest approximation.
         * Best for particle effects or temporary visual effects.
         * @param vertexA First vertex of the triangle.
         * @param vertexB Second vertex of the triangle.
         * @param vertexC Third vertex of the triangle.
         * @return Turbo-normalized normal vector.
         */
        static Vector3D TurboCalcNormal(
            const Vector3D& vertexA,
            const Vector3D& vertexB,
            const Vector3D& vertexC)
        {
            const Vector3D edge1 = vertexB - vertexA;
            const Vector3D edge2 = vertexC - vertexA;
            return edge1.Cross(edge2).TurboNormalize();
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