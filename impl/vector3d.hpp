#pragma once

#include <utility>
#include "vector2d.hpp"
#include "precision.hpp"
#include "sort_order.hpp"

namespace SaturnMath::Types
{
    /**
     * @brief A high-performance three-dimensional vector implementation optimized for Saturn hardware.
     * 
     * @details Vector3D extends Vector2D to provide comprehensive 3D vector operations using
     * fixed-point arithmetic. It inherits all 2D functionality while adding Z-axis operations
     * and 3D-specific algorithms optimized for performance-critical graphics and physics calculations.
     * 
     * Key features:
     * - Memory-efficient representation (three Fxp values)
     * - Comprehensive set of 3D vector operations (cross product, normalization, etc.)
     * - Multiple precision levels for performance-critical operations
     * - Hardware-optimized calculations for Saturn platform
     * - Inheritance from Vector2D for seamless 2D/3D interoperability
     * 
     * Common applications:
     * - 3D positions and translations
     * - Surface normals and orientation vectors
     * - Physics simulations (velocities, forces, torques)
     * - Camera and view transformations
     * - Lighting calculations
     * 
     * The implementation follows a right-handed coordinate system where:
     * - Positive X points right
     * - Positive Y points up
     * - Positive Z points toward the viewer (out of the screen)
     * 
     * Performance considerations:
     * - Most operations are constexpr and can be evaluated at compile time
     * - Precision template parameters allow trading accuracy for speed
     * - Fixed-point arithmetic avoids expensive floating-point operations
     * - Specialized implementations for common operations (unit vectors, etc.)
     * 
     * @note When working with operations that require normalization (like calculating
     * normals), be aware of the performance implications and consider using the
     * appropriate precision level based on your requirements.
     * 
     * @see Vector2D For 2D vector operations
     * @see Fxp For details on the fixed-point implementation
     * @see Precision For available precision levels in calculations
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
         * 
         * @details Computes the cross product, which results in a vector perpendicular 
         * to both input vectors. The magnitude of the result equals the area of the 
         * parallelogram spanned by the input vectors.
         * 
         * Example usage:
         * @code
         * Vector3D v1(1, 0, 0);  // Unit vector along X
         * Vector3D v2(0, 1, 0);  // Unit vector along Y
         * Vector3D cross = v1.Cross(v2);  // Returns (0, 0, 1) - unit vector along Z
         * @endcode
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
         * 
         * @details Calculates the magnitude (Euclidean length) of the vector.
         * The precision template parameter controls the calculation method:
         * - Standard precision: Uses exact square root calculation
         * - Turbo precision: Uses fast approximation with alpha-beta-gamma coefficients
         * 
         * For large vector components that might cause overflow in dot product calculation,
         * a specialized bit-shift-friendly approximation is used regardless of precision level.
         * 
         * Example usage:
         * @code
         * Vector3D v(3, 4, 5);
         * Fxp length = v.Length();  // Returns sqrt(50) with standard precision
         * Fxp fastLength = v.Length<Precision::Turbo>();  // Returns approximate length (faster)
         * @endcode
         */
        template<Precision P = Precision::Default>
        constexpr Fxp Length() const
        {
            // More accurate threshold based on Fxp range
            constexpr Fxp overflowThreshold = 100.0;
            bool potentialOverflow = (X.Abs() > overflowThreshold || Y.Abs() > overflowThreshold || Z.Abs() > overflowThreshold);

            if (potentialOverflow)
            {
                // For large values, use approximation regardless of precision setting
                Vector3D sorted = Abs().Sort<SortOrder::Descending>();

                // Alpha is close to 1, beta is ~0.4, gamma is ~0.25
                // These coefficients give good accuracy and can be calculated with shifts/adds
                Fxp alpha = sorted.X;

                // beta = 0.375 = 3/8 = 1/2 - 1/8
                Fxp beta = (sorted.Y >> 1) - (sorted.Y >> 3);

                // gamma = 0.25 = 1/4
                Fxp gamma = sorted.Z >> 2;

                return alpha + beta + gamma;
            }
            else
            {
                // For smaller values where overflow isn't a concern:
                if constexpr (P == Precision::Turbo)
                {
                    // Use existing fast approximation for small values
                    constexpr Vector3D alphaBetaGamma(
                        0.9398086351723256,
                        0.38928148272372454,
                        0.2987061876143797);
                    return alphaBetaGamma.Dot(Abs().Sort<SortOrder::Descending>());
                }
                else
                {
                    // Use accurate calculation for small values
                    return Dot(*this).Sqrt<P>();
                }
            }
        }

        /**
         * @brief Normalize the vector
         * @tparam P Precision level for calculation
         * @return Normalized vector
         * 
         * @details Creates a unit vector pointing in the same direction as this vector.
         * The precision template parameter controls the length calculation method:
         * - Standard precision: Uses exact square root calculation
         * - Turbo precision: Uses fast approximation with alpha-beta-gamma coefficients
         * 
         * If the vector length is zero, returns a zero vector to avoid division by zero.
         * 
         * Example usage:
         * @code
         * Vector3D v(3, 4, 5);
         * Vector3D unitV = v.Normalize();  // Returns unit vector with standard precision
         * Vector3D fastUnitV = v.Normalize<Precision::Turbo>();  // Returns approximate unit vector (faster)
         * @endcode
         */
        template<Precision P = Precision::Default>
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
         * 
         * @details Calculates the normalized normal vector to a triangle defined by three vertices.
         * The normal is computed using the cross product of two edges of the triangle.
         * The precision template parameter controls the normalization calculation method.
         * 
         * Example usage:
         * @code
         * Vector3D v1(0, 0, 0);
         * Vector3D v2(1, 0, 0);
         * Vector3D v3(0, 1, 0);
         * Vector3D normal = Vector3D::CalcNormal(v1, v2, v3);  // Returns (0, 0, 1)
         * @endcode
         */
        template<Precision P = Precision::Default>
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
         * @tparam P Precision level for calculation
         * @param other The other vector to calculate the distance to.
         * @return The distance as an Fxp value.
         * @details Computes the distance using the formula: sqrt((X - other.X)^2 + (Y - other.Y)^2 + (Z - other.Z)^2).
         * The precision template parameter controls the calculation method:
         * - Standard precision: Uses exact square root calculation
         * - Turbo precision: Uses fast approximation
         * 
         * Example usage:
         * @code
         * Vector3D v1(1, 2, 3);
         * Vector3D v2(4, 6, 8);
         * Fxp distance = v1.DistanceTo(v2);  // Computes distance between the two points
         * Fxp fastDistance = v1.DistanceTo<Precision::Turbo>(v2);  // Computes approximate distance (faster)
         * @endcode
         */
        template<Precision P = Precision::Default>
        constexpr Fxp DistanceTo(const Vector3D& other) const {
            return (*this - other).Length<P>();
        }

        // Scalar multiplication and division
        /**
         * @brief Compound multiplication assignment operator.
         * @param scalar The scalar value to multiply by.
         * @return Reference to the modified Vec3 object.
         * 
         * @details Multiplies each component of the vector by the scalar value.
         * 
         * Example usage:
         * @code
         * Vector3D v(1, 2, 3);
         * v *= 2.5_fxp;  // Results in v = (2.5, 5, 7.5)
         * @endcode
         */
        constexpr Vector3D& operator*=(const Fxp& scalar)
        {
            Vector2D::operator*=(scalar);
            Z *= scalar;
            return *this;
        }

        /**
         * @brief Compound multiplication assignment operator for integral types.
         * @tparam T The integral type of the scalar value.
         * @param scalar The scalar value to multiply by.
         * @return Reference to the modified Vec3 object.
         * 
         * @details Multiplies each component of the vector by the integral scalar value.
         * This specialized version uses Fxp's optimized integral multiplication
         * for better performance on Saturn hardware.
         * 
         * Example usage:
         * @code
         * Vector3D v(1, 2, 3);
         * v *= 2;  // Results in v = (2, 4, 6) with optimized integral multiplication
         * @endcode
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr Vector3D& operator*=(const T& scalar)
        {
            Vector2D::operator*=(scalar);
            Z *= scalar;
            return *this;
        }

        /**
         * @brief Compound division assignment operator.
         * @param scalar The scalar value to divide by.
         * @return Reference to the modified Vec3 object.
         * 
         * @details Divides each component of the vector by the scalar value.
         * 
         * Example usage:
         * @code
         * Vector3D v(4, 6, 8);
         * v /= 2_fxp;  // Results in v = (2, 3, 4)
         * @endcode
         */
        constexpr Vector3D& operator/=(const Fxp& scalar)
        {
            Vector2D::operator/=(scalar);
            Z /= scalar;
            return *this;
        }

        /**
         * @brief Compound division assignment operator for integral types.
         * @tparam T The integral type of the scalar value.
         * @param scalar The scalar value to divide by.
         * @return Reference to the modified Vec3 object.
         * 
         * @details Divides each component of the vector by the integral scalar value.
         * This specialized version uses Fxp's optimized integral division
         * for better performance on Saturn hardware.
         * 
         * Example usage:
         * @code
         * Vector3D v(10, 20, 30);
         * v /= 5;  // Results in v = (2, 4, 6) with optimized integral division
         * @endcode
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr Vector3D& operator/=(const T& scalar)
        {
            Vector2D::operator/=(scalar);
            Z /= scalar;
            return *this;
        }

        /**
         * @brief Scalar multiplication operator.
         * @param scalar The scalar value to multiply by.
         * @return A new vector with each component multiplied by the scalar.
         * 
         * @details Creates a new vector by multiplying each component of this vector
         * by the scalar value.
         * 
         * Example usage:
         * @code
         * Vector3D v(1, 2, 3);
         * Vector3D result = v * 3_fxp;  // Results in result = (3, 6, 9)
         * @endcode
         */
        constexpr Vector3D operator*(const Fxp& scalar) const
        {
            Vector3D result(*this);
            result *= scalar;
            return result;
        }

        /**
         * @brief Scalar multiplication operator for integral types.
         * @tparam T The integral type of the scalar value.
         * @param scalar The scalar value to multiply by.
         * @return A new vector with each component multiplied by the scalar.
         * 
         * @details Creates a new vector by multiplying each component of this vector
         * by the integral scalar value. This specialized version uses Fxp's optimized 
         * integral multiplication for better performance on Saturn hardware.
         * 
         * Example usage:
         * @code
         * Vector3D v(1, 2, 3);
         * Vector3D result = v * 3;  // Results in result = (3, 6, 9) with optimized integral multiplication
         * @endcode
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr Vector3D operator*(const T& scalar) const
        {
            Vector3D result(*this);
            result *= scalar;
            return result;
        }

        /**
         * @brief Multiply an integral scalar by a Vector3D.
         * @tparam T The integral type of the scalar value.
         * @param scalar The scalar value to multiply.
         * @param vec The vector to multiply by.
         * @return A new vector with each component multiplied by the scalar.
         * 
         * @details Creates a new vector by multiplying each component of the input vector
         * by the integral scalar value. This specialized version uses Fxp's optimized 
         * integral multiplication for better performance on Saturn hardware.
         * 
         * Example usage:
         * @code
         * Vector3D v(1, 2, 3);
         * Vector3D result = 3 * v;  // Results in result = (3, 6, 9) with optimized integral multiplication
         * @endcode
         */
        template<typename T>
            requires std::is_integral_v<T>
        friend constexpr Vector3D operator*(const T& scalar, const Vector3D& vec)
        {
            return vec * scalar;
        }

        /**
         * @brief Scalar division operator.
         * @param scalar The scalar value to divide by.
         * @return A new vector with each component divided by the scalar.
         * 
         * @details Creates a new vector by dividing each component of this vector
         * by the scalar value.
         * 
         * Example usage:
         * @code
         * Vector3D v(6, 8, 10);
         * Vector3D result = v / 2_fxp;  // Results in result = (3, 4, 5)
         * @endcode
         */
        constexpr Vector3D operator/(const Fxp& scalar) const
        {
            return Vector3D(Vector2D::operator/(scalar), Z / scalar);
        }

        /**
         * @brief Scalar division operator for integral types.
         * @tparam T The integral type of the scalar value.
         * @param scalar The scalar value to divide by.
         * @return A new vector with each component divided by the scalar.
         * 
         * @details Creates a new vector by dividing each component of this vector
         * by the integral scalar value. This specialized version uses Fxp's optimized 
         * integral division for better performance on Saturn hardware.
         * 
         * Example usage:
         * @code
         * Vector3D v(10, 20, 30);
         * Vector3D result = v / 5;  // Results in result = (2, 4, 6) with optimized integral division
         * @endcode
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr Vector3D operator/(const T& scalar) const
        {
            Vector3D result(*this);
            result /= scalar;
            return result;
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
            return X != vec.X || Y != vec.Y || Z != vec.Z;
        }

        /**
         * @brief Check if two Vec3 objects are equal.
         * @param vec The Vec3 object to compare.
         * @return True if equal, false otherwise.
         */
        constexpr bool operator==(const Vector3D& vec) const
        {
            return X == vec.X && Y == vec.Y && Z == vec.Z;
        }

        /**
         * @brief Less than operator.
         * @param vec The Vector3D object to compare with.
         * @return True if this vector is less than the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y, then Z).
         */
        constexpr bool operator<(const Vector3D& vec) const
        {
            return Vector2D::operator<(vec) || 
                   (Vector2D::operator==(vec) && Z < vec.Z);
        }

        /**
         * @brief Less than or equal operator.
         * @param vec The Vector3D object to compare with.
         * @return True if this vector is less than or equal to the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y, then Z).
         */
        constexpr bool operator<=(const Vector3D& vec) const
        {
            return Vector2D::operator<=(vec) || 
                   (Vector2D::operator==(vec) && Z <= vec.Z);
        }

        /**
         * @brief Greater than operator.
         * @param vec The Vector3D object to compare with.
         * @return True if this vector is greater than the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y, then Z).
         */
        constexpr bool operator>(const Vector3D& vec) const
        {
            return Vector2D::operator>(vec) || 
                   (Vector2D::operator==(vec) && Z > vec.Z);
        }

        /**
         * @brief Greater than or equal operator.
         * @param vec The Vector3D object to compare with.
         * @return True if this vector is greater than or equal to the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y, then Z).
         */
        constexpr bool operator>=(const Vector3D& vec) const
        {
            return Vector2D::operator>=(vec) || 
                   (Vector2D::operator==(vec) && Z >= vec.Z);
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
         * @return The sum as a Vec3 object.
         */
        constexpr Vector3D operator+(const Vector3D& vec) const
        {
            return Vector3D(X + vec.X, Y + vec.Y, Z + vec.Z);
        }

        /**
         * @brief Binary addition operator for adding a Vector2D to a Vector3D.
         * @param vec The Vec2 object to add.
         * @return The sum as a Vec3 object.
         */
        constexpr Vector3D operator+(const Vector2D& vec) const {
            return Vector3D(X + vec.X, Y + vec.Y, Z);
        }

        /**
         * @brief Binary addition operator for adding an Fxp to a Vector3D.
         * @param scalar The Fxp value to add.
         * @return The resulting Vector3D object.
         */
        constexpr Vector3D operator+(const Fxp& scalar) const {
            return Vector3D(X + scalar, Y + scalar, Z + scalar);
        }

        /**
         * @brief Binary subtraction operator.
         * @param vec The Vec3 object to subtract.
         * @return The difference as a Vec3 object.
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