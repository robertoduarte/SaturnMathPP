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
        constexpr Vector3D Sort() const
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
        /**
         * @brief Calculate the cross product of this vector and another vector.
         * @param vec The vector to calculate the cross product with.
         * @return A new vector perpendicular to both input vectors.
         * 
         * @details The cross product follows the right-hand rule and returns a vector 
         * that is perpendicular to both input vectors. The magnitude of the resulting 
         * vector equals the area of the parallelogram formed by the two input vectors.
         * 
         * The cross product is calculated as:
         * cross.X = Y * vec.Z - Z * vec.Y
         * cross.Y = Z * vec.X - X * vec.Z
         * cross.Z = X * vec.Y - Y * vec.X
         * 
         * Example usage:
         * @code
         * Vector3D a(1, 0, 0);  // Unit vector along X
         * Vector3D b(0, 1, 0);  // Unit vector along Y
         * Vector3D cross = a.Cross(b);  // Returns (0, 0, 1) - unit vector along Z
         * @endcode
         */
        constexpr Vector3D Cross(const Vector3D& vec) const
        {
            return Vector3D(
                Y * vec.Z - Z * vec.Y,  // X component
                Z * vec.X - X * vec.Z,  // Y component
                X * vec.Y - Y * vec.X   // Z component
            );
        }

        /**
         * @brief Calculate the squared length of the vector with overflow protection.
         * @return The squared length as an Fxp value, or MaxValue() if the result would overflow.
         * @details Returns Fxp::MaxValue() if the squared magnitude would be too large to represent.
         *          This version includes overflow protection to ensure safe calculations.
         *          
         *          The method checks for potential overflow by comparing against a safe threshold
         *          that ensures the sum of squares won't exceed the maximum representable value.
         *          For 3D vectors, we use a more conservative threshold than 2D to account for
         *          the additional component.
         * 
         * Example usage:
         * @code
         * Vector3D v(3, 4, 5);
         * Fxp lenSq = v.LengthSquared();  // Returns 50 (3*3 + 4*4 + 5*5)
         * 
         * // With large values that would overflow:
         * Vector3D large(1000, 1000, 1000);
         * Fxp largeLenSq = large.LengthSquared();  // Returns Fxp::MaxValue()
         * @endcode
         */
        constexpr Fxp LengthSquared() const {
            // Special case: if any component is MinValue, the square would be MaxValue
            if (X == Fxp::MinValue() || Y == Fxp::MinValue() || Z == Fxp::MinValue()) {
                return Fxp::MaxValue();
            }
            
            // Get absolute values to handle negative numbers
            const Fxp absX = X.Abs();
            const Fxp absY = Y.Abs();
            const Fxp absZ = Z.Abs();
            
            // Calculate maximum possible value before overflow
            // For 16.16 fixed-point with 3 components, we need to be more conservative
            // sqrt(2^31 / 3) ≈ 1193.2, but we use a safer threshold to account for
            // potential intermediate calculations and rounding
            constexpr Fxp maxSafeValue = 100.0;  // Conservative for 3D vectors
            
            // If any component is too large, return MaxValue to prevent overflow
            if (absX >= maxSafeValue || absY >= maxSafeValue || absZ >= maxSafeValue) {
                // For values just above the threshold, try scaling down to avoid false positives
                if (absX < 2 * maxSafeValue && absY < 2 * maxSafeValue && absZ < 2 * maxSafeValue) {
                    // Scale down by 2, calculate, then scale back up
                    const Vector3D scaled = *this >> 1;
                    return scaled.Dot(scaled) << 2;  // Multiply by 4 (2^2)
                }
                return Fxp::MaxValue();
            }
            
            // Safe to calculate normally
            return Dot(*this);
        }

        /**
         * @brief Calculate the length (magnitude) of the vector.
         * @tparam P Precision level for calculation (default: Precision::Default)
         * @return The length as an Fxp value.
         * 
         * @details Calculates the Euclidean length of the vector using the formula:
         *          sqrt(X² + Y² + Z²)
         * 
         * The precision template parameter controls the calculation method:
         * - Precision::Default: Uses exact square root calculation
         * - Precision::Turbo: Uses fast approximation with alpha-beta-gamma coefficients
         * 
         * The method includes overflow protection by scaling down large vectors
         * before calculation and scaling the result back up.
         * 
         * Example usage:
         * @code
         * Vector3D v(3, 4, 5);
         * Fxp len = v.Length();  // Standard precision
         * Fxp fastLen = v.Length<Precision::Turbo>();  // Faster approximation
         * @endcode
         */
        template<Precision P = Precision::Default>
        constexpr Fxp Length() const
        {
            // Use different thresholds based on precision mode
            constexpr Fxp overflowThreshold = (P == Precision::Turbo) ? 
                Fxp(400.0) :  // More permissive for Turbo mode
                Fxp(100.0);   // Conservative for other modes
                
            const Fxp absX = X.Abs();
            const Fxp absY = Y.Abs();
            const Fxp absZ = Z.Abs();
            constexpr bool potentialOverflow = (absX > overflowThreshold || 
                                         absY > overflowThreshold ||
                                         absZ > overflowThreshold);

            // Use a lambda to handle the calculation based on precision
            const auto calculateLength = [](const Vector3D& vec) -> Fxp {
                if constexpr (P == Precision::Turbo) {
                    // Use fast approximation for small values
                    constexpr Vector3D alphaBetaGamma(
                        0.96043387010342,  // Alpha
                        0.39782473475533,   // Beta
                        0.196034280659121   // Gamma
                    );
                    return alphaBetaGamma.Dot(vec.Abs().Sort<SortOrder::Descending>());
                } else {
                    // Use accurate calculation for small values
                    return vec.Dot(vec).Sqrt<P>();
                }
            };

            // Perform calculation with or without overflow protection
            if (potentialOverflow) {
                if constexpr (P == Precision::Turbo) {
                    // For Turbo mode, use less aggressive scaling since alpha-beta-gamma is more robust
                    const Vector3D scaledDown = *this >> 2;  // Divide by 4
                    return calculateLength(scaledDown) << 2;  // Multiply by 4
                } else {
                    // For other precision modes, use more aggressive scaling
                    const Vector3D scaledDown = *this >> 7;  // Divide by 128
                    return calculateLength(scaledDown) << 7;  // Multiply by 128
                }
            } else {
                return calculateLength(*this);
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
         * @brief Get a normalized copy of the vector
         * @tparam P Precision level for calculation
         * @return Normalized vector
         * 
         * @details Creates a unit vector pointing in the same direction as this vector.
         * Unlike Normalize(), this method returns a new vector without modifying the original.
         * 
         * Example usage:
         * @code
         * Vector3D v(3, 4, 5);
         * Vector3D unitV = v.Normalized();  // Original vector remains unchanged
         * @endcode
         */
        template<Precision P = Precision::Default>
        constexpr Vector3D Normalized() const
        {
            Vector3D copy(*this);
            return copy.Normalize<P>();
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

        /**
         * @brief Calculate the squared distance between this point and another 3D point.
         * @param other The other point to measure distance to.
         * @return The squared distance between the two points.
         * 
         * @details This method calculates the squared Euclidean distance between this 3D point
         * and another 3D point. This is more efficient than calculating the actual distance
         * as it avoids a square root operation. Useful for distance comparisons.
         * 
         * Example usage:
         * @code
         * Vector3D a(1, 2, 3);
         * Vector3D b(4, 6, 8);
         * Fxp distSq = a.DistanceSquared(b);  // Returns 50 (3² + 4² + 5²)
         * @endcode
         */
        constexpr Fxp DistanceSquared(const Vector3D& other) const {
            const Fxp dx = X - other.X;
            const Fxp dy = Y - other.Y;
            const Fxp dz = Z - other.Z;
            return dx * dx + dy * dy + dz * dz;
        }

        /**
         * @brief Calculate the angle between two vectors.
         * @tparam P Precision level for calculation
         * @param a First vector
         * @param b Second vector
         * @return Angle between the vectors as an Angle object
         * 
         * @details Calculates the smallest angle between two 3D vectors using the formula:
         * angle = atan2(||a × b||, a · b)
         * 
         * This method is more numerically stable than using acos and handles edge cases better.
         * It returns 0 if either vector has zero length.
         * 
         * The precision template parameter controls the calculation method:
         * - Precision::Default: Uses accurate but slower calculations
         * - Precision::Fast: Uses faster but less precise calculations
         * 
         * Example usage:
         * @code
         * Vector3D v1(1, 0, 0);  // Right vector
         * Vector3D v2(0, 1, 0);  // Up vector
         * Angle angle = Vector3D::Angle(v1, v2);  // Returns 90 degrees (π/2 radians)
         * @endcode
         */
        template<Precision P = Precision::Default>
        static constexpr auto Angle(const Vector3D& a, const Vector3D& b)
        {
            // Handle zero vectors
            const Fxp aLenSq = a.LengthSquared();
            const Fxp bLenSq = b.LengthSquared();
            
            if (aLenSq == 0 || bLenSq == 0) {
                return Angle::Zero();
            }
            
            // Calculate dot product and cross product magnitude squared
            const Fxp dot = a.Dot(b);
            const Vector3D cross = a.Cross(b);
            const Fxp crossLenSq = cross.LengthSquared();
            
            // Handle collinear vectors (cross product is zero)
            if (crossLenSq == 0) {
                // Vectors are in the same direction
                if (dot > 0) {
                    return Angle::Zero();
                }
                // Vectors are in opposite directions
                return Angle::Straight();
            }
            
            // Calculate the angle using atan2 for better numerical stability
            // atan2(||a × b||, a·b) = atan2(√(crossLenSq), dot)
            return Trigonometry::Atan2(crossLenSq.Sqrt(), dot);
        }

        /**
         * @brief Project this vector onto another vector.
         * @param other The vector to project onto.
         * @return The projection of this vector onto the other vector.
         * 
         * @details Calculates the projection of this vector onto another vector.
         * The projection represents the component of this vector that lies along
         * the direction of the other vector.
         * 
         * Formula: (this · other / other · other) * other
         * 
         * @note If the other vector is a zero vector, returns a zero vector.
         * 
         * Example usage:
         * @code
         * Vector3D v(2, 3, 0);
         * Vector3D u(1, 0, 0);
         * Vector3D proj = v.Project(u);  // Returns (2, 0, 0)
         * @endcode
         */
        constexpr Vector3D Project(const Vector3D& other) const
        {
            Fxp denominator = other.Dot(other);
            if (denominator == 0) return Vector3D(); // Avoid division by zero
            Fxp scalar = Dot(other) / denominator;
            return other * scalar;
        }

        /**
         * @brief Reflect this vector over a normal vector.
         * @param normal The normal vector to reflect over (should be normalized).
         * @return The reflected vector.
         * 
         * @details Calculates the reflection of this vector over a surface with the given normal.
         * The normal vector is assumed to be normalized.
         * 
         * Formula: v - 2 * (v · n) * n
         * 
         * @note For best results, the normal vector should be normalized.
         * 
         * Example usage:
         * @code
         * Vector3D v(1, -1, 0);
         * Vector3D n(0, 1, 0); // Up normal
         * Vector3D reflected = v.Reflect(n);  // Returns (1, 1, 0)
         * @endcode
         */
        constexpr Vector3D Reflect(const Vector3D& normal) const
        {
            // Standard reflection formula: v - 2*(v·n)*n
            // Where n is the normal vector (assumed to be normalized)
            // v·n = v.X*n.X + v.Y*n.Y + v.Z*n.Z
            Fxp dot = Dot(normal);
            return *this - normal * (dot * 2);
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

        /**
         * @brief Linear interpolation between two 3D vectors
         * @param start Starting vector
         * @param end Ending vector
         * @param t Interpolation factor [0,1]
         * @return Interpolated vector
         * 
         * @details Performs linear interpolation between start and end vectors.
         * When t=0, returns start. When t=1, returns end. Values outside [0,1] are clamped.
         * 
         * Example usage:
         * @code
         * Vector3D a(1, 2, 3);
         * Vector3D b(5, 6, 7);
         * Vector3D mid = Vector3D::Lerp(a, b, 0.5);  // Returns (3, 4, 5)
         * @endcode
         */
        static constexpr Vector3D Lerp(const Vector3D& start, const Vector3D& end, const Fxp& t)
        {
            // Clamp t to [0, 1] range
            Fxp clampedT = t;
            if (t < 0.0) clampedT = 0.0;
            else if (t > 1.0) clampedT = 1.0;
            
            return start + (end - start) * clampedT;
        }

        // Bitwise shift operators
        /**
         * @brief Bitwise right shift operator.
         * @param shiftAmount The number of positions to shift.
         * @return The resulting Vec3 object.
         */
        constexpr Vector3D operator>>(const size_t& shiftAmount) const
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