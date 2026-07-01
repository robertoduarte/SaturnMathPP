#pragma once

#include <utility>
#include "vector2d.hpp"
#include "precision.hpp"
#include "sort_order.hpp"
#include "utils.hpp"

namespace SaturnMath::Types
{
    /**
     * @brief A high-performance three-dimensional vector implementation optimized for Saturn hardware.
     * 
     * @details Vector3 provides comprehensive 3D vector operations using
     * fixed-point arithmetic. It supports Z-axis operations
     * and 3D-specific algorithms optimized for performance-critical graphics and physics calculations.
     * 
     * Key features:
     * - Memory-efficient representation (three T values)
     * - Comprehensive set of 3D vector operations (cross product, normalization, etc.)
     * - Multiple precision levels for performance-critical operations
     * - Hardware-optimized calculations for Saturn platform
     * - Compatible with Vector2 for 2D/3D interoperability
     * - Concept-constrained to FixedPoint types only
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
     * @see Vector2 For 2D vector operations
     * @see FixedPoint For details on the fixed-point implementation
     * @see Precision For available precision levels in calculations
     */
    template<int I = 16, int F = 16> struct Vector3
    {
        using T = FixedPoint<I, F>;
        T X; /**< The X-coordinate. */
        T Y; /**< The Y-coordinate. */
        T Z; /**< The Z-coordinate. */

        /** @name Constructors */
        ///@{
        /**
         * @brief Default constructor, initializes all coordinates to 0.
         */
        constexpr Vector3() : X(), Y(), Z() {}

        /**
         * @brief Constructor to initialize all coordinates with the same value.
         * @param T The value to initialize all coordinates with.
         */
        constexpr Vector3(const T& value) : X(value), Y(value), Z(value) {}

        /**
         * @brief Copy constructor.
         * @param vec The Vec3 object to copy.
         */
        constexpr Vector3(const Vector3& vec) : X(vec.X), Y(vec.Y), Z(vec.Z) {}

        /**
         * @brief Constructor to initialize coordinates with specific values.
         * @param valueX The X-coordinate.
         * @param valueY The Y-coordinate.
         * @param valueZ The Z-coordinate.
         */
        constexpr Vector3(const T& valueX, const T& valueY, const T& valueZ) : X(valueX), Y(valueY), Z(valueZ) {}

        /**
         * @brief Constructor to initialize from a Vector2 and a Z coordinate.
         * @param vec2d The Vector2 to copy X and Y from.
         * @param valueZ The Z-coordinate.
         */
        constexpr Vector3(const Vector2<I, F>& vec2d, const T& valueZ) : X(vec2d.X), Y(vec2d.Y), Z(valueZ) {}

        ///@}
        /** @name Assignment */
        ///@{
        /**
         * @brief Assignment operator.
         * @param vec The Vec3 object to assign.
         * @return Reference to the modified Vec3 object.
         */
        constexpr Vector3& operator=(const Vector3& vec)
        {
            X = vec.X;
            Y = vec.Y;
            Z = vec.Z;
            return *this;
        }

        /**
         * @brief Calculate the absolute values of each coordinate.
         * @return A new Vec3 object with absolute values.
         */
        [[gnu::always_inline]] constexpr Vector3 Abs() const
        {
            return Vector3(X.Abs(), Y.Abs(), Z.Abs());
        }

        /**
         * @brief Sort coordinates in ascending order.
         * @tparam Ascending If true, sort in ascending order; otherwise, in descending order.
         * @return A new Vec3 object with sorted coordinates.
         */
        template <SortOrder O = SortOrder::Ascending>
        constexpr Vector3 Sort() const
        {
            Vector3 result(*this);
            result.SortInPlace<O>();
            return result;
        }

        /**
         * @brief Sort coordinates in-place.
         * @tparam O Sort order (Ascending or Descending)
         * @details Sorts the coordinates of the vector in-place, modifying the original object.
         * This method is used internally by Sort() to avoid creating a temporary object.
         * It uses a simple swap-based approach to sort the coordinates.
         */
        template <SortOrder O = SortOrder::Ascending>
        constexpr void SortInPlace()
        {
            T temp;

            if constexpr (O == SortOrder::Ascending) {
                if (X > Y) { temp = X; X = Y; Y = temp; }
                if (X > Z) { temp = X; X = Z; Z = temp; }
                if (Y > Z) { temp = Y; Y = Z; Z = temp; }
            } else {
                if (X < Y) { temp = X; X = Y; Y = temp; }
                if (Y < Z) { temp = Y; Y = Z; Z = temp; }
                if (X < Y) { temp = X; X = Y; Y = temp; }
            }
        }

        /**
         * @brief Helper function to perform assembly-level dot product calculation and accumulation
         * @param first First vector
         * @param second Second vector
         * @warning This function MUST be used together with T::ClearMac() and T::ExtractMac().
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
         * T::ClearMac();
         *
         * // Step 2: Call DotAccumulate one or more times
         * DotAccumulate(v1, v2);              // First dot product
         * DotAccumulate(v3, v4);              // Optional: accumulate more dot products
         *
         * // Step 3: Always extract result after last DotAccumulate
         * T result = T::ExtractMac();
         * @endcode
         */
        [[gnu::always_inline]] static void DotAccumulate(const Vector3& first, const Vector3& second)
        {
            auto a = reinterpret_cast<const int32_t*>(&first);
            auto b = reinterpret_cast<const int32_t*>(&second);
            Hardware::MacAccumulate<3>(a, b);
        }

        /**
         * @brief Calculate the dot product of this object and another Vec3 object.
         * @param vec The Vec3 object to calculate the dot product with.
         * @return The dot product as an T value.
         * @details Calculates a single dot product between two vectors. For runtime calculations,
         * this uses the DotAccumulate helper with proper MAC register management.
         *
         * Example usage:
         * @code
         * Vector3 v1(1, 2, 3);
         * Vector3 v2(4, 5, 6);
         * T result = v1.Dot(v2);    // Computes 1*4 + 2*5 + 3*6
         * @endcode
         */
        [[gnu::always_inline]] constexpr T Dot(const Vector3& vec) const
        {
            if consteval
            {
                return X * vec.X + Y * vec.Y + Z * vec.Z;
            }
            else
            {
                Hardware::MacClear();
                DotAccumulate(*this, vec);
                return T::BuildRaw(Hardware::MacExtract());
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
         * Vector3 v1(1, 0, 0), v2(1, 0, 0);  // Unit vectors along X
         * Vector3 v3(0, 1, 0), v4(0, 1, 0);  // Unit vectors along Y
         * Vector3 v5(0, 0, 1), v6(0, 0, 1);  // Unit vectors along Z
         *
         * // Computes (v1·v2) + (v3·v4) + (v5·v6) = 1 + 1 + 1 = 3
         * // All calculations done in parallel using Saturn's MAC registers
         * T result = Vector3::MultiDotAccumulate(
         *     std::pair{v1, v2},
         *     std::pair{v3, v4},
         *     std::pair{v5, v6}
         * );
         * @endcode
         */
        template <typename... Pairs>
        [[gnu::always_inline]] static constexpr T MultiDotAccumulate(const Pairs&... pairs)
        {
            if consteval
            {
                // Compile-time computation using fold expression
                return (... + pairs.first.Dot(pairs.second));
            }
            else
            {
                Hardware::MacClear();

                // Loop through pairs and accumulate dot products
                ([&](const auto& pair)
                {
                    DotAccumulate(pair.first, pair.second);
                }(pairs), ...); // Unpack the variadic arguments

                return T::BuildRaw(Hardware::MacExtract());
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
         * Vector3 v1(1, 0, 0);  // Unit vector along X
         * Vector3 v2(0, 1, 0);  // Unit vector along Y
         * Vector3 cross = v1.Cross(v2);  // Returns (0, 0, 1) - unit vector along Z
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
         * Vector3 a(1, 0, 0);  // Unit vector along X
         * Vector3 b(0, 1, 0);  // Unit vector along Y
         * Vector3 cross = a.Cross(b);  // Returns (0, 0, 1) - unit vector along Z
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector3 Cross(const Vector3& vec) const
        {
            return Vector3(
                Y * vec.Z - Z * vec.Y,  // X component
                Z * vec.X - X * vec.Z,  // Y component
                X * vec.Y - Y * vec.X   // Z component
            );
        }

        /**
         * @brief Compute the maximum safe value for squaring without overflow (3D).
         * @return sqrt(2^(IntBits-1) / 3) in the component type's units.
         * @details For 16.16: ~104.6, for 24.8: ~1673.8, for 8.24: ~6.53.
         *          Values at or above this threshold will overflow when squared
         *          and summed across 3 components.
         */
        static constexpr T MaxSafeSquareValue()
        {
            if constexpr (T::IntBits % 2 == 0)
                return T(static_cast<double>(1u << ((T::IntBits - 1) / 2)) * 0.8164965809277261); // sqrt(2/3)
            else
                return T(static_cast<double>(1u << ((T::IntBits - 1) / 2)) / 1.7320508075688772); // 1/sqrt(3)
        }

        /**
         * @brief Calculate the squared length of the vector with overflow protection.
         * @return The squared length as an T value, or MaxValue() if the result would overflow.
         * @details Returns T::MaxValue() if the squared magnitude would be too large to represent.
         *          This version includes overflow protection to ensure safe calculations.
         *          
         *          The method checks for potential overflow by comparing against a safe threshold
         *          that ensures the sum of squares won't exceed the maximum representable value.
         *          For 3D vectors, we use a more conservative threshold than 2D to account for
         *          the additional component.
         * 
         * Example usage:
         * @code
         * Vector3 v(3, 4, 5);
         * T lenSq = v.LengthSquared();  // Returns 50 (3*3 + 4*4 + 5*5)
         * 
         * // With large values that would overflow:
         * Vector3 large(1000, 1000, 1000);
         * T largeLenSq = large.LengthSquared();  // Returns T::MaxValue()
         * @endcode
         */
        [[gnu::always_inline]] constexpr T LengthSquared() const {
            // Special case: if any component is MinValue, the square would be MaxValue
            if (X == T::MinValue() || Y == T::MinValue() || Z == T::MinValue()) {
                return T::MaxValue();
            }
            
            // Get absolute values to handle negative numbers
            const T absX = X.Abs();
            const T absY = Y.Abs();
            const T absZ = Z.Abs();
            
            // Calculate maximum possible value before overflow
            // sqrt(2^(IntBits-1) / 3) for 3 components
            constexpr T maxSafeValue = MaxSafeSquareValue();
            
            // If any component is too large, return MaxValue to prevent overflow
            if (absX >= maxSafeValue || absY >= maxSafeValue || absZ >= maxSafeValue) {
                // For values just above the threshold, try scaling down to avoid false positives
                if (absX < 2 * maxSafeValue && absY < 2 * maxSafeValue && absZ < 2 * maxSafeValue) {
                    // Scale down by 2, calculate, then scale back up
                    const Vector3 scaled = *this >> 1;
                    T scaledDot = scaled.Dot(scaled);
                    // Check if scaling back by 4 would overflow
                    if (scaledDot > T::MaxValue() >> 2)
                        return T::MaxValue();
                    return scaledDot << 2;  // Multiply by 4 (2^2)
                }
                return T::MaxValue();
            }
            
            // Safe to calculate normally
            return Dot(*this);
        }

        /**
         * @brief Calculate the length (magnitude) of the vector.
         * @tparam P Precision level for calculation (default: Precision::Default)
         * @return The length as an T value.
         * 
         * @details Calculates the Euclidean length of the vector using the formula:
         *          sqrt(X² + Y² + Z²)
         * 
         * The precision template parameter controls the calculation method:
         * - Accurate: Full precision square root calculation
         * - Fast: Fast approximation with good accuracy
         * - Turbo: Fastest approximation using alpha-beta-gamma coefficients
         * 
         * The method includes overflow protection by scaling down large vectors
         * before calculation and scaling the result back up.
         * 
         * Example usage:
         * @code
         * Vector3 v(3, 4, 5);
         * T len = v.Length();  // Default precision (Fast)
         * T accLen = v.Length<Precision::Accurate>();  // Accurate sqrt
         * T fastLen = v.Length<Precision::Fast>();  // Fast sqrt
         * T turboLen = v.Length<Precision::Turbo>();  // Fast approximation (alpha-beta-gamma)
         * @endcode
         */
        [[gnu::always_inline]] constexpr T Length() const
        {
            if consteval
            {
                // Compute the 64-bit dot product (X² + Y² + Z²) the same way the
                // hardware MAC would, then split it into the high/low 32-bit halves
                // expected by InternalSqrtFrom64 (which is itself constexpr-friendly).
                const int64_t acc =
                    static_cast<int64_t>(X.RawValue()) * X.RawValue() +
                    static_cast<int64_t>(Y.RawValue()) * Y.RawValue() +
                    static_cast<int64_t>(Z.RawValue()) * Z.RawValue();
                const uint32_t hi = static_cast<uint32_t>(static_cast<uint64_t>(acc) >> 32);
                const uint32_t lo = static_cast<uint32_t>(static_cast<uint64_t>(acc) & 0xFFFFFFFFu);
                return T::InternalSqrtFrom64(hi, lo);
            }
            else
            {
                Hardware::MacClear();
                DotAccumulate(*this, *this);
                int32_t mach, macl;
                Hardware::MacGet(mach, macl);
                return T::InternalSqrtFrom64(mach, macl);
            }
        }

        /**
         * @brief Fast approximation of vector length using alpha-beta-gamma coefficients.
         * @return Approximate length as an T value.
         *
         * @details Uses the alpha-beta-gamma approximation for square root:
         *          |v| ≈ α·max(|x|,|y|,|z|) + β·mid(|x|,|y|,|z|) + γ·min(|x|,|y|,|z|)
         *
         * The coefficients are stored in 2.30 fixed-point format for maximum precision
         * regardless of the vector's format. This ensures that the coefficient precision
         * does not limit the overall accuracy of the approximation.
         *
         * Trade-offs:
         * - Faster than Length() (no MAC operations, no 64-bit sqrt)
         * - Higher error margin than Length() (typically ~1-2% error)
         * - Suitable for performance-critical code where exact precision is not required
         *
         * Example usage:
         * @code
         * Vector3 v(3, 4, 5);
         * T exactLen = v.Length();        // Exact length (slower)
         * T approxLen = v.TurboLength();   // Approximate length (faster)
         * @endcode
         */
        [[gnu::always_inline]] constexpr T TurboLength() const
        {
            constexpr FixedPoint<8, 24> alpha(0.96043387010342);
            constexpr FixedPoint<8, 24> beta(0.39782473475533);
            constexpr FixedPoint<8, 24> gamma(0.196034280659121);


            Vector3 absolute = Abs();
            absolute.SortInPlace<SortOrder::Descending>();
            absolute.X *= alpha;
            absolute.Y *= beta;
            absolute.Z *= gamma;

            return absolute.X + absolute.Y + absolute.Z;
        }

        /**
         * @brief Calculate the length (magnitude) of the vector (deprecated)
         * @tparam P Precision level for calculation
         * @return The length as an T value.
         * @deprecated Use Length() for exact length, or TurboLength() for fast approximation.
         *             Precision parameter is ignored: Turbo→TurboLength(), others→Length()
         */
        template<Precision P = Precision::Default>
        [[deprecated("Use Length() for exact length, or TurboLength() for fast approximation. Precision parameter is ignored")]]
        [[gnu::always_inline]] constexpr T Length() const
        {
            if constexpr (P == Precision::Turbo) {
                return TurboLength();
            } else {
                return Length();
            }
        }

        /**
         * @brief Creates a unit vector pointing in the same direction as this vector.
         * The precision template parameter controls the length calculation method:
         * - Standard precision: Uses exact square root calculation
         * - Turbo precision: Uses fast approximation with alpha-beta-gamma coefficients
         *
         * If the vector length is zero, returns a zero vector to avoid division by zero.
         *
         * Example usage:
         * @code
         * Vector3 v(3, 4, 5);
         * Vector3 unitV = v.Normalize();  // Returns unit vector with standard precision
         * Vector3 fastUnitV = v.Normalize<Precision::Turbo>();  // Returns approximate unit vector (faster)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector3 Normalize() const
        {
            T length = Length();
            if (length == 0)
                return Vector3();
            auto temp = *this;
            if (length < 0) // Overflow happened
            {
                // Length is always large here, so reciprocal is tiny.
                // Q2.30 has 30 fractional bits — enough precision for all formats.
                // Runtime cost: same Mul64 + Extract32, just different shift constant.
                length = T::BuildRaw(static_cast<uint32_t>(length.RawValue()) >> 1);
                auto reciprocal = FixedPoint<2, 30>(0.5) / length;
                temp.X *= reciprocal;
                temp.Y *= reciprocal;
                temp.Z *= reciprocal;
            }
            else
            {
                // For formats with many integer bits (e.g. Q24.8), large lengths
                // produce tiny reciprocals that truncate to 0 in Q16.16.
                // Q8.24 has 24 fractional bits — enough for lengths up to ~8M.
                // For formats with ≤16 integer bits, Q16.16 is sufficient and
                // preserves the original behavior for Q16.16 and Q8.24.
                using RecipNormal = std::conditional_t<(T::IntBits > 16),
                    FixedPoint<8, 24>, FixedPoint<16, 16>>;
                auto reciprocal = RecipNormal(1.0) / length;
                temp.X *= reciprocal;
                temp.Y *= reciprocal;
                temp.Z *= reciprocal;
            }

            return temp;
        }

        /**
         * @brief Creates a unit vector pointing in the same direction as this vector (deprecated)
         * @tparam P Precision level for calculation (ignored)
         * @return Normalized vector
         * @deprecated Use Normalize() instead - precision parameter is ignored
         */
        template<Precision P = Precision::Default>
        [[deprecated("Use Normalize() instead - precision parameter is ignored")]]
        [[gnu::always_inline]] constexpr Vector3 Normalize() const { return Normalize(); }

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
         * Vector3 v(3, 4, 5);
         * Vector3 unitV = v.Normalized();  // Original vector remains unchanged
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector3 Normalized() const
        {
            Vector3 copy(*this);
            return copy.Normalize();
        }

        /**
         * @brief Get a normalized copy of the vector (deprecated)
         * @tparam P Precision level for calculation (ignored)
         * @return Normalized vector
         * @deprecated Use Normalized() instead - precision parameter is ignored
         */
        template<Precision P = Precision::Default>
        [[deprecated("Use Normalized() instead - precision parameter is ignored")]]
        [[gnu::always_inline]] constexpr Vector3 Normalized() const { return Normalized(); }

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
         * Vector3 v1(0, 0, 0);
         * Vector3 v2(1, 0, 0);
         * Vector3 v3(0, 1, 0);
         * Vector3 normal = Vector3::CalcNormal(v1, v2, v3);  // Returns (0, 0, 1)
         * @endcode
         */
        static Vector3 CalcNormal(
            const Vector3& vertexA,
            const Vector3& vertexB,
            const Vector3& vertexC)
        {
            const Vector3 edge1 = vertexB - vertexA;
            const Vector3 edge2 = vertexC - vertexA;
            return edge1.Cross(edge2).Normalize();
        }

        /**
         * @brief Calculate normal vector for a triangle (deprecated)
         * @tparam P Precision level for calculation (ignored)
         * @deprecated Use CalcNormal() instead - precision parameter is ignored
         */
        template<Precision P = Precision::Default>
        [[deprecated("Use CalcNormal() instead - precision parameter is ignored")]]
        static Vector3 CalcNormal(
            const Vector3& vertexA,
            const Vector3& vertexB,
            const Vector3& vertexC)
        {
            return CalcNormal(vertexA, vertexB, vertexC);
        }

        /**
         * @brief Calculate the Euclidean distance from this vector to another vector.
         * @tparam P Precision level for calculation
         * @param other The other vector to calculate the distance to.
         * @return The distance as an T value.
         * @details Computes the distance using the formula: sqrt((X - other.X)^2 + (Y - other.Y)^2 + (Z - other.Z)^2).
         * The precision template parameter controls the calculation method:
         * - Standard precision: Uses exact square root calculation
         * - Turbo precision: Uses fast approximation
         * 
         * Example usage:
         * @code
         * Vector3 v1(1, 2, 3);
         * Vector3 v2(4, 6, 8);
         * T distance = v1.DistanceTo(v2);  // Computes distance between the two points
         * T fastDistance = v1.DistanceTo<Precision::Turbo>(v2);  // Computes approximate distance (faster)
         * @endcode
         */
        [[gnu::always_inline]] constexpr T DistanceTo(const Vector3& other) const {
            return (*this - other).Length();
        }

        /**
         * @brief Calculate the Euclidean distance from this vector to another vector (deprecated)
         * @tparam P Precision level for calculation (ignored)
         * @param other The other vector to calculate the distance to.
         * @return The distance as an T value.
         * @deprecated Use DistanceTo() instead - precision parameter is ignored
         */
        template<Precision P = Precision::Default>
        [[deprecated("Use DistanceTo() instead - precision parameter is ignored")]]
        [[gnu::always_inline]] constexpr T DistanceTo(const Vector3& other) const {
            return DistanceTo(other);
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
         * Vector3 a(1, 2, 3);
         * Vector3 b(4, 6, 8);
         * T distSq = a.DistanceSquared(b);  // Returns 50 (3² + 4² + 5²)
         * @endcode
         */
        [[gnu::always_inline]] constexpr T DistanceSquared(const Vector3& other) const {
            const T dx = X - other.X;
            const T dy = Y - other.Y;
            const T dz = Z - other.Z;
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
         * Vector3 v1(1, 0, 0);  // Right vector
         * Vector3 v2(0, 1, 0);  // Up vector
         * Angle angle = Vector3::Angle(v1, v2);  // Returns 90 degrees (π/2 radians)
         * @endcode
         */
        template<Precision P = Precision::Default>
        static constexpr auto Angle(const Vector3& a, const Vector3& b)
        {
            // Handle zero vectors
            const T aLenSq = a.LengthSquared();
            const T bLenSq = b.LengthSquared();
            
            if (aLenSq == 0 || bLenSq == 0) {
                return Angle::Zero();
            }
            
            // Calculate dot product and cross product magnitude squared
            const T dot = a.Dot(b);
            const Vector3 cross = a.Cross(b);
            const T crossLenSq = cross.LengthSquared();
            
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
         * Vector3 v(2, 3, 0);
         * Vector3 u(1, 0, 0);
         * Vector3 proj = v.Project(u);  // Returns (2, 0, 0)
         * @endcode
         */
        constexpr Vector3 Project(const Vector3& other) const
        {
            T denominator = other.Dot(other);
            if (denominator == 0) return Vector3(); // Avoid division by zero
            T scalar = Dot(other) / denominator;
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
         * Vector3 v(1, -1, 0);
         * Vector3 n(0, 1, 0); // Up normal
         * Vector3 reflected = v.Reflect(n);  // Returns (1, 1, 0)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector3 Reflect(const Vector3& normal) const
        {
            // Standard reflection formula: v - 2*(v·n)*n
            // Where n is the normal vector (assumed to be normalized)
            // v·n = v.X*n.X + v.Y*n.Y + v.Z*n.Z
            T dot = Dot(normal);
            return *this - normal * (dot * 2);
        }

        ///@}
        /** @name Scalar Multiplication & Division */
        ///@{
        /**
         * @brief Compound multiplication assignment operator.
         * @param scalar The scalar value to multiply by.
         * @return Reference to the modified Vec3 object.
         * 
         * @details Multiplies each component of the vector by the scalar value.
         * 
         * Example usage:
         * @code
         * Vector3 v(1, 2, 3);
         * v *= 2.5_fxp;  // Results in v = (2.5, 5, 7.5)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector3& operator*=(const T& scalar)
        {
            X *= scalar;
            Y *= scalar;
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
         * This specialized version uses T's optimized integral multiplication
         * for better performance on Saturn hardware.
         * 
         * Example usage:
         * @code
         * Vector3 v(1, 2, 3);
         * v *= 2;  // Results in v = (2, 4, 6) with optimized integral multiplication
         * @endcode
         */
        template<typename U>
            requires std::is_integral_v<U>
        [[gnu::always_inline]] constexpr Vector3& operator*=(const U& scalar)
        {
            X *= scalar;
            Y *= scalar;
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
         * Vector3 v(4, 6, 8);
         * v /= 2_fxp;  // Results in v = (2, 3, 4)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector3& operator/=(const T& scalar)
        {
            X /= scalar;
            Y /= scalar;
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
         * This specialized version uses T's optimized integral division
         * for better performance on Saturn hardware.
         * 
         * Example usage:
         * @code
         * Vector3 v(10, 20, 30);
         * v /= 5;  // Results in v = (2, 4, 6) with optimized integral division
         * @endcode
         */
        template<typename U>
            requires std::is_integral_v<U>
        [[gnu::always_inline]] constexpr Vector3& operator/=(const U& scalar)
        {
            X /= scalar;
            Y /= scalar;
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
         * Vector3 v(1, 2, 3);
         * Vector3 result = v * 3_fxp;  // Results in result = (3, 6, 9)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector3 operator*(const T& scalar) const
        {
            Vector3 result(*this);
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
         * by the integral scalar value. This specialized version uses T's optimized 
         * integral multiplication for better performance on Saturn hardware.
         * 
         * Example usage:
         * @code
         * Vector3 v(1, 2, 3);
         * Vector3 result = v * 3;  // Results in result = (3, 6, 9) with optimized integral multiplication
         * @endcode
         */
        template<typename U>
            requires std::is_integral_v<U>
        [[gnu::always_inline]] constexpr Vector3 operator*(const U& scalar) const
        {
            Vector3 result(*this);
            result *= scalar;
            return result;
        }

        /**
         * @brief Multiply an integral scalar by a Vector3.
         * @tparam T The integral type of the scalar value.
         * @param scalar The scalar value to multiply.
         * @param vec The vector to multiply by.
         * @return A new vector with each component multiplied by the scalar.
         * 
         * @details Creates a new vector by multiplying each component of the input vector
         * by the integral scalar value. This specialized version uses T's optimized 
         * integral multiplication for better performance on Saturn hardware.
         * 
         * Example usage:
         * @code
         * Vector3 v(1, 2, 3);
         * Vector3 result = 3 * v;  // Results in result = (3, 6, 9) with optimized integral multiplication
         * @endcode
         */
        template<typename U>
            requires std::is_integral_v<U>
        friend constexpr Vector3 operator*(const U& scalar, const Vector3& vec)
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
         * Vector3 v(6, 8, 10);
         * Vector3 result = v / 2_fxp;  // Results in result = (3, 4, 5)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector3 operator/(const T& scalar) const
        {
            return Vector3(X / scalar, Y / scalar, Z / scalar);
        }

        /**
         * @brief Scalar division operator for integral types.
         * @tparam T The integral type of the scalar value.
         * @param scalar The scalar value to divide by.
         * @return A new vector with each component divided by the scalar.
         * 
         * @details Creates a new vector by dividing each component of this vector
         * by the integral scalar value. This specialized version uses T's optimized 
         * integral division for better performance on Saturn hardware.
         * 
         * Example usage:
         * @code
         * Vector3 v(10, 20, 30);
         * Vector3 result = v / 5;  // Results in result = (2, 4, 6) with optimized integral division
         * @endcode
         */
        template<typename U>
            requires std::is_integral_v<U>
        [[gnu::always_inline]] constexpr Vector3 operator/(const U& scalar) const
        {
            Vector3 result(*this);
            result /= scalar;
            return result;
        }

        ///@}
        /** @name Unit Vectors & Constants */
        ///@{
        /**
         * @brief Get a unit vector pointing along the X axis (1,0,0).
         * @return Unit vector along X axis.
         */
        static consteval Vector3 UnitX()
        {
            return Vector3(1, 0, 0);
        }

        /**
         * @brief Get a unit vector pointing along the Y axis (0,1,0).
         * @return Unit vector along Y axis.
         */
        static consteval Vector3 UnitY()
        {
            return Vector3(0, 1, 0);
        }

        /**
         * @brief Get a unit vector pointing along the Z axis (0,0,1).
         * @return Unit vector along Z axis.
         */
        static consteval Vector3 UnitZ()
        {
            return Vector3(0, 0, 1);
        }

        /**
         * @brief Get a zero vector (0,0,0).
         * @return Zero vector.
         */
        static consteval Vector3 Zero()
        {
            return Vector3(0);
        }

        /**
         * @brief Get a vector with all components set to one (1,1,1).
         * @return Vector with all ones.
         */
        static consteval Vector3 One()
        {
            return Vector3(1);
        }

        ///@}
        /** @name Comparison Operators */
        ///@{
        /**
         * @brief Check if two Vec3 objects are not equal.
         * @param vec The Vec3 object to compare.
         * @return True if not equal, false otherwise.
         */
        [[gnu::always_inline]] constexpr bool operator!=(const Vector3& vec) const
        {
            return X != vec.X || Y != vec.Y || Z != vec.Z;
        }

        /**
         * @brief Check if two Vec3 objects are equal.
         * @param vec The Vec3 object to compare.
         * @return True if equal, false otherwise.
         */
        [[gnu::always_inline]] constexpr bool operator==(const Vector3& vec) const
        {
            return X == vec.X && Y == vec.Y && Z == vec.Z;
        }

        /**
         * @brief Less than operator.
         * @param vec The Vector3 object to compare with.
         * @return True if this vector is less than the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y, then Z).
         */
        [[gnu::always_inline]] constexpr bool operator<(const Vector3& vec) const
        {
            return (X < vec.X) || (X == vec.X && (Y < vec.Y || (Y == vec.Y && Z < vec.Z)));
        }

        /**
         * @brief Less than or equal operator.
         * @param vec The Vector3 object to compare with.
         * @return True if this vector is less than or equal to the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y, then Z).
         */
        [[gnu::always_inline]] constexpr bool operator<=(const Vector3& vec) const
        {
            return (X < vec.X) || (X == vec.X && (Y < vec.Y || (Y == vec.Y && Z <= vec.Z)));
        }

        /**
         * @brief Greater than operator.
         * @param vec The Vector3 object to compare with.
         * @return True if this vector is greater than the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y, then Z).
         */
        [[gnu::always_inline]] constexpr bool operator>(const Vector3& vec) const
        {
            return (X > vec.X) || (X == vec.X && (Y > vec.Y || (Y == vec.Y && Z > vec.Z)));
        }

        /**
         * @brief Greater than or equal operator.
         * @param vec The Vector3 object to compare with.
         * @return True if this vector is greater than or equal to the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y, then Z).
         */
        [[gnu::always_inline]] constexpr bool operator>=(const Vector3& vec) const
        {
            return (X > vec.X) || (X == vec.X && (Y > vec.Y || (Y == vec.Y && Z >= vec.Z)));
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
         * Vector3 a(1, 2, 3);
         * Vector3 b(5, 6, 7);
         * Vector3 mid = Vector3::Lerp(a, b, 0.5);  // Returns (3, 4, 5)
         * @endcode
         */
        static constexpr Vector3 Lerp(const Vector3& start, const Vector3& end, const T& t)
        {
            // Clamp t to [0, 1] range
            T clampedT = t;
            if (t < 0.0) clampedT = 0.0;
            else if (t > 1.0) clampedT = 1.0;
            
            return start + (end - start) * clampedT;
        }

        /**
         * @brief Smoothstep interpolation between two 3D vectors
         * @param start Start vector
         * @param end End vector
         * @param t Interpolation factor [0, 1]
         * @return Interpolated vector between start and end
         */
        static constexpr Vector3 Smoothstep(const Vector3& start, const Vector3& end, const T& t)
        {
            T x = (t < 0) ? 0 : ((t > 1) ? 1 : t);
            T factor = x * x * (T(3) - T(2) * x);
            return Vector3(
                T::Lerp(start.X, end.X, factor),
                T::Lerp(start.Y, end.Y, factor),
                T::Lerp(start.Z, end.Z, factor)
            );
        }

        ///@}
        /** @name Bitwise Shift Operators */
        ///@{
        /**
         * @brief Bitwise right shift operator.
         * @param shiftAmount The number of positions to shift.
         * @return The resulting Vec3 object.
         */
        [[gnu::always_inline]] constexpr Vector3 operator>>(const size_t& shiftAmount) const
        {
            return Vector3(X >> shiftAmount, Y >> shiftAmount, Z >> shiftAmount);
        }

        /**
         * @brief Bitwise right shift assignment operator.
         * @param shiftAmount The number of positions to shift.
         * @return Reference to the modified Vec3 object.
         */
        [[gnu::always_inline]] constexpr Vector3& operator>>=(const size_t& shiftAmount)
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
        [[gnu::always_inline]] constexpr Vector3 operator<<(const size_t& shiftAmount) const
        {
            return Vector3(X << shiftAmount, Y << shiftAmount, Z << shiftAmount);
        }

        /**
         * @brief Bitwise left shift assignment operator.
         * @param shiftAmount The number of positions to shift.
         * @return Reference to the modified Vec3 object.
         */
        [[gnu::always_inline]] constexpr Vector3& operator<<=(const size_t& shiftAmount)
        {
            X <<= shiftAmount;
            Y <<= shiftAmount;
            Z <<= shiftAmount;
            return *this;
        }

        ///@}
        /** @name Unary Operators */
        ///@{
        /**
         * @brief Unary negation operator.
         * @return A new Vec3 object with negated coordinates.
         */
        [[gnu::always_inline]] constexpr Vector3 operator-() const
        {
            return Vector3(-X, -Y, -Z);
        }

        // Binary operators
        /**
         * @brief Binary addition operator.
         * @param vec The Vec3 object to add.
         * @return The sum as a Vec3 object.
         */
        [[gnu::always_inline]] constexpr Vector3 operator+(const Vector3& vec) const
        {
            return Vector3(X + vec.X, Y + vec.Y, Z + vec.Z);
        }

        /**
         * @brief Binary addition operator for adding a Vector2 to a Vector3.
         * @param vec The Vec2 object to add.
         * @return The sum as a Vec3 object.
         */
        [[gnu::always_inline]] constexpr Vector3 operator+(const Vector2<I, F>& vec) const {
            return Vector3(X + vec.X, Y + vec.Y, Z);
        }

        /**
         * @brief Binary addition operator for adding an T to a Vector3.
         * @param scalar The T value to add.
         * @return The resulting Vector3 object.
         */
        [[gnu::always_inline]] constexpr Vector3 operator+(const T& scalar) const {
            return Vector3(X + scalar, Y + scalar, Z + scalar);
        }

        /**
         * @brief Binary subtraction operator.
         * @param vec The Vec3 object to subtract.
         * @return The difference as a Vec3 object.
         */
        /**
         * @brief Binary subtraction operator.
         * @param vec The Vec3 object to subtract.
         * @return The difference as a Vec3 object.
         * 
         * @details Creates a new vector by subtracting each component of the input vector
         * from the corresponding component of this vector.
         * 
         * Example usage:
         * @code
         * Vector3 v1(5, 7, 9);
         * Vector3 v2(2, 3, 4);
         * Vector3 result = v1 - v2;  // Results in result = (3, 4, 5)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector3 operator-(const Vector3& vec) const
        {
            return Vector3(X - vec.X, Y - vec.Y, Z - vec.Z);
        }

        /**
         * @brief Compound addition assignment operator.
         * @param vec The Vec3 object to add.
         * @return Reference to the modified Vec3 object.
         */
        [[gnu::always_inline]] constexpr Vector3 operator+=(const Vector3& vec)
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
        [[gnu::always_inline]] constexpr Vector3 operator-=(const Vector3& vec)
        {
            X -= vec.X;
            Y -= vec.Y;
            Z -= vec.Z;
            return *this;
        }
        ///@}
    };

    using Vector3D  = Vector3<>;
}