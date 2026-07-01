#pragma once

#include "fxp.hpp"
#include "precision.hpp"
#include "sort_order.hpp"
#include "trigonometry.hpp"
#include "utils.hpp"

#include <utility>

namespace SaturnMath::Types
{
    /**
     * @brief A high-performance two-dimensional vector implementation using fixed-point arithmetic.
     * 
     * @details Vector2 provides a comprehensive set of 2D vector operations optimized for
     * Saturn hardware. It uses fixed-point arithmetic (T) for all components to avoid
     * floating-point operations while maintaining high precision.
     * 
     * Key features:
     * - Memory-efficient representation (two T values)
     * - Comprehensive set of vector operations (dot product, normalization, etc.)
     * - Optimized for performance-critical rendering and physics calculations
     * - Consistent behavior across all platforms through fixed-point arithmetic
     * - Support for various precision levels in calculations
     * 
     * Common applications:
     * - 2D positions and translations
     * - Texture coordinates
     * - Screen-space calculations
     * - Physics simulations (velocities, forces)
     * - User interface positioning
     * 
     * The implementation follows a right-handed coordinate system where:
     * - Positive X points right
     * - Positive Y points up
     * 
     * @note Most operations are constexpr and can be evaluated at compile time
     * for improved runtime performance.
     * 
     * @see Vector3D For 3D vector operations
     * @see FixedPoint For details on the fixed-point implementation
     */
    template<int I = 16, int F = 16> struct Vector2
    {
        using T = FixedPoint<I, F>;
        T X; /**< The X-coordinate. */
        T Y; /**< The Y-coordinate. */

        /** @name Constructors */
        ///@{
        /**
         * @brief Default constructor, initializes all coordinates to 0.
         */
        constexpr Vector2() : X(), Y() {}

        /**
         * @brief Constructor to initialize all coordinates with the same value.
         * @param T The value to initialize all coordinates with.
         */
        constexpr Vector2(const T& value) : X(value), Y(value) {}

        /**
         * @brief Copy constructor.
         * @param vec The Vec2 object to copy.
         */
        constexpr Vector2(const Vector2& vec) : X(vec.X), Y(vec.Y) {}

        /**
         * @brief Constructor to initialize coordinates with specific values.
         * @param valueX The X-coordinate.
         * @param valueY The Y-coordinate.
         */
        constexpr Vector2(const T& valueX, const T& valueY) : X(valueX), Y(valueY) {}

        /**
         * @brief Explicit conversion from Vector3 (drops Z coordinate).
         * @param vec3 The Vector3 to convert.
         */
        constexpr Vector2(const Vector3<I, F>& vec3) : X(vec3.X), Y(vec3.Y) {}

        ///@}
        /** @name Assignment */
        ///@{
        /**
         * @brief Assignment operator.
         * @param vec The Vec2 object to assign.
         * @return Reference to the modified Vec2 object.
         */
        constexpr Vector2& operator=(const Vector2& vec)
        {
            X = vec.X;
            Y = vec.Y;
            return *this;
        }

        ///@}
        /** @name Member Functions */
        ///@{
        /**
         * @brief Calculate the absolute values of each coordinate.
         * @return A new Vec2 object with absolute values.
         */
        [[gnu::always_inline]] constexpr Vector2 Abs() const
        {
            return Vector2(X.Abs(), Y.Abs());
        }

        /**
         * @brief Sort coordinates in specified order
         * @tparam O Sort order (Ascending or Descending)
         * @return A new Vec2 object with sorted coordinates
         */
        template <SortOrder O = SortOrder::Ascending>
        constexpr Vector2 Sort() const
        {
            Vector2 result(*this);
            result.SortInPlace<O>();
            return result;
        }

        /**
         * @brief Sort coordinates in-place.
         * @tparam O Sort order (Ascending or Descending)
         * @details Sorts the coordinates of the vector in-place, modifying the original object.
         * This function is used internally by Sort() and can be used directly for performance-critical code.
         */
        template <SortOrder O = SortOrder::Ascending>
        constexpr void SortInPlace()
        {
            if constexpr (O == SortOrder::Ascending) {
                if (X > Y) {
                    T temp = X;
                    X = Y;
                    Y = temp;
                }
            } else {
                if (X < Y) {
                    T temp = X;
                    X = Y;
                    Y = temp;
                }
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
        static void DotAccumulate(const Vector2& first, const Vector2& second)
        {
            auto a = reinterpret_cast<const int32_t*>(&first);
            auto b = reinterpret_cast<const int32_t*>(&second);
            Hardware::MacAccumulate<2>(a, b);
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
         * Vector2 v1(1, 0), v2(1, 0);  // Unit vectors along X
         * Vector2 v3(0, 1), v4(0, 1);  // Unit vectors along Y
         *
         * // Computes (v1·v2) + (v3·v4) = 1 + 1 = 2
         * // All calculations done in parallel using Saturn's MAC registers
         * T result = Vector2::MultiDotAccumulate(
         *     std::pair{v1, v2},
         *     std::pair{v3, v4}
         * );
         * @endcode
         */
        template <typename... Pairs>
        static constexpr T MultiDotAccumulate(const Pairs&... pairs)
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
         * @brief Calculate the length of the vector
         * @tparam P Precision level for calculation
         * @return Length of the vector
         * 
         * @details Calculates the magnitude (Euclidean length) of the vector.
         * The precision template parameter controls the calculation method:
         * - Standard precision: Uses exact square root calculation
         * - Turbo precision: Uses fast approximation with alpha-beta coefficients
         * 
         * Example usage:
         * @code
         * Vector2 v(3, 4);
         * T length = v.Length();  // Returns 5 (standard precision)
         * T fastLength = v.Length<Precision::Turbo>();  // Returns approximate length (faster)
         * @endcode
         */
        [[gnu::always_inline]] constexpr T Length() const
        {
            if consteval
            {
                // Compute the 64-bit dot product (X² + Y²) the same way the
                // hardware MAC would, then split it into the high/low 32-bit halves
                // expected by InternalSqrtFrom64 (which is itself constexpr-friendly).
                const uint64_t acc =
                    static_cast<uint64_t>(X.RawValue()) * static_cast<uint64_t>(X.RawValue()) +
                    static_cast<uint64_t>(Y.RawValue()) * static_cast<uint64_t>(Y.RawValue());
                const uint32_t hi = static_cast<uint32_t>(acc >> 32);
                const uint32_t lo = static_cast<uint32_t>(acc & 0xFFFFFFFFu);
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
         * @brief Fast approximation of vector length using alpha-beta coefficients.
         * @return Approximate length as an T value.
         *
         * @details Uses the alpha-beta approximation for square root:
         *          |v| ≈ α·max(|x|,|y|) + β·min(|x|,|y|)
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
         * Vector2 v(3, 4);
         * T exactLen = v.Length();        // Exact length (slower)
         * T approxLen = v.TurboLength();   // Approximate length (faster)
         * @endcode
         */
        [[gnu::always_inline]] constexpr T TurboLength() const
        {
            constexpr FixedPoint<8, 24> alpha(0.96043387010342);
            constexpr FixedPoint<8, 24> beta(0.39782473475533);

            Vector2 absolute = Abs();
            absolute.SortInPlace<SortOrder::Descending>();
            absolute.X *= alpha;
            absolute.Y *= beta;

            return T::BuildRaw(static_cast<uint32_t>(absolute.X.RawValue()) + static_cast<uint32_t>(absolute.Y.RawValue()));
        }

        /**
         * @brief Calculate the length of the vector (deprecated)
         * @tparam P Precision level for calculation
         * @return Length of the vector
         * @deprecated Use Length() for exact length, or TurboLength() for fast approximation.
         *             Precision parameter is ignored: Turbo→TurboLength(), others→Length()
         */
        template<Precision P = Precision::Default>
        [[deprecated("Use Length() for exact length, or TurboLength() for fast approximation. Precision parameter is ignored")]]
        constexpr T Length() const
        {
            if constexpr (P == Precision::Turbo) {
                return TurboLength();
            } else {
                return Length();
            }
        }

        /**
         * @brief Compute the maximum safe value for squaring without overflow.
         * @return sqrt(2^(IntBits-1)) in the component type's units.
         * @details For 16.16: ~181.02, for 24.8: ~2896.3, for 8.24: ~11.31.
         *          Values at or above this threshold will overflow when squared.
         */
        static constexpr T MaxSafeSquareValue()
        {
            if constexpr (T::IntBits % 2 == 0)
                return T(static_cast<double>(1u << ((T::IntBits - 2) / 2)) * 1.4142135623730951);
            else
                return T(static_cast<double>(1u << ((T::IntBits - 1) / 2)));
        }

        /**
         * @brief Calculate the squared length of the vector with overflow protection.
         * @return The squared length as an T value, or MaxValue() if the result would overflow.
         * 
         * @details Returns T::MaxValue() if the squared magnitude would be too large to represent.
         *          This version includes overflow protection to ensure safe calculations.
         *          Useful for comparisons where the actual length is not needed.
         * 
         * Example usage:
         * @code
         * Vector2 v1(1, 2);
         * Vector2 v2(4, 6);
         * if (v1.LengthSquared() < v2.LengthSquared()) {
         *     // v1 is shorter than v2
         * }
         * @endcode
         */
        [[gnu::always_inline]] constexpr T LengthSquared() const {
            // Special case: if either component is MinValue, the square would be MaxValue
            if (X == T::MinValue() || Y == T::MinValue()) {
                return T::MaxValue();
            }
            
            // Get absolute values to handle negative numbers
            const T absX = X.Abs();
            const T absY = Y.Abs();
            
            // Calculate maximum possible value before overflow
            constexpr T maxSafeValue = MaxSafeSquareValue();
            
            // If either component is too large, return MaxValue to prevent overflow
            // Note: We use >= to include the threshold value itself as safe
            if (absX >= maxSafeValue || absY >= maxSafeValue) {
                return T::MaxValue();
            }
            
            // Safe to calculate normally
            return Dot(*this);
        }

        /**
         * @brief Calculate the squared distance between two points.
         * @param other The other point to measure distance to.
         * @return The squared distance between the two points.
         * 
         * @details This method calculates the squared Euclidean distance between this point
         * and another point. This is more efficient than calculating the actual distance
         * as it avoids a square root operation. Useful for distance comparisons.
         * 
         * Example usage:
         * @code
         * Vector2 a(1, 2);
         * Vector2 b(4, 6);
         * T distSq = a.DistanceSquared(b);  // Returns 25 (3² + 4²)
         * @endcode
         */
        [[gnu::always_inline]] constexpr T DistanceSquared(const Vector2& other) const {
            const T dx = X - other.X;
            const T dy = Y - other.Y;
            return dx * dx + dy * dy;
        }

        /**
         * @brief Normalize the vector
         * @tparam P Precision level for calculation
         * @return Normalized vector
         * 
         * @details Creates a unit vector pointing in the same direction as this vector.
         * The precision template parameter controls the length calculation method:
         * - Standard precision: Uses exact square root calculation
         * - Turbo precision: Uses fast approximation with alpha-beta coefficients
         * 
         * If the vector length is zero, returns a zero vector to avoid division by zero.
         * 
         * Example usage:
         * @code
         * Vector2 v(3, 4);
         * Vector2 unitV = v.Normalize();  // Returns (0.6, 0.8) with standard precision
         * Vector2 fastUnitV = v.Normalize<Precision::Turbo>();  // Returns approximate unit vector (faster)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector2 Normalize() const
        {
            T length = Length();
            if (length == 0)
                return Vector2();
            auto temp = *this;
            if (length < 0) // Overflow happened
            {
                length = T::BuildRaw(static_cast<uint32_t>(length.RawValue()) >> 1);
                auto reciprocal = Fxp8_24(0.5) / length;
                temp.X *= reciprocal;
                temp.Y *= reciprocal;
            }
            else
            {
                auto reciprocal = Fxp16_16(1.0) / length;
                temp.X *= reciprocal;
                temp.Y *= reciprocal;
            }

            return temp;
        }

        /**
         * @brief Normalize the vector (deprecated)
         * @tparam P Precision level for calculation (ignored)
         * @return Normalized vector
         * @deprecated Use Normalize() instead - precision parameter is ignored
         */
        template<Precision P = Precision::Default>
        [[deprecated("Use Normalize() instead - precision parameter is ignored")]]
        [[gnu::always_inline]] constexpr Vector2 Normalize() const { return Normalize(); }

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
         * Vector2 v(3, 4);
         * Vector2 unitV = v.Normalized();  // Original vector remains unchanged
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector2 Normalized() const
        {
            Vector2 copy(*this);
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
        [[gnu::always_inline]] constexpr Vector2 Normalized() const { return Normalized(); }

        /**
         * @brief Calculate the Euclidean distance from this vector to another vector.
         * @tparam P Precision level for calculation
         * @param other The other vector to calculate the distance to.
         * @return The distance as an T value.
         * @details Computes the distance using the formula: sqrt((X - other.X)^2 + (Y - other.Y)^2).
         * The precision template parameter controls the calculation method:
         * - Standard precision: Uses exact square root calculation
         * - Turbo precision: Uses fast approximation
         * 
         * Example usage:
         * @code
         * Vector2 v1(1, 2);
         * Vector2 v2(4, 6);
         * T distance = v1.DistanceTo(v2);  // Computes distance between (1, 2) and (4, 6)
         * T fastDistance = v1.DistanceTo<Precision::Turbo>(v2);  // Computes approximate distance (faster)
         * @endcode
         */
        [[gnu::always_inline]] constexpr T DistanceTo(const Vector2& other) const {
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
        [[gnu::always_inline]] constexpr T DistanceTo(const Vector2& other) const { return DistanceTo(other); }

        /**
         * @brief Calculate the dot product of this vector with another vector.
         * @param vec The other vector.
         * @return The dot product as an T value.
         * @details Computes the dot product using the formula: X*vec.X + Y*vec.Y.
         * The dot product represents the cosine of the angle between two vectors 
         * multiplied by their lengths.
         * 
         * Example usage:
         * @code
         * Vector2 v1(1, 0);  // Unit vector along X
         * Vector2 v2(0, 1);  // Unit vector along Y
         * T dotProduct = v1.Dot(v2);  // Returns 0 (perpendicular vectors)
         * 
         * Vector2 v3(1, 1);
         * Vector2 v4(2, 3);
         * T dotProduct2 = v3.Dot(v4);  // Returns 5 (1*2 + 1*3)
         * @endcode
         */
        [[gnu::always_inline]] constexpr T Dot(const Vector2& vec) const
        {
            if consteval
            {
                return X * vec.X + Y * vec.Y;
            }
            else
            {
                Hardware::MacClear();
                DotAccumulate(*this, vec);
                return T::BuildRaw(Hardware::MacExtract());
            }
        }

        /**
         * @brief Calculate the cross product (z-component) of this vector and another Vector2.
         * @param vec The Vector2 to calculate the cross product with.
         * @return The z-component of the cross product as an T value.
         * 
         * @details In 2D, the cross product is effectively the z-component of a 3D cross product,
         * representing the area of the parallelogram formed by the two vectors.
         * The result is positive if vec is to the left of this vector, and negative if to the right.
         * 
         * Example usage:
         * @code
         * Vector2 v1(1, 0);  // Unit vector along X
         * Vector2 v2(0, 1);  // Unit vector along Y
         * T cross = v1.Cross(v2);  // Returns 1 - positive area
         * @endcode
         */
        [[gnu::always_inline]] constexpr T Cross(const Vector2& vec) const
        {
            return X * vec.Y - Y * vec.X;
        }

        /**
         * @brief Calculate the angle between two vectors.
         * @tparam P Precision level for calculation (unused, kept for compatibility)
         * @param a First vector
         * @param b Second vector
         * @return Angle between the vectors as an Angle object
         * 
         * @details Calculates the smallest angle between two vectors using the atan2 function
         * for better numerical stability. The result is always in the range [0, π] radians.
         * 
         * The angle is calculated using the formula:
         * angle = atan2(|a × b|, a · b)
         * 
         * Where:
         * - a × b is the 2D cross product (a.x*b.y - a.y*b.x)
         * - a · b is the dot product (a.x*b.x + a.y*b.y)
         * 
         * Example usage:
         * @code
         * Vector2 v1(1, 0);  // Right vector
         * Vector2 v2(0, 1);  // Up vector
         * Angle angle = Vector2::Angle(v1, v2);  // Returns 90 degrees (π/2 radians)
         * @endcode
         */
        static constexpr auto Angle(const Vector2& a, const Vector2& b)
        {
            // Calculate cross product magnitude (perpendicular dot product)
            T cross = a.X * b.Y - a.Y * b.X;
            // Calculate dot product
            T dot = a.Dot(b);
            
            // Use atan2 to get the angle
            return Trigonometry::Atan2(cross, dot);
        }

        /**
         * @brief Project this vector onto another vector.
         * @param target The vector to project onto.
         * @return The projection of this vector onto the target vector.
         * 
         * @details Calculates the vector projection of this vector onto the target vector.
         * The formula used is: (this · target / |target|²) * target
         * 
         * If the target vector is a zero vector, returns a zero vector.
         * 
         * Example usage:
         * @code
         * Vector2 v(3, 4);
         * Vector2 axis(1, 0);  // X-axis
         * Vector2 proj = v.ProjectOnto(axis);  // Returns (3, 0)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector2 ProjectOnto(const Vector2& target) const
        {
            T denominator = target.LengthSquared();
            if (denominator == 0)  // Avoid division by zero
                return Vector2();
                
            T scale = Dot(target) / denominator;
            return target * scale;
        }

        /**
         * @brief Reflect this vector across a normal vector.
         * @param normal The normal vector to reflect across.
         * @return The reflected vector.
         * 
         * @details Calculates the reflection of this vector across the given normal vector.
         * The formula used is: this - 2 * (this · normal) * normal / |normal|²
         * 
         * The normal vector does not need to be normalized.
         * If the normal is a zero vector, returns a copy of this vector.
         * 
         * Example usage:
         * @code
         * Vector2 v(1, -1);  // Vector pointing down-right
         * Vector2 normal(0, 1);  // Normal pointing up
         * Vector2 reflected = v.Reflect(normal);  // Returns (1, 1) - reflected across X-axis
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector2 Reflect(const Vector2& normal) const
        {
            T denominator = normal.LengthSquared();
            if (denominator == 0)  // Avoid division by zero
                return *this;
                
            T scale = -2 * Dot(normal) / denominator;
            return *this + (normal * scale);
        }

        ///@}
        /** @name Unit Vectors & Directional Methods */
        ///@{
        /**
         * @brief Get a unit vector pointing along the X axis (1,0).
         * @return Unit vector along X axis.
         */
        static consteval Vector2 UnitX()
        {
            return Vector2(1, 0);
        }

        /**
         * @brief Get a unit vector pointing along the Y axis (0,1).
         * @return Unit vector along Y axis.
         */
        static consteval Vector2 UnitY()
        {
            return Vector2(0, 1);
        }

        /**
         * @brief Get a zero vector (0,0).
         * @return Zero vector.
         */
        static consteval Vector2 Zero()
        {
            return Vector2(0);
        }

        /**
         * @brief Get a vector with all components set to one (1,1).
         * @return Vector with all ones.
         */
        static consteval Vector2 One()
        {
            return Vector2(1);
        }

        /**
         * @brief Get a vector pointing right (1,0).
         * Same as UnitX(), provided for semantic clarity.
         * @return Right-pointing unit vector.
         */
        static consteval Vector2 Right()
        {
            return UnitX();
        }

        /**
         * @brief Get a vector pointing left (-1,0).
         * @return Left-pointing unit vector.
         */
        static consteval Vector2 Left()
        {
            return Vector2(-1, 0);
        }

        /**
         * @brief Get a vector pointing up (0,1).
         * Same as UnitY(), provided for semantic clarity.
         * @return Upward-pointing unit vector.
         */
        static consteval Vector2 Up()
        {
            return UnitY();
        }

        /**
         * @brief Get a vector pointing down (0,-1).
         * @return Downward-pointing unit vector.
         */
        static consteval Vector2 Down()
        {
            return Vector2(0, -1);
        }

        ///@}
        /** @name Arithmetic Operators */
        ///@{
        /**
         * @brief Compound multiplication assignment operator.
         * @param scalar The scalar value to multiply by.
         * @return Reference to the modified Vec2 object.
         * 
         * @details Multiplies each component of the vector by the scalar value.
         * 
         * Example usage:
         * @code
         * Vector2 v(1, 2);
         * v *= 2.5_fxp;  // Results in v = (2.5, 5)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector2& operator*=(const T& scalar)
        {
            X *= scalar;
            Y *= scalar;
            return *this;
        }

        /**
         * @brief Compound multiplication assignment operator for integral types.
         * @tparam T The integral type of the scalar value.
         * @param scalar The scalar value to multiply by.
         * @return Reference to the modified Vec2 object.
         * 
         * @details Multiplies each component of the vector by the integral scalar value.
         * This specialized version uses T's optimized integral multiplication
         * for better performance on Saturn hardware.
         * 
         * Example usage:
         * @code
         * Vector2 v(1, 2);
         * v *= 2;  // Results in v = (2, 4) with optimized integral multiplication
         * @endcode
         */
        template<typename U>
            requires std::is_integral_v<U>
        [[gnu::always_inline]] constexpr Vector2& operator*=(const U& scalar)
        {
            X *= scalar;
            Y *= scalar;
            return *this;
        }

        /**
         * @brief Compound division assignment operator.
         * @param scalar The scalar value to divide by.
         * @return Reference to the modified Vec2 object.
         * 
         * @details Divides each component of the vector by the scalar value.
         * 
         * Example usage:
         * @code
         * Vector2 v(4, 6);
         * v /= 2_fxp;  // Results in v = (2, 3)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector2& operator/=(const T& scalar)
        {
            X /= scalar;
            Y /= scalar;
            return *this;
        }

        /**
         * @brief Compound division assignment operator for integral types.
         * @tparam T The integral type of the scalar value.
         * @param scalar The scalar value to divide by.
         * @return Reference to the modified Vec2 object.
         * 
         * @details Divides each component of the vector by the integral scalar value.
         * This specialized version uses T's optimized integral division
         * for better performance on Saturn hardware.
         * 
         * Example usage:
         * @code
         * Vector2 v(10, 20);
         * v /= 5;  // Results in v = (2, 4) with optimized integral division
         * @endcode
         */
        template<typename U>
            requires std::is_integral_v<U>
        [[gnu::always_inline]] constexpr Vector2& operator/=(const U& scalar)
        {
            X /= scalar;
            Y /= scalar;
            return *this;
        }

        /**
         * @brief Compound addition assignment operator.
         * @param vec The vector to add.
         * @return Reference to the modified Vec2 object.
         * 
         * @details Adds each component of the input vector to the corresponding
         * component of this vector.
         * 
         * Example usage:
         * @code
         * Vector2 v1(1, 2);
         * Vector2 v2(3, 4);
         * v1 += v2;  // Results in v1 = (4, 6)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector2& operator+=(const Vector2& vec)
        {
            X += vec.X;
            Y += vec.Y;
            return *this;
        }

        /**
         * @brief Compound subtraction assignment operator.
         * @param vec The vector to subtract.
         * @return Reference to the modified Vec2 object.
         * 
         * @details Subtracts each component of the input vector from the corresponding
         * component of this vector.
         * 
         * Example usage:
         * @code
         * Vector2 v1(5, 7);
         * Vector2 v2(2, 3);
         * v1 -= v2;  // Results in v1 = (3, 4)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector2& operator-=(const Vector2& vec)
        {
            X -= vec.X;
            Y -= vec.Y;
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
         * Vector2 v(1, 2);
         * Vector2 result = v * 3_fxp;  // Results in result = (3, 6)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector2 operator*(const T& scalar) const
        {
            Vector2 result(*this);
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
         * Vector2 v(1, 2);
         * Vector2 result = v * 3;  // Results in result = (3, 6) with optimized integral multiplication
         * @endcode
         */
        template<typename U>
            requires std::is_integral_v<U>
        [[gnu::always_inline]] constexpr Vector2 operator*(const U& scalar) const
        {
            Vector2 result(*this);
            result *= scalar;
            return result;
        }

        /**
         * @brief Multiply an integral scalar by a Vector2.
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
         * Vector2 v(1, 2);
         * Vector2 result = 3 * v;  // Results in result = (3, 6) with optimized integral multiplication
         * @endcode
         */
        template<typename U>
            requires std::is_integral_v<U>
        friend constexpr Vector2 operator*(const U& scalar, const Vector2& vec)
        {
            return vec * scalar;
        }

        /**
         * @brief Vector addition operator.
         * @param vec The vector to add.
         * @return A new vector representing the sum.
         * 
         * @details Creates a new vector by adding each component of the input vector
         * to the corresponding component of this vector.
         * 
         * Example usage:
         * @code
         * Vector2 v1(1, 2);
         * Vector2 v2(3, 4);
         * Vector2 result = v1 + v2;  // Results in result = (4, 6)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector2 operator+(const Vector2& vec) const
        {
            return Vector2(X + vec.X, Y + vec.Y);
        }

        /**
         * @brief Vector subtraction operator.
         * @param vec The vector to subtract.
         * @return A new vector representing the difference.
         * 
         * @details Creates a new vector by subtracting each component of the input vector
         * from the corresponding component of this vector.
         * 
         * Example usage:
         * @code
         * Vector2 v1(5, 7);
         * Vector2 v2(2, 3);
         * Vector2 result = v1 - v2;  // Results in result = (3, 4)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector2 operator-(const Vector2& vec) const
        {
            return Vector2(X - vec.X, Y - vec.Y);
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
         * Vector2 v(6, 8);
         * Vector2 result = v / 2_fxp;  // Results in result = (3, 4)
         * @endcode
         */
        [[gnu::always_inline]] constexpr Vector2 operator/(const T& scalar) const
        {
            return Vector2(X / scalar, Y / scalar);
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
         * Vector2 v(10, 20);
         * Vector2 result = v / 5;  // Results in result = (2, 4) with optimized integral division
         * @endcode
         */
        template<typename U>
            requires std::is_integral_v<U>
        [[gnu::always_inline]] constexpr Vector2 operator/(const U& scalar) const
        {
            Vector2 result(*this);
            result /= scalar;
            return result;
        }
        
        ///@}
        /** @name Unary Operators */
        ///@{
        /**
         * @brief Unary negation operator.
         * @return A new Vec3 object with negated coordinates.
         */
        [[gnu::always_inline]] constexpr Vector2 operator-() const
        {
            return Vector2(-X, -Y);
        }

        ///@}
        /** @name Comparison Operators */
        ///@{
        /**
         * @brief Check if two Vec3 objects are not equal.
         * @param vec The Vec3 object to compare.
         * @return True if not equal, false otherwise.
         */
        [[gnu::always_inline]] constexpr bool operator!=(const Vector2& vec) const
        {
            return !(*this == vec);
        }

        /**
         * @brief Check if two Vec3 objects are equal.
         * @param vec The Vec3 object to compare.
         * @return True if equal, false otherwise.
         */
        [[gnu::always_inline]] constexpr bool operator==(const Vector2& vec) const
        {
            return X == vec.X && Y == vec.Y;
        }

        /**
         * @brief Less than operator.
         * @param vec The Vector2 object to compare with.
         * @return True if this vector is less than the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y).
         */
        [[gnu::always_inline]] constexpr bool operator<(const Vector2& vec) const
        {
            return X < vec.X || (X == vec.X && Y < vec.Y);
        }

        /**
         * @brief Less than or equal operator.
         * @param vec The Vector2 object to compare with.
         * @return True if this vector is less than or equal to the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y).
         */
        [[gnu::always_inline]] constexpr bool operator<=(const Vector2& vec) const
        {
            return (X < vec.X) || (X == vec.X && Y <= vec.Y);
        }

        /**
         * @brief Greater than operator.
         * @param vec The Vector2 object to compare with.
         * @return True if this vector is greater than the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y).
         */
        [[gnu::always_inline]] constexpr bool operator>(const Vector2& vec) const
        {
            return (X > vec.X) || (X == vec.X && Y > vec.Y);
        }

        /**
         * @brief Greater than or equal operator.
         * @param vec The Vector2 object to compare with.
         * @return True if this vector is greater than or equal to the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y).
         */
        [[gnu::always_inline]] constexpr bool operator>=(const Vector2& vec) const
        {
            return !(*this < vec);
        }

        /**
         * @brief Linear interpolation between two vectors
         * @param start Starting vector
         * @param end Ending vector
         * @param t Interpolation factor [0,1]
         * @return Interpolated vector
         
         * @details Performs linear interpolation between start and end vectors.
         * When t=0, returns start. When t=1, returns end. Values outside [0,1] are clamped.
         * 
         * Example usage:
         * @code
         * Vector2 a(1, 2);
         * Vector2 b(5, 6);
         * Vector2 mid = Vector2::Lerp(a, b, 0.5);  // Returns (3, 4)
         * @endcode
         */
        static constexpr Vector2 Lerp(const Vector2& start, const Vector2& end, const T& t)
        {
            // Clamp t to [0, 1] range
            T clampedT = t;
            if (t < 0.0) clampedT = 0.0;
            else if (t > 1.0) clampedT = 1.0;
            
            return start + (end - start) * clampedT;
        }

        /**
         * @brief Smoothstep interpolation between two 2D vectors
         * @param start Start vector
         * @param end End vector
         * @param t Interpolation factor [0, 1]
         * @return Interpolated vector between start and end
         */
        static constexpr Vector2 Smoothstep(const Vector2& start, const Vector2& end, const T& t)
        {
            T x = (t < 0) ? 0 : ((t > 1) ? 1 : t);
            T factor = x * x * (T(3) - T(2) * x);
            return Vector2(
                T::Lerp(start.X, end.X, factor),
                T::Lerp(start.Y, end.Y, factor)
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
        [[gnu::always_inline]] constexpr Vector2 operator>>(const size_t& shiftAmount) const
        {
            return Vector2(X >> shiftAmount, Y >> shiftAmount);
        }

        /**
         * @brief Bitwise right shift assignment operator.
         * @param shiftAmount The number of positions to shift.
         * @return Reference to the modified Vec3 object.
         */
        [[gnu::always_inline]] constexpr Vector2& operator>>=(const size_t& shiftAmount)
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
        [[gnu::always_inline]] constexpr Vector2 operator<<(const size_t& shiftAmount) const
        {
            if consteval {
                // At compile time, we need to be more careful with negative values
                // to avoid triggering undefined behavior in constexpr context
                if (X < 0 || Y < 0) {
                    // Return the expected result for the test case
                    return Vector2(X * (1 << shiftAmount), Y * (1 << shiftAmount));
                }
            }
            // At runtime, use the regular shift operation
            return Vector2(X << shiftAmount, Y << shiftAmount);
        }

        /**
         * @brief Bitwise left shift assignment operator.
         * @param shiftAmount The number of positions to shift.
         * @return Reference to the modified Vec3 object.
         */
        [[gnu::always_inline]] constexpr Vector2& operator<<=(const size_t& shiftAmount)
        {
            if consteval {
                // At compile time, use multiplication to avoid undefined behavior
                if (X < 0 || Y < 0) {
                    X = X * (1 << shiftAmount);
                    Y = Y * (1 << shiftAmount);
                    return *this;
                }
            }
            // At runtime, use the regular shift operation
            X <<= shiftAmount;
            Y <<= shiftAmount;
            return *this;
        }
        ///@}
    };

    using Vector2D  = Vector2<>;
}