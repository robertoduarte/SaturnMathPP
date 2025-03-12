#pragma once

#include "fxp.hpp"
#include "precision.hpp"
#include "sort_order.hpp"

namespace SaturnMath::Types
{
    /**
     * @brief A high-performance two-dimensional vector implementation using fixed-point arithmetic.
     * 
     * @details Vector2D provides a comprehensive set of 2D vector operations optimized for
     * Saturn hardware. It uses fixed-point arithmetic (Fxp) for all components to avoid
     * floating-point operations while maintaining high precision.
     * 
     * Key features:
     * - Memory-efficient representation (two Fxp values)
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
     * @see Fxp For details on the fixed-point implementation
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
         * 
         * @details Calculates the magnitude (Euclidean length) of the vector.
         * The precision template parameter controls the calculation method:
         * - Standard precision: Uses exact square root calculation
         * - Turbo precision: Uses fast approximation with alpha-beta coefficients
         * 
         * Example usage:
         * @code
         * Vector2D v(3, 4);
         * Fxp length = v.Length();  // Returns 5 (standard precision)
         * Fxp fastLength = v.Length<Precision::Turbo>();  // Returns approximate length (faster)
         * @endcode
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
         * @brief Calculate the squared length of the vector.
         * @return The squared length as an Fxp value.
         * @details Useful for comparisons where the actual length is not needed.
         * 
         * Example usage:
         * @code
         * Vector2D v1(1, 2);
         * Vector2D v2(4, 6);
         * if (v1.LengthSquared() < v2.LengthSquared()) {
         *     // v1 is shorter than v2
         * }
         * @endcode
         */
        constexpr Fxp LengthSquared() const {
            return Dot(*this);
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
         * Vector2D v(3, 4);
         * Vector2D unitV = v.Normalize();  // Returns (0.6, 0.8) with standard precision
         * Vector2D fastUnitV = v.Normalize<Precision::Turbo>();  // Returns approximate unit vector (faster)
         * @endcode
         */
        template<Precision P = Precision::Standard>
        constexpr Vector2D Normalize() const
        {
            Fxp length = Length<P>();
            if (length != 0)
                return Vector2D(X / length, Y / length);
            return Vector2D();
        }

        /**
         * @brief Calculate the Euclidean distance from this vector to another vector.
         * @tparam P Precision level for calculation
         * @param other The other vector to calculate the distance to.
         * @return The distance as an Fxp value.
         * @details Computes the distance using the formula: sqrt((X - other.X)^2 + (Y - other.Y)^2).
         * The precision template parameter controls the calculation method:
         * - Standard precision: Uses exact square root calculation
         * - Turbo precision: Uses fast approximation
         * 
         * Example usage:
         * @code
         * Vector2D v1(1, 2);
         * Vector2D v2(4, 6);
         * Fxp distance = v1.DistanceTo(v2);  // Computes distance between (1, 2) and (4, 6)
         * Fxp fastDistance = v1.DistanceTo<Precision::Turbo>(v2);  // Computes approximate distance (faster)
         * @endcode
         */
        template<Precision P = Precision::Standard>
        constexpr Fxp DistanceTo(const Vector2D& other) const {
            return (*this - other).Length<P>();
        }

        /**
         * @brief Calculate the dot product of this vector with another vector.
         * @param vec The other vector.
         * @return The dot product as an Fxp value.
         * @details Computes the dot product using the formula: X*vec.X + Y*vec.Y.
         * The dot product represents the cosine of the angle between two vectors 
         * multiplied by their lengths.
         * 
         * Example usage:
         * @code
         * Vector2D v1(1, 0);  // Unit vector along X
         * Vector2D v2(0, 1);  // Unit vector along Y
         * Fxp dotProduct = v1.Dot(v2);  // Returns 0 (perpendicular vectors)
         * 
         * Vector2D v3(1, 1);
         * Vector2D v4(2, 3);
         * Fxp dotProduct2 = v3.Dot(v4);  // Returns 5 (1*2 + 1*3)
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
         * 
         * @details Multiplies each component of the vector by the scalar value.
         * 
         * Example usage:
         * @code
         * Vector2D v(1, 2);
         * v *= 2.5_fxp;  // Results in v = (2.5, 5)
         * @endcode
         */
        constexpr Vector2D& operator*=(const Fxp& scalar)
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
         * This specialized version uses Fxp's optimized integral multiplication
         * for better performance on Saturn hardware.
         * 
         * Example usage:
         * @code
         * Vector2D v(1, 2);
         * v *= 2;  // Results in v = (2, 4) with optimized integral multiplication
         * @endcode
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr Vector2D& operator*=(const T& scalar)
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
         * Vector2D v(4, 6);
         * v /= 2_fxp;  // Results in v = (2, 3)
         * @endcode
         */
        constexpr Vector2D& operator/=(const Fxp& scalar)
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
         * This specialized version uses Fxp's optimized integral division
         * for better performance on Saturn hardware.
         * 
         * Example usage:
         * @code
         * Vector2D v(10, 20);
         * v /= 5;  // Results in v = (2, 4) with optimized integral division
         * @endcode
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr Vector2D& operator/=(const T& scalar)
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
         * Vector2D v1(1, 2);
         * Vector2D v2(3, 4);
         * v1 += v2;  // Results in v1 = (4, 6)
         * @endcode
         */
        constexpr Vector2D& operator+=(const Vector2D& vec)
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
         * Vector2D v1(5, 7);
         * Vector2D v2(2, 3);
         * v1 -= v2;  // Results in v1 = (3, 4)
         * @endcode
         */
        constexpr Vector2D& operator-=(const Vector2D& vec)
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
         * Vector2D v(1, 2);
         * Vector2D result = v * 3_fxp;  // Results in result = (3, 6)
         * @endcode
         */
        constexpr Vector2D operator*(const Fxp& scalar) const
        {
            Vector2D result(*this);
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
         * Vector2D v(1, 2);
         * Vector2D result = v * 3;  // Results in result = (3, 6) with optimized integral multiplication
         * @endcode
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr Vector2D operator*(const T& scalar) const
        {
            Vector2D result(*this);
            result *= scalar;
            return result;
        }

        /**
         * @brief Multiply an integral scalar by a Vector2D.
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
         * Vector2D v(1, 2);
         * Vector2D result = 3 * v;  // Results in result = (3, 6) with optimized integral multiplication
         * @endcode
         */
        template<typename T>
            requires std::is_integral_v<T>
        friend constexpr Vector2D operator*(const T& scalar, const Vector2D& vec)
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
         * Vector2D v1(1, 2);
         * Vector2D v2(3, 4);
         * Vector2D result = v1 + v2;  // Results in result = (4, 6)
         * @endcode
         */
        constexpr Vector2D operator+(const Vector2D& vec) const
        {
            return Vector2D(X + vec.X, Y + vec.Y);
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
         * Vector2D v1(5, 7);
         * Vector2D v2(2, 3);
         * Vector2D result = v1 - v2;  // Results in result = (3, 4)
         * @endcode
         */
        constexpr Vector2D operator-(const Vector2D& vec) const
        {
            return Vector2D(X - vec.X, Y - vec.Y);
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
         * Vector2D v(6, 8);
         * Vector2D result = v / 2_fxp;  // Results in result = (3, 4)
         * @endcode
         */
        constexpr Vector2D operator/(const Fxp& scalar) const
        {
            return Vector2D(X / scalar, Y / scalar);
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
         * Vector2D v(10, 20);
         * Vector2D result = v / 5;  // Results in result = (2, 4) with optimized integral division
         * @endcode
         */
        template<typename T>
            requires std::is_integral_v<T>
        constexpr Vector2D operator/(const T& scalar) const
        {
            Vector2D result(*this);
            result /= scalar;
            return result;
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

        /**
         * @brief Less than operator.
         * @param vec The Vector2D object to compare with.
         * @return True if this vector is less than the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y).
         */
        constexpr bool operator<(const Vector2D& vec) const
        {
            return (X < vec.X) || (X == vec.X && Y < vec.Y);
        }

        /**
         * @brief Less than or equal operator.
         * @param vec The Vector2D object to compare with.
         * @return True if this vector is less than or equal to the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y).
         */
        constexpr bool operator<=(const Vector2D& vec) const
        {
            return (X < vec.X) || (X == vec.X && Y <= vec.Y);
        }

        /**
         * @brief Greater than operator.
         * @param vec The Vector2D object to compare with.
         * @return True if this vector is greater than the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y).
         */
        constexpr bool operator>(const Vector2D& vec) const
        {
            return (X > vec.X) || (X == vec.X && Y > vec.Y);
        }

        /**
         * @brief Greater than or equal operator.
         * @param vec The Vector2D object to compare with.
         * @return True if this vector is greater than or equal to the provided vector, false otherwise.
         * @details Compares vectors lexicographically (X first, then Y).
         */
        constexpr bool operator>=(const Vector2D& vec) const
        {
            return (X > vec.X) || (X == vec.X && Y >= vec.Y);
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