#pragma once

#include "mat33.hpp"

namespace SaturnMath::Types
{
    /**
     * @brief High-performance 4x3 transformation matrix optimized for Saturn hardware.
     * 
     * @details The Matrix4x3 class extends Matrix33 to provide a complete set of
     * affine transformation operations (rotation, scaling, translation) optimized
     * for 3D graphics and physics on Saturn hardware. It uses a memory-efficient
     * 4x3 layout where the last row [0,0,0,1] is implicit.
     * 
     * Key features:
     * - Memory-efficient representation (three rotation rows + translation vector)
     * - Complete set of affine transformation operations
     * - Optimized for performance-critical rendering and physics calculations
     * - Fixed-point arithmetic for consistent precision and hardware acceleration
     * - Support for various precision levels in calculations
     * 
     * Matrix layout:
     *
     *     | Row0.x Row0.y Row0.z |
     *     | Row1.x Row1.y Row1.z |
     *     | Row2.x Row2.y Row2.z |
     *     | Row3.x Row3.y Row3.z |
     *
     * Where:
     * - Row0, Row1, Row2: Rotation and scale components (from Matrix33)
     * - Row3: Translation vector (position)
     * - The implicit 4th column [0,0,0,1] is not stored for memory efficiency
     *
     * This represents the transformation matrix:
     *
     *     | Row0.x Row0.y Row0.z 0 |
     *     | Row1.x Row1.y Row1.z 0 |
     *     | Row2.x Row2.y Row2.z 0 |
     *     | Row3.x Row3.y Row3.z 1 |
     *
     * Common applications:
     * - Object transformations (position, rotation, scale)
     * - Camera view matrices
     * - Hierarchical transformations (parent-child relationships)
     * - Skeletal animation
     * - Coordinate system conversions
     * 
     * Performance considerations:
     * - Matrix multiplication is an O(n³) operation and can be expensive
     * - Inverse calculation is even more costly and should be cached when possible
     * - For hierarchical transformations, consider using MatrixStack
     * - When applying multiple transformations, combine them into a single matrix
     *   when possible to reduce the number of matrix multiplications
     * 
     * Implementation notes:
     * - Inherits rotation/scale components from Matrix33
     * - Adds translation component as Row3
     * - The implicit last row [0,0,0,1] enables affine transformations
     * - Critical operations have specialized implementations for Saturn hardware
     * 
     * @see Matrix33 For rotation-only transformations
     * @see MatrixStack For hierarchical transformations
     * @see Vector3D For the underlying vector implementation
     */
    template<int I = 16, int F = 16> struct Matrix4x3 : public Matrix3x3<I, F>
    {
        using T = FixedPoint<I, F>;
        Vector3<I, F> Row3; /**< Translation vector (position). */

        /**
         * @brief Default constructor initializing to a zero matrix.
         *
         * This constructor initializes the matrix to a zero matrix, which means all elements are set to zero.
         * This is often used as a starting point for transformations before any operations are applied.
         *
         * @details The zero matrix for a 4x3 matrix is represented as:
         *
         *     | 0 0 0 |
         *     | 0 0 0 |
         *     | 0 0 0 |
         *     | 0 0 0 |
         *
         * @note This constructor is typically used when no transformation is required, and the matrix should
         * represent the default state.
         */
        constexpr Matrix4x3() : Matrix3x3<I, F>(), Row3() {}

        /**
         * @brief Creates transformation from orientation and position.
         *
         * This constructor initializes a 4x3 matrix using an up vector, a direction vector, and a position vector.
         * The resulting matrix represents a transformation that combines rotation (defined by the up and direction
         * vectors) and translation (defined by the position vector).
         *
         * @param up The up vector for orientation, defining the vertical direction of the transformation.
         * @param direction The forward vector for orientation, defining the direction the transformation is facing.
         * @param position The translation vector that defines the position of the transformation in world space.
         *
         * @note Ensure that the up and direction vectors are orthogonal to avoid unexpected transformations.
         */
        constexpr Matrix4x3(const Vector3<I, F>& up, const Vector3<I, F>& direction, const Vector3<I, F>& position)
            : Matrix3x3<I, F>(up, direction), Row3(position)
        {
        }

        /**
         * @brief Combines rotation matrix with translation.
         * 
         * This constructor initializes a 4x3 matrix using a base 3x3 rotation/scale matrix and a translation vector. 
         * The resulting matrix represents a transformation that combines the specified rotation and translation.
         * 
         * @param rotation The base 3x3 rotation/scale matrix that defines the rotation component of the transformation.
         * @param translation The translation vector that defines the position of the transformation in world space.
         * 
         * @note This constructor is useful for creating transformation matrices that include both rotation and translation.
         */
        constexpr Matrix4x3(const Matrix3x3<I, F>& rotation, const Vector3<I, F>& translation)
            : Matrix3x3<I, F>(rotation), Row3(translation)
        {
        }

        /**
         * @brief Creates matrix from individual row vectors.
         * 
         * This constructor initializes a 4x3 matrix using individual row vectors. Each row vector defines a direction 
         * in world space, allowing for flexible matrix creation based on specific row definitions.
         * 
         * @param row0 The right vector, representing the first row of the matrix.
         * @param row1 The up vector, representing the second row of the matrix.
         * @param row2 The forward vector, representing the third row of the matrix.
         * @param row3 The translation vector, representing the position of the transformation in world space.
         * 
         * @note This constructor is useful when the individual row vectors are known and need to be combined into a matrix.
         */
        constexpr Matrix4x3(const Vector3<I, F>& row0, const Vector3<I, F>& row1, const Vector3<I, F>& row2, const Vector3<I, F>& row3)
            : Matrix3x3<I, F>(row0, row1, row2), Row3(row3)
        {
        }

        /**
         * @brief Modifies current matrix by adding translation.
         * 
         * This method updates the current matrix by adding a translation vector to the existing translation component. 
         * This is particularly useful for continuous movement and animation, allowing for incremental updates to the position.
         * 
         * @param translation The translation vector to add to the current transformation.
         * 
         * @return Reference to this matrix, allowing for method chaining.
         * 
         * @code {.cpp}
         * Matrix4x3 matrix;
         * matrix.Translate(Vector3D(1, 0, 0)); // Moves the matrix by (1, 0, 0)
         * @endcode 
         */
        constexpr Matrix4x3& Translate(const Vector3<I, F>& translation)
        {
            Row3 += translation;
            return *this;
        }

        /**
         * @brief Multiplies this matrix by another, combining transformations.
         * 
         * This operator overload allows for the multiplication of this matrix by another transformation matrix, 
         * effectively combining the transformations. The result is a new matrix that represents the combined 
         * effect of both transformations.
         * 
         * @param other The matrix to multiply with.
         * 
         * @return Reference to this matrix after multiplication, allowing for method chaining.
         * 
         * @code {.cpp}
         * Matrix4x3 result = matrix1 * matrix2; // Combines transformations of matrix1 and matrix2
         * @endcode 
         */
        constexpr Matrix4x3& operator*=(const Matrix4x3& other)
        {
            // Save the original rows and translation
            const Vector3<I, F> oldRow0 = this->Row0;
            const Vector3<I, F> oldRow1 = this->Row1;
            const Vector3<I, F> oldRow2 = this->Row2;
            const Vector3<I, F> oldRow3 = Row3;

            // Multiply 3x3 part (rotation/scale)
            Matrix3x3<I, F>::operator*=(other);

            // Transform the other matrix's translation by our rotation/scale
            // and add our original translation
            Row3 = Vector3<I, F>(
                oldRow0.Dot(other.Row3),
                oldRow1.Dot(other.Row3),
                oldRow2.Dot(other.Row3)
            ) + oldRow3;

            return *this;
        }

        /**
         * @brief Equality comparison operator.
         *
         * Compares two matrices for exact equality by comparing each component.
         * The comparison includes both the rotation/scale part (from Matrix33)
         * and the translation part (Row3).
         *
         * @param other The matrix to compare with.
         * @return true if all components are equal, false otherwise.
         *
         * @code {.cpp}
         * Matrix4x3 a = Matrix4x3::Identity();
         * Matrix4x3 b = Matrix4x3::Identity();
         * bool equal = (a == b);  // true
         * @endcode
         */
        constexpr bool operator==(const Matrix4x3& other) const
        {
            // Compare both the 3x3 part (via Matrix33::operator==) and the translation
            return Matrix3x3<I, F>::operator==(other) && Row3 == other.Row3;
        }

        /**
         * @brief Inequality comparison operator.
         *
         * Compares two matrices for inequality by comparing each component.
         * The comparison includes both the rotation/scale part (from Matrix33)
         * and the translation part (Row3).
         *
         * @param other The matrix to compare with.
         * @return true if any component is not equal, false otherwise.
         *
         * @code {.cpp}
         * Matrix4x3 a = Matrix4x3::Identity();
         * Matrix4x3 b = Matrix4x3::CreateTranslation(Vector3D(1, 2, 3));
         * bool notEqual = (a != b);  // true
         * @endcode
         */
        constexpr bool operator!=(const Matrix4x3& other) const
        {
            return !(*this == other);
        }

        /**
         * @brief Creates new matrix as sum of this and other.
         * 
         * @param other The matrix to add.
         * @return A new matrix that is the sum of this matrix and the other matrix.
         */
        constexpr Matrix4x3 operator+(const Matrix4x3& other) const
        {
            return Matrix4x3(
                this->Row0 + other.Row0,
                this->Row1 + other.Row1,
                this->Row2 + other.Row2,
                Row3 + other.Row3
            );
        }

        /**
         * @brief Creates new matrix as difference of this and other.
         * 
         * @param other The matrix to subtract.
         * @return A new matrix that is the difference between this matrix and the other matrix.
         */
        constexpr Matrix4x3 operator-(const Matrix4x3& other) const
        {
            return Matrix4x3(
                this->Row0 - other.Row0,
                this->Row1 - other.Row1,
                this->Row2 - other.Row2,
                Row3 - other.Row3
            );
        }

        /**
         * @brief Multiplies this matrix by a scalar value.
         * 
         * @param scalar The scalar value to multiply by.
         * @return A new matrix with each component multiplied by the scalar.
         */
        constexpr Matrix4x3 operator*(const T& scalar) const
        {
            return Matrix4x3(
                this->Row0 * scalar,
                this->Row1 * scalar,
                this->Row2 * scalar,
                Row3 * scalar
            );
        }

        /**
         * @brief Multiplies this matrix by a scalar value.
         * 
         * @tparam U The type of the scalar (integral type)
         * @param scalar The scalar value to multiply by.
         * @return A new matrix with each component multiplied by the scalar.
         */
        template <typename U>
            requires std::is_integral_v<U>
        constexpr Matrix4x3 operator*(const U& scalar) const
        {
            return Matrix4x3(
                this->Row0 * scalar,
                this->Row1 * scalar,
                this->Row2 * scalar,
                Row3 * scalar
            );
        }

        /**
         * @brief Divides this matrix by a scalar value.
         * 
         * @param scalar The scalar value to divide by.
         * @return A new matrix with each component divided by the scalar.
         * @note Division by zero will result in undefined behavior.
         */
        constexpr Matrix4x3 operator/(const T& scalar) const
        {
            return Matrix4x3(
                this->Row0 / scalar,
                this->Row1 / scalar,
                this->Row2 / scalar,
                Row3 / scalar
            );
        }

        /**
         * @brief Divides this matrix by a scalar value.
         * 
         * @tparam U The type of the scalar (integral type)
         * @param scalar The scalar value to divide by.
         * @return A new matrix with each component divided by the scalar.
         * @note Division by zero will result in undefined behavior.
         */
        template <typename U>
            requires std::is_integral_v<U>
        constexpr Matrix4x3 operator/(const U& scalar) const
        {
            return Matrix4x3(
                this->Row0 / scalar,
                this->Row1 / scalar,
                this->Row2 / scalar,
                Row3 / scalar
            );
        }

        /**
         * @brief Adds another matrix to this matrix.
         * 
         * @param other The matrix to add.
         * @return Reference to this matrix after addition.
         */
        constexpr Matrix4x3& operator+=(const Matrix4x3& other)
        {
            this->Row0 += other.Row0;
            this->Row1 += other.Row1;
            this->Row2 += other.Row2;
            Row3 += other.Row3;
            return *this;
        }

        /**
         * @brief Subtracts another matrix from this matrix.
         * 
         * @param other The matrix to subtract.
         * @return Reference to this matrix after subtraction.
         */
        constexpr Matrix4x3& operator-=(const Matrix4x3& other)
        {
            this->Row0 -= other.Row0;
            this->Row1 -= other.Row1;
            this->Row2 -= other.Row2;
            Row3 -= other.Row3;
            return *this;
        }

        /**
         * @brief Multiplies this matrix by a scalar value in-place.
         * 
         * @param scalar The scalar value to multiply by.
         * @return Reference to this matrix after multiplication.
         */
        constexpr Matrix4x3& operator*=(const T& scalar)
        {
            Matrix3x3<I, F>::operator*=(scalar);
            Row3 *= scalar;
            return *this;
        }

        /**
         * @brief Multiplies this matrix by a scalar value in-place.
         * 
         * @tparam U The type of the scalar (integral type)
         * @param scalar The scalar value to multiply by.
         * @return Reference to this matrix after multiplication.
         */
        template <typename U>
            requires std::is_integral_v<U>
        constexpr Matrix4x3& operator*=(const U& scalar)
        {
            Matrix3x3<I, F>::operator*=(scalar);
            Row3 *= scalar;
            return *this;
        }

        /**
         * @brief Divides this matrix by a scalar value in-place.
         * 
         * @param scalar The scalar value to divide by.
         * @return Reference to this matrix after division.
         * @note Division by zero will result in undefined behavior.
         */
        constexpr Matrix4x3& operator/=(const T& scalar)
        {
            Matrix3x3<I, F>::operator/=(scalar);
            Row3 /= scalar;
            return *this;
        }

        /**
         * @brief Divides this matrix by a scalar value in-place.
         * 
         * @tparam U The type of the scalar (integral type)
         * @param scalar The scalar value to divide by.
         * @return Reference to this matrix after division.
         * @note Division by zero will result in undefined behavior.
         */
        template <typename U>
            requires std::is_integral_v<U>
        constexpr Matrix4x3& operator/=(const U& scalar)
        {
            Matrix3x3<I, F>::operator/=(scalar);
            Row3 /= scalar;
            return *this;
        }

        /**
         * @brief Returns the negation of this matrix.
         * 
         * @return A new matrix with all components negated.
         */
        constexpr Matrix4x3 operator-() const
        {
            return Matrix4x3(
                -this->Row0,
                -this->Row1,
                -this->Row2,
                -Row3
            );
        }

        /**
         * @brief Multiplies a scalar by a matrix (scalar * matrix).
         * 
         * @param scalar The scalar value to multiply by.
         * @param matrix The matrix to be multiplied.
         * @return A new matrix with each component multiplied by the scalar.
         */
        friend constexpr Matrix4x3 operator*(const T& scalar, const Matrix4x3& matrix)
        {
            return matrix * scalar;
        }

        /**
         * @brief Multiplies a scalar by a matrix (scalar * matrix).
         * 
         * @tparam U The type of the scalar (integral type)
         * @param scalar The scalar value to multiply by.
         * @param matrix The matrix to be multiplied.
         * @return A new matrix with each component multiplied by the scalar.
         */
        template <typename U>
            requires std::is_integral_v<U>
        friend constexpr Matrix4x3 operator*(const U& scalar, const Matrix4x3& matrix)
        {
            return matrix * scalar;
        }

        /**
         * @brief Creates new matrix as product of this and other.
         * 
         * This operator overload creates a new matrix that is the result of multiplying this matrix by another. 
         * The resulting matrix represents the combined transformations of both matrices, without modifying the original.
         * 
         * @param other The matrix to multiply with.
         * 
         * @return A new Matrix4x3 object that is the product of this matrix and the other matrix.
         * 
         * @code {.cpp}
         * Matrix4x3 result = matrix1 * matrix2; // Creates a new matrix as the product of matrix1 and matrix2
         * @endcode 
         */
        constexpr Matrix4x3 operator*(const Matrix4x3& other) const
        {
            Matrix4x3 result(*this);
            result *= other;
            return result;
        }

        /**
         * @brief Multiplies this matrix by another 3x3 matrix.
         * 
         * This operator overload allows for the multiplication of this matrix by a 3x3 matrix, modifying only the 
         * rotation part of this matrix. The translation component remains unchanged.
         * 
         * @param other The 3x3 matrix to multiply with.
         * 
         * @return Reference to this matrix after multiplication, allowing for method chaining.
         * 
         * @code {.cpp}
         * matrix *= rotationMatrix; // Updates the matrix with the rotation from rotationMatrix
         * @endcode 
         */
        constexpr Matrix4x3& operator*=(const Matrix3x3<I, F>& other)
        {
            Matrix3x3<I, F>::operator*=(other);
            return *this;
        }

        /**
         * @brief Creates new matrix as product of this and another 3x3 matrix.
         * 
         * This operator overload creates a new matrix that is the result of multiplying this matrix by a 3x3 matrix. 
         * The resulting matrix modifies only the rotation part of this matrix, while the translation component remains unchanged.
         * 
         * @param other The 3x3 matrix to multiply with.
         * 
         * @return A new Matrix4x3 object that is the product of this matrix and the 3x3 matrix.
         * 
         * @code {.cpp}
         * Matrix4x3 result = matrix * rotationMatrix; // Creates a new matrix as the product of matrix and rotationMatrix
         * @endcode 
         */
        constexpr Matrix4x3 operator*(const Matrix3x3<I, F>& other) const
        {
            Matrix4x3 result(*this);
            result *= other;
            return result;
        }

        /**
         * @brief Transforms a point by this matrix.
         * 
         * This method applies both rotation and translation to a given point in world space, transforming it 
         * to the corresponding point in the camera's local space.
         * 
         * @param point The point to transform, represented as a Vector3D.
         * 
         * @return The transformed point as a Vector3D.
         * 
         * @code {.cpp}
         * Vector3D transformedPoint = matrix.TransformPoint(Vector3D(1, 2, 3)); // Transforms the point (1, 2, 3)
         * @endcode 
         */
        constexpr Vector3<I, F> TransformPoint(const Vector3<I, F>& point) const
        {
            return Vector3<I, F>(
                this->Row0.Dot(point) + Row3.X,
                this->Row1.Dot(point) + Row3.Y,
                this->Row2.Dot(point) + Row3.Z
            );
        }

        /**
         * @brief Transforms a vector by this matrix.
         * 
         * This method applies only the rotation component of the matrix to a given vector, ignoring any translation. 
         * This is useful for transforming direction vectors without affecting their position.
         * 
         * @param vector The vector to transform, represented as a Vector3D.
         * 
         * @return The transformed vector as a Vector3D.
         * 
         * @code {.cpp}
         * Vector3D transformedVector = matrix.TransformVector(Vector3D(1, 0, 0)); // Transforms the vector (1, 0, 0)
         * @endcode 
         */
        constexpr Vector3<I, F> TransformVector(const Vector3<I, F>& vector) const
        {
            return Matrix3x3<I, F>::operator*(vector);
        }

        /**
         * @brief Inverts the matrix.
         *
         * For orthogonal matrices (e.g., pure rotation matrices), the inverse is the transpose.
         * This method computes the inverse by transposing the rotation part and negating the translation.
         *
         * @return The inverted matrix.
         *
         * @note This method assumes the matrix is orthogonal. For non-orthogonal matrices,
         * this will not produce the correct inverse.
         *
         * @code {.cpp}
         * Matrix4x3 transform = Matrix4x3::CreateTranslation(Vector3D(1, 2, 3));
         * Matrix4x3 inverse = transform.Invert(); // Computes the inverse of the transformation matrix
         * @endcode
         */
        constexpr Matrix4x3 Invert() const {
            Matrix4x3 result;

            // Invert the 3x3 rotation part (transpose for orthogonal matrices)
            result.Row0 = Vector3<I, F>(this->Row0.X, this->Row1.X, this->Row2.X);
            result.Row1 = Vector3<I, F>(this->Row0.Y, this->Row1.Y, this->Row2.Y);
            result.Row2 = Vector3<I, F>(this->Row0.Z, this->Row1.Z, this->Row2.Z);

            // Invert the translation part
            result.Row3 = -Vector3<I, F>(
                result.Row0.Dot(Row3),
                result.Row1.Dot(Row3),
                result.Row2.Dot(Row3)
            );

            return result;
        }

        ///@}
        /** @name Static Creation Methods */
        ///@{

        /**
         * @brief Creates translation matrix.
         * 
         * This static method creates a 4x3 matrix that represents a translation transformation. 
         * The resulting matrix can be used to translate points in world space by the specified translation vector.
         * 
         * @param translation The desired translation vector, represented as a Vector3D.
         * 
         * @return A new Matrix4x3 object representing the translation transformation.
         * 
         * @code {.cpp}
         * Matrix4x3 translationMatrix = Matrix4x3::CreateTranslation(Vector3D(5, 0, 0)); // Translates by (5, 0, 0)
         * @endcode 
         */
        static constexpr Matrix4x3 CreateTranslation(const Vector3<I, F>& translation)
        {
            return Matrix4x3(
                Vector3<I, F>(1, 0, 0),
                Vector3<I, F>(0, 1, 0),
                Vector3<I, F>(0, 0, 1),
                translation
            );
        }

        /**
         * @brief Creates a rotation matrix around the X axis.
         * 
         * This static method creates a 4x3 matrix that represents a rotation around the X axis.
         * The resulting matrix can be used to rotate points around the X axis by the specified angle.
         * 
         * @param angle The rotation angle.
         * @return A new Matrix4x3 object representing the rotation transformation.
         * 
         * @code {.cpp}
         * Matrix4x3 rotationX = Matrix4x3::CreateRotationX(Angle::FromDegrees(90)); // Rotates 90 degrees around X axis
         * @endcode
         */
        static constexpr Matrix4x3 CreateRotationX(const Angle& angle)
        {
            T cosA = Trigonometry::Cos<T>(angle);
            T sinA = Trigonometry::Sin<T>(angle);
            
            return Matrix4x3(
                Vector3<I, F>(1, 0, 0),
                Vector3<I, F>(0, cosA, sinA),
                Vector3<I, F>(0, -sinA, cosA),
                Vector3<I, F>(0, 0, 0)
            );
        }

        /**
         * @brief Rotate this matrix around the X axis.
         * 
         * This method applies a rotation around the X axis to this matrix.
         * The rotation is applied by multiplying this matrix with a rotation matrix.
         * 
         * @param angle The rotation angle.
         * @return Reference to this matrix after rotation.
         * 
         * @code {.cpp}
         * Matrix4x3 transform = Matrix4x3::Identity();
         * transform.RotateX(Angle::FromDegrees(45)); // Rotate 45° around X
         * @endcode
         */
        constexpr Matrix4x3& RotateX(const Angle& angle)
        {
            Matrix3x3<I, F>::RotateX(angle);
            
            // Translation remains unchanged
            return *this;
        }

        /**
         * @brief Creates a rotation matrix around the Y axis.
         * 
         * This static method creates a 4x3 matrix that represents a rotation around the Y axis.
         * The resulting matrix can be used to rotate points around the Y axis by the specified angle.
         * 
         * @param angle The rotation angle.
         * @return A new Matrix4x3 object representing the rotation transformation.
         * 
         * @code {.cpp}
         * Matrix4x3 rotationY = Matrix4x3::CreateRotationY(Angle::FromDegrees(90)); // Rotates 90 degrees around Y axis
         * @endcode
         */
        static constexpr Matrix4x3 CreateRotationY(const Angle& angle)
        {
            T cosA = Trigonometry::Cos<T>(angle);
            T sinA = Trigonometry::Sin<T>(angle);
            
            return Matrix4x3(
                Vector3<I, F>(cosA, 0, -sinA),
                Vector3<I, F>(0, 1, 0),
                Vector3<I, F>(sinA, 0, cosA),
                Vector3<I, F>(0, 0, 0)
            );
        }

        /**
         * @brief Rotate this matrix around the Y axis.
         * 
         * This method applies a rotation around the Y axis to this matrix.
         * The rotation is applied by multiplying this matrix with a rotation matrix.
         * 
         * @param angle The rotation angle.
         * @return Reference to this matrix after rotation.
         * 
         * @code {.cpp}
         * Matrix4x3 transform = Matrix4x3::Identity();
         * transform.RotateY(Angle::FromDegrees(45)); // Rotate 45° around Y
         * @endcode
         */
        constexpr Matrix4x3& RotateY(const Angle& angle)
        {
            // Apply rotation to the 3x3 part (rotation/scale)
            Matrix3x3<I, F>::RotateY(angle);
            
            // Translation remains unchanged
            return *this;
        }

        /**
         * @brief Creates a rotation matrix around the Z axis.
         * 
         * This static method creates a 4x3 matrix that represents a rotation around the Z axis.
         * The resulting matrix can be used to rotate points around the Z axis by the specified angle.
         * 
         * @param angle The rotation angle.
         * @return A new Matrix4x3 object representing the rotation transformation.
         * 
         * @code {.cpp}
         * Matrix4x3 rotationZ = Matrix4x3::CreateRotationZ(Angle::FromDegrees(90)); // Rotates 90 degrees around Z axis
         * @endcode
         */
        static constexpr Matrix4x3 CreateRotationZ(const Angle& angle)
        {
            T cosA = Trigonometry::Cos<T>(angle);
            T sinA = Trigonometry::Sin<T>(angle);
            
            return Matrix4x3(
                Vector3<I, F>(cosA, sinA, 0),
                Vector3<I, F>(-sinA, cosA, 0),
                Vector3<I, F>(0, 0, 1),
                Vector3<I, F>(0, 0, 0)
            );
        }

        /**
         * @brief Rotate this matrix around the Z axis.
         * 
         * This method applies a rotation around the Z axis to this matrix.
         * The rotation is applied by multiplying this matrix with a rotation matrix.
         * 
         * @param angle The rotation angle.
         * @return Reference to this matrix after rotation.
         * 
         * @code {.cpp}
         * Matrix4x3 transform = Matrix4x3::Identity();
         * transform.RotateZ(Angle::FromDegrees(45)); // Rotate 45° around Z
         * @endcode
         */
        constexpr Matrix4x3& RotateZ(const Angle& angle)
        {
            // Apply rotation to the 3x3 part (rotation/scale)
            Matrix3x3<I, F>::RotateZ(angle);
            
            // Translation remains unchanged
            return *this;
        }

        /**
         * @brief Create a billboard matrix that always faces the camera.
         * 
         * This static method generates a billboard matrix that ensures an object always faces the camera. 
         * The resulting matrix can be used for rendering objects like sprites that should always face the camera.
         * 
         * @param position The position of the billboard in world coordinates.
         * @param cameraPosition The position of the camera in world coordinates.
         * @param up The up vector (usually Vector3D::UnitY()), defining the vertical orientation of the billboard.
         * 
         * @return The billboard matrix that transforms points from world space to the billboard's local space.
         * 
         * @code {.cpp}
         * Matrix4x3 billboardMatrix = Matrix4x3::CreateBillboard(
         *     Vector3D(0, 0, 0),   // Billboard position
         *     Vector3D(0, 0, 5),   // Camera position
         *     Vector3D(0, 1, 0)    // Up vector
         * );
         * @endcode 
         * 
         * @note Ensure that the up vector is not collinear with the look vector to avoid undefined behavior.
         */
        static constexpr Matrix4x3 CreateBillboard(
            const Vector3<I, F>& position,
            const Vector3<I, F>& cameraPosition,
            const Vector3<I, F>& up = Vector3<I, F>::UnitY())
        {
            // Calculate the look vector from billboard to camera
            Vector3<I, F> look = (cameraPosition - position).Normalize();

            // Calculate right vector as cross product of up and look
            Vector3<I, F> right = up.Cross(look).Normalize();

            // Calculate actual up vector as cross product of look and right
            Vector3<I, F> actualUp = look.Cross(right);

            // Construct the matrix
            return Matrix4x3(
                Vector3<I, F>(right.X, right.Y, right.Z),
                Vector3<I, F>(actualUp.X, actualUp.Y, actualUp.Z),
                Vector3<I, F>(look.X, look.Y, look.Z),
                position
            );
        }

        /**
         * @brief Creates a look-at matrix for positioning a camera in 3D space.
         *
         * This function generates a view matrix that can be used to position a camera
         * looking at a specific target point from a given eye position, with an up vector
         * to define the camera's vertical orientation.
         *
         * @param eye The position of the camera in world coordinates.
         * @param target The point in world space that the camera is looking at.
         * @param up The up vector, which defines the camera's vertical direction (default is Vector3D::UnitY()).
         * @return A Matrix4x3 representing the look-at transformation.
         *
         * @note This function assumes that the up vector is not collinear with the look vector.
         *
         * @code {.cpp}
         * Matrix4x3 viewMatrix = Matrix4x3::CreateLookAt(
         *     Vector3D(0.0, 0.0, 5.0),  // Eye position
         *     Vector3D(0.0, 0.0, 0.0),  // Target position
         *     Vector3D::UnitY()            // Up vector
         * );
         * @endcode 
         * 
         * @details This function is used to create a view matrix that can be used to position a camera in 3D space.
         * The resulting matrix can be used to transform points from world space to the camera's local space.
         */
        static constexpr Matrix4x3 CreateLookAt(
            const Vector3<I, F>& eye,
            const Vector3<I, F>& target,
            const Vector3<I, F>& up = Vector3<I, F>::UnitY())
        {
            // Calculate the look vector (direction from eye to target)
            Vector3<I, F> look = (target - eye).Normalized();

            // Calculate right vector as cross product of look and up
            Vector3<I, F> right = look.Cross(up).Normalized();

            // Calculate actual up vector as cross product of right and look
            Vector3<I, F> actualUp = right.Cross(look);

            // In a right-handed coordinate system, the camera looks down the negative Z-axis
            // So we need to use -look for the view matrix's Z-axis
            Vector3<I, F> viewZ = -look;

            // Construct the view matrix
            // The translation components are the negative dot product of each basis vector with the eye position
            return Matrix4x3(
                right,                                          // Right vector (X axis)
                actualUp,                                       // Up vector (Y axis)
                viewZ,                                          // View Z axis (points away from camera)
                Vector3<I, F>(-right.Dot(eye), -actualUp.Dot(eye), -viewZ.Dot(eye))  // Translation
            );
        }

        /**
         * @brief Creates transformation matrix.
         * 
         * This static method generates a transformation matrix that combines translation, rotation, and scale. 
         * The resulting matrix can be used to transform points in world space by applying the specified translation, 
         * rotation angles, and scale factors.
         *
         * 
         * @param translation The position offset as a Vector3D.
         * @param rotation The rotation as EulerAngles (pitch, yaw, roll).
         * @param scale The scale factors as a Vector3D (default: 1, 1, 1).
         * 
         * @return A new Matrix4x3 object representing the combined transformation.
         * 
         * @code {.cpp}
         * Matrix4x3 transformMatrix = Matrix4x3::CreateTransform(
         *     Vector3D(1, 2, 3),                                  // Translation
         *     EulerAngles(Angle::Zero(), Angle::HalfPi(), Angle::Zero()),  // 90° rotation around Y axis
         *     Vector3D(2, 2, 2)                                   // Scale
         * );
         * @endcode 
         */
        static constexpr Matrix4x3 CreateTransform(
            const Vector3<I, F>& translation,
            const EulerAngles& rotation,
            const Vector3<I, F>& scale = Vector3<I, F>(1, 1, 1))
        {
            // Create rotation matrix from Euler angles
            Matrix3x3<I, F> rotationMatrix = Matrix3x3<I, F>::CreateRotation(rotation.pitch, rotation.yaw, rotation.roll);

            // Create the transformation matrix using the rotation matrix and translation
            Matrix4x3 result(rotationMatrix, translation);

            // Apply scaling
            result.Row0 *= scale.X;
            result.Row1 *= scale.Y;
            result.Row2 *= scale.Z;

            return result;
        }

        /**
         * @brief Extracts scale, rotation, and translation components.
         * 
         * This method decomposes the transformation matrix into its constituent components: scale, rotation, 
         * and translation. The extracted values can be used for further processing or analysis of the transformation.
         *
         * 
         * @param scale Output parameter for the scale vector.
         * @param rotation Output parameter for the rotation angles (X=pitch, Y=yaw, Z=roll).
         * @param translation Output parameter for the translation vector.
         * 
         * @note The method assumes that the input matrix is a valid transformation matrix. Ensure that the matrix 
         * has not been skewed or sheared, as this may affect the accuracy of the extracted values.
         */
        void Decompose(
            Vector3<I, F>& scale,
            Vector3<I, F>& rotation,
            Vector3<I, F>& translation) const
        {
            // Extract translation
            translation = Row3;

            // Extract scale
            scale.X = Vector3<I, F>(this->Row0.X, this->Row0.Y, this->Row0.Z).Length();
            scale.Y = Vector3<I, F>(this->Row1.X, this->Row1.Y, this->Row1.Z).Length();
            scale.Z = Vector3<I, F>(this->Row2.X, this->Row2.Y, this->Row2.Z).Length();

            // Create rotation matrix by removing scale
            Matrix3x3<I, F> rotMat(
                Vector3<I, F>(this->Row0.X / scale.X, this->Row0.Y / scale.X, this->Row0.Z / scale.X),
                Vector3<I, F>(this->Row1.X / scale.Y, this->Row1.Y / scale.Y, this->Row1.Z / scale.Y),
                Vector3<I, F>(this->Row2.X / scale.Z, this->Row2.Y / scale.Z, this->Row2.Z / scale.Z)
            );

            // Extract rotation angles (Euler angles in XYZ order)
            rotation.Y = Trigonometry::Asin(-rotMat.Row2.Z).ToFxp();

            // Check for gimbal lock
            if (rotMat.Row2.Z < 0.999999 && rotMat.Row2.Z > -0.999999)
            {
                rotation.X = Trigonometry::Atan2(rotMat.Row2.Z, rotMat.Row2.Z).ToFxp();
                rotation.Z = Trigonometry::Atan2(rotMat.Row1.Y, rotMat.Row1.X).ToFxp();
            }
            else
            {
                // Gimbal lock has occurred
                rotation.X = 0;
                rotation.Z = Trigonometry::Atan2(-rotMat.Row2.X, rotMat.Row2.Y).ToFxp();
            }
        }

        /**
         * @brief Creates identity matrix.
         * 
         * This static method generates a 4x3 identity matrix, which is a special type of matrix that does not 
         * change the value of any vector when multiplied by it. The identity matrix is often used as a starting 
         * point for transformations or to reset a transformation.
         * 
         * @return A new Matrix4x3 object representing the identity transformation.
         * 
         * @code {.cpp}
         * Matrix4x3 identityMatrix = Matrix4x3::Identity(); // Creates an identity matrix
         * @endcode 
         */
        static consteval Matrix4x3 Identity()
        {
            return Matrix4x3(
                Vector3<I, F>(1, 0, 0),
                Vector3<I, F>(0, 1, 0),
                Vector3<I, F>(0, 0, 1),
                Vector3<I, F>(0, 0, 0)
            );
        }

        /**
         * @brief Creates a translation matrix.
         * 
         * This static method generates a 4x3 translation matrix that represents a transformation that moves 
         * points in world space by the specified translation vector.
         * 
         * @param x The X translation component.
         * @param y The Y translation component.
         * @param z The Z translation component.
         * 
         * @return A new Matrix4x3 object representing the translation transformation.
         * 
         * @code {.cpp}
         * Matrix4x3 translationMatrix = Matrix4x3::Translation(5, 0, 0); // Translates by (5, 0, 0)
         * @endcode 
         */
        static constexpr Matrix4x3 Translation(const T& x, const T& y, const T& z)
        {
            return Matrix4x3(
                Vector3<I, F>(1, 0, 0),
                Vector3<I, F>(0, 1, 0),
                Vector3<I, F>(0, 0, 1),
                Vector3<I, F>(x, y, z)
            );
        }

        /**
         * @brief Creates a uniform scale matrix.
         * 
         * This static method generates a 4x3 scale matrix that represents a transformation that scales 
         * points uniformly along all axes by the specified scale factor.
         * 
         * @param scale The scale factor for all axes.
         * 
         * @return A new Matrix4x3 object representing the uniform scale transformation.
         * 
         * @code {.cpp}
         * Matrix4x3 scaleMatrix = Matrix4x3::Scale(2); // Scales by a factor of 2
         * @endcode
         */
        static constexpr Matrix4x3 Scale(const T& scale)
        {
            return Matrix4x3(
                Vector3<I, F>(scale, 0, 0),
                Vector3<I, F>(0, scale, 0),
                Vector3<I, F>(0, 0, scale),
                Vector3<I, F>(0, 0, 0)
            );
        }

        /**
         * @brief Creates a non-uniform scale matrix.
         * 
         * This static method generates a 4x3 scale matrix that represents a transformation that scales 
         * points by different factors along each axis, allowing for non-uniform scaling.
         * 
         * @param x The X scale factor.
         * @param y The Y scale factor.
         * @param z The Z scale factor.
         * 
         * @return A new Matrix4x3 object representing the non-uniform scale transformation.
         * 
         * @code {.cpp}
         * Matrix4x3 nonUniformScaleMatrix = Matrix4x3::Scale(2, 1, 0.5); // Scales by (2, 1, 0.5)
         * @endcode 
         */
        static constexpr Matrix4x3 Scale(const T& x, const T& y, const T& z)
        {
            return Matrix4x3(
                Vector3<I, F>(x, 0, 0),
                Vector3<I, F>(0, y, 0),
                Vector3<I, F>(0, 0, z),
                Vector3<I, F>(0, 0, 0)
            );
        }

        /**
         * @brief Creates a uniform scale matrix from a Vector3D.
         * 
         * This static method generates a 4x3 scale matrix that represents a transformation that scales 
         * points uniformly along all axes by the specified scale factors from a Vector3D.
         * 
         * @param scale The Vector3D containing scale factors for all axes.
         * 
         * @return A new Matrix4x3 object representing the uniform scale transformation.
         * 
         * @code {.cpp}
         * Matrix4x3 scaleMatrix = Matrix4x3::Scale(Vector3D(2, 2, 2)); // Scales by a factor of 2
         * @endcode
         */
        static constexpr Matrix4x3 Scale(const Vector3<I, F>& scale)
        {
            return Scale(scale.X, scale.Y, scale.Z);
        }
        ///@}
    };

    // Legacy alias for default precision (Q16.16)
    using Matrix43    = Matrix4x3<>;
}