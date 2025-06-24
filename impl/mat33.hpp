#pragma once

#include "vector3d.hpp"
#include "trigonometry.hpp"

namespace SaturnMath::Types
{
    /**
     * @brief High-performance 3x3 matrix implementation optimized for Saturn hardware.
     *
     * @details The Matrix33 class provides a comprehensive set of matrix operations
     * optimized for 3D transformations, rotations, and linear algebra calculations
     * on Saturn hardware. It uses fixed-point arithmetic for all operations to ensure
     * consistent behavior across platforms and maximize performance.
     * 
     * Key features:
     * - Memory-efficient representation (three row vectors)
     * - Comprehensive set of transformation operations
     * - Optimized for performance-critical graphics and physics calculations
     * - Fixed-point arithmetic for consistent precision and hardware acceleration
     * - Support for various precision levels in calculations
     * 
     * Matrix layout:
     *
     *     | Row0.x Row0.y Row0.z |
     *     | Row1.x Row1.y Row1.z |
     *     | Row2.x Row2.y Row2.z |
     *
     * Where:
     * - Row0 (XAxis): Right vector, defines the local X direction
     * - Row1 (YAxis): Up vector, defines the local Y direction
     * - Row2 (ZAxis): Forward vector, defines the local Z direction
     *
     * Common applications:
     * - 3D rotations and orientations
     * - Local-to-world and world-to-local transformations
     * - Inertia tensors for rigid body physics
     * - Basis transformations
     * - Normal transformations for lighting calculations
     * 
     * Performance considerations:
     * - Matrix multiplication is an O(n³) operation and can be expensive
     * - Inverse calculation is even more costly and should be cached when possible
     * - Consider using specialized functions for common operations:
     *   - CreateRotationX/Y/Z for single-axis rotations
     *   - CreateScale for uniform/non-uniform scaling
     *   - CreateFromAxisAngle for arbitrary axis rotations
     * 
     * Implementation notes:
     * - The rows form an orthonormal basis in a right-handed coordinate system
     * - Matrix operations are designed to minimize temporary object creation
     * - Critical operations have specialized implementations for Saturn hardware
     * 
     * @see Matrix43 For 4x3 matrix with translation component
     * @see MatrixStack For hierarchical transformations
     * @see Vector3D For the underlying vector implementation
     */
    struct Matrix33
    {
        Vector3D Row0; /**< The right vector (XAxis) of the transformation. */
        Vector3D Row1; /**< The up vector (YAxis) of the transformation. */
        Vector3D Row2; /**< The forward vector (ZAxis) of the transformation. */

        /**
         * @brief Default constructor initializing to a zero matrix.
         *
         * This constructor initializes the matrix to a zero matrix, which means all elements are set to zero.
         * This is often used as a starting point for transformations before any operations are applied.
         *
         * @details The zero matrix is represented as:
         *
         *     | 0 0 0 |
         *     | 0 0 0 |
         *     | 0 0 0 |
         */
        constexpr Matrix33() : Row0(), Row1(), Row2() {}

        /**
         * @brief Creates rotation matrix from up and direction vectors.
         *
         * This constructor builds a rotation matrix using an up vector and a direction vector.
         * The right vector is automatically computed as the cross product of up and direction.
         *
         * @param up The up vector defining the local Y axis.
         * @param direction The direction vector defining the local Z axis.
         *
         * @note Ensure that up and direction vectors are not collinear to avoid undefined behavior.
         *
         * @code {.cpp}
         * Matrix33 rotation = Matrix33(
         *     Vector3D(0, 1, 0),    // Up vector
         *     Vector3D(0, 0, 1)     // Direction vector
         * );
         * @endcode 
         */
        constexpr Matrix33(const Vector3D& up, const Vector3D& direction)
        {
            // Normalize direction first
            Row2 = direction.Normalize();
            
            // Compute right vector and normalize only once
            Row0 = up.Cross(Row2).Normalize();
            
            // Recompute up vector using already normalized vectors
            Row1 = Row2.Cross(Row0);  // No need to normalize as Row0 and Row2 are orthonormal
        }

        /**
         * @brief Creates matrix from individual row vectors.
         *
         * This constructor initializes a 3x3 matrix using individual row vectors. Each row vector 
         * defines a direction in world space, allowing for flexible matrix creation.
         *
         * @param row0In The right vector (XAxis).
         * @param row1In The up vector (YAxis).
         * @param row2In The forward vector (ZAxis).
         *
         * @note For proper rotation matrices, ensure the row vectors are orthonormal.
         *
         * @code {.cpp}
         * Matrix33 matrix = Matrix33(
         *     Vector3D(1, 0, 0),    // Right vector
         *     Vector3D(0, 1, 0),    // Up vector
         *     Vector3D(0, 0, 1)     // Forward vector
         * );
         * @endcode 
         */
        constexpr Matrix33(const Vector3D& row0In, const Vector3D& row1In, const Vector3D& row2In) : Row0(row0In), Row1(row1In), Row2(row2In) {}

        /**
         * @brief Multiply this matrix by another matrix in-place.
         *
         * This operation performs matrix multiplication (M = M * other) and stores the result in this matrix.
         * The operation is performed by computing the dot product of each row of this matrix with each column
         * of the other matrix.
         *
         * @param other The matrix to multiply with.
         * @return Reference to this matrix after multiplication.
         *
         * @note Matrix multiplication is not commutative, meaning A * B ≠ B * A.
         *
         * @code {.cpp}
         * Matrix33 matA = Matrix33::CreateRotationX(Angle::FromDegrees(90));
         * Matrix33 matB = Matrix33::CreateRotationY(Angle::FromDegrees(45));
         * matA *= matB; // Combines rotations, first X then Y
         * @endcode 
         */
        constexpr Matrix33& operator*=(const Matrix33& other)
        {
            // Store current values since we'll be modifying the matrix
            const Vector3D oldRow0 = Row0;
            const Vector3D oldRow1 = Row1;
            const Vector3D oldRow2 = Row2;

            // Create a transposed version of the other matrix to access columns efficiently
            const Matrix33 transposed = other.Transposed();

            // Compute new rows using Dot product for better performance
            Row0 = Vector3D(oldRow0.Dot(transposed.Row0), oldRow0.Dot(transposed.Row1), oldRow0.Dot(transposed.Row2));
            Row1 = Vector3D(oldRow1.Dot(transposed.Row0), oldRow1.Dot(transposed.Row1), oldRow1.Dot(transposed.Row2));
            Row2 = Vector3D(oldRow2.Dot(transposed.Row0), oldRow2.Dot(transposed.Row1), oldRow2.Dot(transposed.Row2));

            return *this;
        }

        /**
         * @brief Create a new matrix as the product of this and another matrix.
         *
         * This operation performs matrix multiplication (result = this * other) and returns a new matrix.
         * Unlike operator*=, this operation does not modify the original matrices.
         *
         * @param other The matrix to multiply with.
         * @return A new matrix containing the result of the multiplication.
         *
         * @note This is equivalent to creating a copy of this matrix and using operator*=.
         *
         * @code {.cpp}
         * Matrix33 combined = rotationMatrix * scaleMatrix;
         * @endcode 
         */
        constexpr Matrix33 operator*(const Matrix33& other) const
        {
            Matrix33 result(*this);
            result *= other;
            return result;
        }

        /**
         * @brief Equality comparison operator.
         *
         * Compares two matrices for exact equality by comparing each component.
         *
         * @param other The matrix to compare with.
         * @return true if all components are equal, false otherwise.
         *
         * @code {.cpp}
         * Matrix33 a = Matrix33::Identity();
         * Matrix33 b = Matrix33::Identity();
         * bool equal = (a == b);  // true
         * @endcode
         */
        constexpr bool operator==(const Matrix33& other) const
        {
            return Row0 == other.Row0 && 
                   Row1 == other.Row1 && 
                   Row2 == other.Row2;
        }

        /**
         * @brief Inequality comparison operator.
         *
         * Compares two matrices for inequality by comparing each component.
         *
         * @param other The matrix to compare with.
         * @return true if any component is not equal, false otherwise.
         *
         * @code {.cpp}
         * Matrix33 a = Matrix33::Identity();
         * Matrix33 b = Matrix33::CreateScale(2.0f);
         * bool notEqual = (a != b);  // true
         * @endcode
         */
        constexpr bool operator!=(const Matrix33& other) const
        {
            return !(*this == other);
        }

        /**
         * @brief Transform a vector by this matrix.
         *
         * Applies the transformation represented by this matrix to a vector through
         * matrix-vector multiplication. This is commonly used to:
         * - Transform a point in local space to world space
         * - Apply a rotation to a direction vector
         * - Scale a vector
         *
         * @param v The vector to transform.
         * @return The transformed vector.
         *
         * @note For a rotation matrix, the length of the input vector is preserved.
         *
         * @code {.cpp}
         * Vector3D direction(0, 0, 1);
         * Matrix33 rotation = Matrix33::CreateRotationY(Angle::FromDegrees(90));
         * Vector3D rotated = rotation * direction; // Rotates the vector 90° around Y axis
         * @endcode 
         */
        constexpr Vector3D operator*(const Vector3D& v) const 
        { 
            return Vector3D(Row0.Dot(v), Row1.Dot(v), Row2.Dot(v)); 
        }

        /**
         * @brief Transpose the matrix in-place.
         *
         * Transposes the matrix by swapping elements across the main diagonal.
         * For a matrix M, the transpose is defined as: M[i][j] = M[j][i]
         *
         * @details The transposition is performed as follows:
         *     | a b c |    | a d g |
         *     | d e f | -> | b e h |
         *     | g h i |    | c f i |
         *
         * @return Reference to this matrix after transposition.
         *
         * @note For orthogonal matrices (like pure rotation matrices), 
         * the transpose is equal to the inverse.
         *
         * @code {.cpp}
         * Matrix33 mat = Matrix33::CreateRotationX(Angle::FromDegrees(45));
         * mat.Transpose(); // Transposes the matrix in-place
         * @endcode 
         */
        constexpr Matrix33& Transpose()
        {
            // Swap row0.y and row1.x
            const Fxp m01 = Row0.Y;
            Row0.Y = Row1.X;
            Row1.X = m01;

            // Swap row0.z and row2.x
            const Fxp m02 = Row0.Z;
            Row0.Z = Row2.X;
            Row2.X = m02;

            // Swap row1.z and row2.y
            const Fxp m12 = Row1.Z;
            Row1.Z = Row2.Y;
            Row2.Y = m12;

            return *this;
        }

        /**
         * @brief Apply X-axis rotation to the current matrix.
         *
         * Modifies the current matrix by applying a rotation around the X-axis.
         * This is equivalent to multiplying the current matrix by a rotation matrix.
         *
         * @details The X-axis rotation matrix is:
         *     | 1    0        0    |
         *     | 0  cos(θ)  -sin(θ) |
         *     | 0  sin(θ)   cos(θ) |
         *
         * @param angleX Rotation angle around X-axis.
         * @return Reference to this matrix after rotation.
         *
         * @note This is ideal for continuous transformations like animation
         * as it modifies the existing matrix rather than creating a new one.
         *
         * @code {.cpp}
         * Matrix33 transform = Matrix33::Identity();
         * transform.RotateX(Angle::FromDegrees(45)); // Rotate 45° around X
         * @endcode 
         */
        constexpr Matrix33& RotateX(const Angle& angleX)
        {
            // Compute sin and cos values for the angleX using SinCos
 
            const Fxp sinValue = Trigonometry::Sin(angleX);
            const Fxp cosValue = Trigonometry::Cos(angleX);

            // Update matrix elements to perform rotation around the X-axis
            const Fxp m01 = Row0.Y;
            const Fxp m02 = Row0.Z;
            const Fxp m11 = Row1.Y;
            const Fxp m12 = Row1.Z;
            const Fxp m21 = Row2.Y;
            const Fxp m22 = Row2.Z;

            Row0.Y = (m01 * cosValue) + (m02 * sinValue);
            Row0.Z = -(m01 * sinValue) + (m02 * cosValue);

            Row1.Y = (m11 * cosValue) + (m12 * sinValue);
            Row1.Z = -(m11 * sinValue) + (m12 * cosValue);

            Row2.Y = (m21 * cosValue) + (m22 * sinValue);
            Row2.Z = -(m21 * sinValue) + (m22 * cosValue);

            return *this;
        }

        /**
         * @brief Create a new X-axis rotation matrix.
         *
         * Creates a fresh matrix representing a rotation around the X-axis.
         * This is ideal for initialization and clean transformations.
         *
         * @details The resulting matrix will be:
         *     | 1    0        0    |
         *     | 0  cos(θ)  -sin(θ) |
         *     | 0  sin(θ)   cos(θ) |
         *
         * @param angle Rotation angle around X-axis.
         * @return A new rotation matrix.
         *
         * @code {.cpp}
         * Matrix33 rotation = Matrix33::CreateRotationX(Angle::FromDegrees(90));
         * Vector3D rotated = rotation * Vector3D(0, 1, 0); // Rotates (0,1,0) to (0,0,1)
         * @endcode 
         */
        static constexpr Matrix33 CreateRotationX(const Angle& angle)
        {
            const Fxp sinValue = Trigonometry::Sin(angle);
            const Fxp cosValue = Trigonometry::Cos(angle);
            return Matrix33{
                Vector3D(1, 0, 0),
                Vector3D(0, cosValue, -sinValue),
                Vector3D(0, sinValue, cosValue)
            };
        }

        /**
         * @brief Apply Y-axis rotation to the current matrix.
         *
         * Modifies the current matrix by applying a rotation around the Y-axis.
         * This is equivalent to multiplying the current matrix by a rotation matrix.
         *
         * @details The Y-axis rotation matrix is:
         *     |  cos(θ)  0  sin(θ) |
         *     |    0     1    0    |
         *     | -sin(θ)  0  cos(θ) |
         *
         * @param angleY Rotation angle around Y-axis.
         * @return Reference to this matrix after rotation.
         *
         * @note This is ideal for continuous transformations like animation
         * as it modifies the existing matrix rather than creating a new one.
         *
         * @code {.cpp}
         * Matrix33 transform = Matrix33::Identity();
         * transform.RotateY(Angle::FromDegrees(45)); // Rotate 45° around Y
         * @endcode 
         */
        constexpr Matrix33& RotateY(const Angle& angleY)
        {
            const Fxp sinValue = Trigonometry::Sin(angleY);
            const Fxp cosValue = Trigonometry::Cos(angleY);

            // Update matrix elements to perform rotation around the Y-axis
            const Fxp m00 = Row0.X;
            const Fxp m02 = Row0.Z;
            const Fxp m10 = Row1.X;
            const Fxp m12 = Row1.Z;
            const Fxp m20 = Row2.X;
            const Fxp m22 = Row2.Z;

            Row0.X = (m00 * cosValue) - (m02 * sinValue);
            Row0.Z = (m00 * sinValue) + (m02 * cosValue);

            Row1.X = (m10 * cosValue) - (m12 * sinValue);
            Row1.Z = (m10 * sinValue) + (m12 * cosValue);

            Row2.X = (m20 * cosValue) - (m22 * sinValue);
            Row2.Z = (m20 * sinValue) + (m22 * cosValue);

            return *this;
        }

        /**
         * @brief Create a new Y-axis rotation matrix.
         *
         * Creates a fresh matrix representing a rotation around the Y-axis.
         * This is ideal for initialization and clean transformations.
         *
         * @details The resulting matrix will be:
         *     |  cos(θ)  0  sin(θ) |
         *     |    0     1    0    |
         *     | -sin(θ)  0  cos(θ) |
         *
         * @param angle Rotation angle around Y-axis.
         * @return A new rotation matrix.
         *
         * @code {.cpp}
         * Matrix33 rotation = Matrix33::CreateRotationY(Angle::FromDegrees(90));
         * Vector3D rotated = rotation * Vector3D(1, 0, 0); // Rotates (1,0,0) to (0,0,-1)
         * @endcode 
         */
        static constexpr Matrix33 CreateRotationY(const Angle& angle)
        {
            const Fxp sinValue = Trigonometry::Sin(angle);
            const Fxp cosValue = Trigonometry::Cos(angle);

            return Matrix33{
                Vector3D(cosValue, 0, sinValue),
                Vector3D(0, 1, 0),
                Vector3D(-sinValue, 0, cosValue)
            };
        }

        /**
         * @brief Apply Z-axis rotation to the current matrix.
         *
         * Modifies the current matrix by applying a rotation around the Z-axis.
         * This is equivalent to multiplying the current matrix by a rotation matrix.
         *
         * @details The Z-axis rotation matrix is:
         *     |  cos(θ)  -sin(θ)  0 |
         *     |  sin(θ)   cos(θ)  0 |
         *     |    0        0     1 |
         *
         * @param angleZ Rotation angle around Z-axis.
         * @return Reference to this matrix after rotation.
         *
         * @note This is ideal for continuous transformations like animation
         * as it modifies the existing matrix rather than creating a new one.
         *
         * @code {.cpp}
         * Matrix33 transform = Matrix33::Identity();
         * transform.RotateZ(Angle::FromDegrees(45)); // Rotate 45° around Z
         * @endcode 
         */
        constexpr Matrix33& RotateZ(const Angle& angleZ)
        {
            const Fxp sinValue = Trigonometry::Sin(angleZ);
            const Fxp cosValue = Trigonometry::Cos(angleZ);

            // Update matrix elements to perform rotation around the Z-axis
            const Fxp m00 = Row0.X;
            const Fxp m01 = Row0.Y;
            const Fxp m10 = Row1.X;
            const Fxp m11 = Row1.Y;
            const Fxp m20 = Row2.X;
            const Fxp m21 = Row2.Y;

            Row0.X = (m00 * cosValue) + (m01 * sinValue);
            Row0.Y = -(m00 * sinValue) + (m01 * cosValue);

            Row1.X = (m10 * cosValue) + (m11 * sinValue);
            Row1.Y = -(m10 * sinValue) + (m11 * cosValue);

            Row2.X = (m20 * cosValue) + (m21 * sinValue);
            Row2.Y = -(m20 * sinValue) + (m21 * cosValue);

            return *this;
        }

        /**
         * @brief Create a new Z-axis rotation matrix.
         *
         * Creates a fresh matrix representing a rotation around the Z-axis.
         * This is ideal for initialization and clean transformations.
         *
         * @details The resulting matrix will be:
         *     |  cos(θ)  -sin(θ)  0 |
         *     |  sin(θ)   cos(θ)  0 |
         *     |    0        0     1 |
         *
         * @param angle Rotation angle around Z-axis.
         * @return A new rotation matrix.
         *
         * @code {.cpp}
         * Matrix33 rotation = Matrix33::CreateRotationZ(Angle::FromDegrees(90));
         * Vector3D rotated = rotation * Vector3D(1, 0, 0); // Rotates (1,0,0) to (0,1,0)
         * @endcode 
         */
        static constexpr Matrix33 CreateRotationZ(const Angle& angle)
        {
            const Fxp sinValue = Trigonometry::Sin(angle);
            const Fxp cosValue = Trigonometry::Cos(angle);
            
            return Matrix33{
                Vector3D(cosValue, -sinValue, 0),
                Vector3D(sinValue, cosValue, 0),
                Vector3D(0, 0, 1)
            };
        }

        
        /**
         * @brief Create a rotation matrix from Euler angles.
         *
         * Creates a fresh matrix representing a combined rotation around all three axes.
         * The rotations are applied in the order: Z, then Y, then X (intrinsic rotations).
         *
         * @details The resulting matrix is a combination of three rotations:
         * M = Rx * Ry * Rz, where:
         * - Rx is rotation around X-axis
         * - Ry is rotation around Y-axis
         * - Rz is rotation around Z-axis
         *
         * @param angleX Rotation angle around X-axis (pitch).
         * @param angleY Rotation angle around Y-axis (yaw).
         * @param angleZ Rotation angle around Z-axis (roll).
         * @return A new rotation matrix.
         *
         * @note The order of rotations matters. Changing the order will result
         * in a different final orientation.
         *
         * @code {.cpp}
         * Matrix33 rotation = Matrix33::CreateRotation(
         *     Angle::FromDegrees(30),  // X rotation (pitch)
         *     Angle::FromDegrees(45),  // Y rotation (yaw)
         *     Angle::FromDegrees(60)   // Z rotation (roll)
         * );
         * @endcode 
         */
        static constexpr Matrix33 CreateRotation(const Angle& angleX, const Angle& angleY, const Angle& angleZ)
        {
            const Fxp sinX = Trigonometry::Sin(angleX);
            const Fxp cosX = Trigonometry::Cos(angleX);
            const Fxp sinY = Trigonometry::Sin(angleY);
            const Fxp cosY = Trigonometry::Cos(angleY);
            const Fxp sinZ = Trigonometry::Sin(angleZ);
            const Fxp cosZ = Trigonometry::Cos(angleZ);

            const Fxp m00 = cosY * cosZ;
            const Fxp m01 = -cosY * sinZ;
            const Fxp m02 = sinY;
            const Fxp m10 = sinX * sinY * cosZ + cosX * sinZ;
            const Fxp m11 = -sinX * sinY * sinZ + cosX * cosZ;
            const Fxp m12 = -sinX * cosY;
            const Fxp m20 = -cosX * sinY * cosZ + sinX * sinZ;
            const Fxp m21 = cosX * sinY * sinZ + sinX * cosZ;
            const Fxp m22 = cosX * cosY;

            return Matrix33{
                Vector3D(m00, m01, m02),
                Vector3D(m10, m11, m12),
                Vector3D(m20, m21, m22)
            };
        }


        /**
         * @brief Scale the matrix in-place along each axis.
         *
         * Modifies the current matrix by applying non-uniform scaling along each axis.
         * This is equivalent to multiplying each row by the corresponding scale factor.
         *
         * @details The scaling is applied as follows:
         *     | sx*m00  sx*m01  sx*m02 |
         *     | sy*m10  sy*m11  sy*m12 |
         *     | sz*m20  sz*m21  sz*m22 |
         *
         * @param scale Vector containing scale factors for each axis.
         * @return Reference to this matrix after scaling.
         *
         * @note This operation affects both the orientation and scale of the transformation.
         * For pure scaling, use CreateScale instead.
         *
         * @code {.cpp}
         * Matrix33 transform = Matrix33::Identity();
         * transform.Scale(Vector3D(2, 1, 0.5)); // Scale x by 2, y by 1, z by 0.5
         * @endcode 
         */
        constexpr Matrix33& Scale(const Vector3D& scale)
        {
            Row0.X *= scale.X;
            Row0.Y *= scale.Y;
            Row0.Z *= scale.Z;
            Row1.X *= scale.X;
            Row1.Y *= scale.Y;
            Row1.Z *= scale.Z;
            Row2.X *= scale.X;
            Row2.Y *= scale.Y;
            Row2.Z *= scale.Z;
            return *this;
        }

        /**
         * @brief Calculate the determinant of the matrix.
         *
         * Computes the determinant using the Laplace expansion along the first row.
         * The determinant is a scalar value that represents the scaling factor
         * of the transformation represented by this matrix.
         *
         * @details For a 3x3 matrix:
         *     | a b c |
         *     | d e f |
         *     | g h i |
         * 
         * det = a(ei-fh) - b(di-fg) + c(dh-eg)
         *
         * @return The determinant value.
         *
         * @note Properties of the determinant:
         * - det = 0 indicates a singular (non-invertible) matrix
         * - det = 1 for pure rotation matrices
         * - |det| represents the scale factor of the transformation
         *
         * @code {.cpp}
         * Matrix33 rotation = Matrix33::CreateRotationX(Angle::FromDegrees(90));
         * Fxp det = rotation.Determinant(); // Should be close to 1
         * @endcode 
         */
        constexpr Fxp Determinant() const
        {
            return Row0.X * (Row1.Y * Row2.Z - Row1.Z * Row2.Y) -
                   Row0.Y * (Row1.X * Row2.Z - Row1.Z * Row2.X) +
                   Row0.Z * (Row1.X * Row2.Y - Row1.Y * Row2.X);
        }

        /**
         * @brief Attempt to compute the inverse of the matrix.
         *
         * Computes the inverse matrix if possible. A matrix is invertible if and only if
         * its determinant is non-zero. The inverse of a transformation matrix represents
         * the opposite transformation.
         *
         * @details The inverse is computed using the adjugate matrix method:
         * 1. Calculate the determinant
         * 2. If determinant is non-zero:
         *    - Compute the matrix of cofactors
         *    - Transpose to get the adjugate matrix
         *    - Multiply by 1/determinant
         *
         * @param out The resulting inverse matrix if successful.
         * @return True if matrix was invertible (det ≠ 0), false otherwise.
         *
         * @note For orthogonal matrices (like pure rotation matrices),
         * the inverse is equal to the transpose.
         *
         * @code {.cpp}
         * Matrix33 transform = Matrix33::CreateRotationY(Angle::FromDegrees(45));
         * Matrix33 inverse;
         * if (transform.TryInverse(inverse)) {
         *     // inverse * transform ≈ Identity
         * }
         * @endcode 
         */
        bool constexpr TryInverse(Matrix33& out) const
        {
            const Fxp det = Determinant();
            if (det == 0.0) return false;

            const Fxp invDet = 1.0 / det;

            // Calculate cofactors and adjugate matrix
            out.Row0.X = (Row1.Y * Row2.Z - Row1.Z * Row2.Y) * invDet;
            out.Row0.Y = (Row0.Z * Row2.Y - Row0.Y * Row2.Z) * invDet;
            out.Row0.Z = (Row0.Y * Row1.Z - Row0.Z * Row1.Y) * invDet;

            out.Row1.X = (Row1.Z * Row2.X - Row1.X * Row2.Z) * invDet;
            out.Row1.Y = (Row0.X * Row2.Z - Row0.Z * Row2.X) * invDet;
            out.Row1.Z = (Row0.Z * Row1.X - Row0.X * Row1.Z) * invDet;

            out.Row2.X = (Row1.X * Row2.Y - Row1.Y * Row2.X) * invDet;
            out.Row2.Y = (Row0.Y * Row2.X - Row0.X * Row2.Y) * invDet;
            out.Row2.Z = (Row0.X * Row1.Y - Row0.Y * Row1.X) * invDet;

            return true;
        }

        /**
         * @brief Create a scale matrix.
         *
         * Creates a diagonal matrix that represents a non-uniform scaling transformation.
         * When applied to a vector, each component is multiplied by the corresponding scale factor.
         *
         * @details The resulting matrix will be:
         *     | sx  0   0  |
         *     | 0   sy  0  |
         *     | 0   0   sz |
         *
         * @param scale Vector containing scale factors for each axis.
         * @return A new scale matrix.
         *
         * @note Unlike the Scale() method, this creates a fresh matrix
         * that only represents scaling, without affecting rotation.
         *
         * @code {.cpp}
         * Matrix33 scaleMatrix = Matrix33::CreateScale(Vector3D(2, 2, 2)); // Uniform scale by 2
         * Vector3D scaled = scaleMatrix * Vector3D(1, 1, 1); // Results in (2, 2, 2)
         * @endcode 
         */
        static constexpr Matrix33 CreateScale(const Vector3D& scale)
        {
            return Matrix33(
                Vector3D(scale.X, 0, 0),
                Vector3D(0, scale.Y, 0),
                Vector3D(0, 0, scale.Z)
            );
        }

        /**
         * @brief Create an identity matrix.
         *
         * Creates a 3x3 identity matrix, which represents a null transformation
         * (no rotation, no scale). This is the multiplicative identity for matrices.
         *
         * @details The identity matrix is:
         *     | 1 0 0 |
         *     | 0 1 0 |
         *     | 0 0 1 |
         *
         * Properties of the identity matrix:
         * - M * I = I * M = M for any matrix M
         * - Represents no transformation
         * - Has a determinant of 1
         *
         * @return The 3x3 identity matrix.
         *
         * @code {.cpp}
         * Matrix33 identity = Matrix33::Identity();
         * Vector3D v(1, 2, 3);
         * Vector3D result = identity * v; // Same as v
         * @endcode 
         */
        static consteval Matrix33 Identity()
        {
            return Matrix33(
                Vector3D(1, 0, 0),
                Vector3D(0, 1, 0),
                Vector3D(0, 0, 1)
            );
        }

        /**
         * @brief Returns a new matrix that is the transpose of this matrix.
         * 
         * @return A new transposed matrix.
         */
        constexpr Matrix33 Transposed() const
        {
            return Matrix33(
                Vector3D(Row0.X, Row1.X, Row2.X),
                Vector3D(Row0.Y, Row1.Y, Row2.Y),
                Vector3D(Row0.Z, Row1.Z, Row2.Z)
            );
        }


        // Matrix addition
        constexpr Matrix33 operator+(const Matrix33& other) const
        {
            return Matrix33(
                Row0 + other.Row0,
                Row1 + other.Row1,
                Row2 + other.Row2
            );
        }

        // Matrix subtraction
        constexpr Matrix33 operator-(const Matrix33& other) const
        {
            return Matrix33(
                Row0 - other.Row0,
                Row1 - other.Row1,
                Row2 - other.Row2
            );
        }

        // Scalar multiplication
        constexpr Matrix33 operator*(const Fxp& scalar) const
        {
            return Matrix33(
                Row0 * scalar,
                Row1 * scalar,
                Row2 * scalar
            );
        }

        // Scalar multiplication for integral types (optimized)
        template <typename T>
            requires std::is_integral_v<T>
        constexpr Matrix33 operator*(const T& scalar) const
        {
            return Matrix33(
                Row0 * scalar,
                Row1 * scalar,
                Row2 * scalar
            );
        }

        // Scalar division
        constexpr Matrix33 operator/(const Fxp& scalar) const
        {
            return Matrix33(
                Row0 / scalar,
                Row1 / scalar,
                Row2 / scalar
            );
        }

        // Scalar division for integral types (optimized)
        template <typename T>
            requires std::is_integral_v<T>
        constexpr Matrix33 operator/(const T& scalar) const
        {
            return Matrix33(
                Row0 / scalar,
                Row1 / scalar,
                Row2 / scalar
            );
        }

        // Compound addition assignment
        constexpr Matrix33& operator+=(const Matrix33& other)
        {
            Row0 += other.Row0;
            Row1 += other.Row1;
            Row2 += other.Row2;
            return *this;
        }

        // Compound subtraction assignment
        constexpr Matrix33& operator-=(const Matrix33& other)
        {
            Row0 -= other.Row0;
            Row1 -= other.Row1;
            Row2 -= other.Row2;
            return *this;
        }

        // Compound scalar multiplication assignment
        constexpr Matrix33& operator*=(const Fxp& scalar)
        {
            Row0 *= scalar;
            Row1 *= scalar;
            Row2 *= scalar;
            return *this;
        }

        // Compound scalar multiplication assignment for integral types (optimized)
        template <typename T>
            requires std::is_integral_v<T>
        constexpr Matrix33& operator*=(const T& scalar)
        {
            Row0 *= scalar;
            Row1 *= scalar;
            Row2 *= scalar;
            return *this;
        }

        // Compound scalar division assignment
        constexpr Matrix33& operator/=(const Fxp& scalar)
        {
            Row0 /= scalar;
            Row1 /= scalar;
            Row2 /= scalar;
            return *this;
        }

        // Compound scalar division assignment for integral types (optimized)
        template <typename T>
            requires std::is_integral_v<T>
        constexpr Matrix33& operator/=(const T& scalar)
        {
            Row0 /= scalar;
            Row1 /= scalar;
            Row2 /= scalar;
            return *this;
        }

        // Friend function to allow scalar * matrix
        friend constexpr Matrix33 operator*(const Fxp& scalar, const Matrix33& mat)
        {
            return mat * scalar;
        }

        // Friend function to allow integral scalar * matrix (optimized)
        template <typename T>
            requires std::is_integral_v<T>
        friend constexpr Matrix33 operator*(const T& scalar, const Matrix33& mat)
        {
            return mat * scalar;
        }

        /**
         * @brief Unary negation operator.
         * @return A new matrix with all components negated.
         * 
         * @details Creates a new matrix where each component is the negation
         * of the corresponding component in the original matrix.
         * 
         * Example usage:
         * @code
         * Matrix33 m(Vector3D(1, 2, 3), Vector3D(4, 5, 6), Vector3D(7, 8, 9));
         * Matrix33 neg = -m;  // Results in [(-1,-2,-3), (-4,-5,-6), (-7,-8,-9)]
         * @endcode
         */
        constexpr Matrix33 operator-() const
        {
            return Matrix33(
                -Row0,
                -Row1,
                -Row2
            );
        }
    };
}