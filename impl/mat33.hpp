#pragma once

#include "vector3d.hpp"
#include "trigonometry.hpp"

namespace SaturnMath
{
    struct Matrix33
    {
        Vector3D Row0; /**< The first row of the 3x3 matrix. */
        Vector3D Row1; /**< The second row of the 3x3 matrix. */
        Vector3D Row2; /**< The third row of the 3x3 matrix. */

        using XAxis = Row0;
        using YAxis = Row1;
        using ZAxis = Row2;

        /**
         * @brief Default constructor initializing a 3x3 matrix with zeros.
         */
        constexpr Matrix33() : Row0(), Row1(), Row2() {}

        /**
         * @brief Constructor initializing a 3x3 matrix with a given up vector and direction vector.
         * @param up The up vector.
         * @param direction The direction vector.
         */
        constexpr Matrix33(const Vector3D& up, const Vector3D& direction) : Row0(up.Cross(direction)), Row1(up), Row2(direction) {}

        /**
         * @brief Constructor initializing a 3x3 matrix with specified rows.
         * @param row0In The first row of the matrix.
         * @param row1In The second row of the matrix.
         * @param row2In The third row of the matrix.
         */
        constexpr Matrix33(const Vector3D& row0In, const Vector3D& row1In, const Vector3D& row2In) : Row0(row0In), Row1(row1In), Row2(row2In) {}

        /**
         * @brief Multiply this matrix by another matrix.
         * @param other The matrix to multiply with.
         * @return The result of the matrix multiplication.
         */
        Matrix33& operator*=(const Matrix33& other)
        {
            // Store current values since we'll be modifying the matrix
            const Vector3D oldRow0 = Row0;
            const Vector3D oldRow1 = Row1;
            const Vector3D oldRow2 = Row2;

            Matrix33 transposed = other;
            transposed.Transpose();

            Row0 = Vector3D(oldRow0.Dot(transposed.Row0), oldRow0.Dot(transposed.Row1), oldRow0.Dot(transposed.Row2));
            Row1 = Vector3D(oldRow1.Dot(transposed.Row0), oldRow1.Dot(transposed.Row1), oldRow1.Dot(transposed.Row2));
            Row2 = Vector3D(oldRow2.Dot(transposed.Row0), oldRow2.Dot(transposed.Row1), oldRow2.Dot(transposed.Row2));

            return *this;
        }

        /**
         * @brief Multiply two matrices.
         * @param other The matrix to multiply with.
         * @return The result of the matrix multiplication.
         */
        Matrix33 operator*(const Matrix33& other) const
        {
            Matrix33 result(*this);
            result *= other;
            return result;
        }

        /**
         * @brief Multiply this matrix by a 3D vector.
         * @param v The vector to multiply with.
         * @return The result of the matrix-vector multiplication.
         */
        Vector3D operator*(const Vector3D& v) const 
        { 
            return Vector3D(Row0.Dot(v), Row1.Dot(v), Row2.Dot(v)); 
        }

        /**
         * @brief Transposes the 3x3 matrix.
         * @return A reference to the modified matrix.
         */
        Matrix33& Transpose()
        {
            // Swap row0.y and row1.x
            const Fxp m01 = Row0.y;
            Row0.y = Row1.x;
            Row1.x = m01;

            // Swap row0.z and row2.x
            const Fxp m02 = Row0.z;
            Row0.z = Row2.x;
            Row2.x = m02;

            // Swap row1.z and row2.y
            const Fxp m12 = Row1.z;
            Row1.z = Row2.y;
            Row2.y = m12;

            return *this;
        }

        /**
         * @brief Modifies current matrix by adding X-axis rotation.
         * Ideal for continuous transformations like animation.
         * @param angleX Rotation angle.
         * @return Reference to this matrix.
         */
        Matrix33& RotateX(const Fxp& angleX)
        {
            // Compute sin and cos values for the angleX
            Fxp sinValue = Trigonometry::Sin(angleX);
            Fxp cosValue = Trigonometry::Cos(angleX);

            // Update matrix elements to perform rotation around the X-axis
            const Fxp m01 = Row0.y;
            const Fxp m02 = Row0.z;
            const Fxp m11 = Row1.y;
            const Fxp m12 = Row1.z;
            const Fxp m21 = Row2.y;
            const Fxp m22 = Row2.z;

            Row0.y = (m01 * cosValue) + (m02 * sinValue);
            Row0.z = -(m01 * sinValue) + (m02 * cosValue);

            Row1.y = (m11 * cosValue) + (m12 * sinValue);
            Row1.z = -(m11 * sinValue) + (m12 * cosValue);

            Row2.y = (m21 * cosValue) + (m22 * sinValue);
            Row2.z = -(m21 * sinValue) + (m22 * cosValue);

            return *this;
        }

        /**
         * @brief Creates a fresh X-axis rotation matrix.
         * Ideal for initialization and clean transformations.
         * @param angle Rotation angle.
         * @return New rotation matrix.
         */
        static constexpr Matrix33 CreateRotationX(const Fxp& angle)
        {
            const Fxp s = Trigonometry::Sin(angle);
            const Fxp c = Trigonometry::Cos(angle);
            return Matrix33(
                Vector3D(1.0, 0.0, 0.0),
                Vector3D(0.0, c, s),
                Vector3D(0.0, -s, c)
            );
        }

        /**
         * @brief Modifies current matrix by adding Y-axis rotation.
         * Ideal for continuous transformations like animation.
         * @param angleY Rotation angle.
         * @return Reference to this matrix.
         */
        Matrix33& RotateY(const Fxp& angleY)
        {
            // Compute sin and cos values for the angleY
            Fxp sinValue = Trigonometry::Sin(angleY);
            Fxp cosValue = Trigonometry::Cos(angleY);

            // Update matrix elements to perform rotation around the Y-axis
            const Fxp m00 = Row0.x;
            const Fxp m02 = Row0.z;
            const Fxp m10 = Row1.x;
            const Fxp m12 = Row1.z;
            const Fxp m20 = Row2.x;
            const Fxp m22 = Row2.z;

            Row0.x = (m00 * cosValue) - (m02 * sinValue);
            Row0.z = (m00 * sinValue) + (m02 * cosValue);

            Row1.x = (m10 * cosValue) - (m12 * sinValue);
            Row1.z = (m10 * sinValue) + (m12 * cosValue);

            Row2.x = (m20 * cosValue) - (m22 * sinValue);
            Row2.z = (m20 * sinValue) + (m22 * cosValue);

            return *this;
        }

        /**
         * @brief Creates a fresh Y-axis rotation matrix.
         * Ideal for initialization and clean transformations.
         * @param angle Rotation angle.
         * @return New rotation matrix.
         */
        static constexpr Matrix33 CreateRotationY(const Fxp& angle)
        {
            const Fxp s = Trigonometry::Sin(angle);
            const Fxp c = Trigonometry::Cos(angle);
            return Matrix33(
                Vector3D(c, 0.0, -s),
                Vector3D(0.0, 1.0, 0.0),
                Vector3D(s, 0.0, c)
            );
        }

        /**
         * @brief Modifies current matrix by adding Z-axis rotation.
         * Ideal for continuous transformations like animation.
         * @param angleZ Rotation angle.
         * @return Reference to this matrix.
         */
        Matrix33& RotateZ(const Fxp& angleZ)
        {
            // Compute sin and cos values for the angleZ
            Fxp sinValue = Trigonometry::Sin(angleZ);
            Fxp cosValue = Trigonometry::Cos(angleZ);

            // Update matrix elements to perform rotation around the Z-axis
            const Fxp m00 = Row0.x;
            const Fxp m01 = Row0.y;
            const Fxp m10 = Row1.x;
            const Fxp m11 = Row1.y;
            const Fxp m20 = Row2.x;
            const Fxp m21 = Row2.y;

            Row0.x = (m00 * cosValue) + (m01 * sinValue);
            Row0.y = -(m00 * sinValue) + (m01 * cosValue);

            Row1.x = (m10 * cosValue) + (m11 * sinValue);
            Row1.y = -(m10 * sinValue) + (m11 * cosValue);

            Row2.x = (m20 * cosValue) + (m21 * sinValue);
            Row2.y = -(m20 * sinValue) + (m21 * cosValue);

            return *this;
        }

        /**
         * @brief Creates a fresh Z-axis rotation matrix.
         * Ideal for initialization and clean transformations.
         * @param angle Rotation angle.
         * @return New rotation matrix.
         */
        static constexpr Matrix33 CreateRotationZ(const Fxp& angle)
        {
            const Fxp s = Trigonometry::Sin(angle);
            const Fxp c = Trigonometry::Cos(angle);
            return Matrix33(
                Vector3D(c, s, 0.0),
                Vector3D(-s, c, 0.0),
                Vector3D(0.0, 0.0, 1.0)
            );
        }

        /**
         * @brief Creates a fresh rotation matrix from Euler angles (X,Y,Z order).
         * Combines all three rotations into a single clean transformation.
         * @param angles Rotation angles for each axis.
         * @return Combined rotation matrix.
         */
        static constexpr Matrix33 CreateRotation(const Vector3D& angles)
        {
            return CreateRotationZ(angles.z) * 
                   CreateRotationY(angles.y) * 
                   CreateRotationX(angles.x);
        }

        /**
         * @brief Scale the matrix in-place by the given factors.
         * @param scale Scale factors for each axis.
         * @return Reference to this matrix.
         */
        Matrix33& Scale(const Vector3D& scale)
        {
            Row0.x *= scale.x;
            Row0.y *= scale.y;
            Row0.z *= scale.z;
            Row1.x *= scale.x;
            Row1.y *= scale.y;
            Row1.z *= scale.z;
            Row2.x *= scale.x;
            Row2.y *= scale.y;
            Row2.z *= scale.z;
            return *this;
        }

        /**
         * @brief Calculate matrix determinant.
         * @return The determinant value.
         */
        constexpr Fxp Determinant() const
        {
            return Row0.x * (Row1.y * Row2.z - Row1.z * Row2.y) -
                   Row0.y * (Row1.x * Row2.z - Row1.z * Row2.x) +
                   Row0.z * (Row1.x * Row2.y - Row1.y * Row2.x);
        }

        /**
         * @brief Try to compute inverse matrix.
         * @param out The resulting inverse matrix if successful.
         * @return True if matrix was invertible, false otherwise.
         */
        bool TryInverse(Matrix33& out) const
        {
            const Fxp det = Determinant();
            if (det == 0) return false;

            const Fxp invDet = Fxp(1) / det;

            // Calculate cofactors and adjugate matrix
            out.Row0.x = (Row1.y * Row2.z - Row1.z * Row2.y) * invDet;
            out.Row0.y = (Row0.z * Row2.y - Row0.y * Row2.z) * invDet;
            out.Row0.z = (Row0.y * Row1.z - Row0.z * Row1.y) * invDet;

            out.Row1.x = (Row1.z * Row2.x - Row1.x * Row2.z) * invDet;
            out.Row1.y = (Row0.x * Row2.z - Row0.z * Row2.x) * invDet;
            out.Row1.z = (Row0.z * Row1.x - Row0.x * Row1.z) * invDet;

            out.Row2.x = (Row1.x * Row2.y - Row1.y * Row2.x) * invDet;
            out.Row2.y = (Row0.y * Row2.x - Row0.x * Row2.y) * invDet;
            out.Row2.z = (Row0.x * Row1.y - Row0.y * Row1.x) * invDet;

            return true;
        }

        /**
         * @brief Create scale matrix.
         * @param scale Scale factors for each axis.
         * @return The scale matrix.
         */
        static constexpr Matrix33 CreateScale(const Vector3D& scale)
        {
            return Matrix33(
                Vector3D(scale.x, 0.0, 0.0),
                Vector3D(0.0, scale.y, 0.0),
                Vector3D(0.0, 0.0, scale.z)
            );
        }

        /**
         * @brief Static function to create an identity 3x3 matrix.
         * @return The identity matrix.
         */
        static consteval Matrix33 Identity()
        {
            return Matrix33(
                Vector3D(1.0, 0.0, 0.0),
                Vector3D(0.0, 1.0, 0.0),
                Vector3D(0.0, 0.0, 1.0)
            );
        }
    };
}