#pragma once

#include "mat33.hpp"

namespace SaturnMath
{
    /**
     * @brief 4x3 matrix optimized for 3D transformations.
     * Inherits rotation from Matrix33 and adds translation.
     * Last row [0,0,0,1] is implicit for memory efficiency.
     */
    struct Matrix43 : public Matrix33
    {
        Vector3D Row3; /**< Translation vector (position). */

        /**
         * @brief Default constructor initializing to identity matrix.
         */
        constexpr Matrix43() : Matrix33(), Row3() {}

        /**
         * @brief Creates transformation from orientation and position.
         * @param up Up vector for orientation.
         * @param direction Forward vector for orientation.
         * @param position Translation vector.
         */
        constexpr Matrix43(const Vector3D& up, const Vector3D& direction, const Vector3D& position)
            : Matrix33(up, direction), Row3(position) {}

        /**
         * @brief Combines rotation matrix with translation.
         * @param rotation Base 3x3 rotation/scale matrix.
         * @param translation Translation vector.
         */
        constexpr Matrix43(const Matrix33& rotation, const Vector3D& translation)
            : Matrix33(rotation), Row3(translation) {}

        /**
         * @brief Creates matrix from individual row vectors.
         * @param row0 Right vector.
         * @param row1 Up vector.
         * @param row2 Forward vector.
         * @param row3 Translation vector.
         */
        constexpr Matrix43(const Vector3D& row0, const Vector3D& row1, const Vector3D& row2, const Vector3D& row3)
            : Matrix33(row0, row1, row2), Row3(row3) {}

        /**
         * @brief Modifies current matrix by adding translation.
         * Ideal for continuous movement and animation.
         * @param translation Translation vector to add.
         * @return Reference to this matrix.
         */
        Matrix43& Translate(const Vector3D& translation)
        {
            Row3 += translation;
            return *this;
        }

        /**
         * @brief Multiplies this matrix by another, combining transformations.
         * @param other Matrix to multiply with.
         * @return Reference to this matrix.
         */
        Matrix43& operator*=(const Matrix43& other)
        {
            const Vector3D oldRow3 = Row3;
            
            // Multiply 3x3 part
            Matrix33::operator*=(other);
            
            // Transform and add translation
            Row3 = oldRow3 + Vector3D(
                Row0.Dot(other.Row3),
                Row1.Dot(other.Row3),
                Row2.Dot(other.Row3)
            );
            
            return *this;
        }

        /**
         * @brief Creates new matrix as product of this and other.
         * @param other Matrix to multiply with.
         * @return Resulting transformation matrix.
         */
        Matrix43 operator*(const Matrix43& other) const
        {
            Matrix43 result(*this);
            result *= other;
            return result;
        }

        /**
         * @brief Transforms a point by this matrix.
         * Applies both rotation and translation.
         * @param point Point to transform.
         * @return Transformed point.
         */
        Vector3D TransformPoint(const Vector3D& point) const
        {
            return Vector3D(
                Row0.Dot(point) + Row3.X,
                Row1.Dot(point) + Row3.Y,
                Row2.Dot(point) + Row3.Z
            );
        }

        /**
         * @brief Transforms a vector by this matrix.
         * Applies only rotation, ignores translation.
         * @param vector Vector to transform.
         * @return Transformed vector.
         */
        Vector3D TransformVector(const Vector3D& vector) const
        {
            return Matrix33::operator*(vector);
        }

        // Static creation methods

        /**
         * @brief Creates translation matrix.
         * @param translation Desired translation vector.
         * @return New translation matrix.
         */
        static constexpr Matrix43 CreateTranslation(const Vector3D& translation)
        {
            return Matrix43(
                Vector3D(1, 0, 0),
                Vector3D(0, 1, 0),
                Vector3D(0, 0, 1),
                translation
            );
        }

        /**
         * @brief Create a billboard matrix that always faces the camera
         * @tparam P Precision level for calculation
         * @param position The position of the billboard
         * @param cameraPosition The position of the camera
         * @param up The up vector (usually Vector3D::UnitY())
         * @return The billboard matrix
         */
        template<Precision P = Precision::Standard>
        static Matrix43 CreateBillboard(
            const Vector3D& position,
            const Vector3D& cameraPosition,
            const Vector3D& up = Vector3D::UnitY())
        {
            // Calculate the look vector from billboard to camera
            Vector3D look = (cameraPosition - position).Normalize<P>();
            
            // Calculate right vector as cross product of up and look
            Vector3D right = up.Cross(look).Normalize<P>();
            
            // Calculate actual up vector as cross product of look and right
            Vector3D actualUp = look.Cross(right);

            // Construct the matrix
            return Matrix43(
                right.X, right.Y, right.Z,
                actualUp.X, actualUp.Y, actualUp.Z,
                look.X, look.Y, look.Z,
                position.X, position.Y, position.Z
            );
        }

        /**
         * @brief Create a look-at matrix for camera positioning
         * @tparam P Precision level for calculation
         * @param eye The position of the camera
         * @param target The point to look at
         * @param up The up vector (usually Vector3D::UnitY())
         * @return The look-at matrix
         */
        template<Precision P = Precision::Standard>
        static Matrix43 CreateLookAt(
            const Vector3D& eye,
            const Vector3D& target,
            const Vector3D& up = Vector3D::UnitY())
        {
            // Calculate the look vector
            Vector3D look = (target - eye).Normalize<P>();
            
            // Calculate right vector as cross product of up and look
            Vector3D right = up.Cross(look).Normalize<P>();
            
            // Calculate actual up vector as cross product of look and right
            Vector3D actualUp = look.Cross(right);

            // Construct the view matrix
            return Matrix43(
                right.X, right.Y, right.Z,
                actualUp.X, actualUp.Y, actualUp.Z,
                look.X, look.Y, look.Z,
                eye.X, eye.Y, eye.Z
            );
        }

        /**
         * @brief Creates transformation matrix
         * @tparam P Precision level for calculation (Turbo not supported)
         * @param translation Position offset
         * @param rotation Rotation angles (X,Y,Z)
         * @param scale Scale factors (default: 1,1,1)
         * @return Transformation matrix
         */
        template<Precision P = Precision::Standard>
        static Matrix43 CreateTransform(
            const Vector3D& translation,
            const Vector3D& rotation,
            const Vector3D& scale = Vector3D(1, 1, 1))
        {
            static_assert(P != Precision::Turbo, "Turbo precision is not supported for CreateTransform");

            const auto [sinX, cosX] = Trigonometry::SinCos(rotation.X);
            const auto [sinY, cosY] = Trigonometry::SinCos(rotation.Y);
            const auto [sinZ, cosZ] = Trigonometry::SinCos(rotation.Z);

            // Create rotation matrix
            Matrix43 result(
                (cosY * cosZ) * scale.X,
                (cosY * sinZ) * scale.X,
                (-sinY) * scale.X,
                ((sinX * sinY * cosZ) - (cosX * sinZ)) * scale.Y,
                ((sinX * sinY * sinZ) + (cosX * cosZ)) * scale.Y,
                (sinX * cosY) * scale.Y,
                ((cosX * sinY * cosZ) + (sinX * sinZ)) * scale.Z,
                ((cosX * sinY * sinZ) - (sinX * cosZ)) * scale.Z,
                (cosX * cosY) * scale.Z,
                translation.X,
                translation.Y,
                translation.Z
            );

            return result;
        }

        /**
         * @brief Extracts scale, rotation, and translation components
         * @tparam P Precision level for calculation (Turbo not supported)
         * @param scale Output scale vector
         * @param rotation Output Euler angles (X=pitch, Y=yaw, Z=roll)
         * @param translation Output translation vector
         */
        template<Precision P = Precision::Standard>
        void Decompose(
            Vector3D& scale,
            Vector3D& rotation,
            Vector3D& translation) const
        {
            static_assert(P != Precision::Turbo, "Turbo precision is not supported for Decompose");

            // Extract translation
            translation = Row3;

            // Extract scale
            scale.X = Vector3D(Row0.X, Row0.Y, Row0.Z).Length<P>();
            scale.Y = Vector3D(Row1.X, Row1.Y, Row1.Z).Length<P>();
            scale.Z = Vector3D(Row2.X, Row2.Y, Row2.Z).Length<P>();

            // Create rotation matrix by removing scale
            Matrix33 rotMat(
                Row0.X / scale.X, Row0.Y / scale.X, Row0.Z / scale.X,
                Row1.X / scale.Y, Row1.Y / scale.Y, Row1.Z / scale.Y,
                Row2.X / scale.Z, Row2.Y / scale.Z, Row2.Z / scale.Z
            );

            // Extract rotation angles (Euler angles in XYZ order)
            rotation.Y = Trigonometry::Asin(-rotMat.Row1.Z);

            // Check for gimbal lock
            if (rotMat.Row1.Z < 0.999999 && rotMat.Row1.Z > -0.999999)
            {
                rotation.X = Trigonometry::Atan2(rotMat.Row2.Z, rotMat.Row3.z);
                rotation.Z = Trigonometry::Atan2(rotMat.Row1.Y, rotMat.Row1.X);
            }
            else
            {
                // Gimbal lock has occurred
                rotation.X = 0;
                rotation.Z = Trigonometry::Atan2(-rotMat.Row2.X, rotMat.Row2.Y);
            }
        }

        /**
         * @brief Creates identity matrix.
         * @return Identity transformation matrix.
         */
        static consteval Matrix43 Identity()
        {
            return Matrix43(
                Vector3D(1, 0, 0),
                Vector3D(0, 1, 0),
                Vector3D(0, 0, 1),
                Vector3D(0, 0, 0)
            );
        }

        /**
         * @brief Creates a translation matrix.
         * @param x X translation
         * @param y Y translation
         * @param z Z translation
         * @return Translation matrix
         */
        static constexpr Matrix43 Translation(const Fxp& x, const Fxp& y, const Fxp& z)
        {
            return Matrix43(
                Vector3D(1, 0, 0),
                Vector3D(0, 1, 0),
                Vector3D(0, 0, 1),
                Vector3D(x, y, z)
            );
        }

        /**
         * @brief Creates a uniform scale matrix.
         * @param scale Scale factor for all axes
         * @return Scale matrix
         */
        static constexpr Matrix43 Scale(const Fxp& scale)
        {
            return Matrix43(
                Vector3D(scale, 0, 0),
                Vector3D(0, scale, 0),
                Vector3D(0, 0, scale),
                Vector3D(0, 0, 0)
            );
        }

        /**
         * @brief Creates a non-uniform scale matrix.
         * @param x X scale factor
         * @param y Y scale factor
         * @param z Z scale factor
         * @return Scale matrix
         */
        static constexpr Matrix43 Scale(const Fxp& x, const Fxp& y, const Fxp& z)
        {
            return Matrix43(
                Vector3D(x, 0, 0),
                Vector3D(0, y, 0),
                Vector3D(0, 0, z),
                Vector3D(0, 0, 0)
            );
        }
    };
}