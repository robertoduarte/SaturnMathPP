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

        using Position = Row3;

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
                Row0.Dot(point) + Row3.x,
                Row1.Dot(point) + Row3.y,
                Row2.Dot(point) + Row3.z
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
                Vector3D(1.0, 0.0, 0.0),
                Vector3D(0.0, 1.0, 0.0),
                Vector3D(0.0, 0.0, 1.0),
                translation
            );
        }

        /**
         * @brief Creates view matrix for camera with standard precision.
         * Ideal for initial camera setup and important viewpoints.
         * @param eye Camera position.
         * @param target Point camera is looking at.
         * @param up World up vector (usually 0,1,0).
         * @return View transformation matrix.
         */
        static Matrix43 CreateLookAt(
            const Vector3D& eye,
            const Vector3D& target,
            const Vector3D& up)
        {
            const Vector3D zaxis = (eye - target).Normalize();  // Forward
            const Vector3D xaxis = up.Cross(zaxis).Normalize(); // Right
            const Vector3D yaxis = zaxis.Cross(xaxis);          // Up (already normalized)

            return Matrix43(
                xaxis,
                yaxis,
                zaxis,
                Vector3D(
                    -xaxis.Dot(eye),
                    -yaxis.Dot(eye),
                    -zaxis.Dot(eye)
                )
            );
        }

        /**
         * @brief Creates view matrix for camera with fast approximation.
         * Good for dynamic camera movement where slight imprecision is acceptable.
         * @param eye Camera position.
         * @param target Point camera is looking at.
         * @param up World up vector (usually 0,1,0).
         * @return View transformation matrix.
         */
        static Matrix43 FastCreateLookAt(
            const Vector3D& eye,
            const Vector3D& target,
            const Vector3D& up)
        {
            const Vector3D zaxis = (eye - target).FastNormalize();  // Forward
            const Vector3D xaxis = up.Cross(zaxis).FastNormalize(); // Right
            const Vector3D yaxis = zaxis.Cross(xaxis);              // Up (already normalized)

            return Matrix43(
                xaxis,
                yaxis,
                zaxis,
                Vector3D(
                    -xaxis.Dot(eye),
                    -yaxis.Dot(eye),
                    -zaxis.Dot(eye)
                )
            );
        }

        /**
         * @brief Creates view matrix for camera with fastest approximation.
         * Best for temporary views or rapid camera updates.
         * @param eye Camera position.
         * @param target Point camera is looking at.
         * @param up World up vector (usually 0,1,0).
         * @return View transformation matrix.
         */
        static Matrix43 TurboCreateLookAt(
            const Vector3D& eye,
            const Vector3D& target,
            const Vector3D& up)
        {
            const Vector3D zaxis = (eye - target).TurboNormalize();  // Forward
            const Vector3D xaxis = up.Cross(zaxis).TurboNormalize(); // Right
            const Vector3D yaxis = zaxis.Cross(xaxis);               // Up (already normalized)

            return Matrix43(
                xaxis,
                yaxis,
                zaxis,
                Vector3D(
                    -xaxis.Dot(eye),
                    -yaxis.Dot(eye),
                    -zaxis.Dot(eye)
                )
            );
        }

        /**
         * @brief Creates billboard matrix with standard precision.
         * Ideal for important UI elements or precise sprite positioning.
         * @param position Center position of the billboard.
         * @param cameraPos Position of the viewing camera.
         * @param up World up vector (usually 0,1,0).
         * @return Billboard transformation matrix.
         */
        static Matrix43 CreateBillboard(
            const Vector3D& position,
            const Vector3D& cameraPos,
            const Vector3D& up = Vector3D(0.0, 1.0, 0.0))
        {
            const Vector3D look = (cameraPos - position).Normalize();
            const Vector3D right = up.Cross(look).Normalize();
            const Vector3D upVec = look.Cross(right);  // Already normalized

            return Matrix43(
                right,
                upVec,
                look,
                position
            );
        }

        /**
         * @brief Creates billboard matrix with fast approximation.
         * Good for particle effects and dynamic UI elements.
         * @param position Center position of the billboard.
         * @param cameraPos Position of the viewing camera.
         * @param up World up vector (usually 0,1,0).
         * @return Billboard transformation matrix.
         */
        static Matrix43 FastCreateBillboard(
            const Vector3D& position,
            const Vector3D& cameraPos,
            const Vector3D& up = Vector3D(0.0, 1.0, 0.0))
        {
            const Vector3D look = (cameraPos - position).FastNormalize();
            const Vector3D right = up.Cross(look).FastNormalize();
            const Vector3D upVec = look.Cross(right);  // Already normalized

            return Matrix43(
                right,
                upVec,
                look,
                position
            );
        }

        /**
         * @brief Creates billboard matrix with fastest approximation.
         * Best for numerous particles or temporary effects.
         * @param position Center position of the billboard.
         * @param cameraPos Position of the viewing camera.
         * @param up World up vector (usually 0,1,0).
         * @return Billboard transformation matrix.
         */
        static Matrix43 TurboCreateBillboard(
            const Vector3D& position,
            const Vector3D& cameraPos,
            const Vector3D& up = Vector3D(0.0, 1.0, 0.0))
        {
            const Vector3D look = (cameraPos - position).TurboNormalize();
            const Vector3D right = up.Cross(look).TurboNormalize();
            const Vector3D upVec = look.Cross(right);  // Already normalized

            return Matrix43(
                right,
                upVec,
                look,
                position
            );
        }

        /**
         * @brief Extracts scale, rotation, and translation components.
         * Useful for animation, physics, and transformation debugging.
         * Note: Assumes standard TRS (Translation * Rotation * Scale) order.
         * @param scale Output scale vector.
         * @param rotation Output Euler angles (X=pitch, Y=yaw, Z=roll).
         * @param translation Output position vector.
         */
        void Decompose(
            Vector3D& scale,
            Vector3D& rotation,
            Vector3D& translation) const
        {
            // Extract translation (always safe)
            translation = Row3;

            // Extract scale (length of axes)
            scale.x = Vector3D(Row0.x, Row0.y, Row0.z).Length();
            scale.y = Vector3D(Row1.x, Row1.y, Row1.z).Length();
            scale.z = Vector3D(Row2.x, Row2.y, Row2.z).Length();

            // Remove scale from rotation matrix
            Matrix33 rotMat(
                Vector3D(Row0.x / scale.x, Row0.y / scale.x, Row0.z / scale.x),
                Vector3D(Row1.x / scale.y, Row1.y / scale.y, Row1.z / scale.y),
                Vector3D(Row2.x / scale.z, Row2.y / scale.z, Row2.z / scale.z)
            );

            // Extract Euler angles using only Atan2
            // This method avoids ArcSin and is more numerically stable
            
            // For Y (yaw), we can use atan2(x,z) from the rotated x-axis
            rotation.y = Trigonometry::Atan2(rotMat.Row0.x, rotMat.Row0.z);

            // For X (pitch), use atan2 of y component and length of xz projection
            Fxp xzLen = Trigonometry::Sqrt(rotMat.Row0.x * rotMat.Row0.x + rotMat.Row0.z * rotMat.Row0.z);
            rotation.x = Trigonometry::Atan2(rotMat.Row0.y, xzLen);

            if (xzLen > 0.0001f)  // Small threshold for gimbal lock detection
            {
                rotation.z = Trigonometry::Atan2(rotMat.Row1.x * rotMat.Row0.z - rotMat.Row1.z * rotMat.Row0.x,
                                               rotMat.Row2.x * rotMat.Row0.z - rotMat.Row2.z * rotMat.Row0.x);
            }
            else
            {
                // Near gimbal lock: simplified roll calculation
                rotation.z = Trigonometry::Atan2(-rotMat.Row2.x, rotMat.Row1.x);
            }
        }

        /**
         * @brief Fast but less accurate matrix decomposition.
         * Ideal for real-time updates where speed is more important than precision.
         * @param scale Output scale vector.
         * @param rotation Output Euler angles (X=pitch, Y=yaw, Z=roll).
         * @param translation Output position vector.
         */
        void FastDecompose(
            Vector3D& scale,
            Vector3D& rotation,
            Vector3D& translation) const
        {
            // Extract translation (always safe)
            translation = Row3;

            // Extract scale using fast square root
            scale.x = Vector3D(Row0.x, Row0.y, Row0.z).FastLength();
            scale.y = Vector3D(Row1.x, Row1.y, Row1.z).FastLength();
            scale.z = Vector3D(Row2.x, Row2.y, Row2.z).FastLength();

            // Remove scale from rotation matrix
            Matrix33 rotMat(
                Vector3D(Row0.x / scale.x, Row0.y / scale.x, Row0.z / scale.x),
                Vector3D(Row1.x / scale.y, Row1.y / scale.y, Row1.z / scale.y),
                Vector3D(Row2.x / scale.z, Row2.y / scale.z, Row2.z / scale.z)
            );

            // Extract Euler angles using only Atan2
            rotation.y = Trigonometry::Atan2(rotMat.Row0.x, rotMat.Row0.z);

            // Use FastSqrt for xz length
            Fxp xzLen = Trigonometry::FastSqrt(rotMat.Row0.x * rotMat.Row0.x + rotMat.Row0.z * rotMat.Row0.z);
            rotation.x = Trigonometry::Atan2(rotMat.Row0.y, xzLen);

            if (xzLen > 0.0001)  // Small threshold for gimbal lock detection
            {
                rotation.z = Trigonometry::Atan2(rotMat.Row1.x * rotMat.Row0.z - rotMat.Row1.z * rotMat.Row0.x,
                                               rotMat.Row2.x * rotMat.Row0.z - rotMat.Row2.z * rotMat.Row0.x);
            }
            else
            {
                rotation.z = Trigonometry::Atan2(-rotMat.Row2.x, rotMat.Row1.x);
            }
        }

        /**
         * @brief Creates billboard matrix with fast approximations.
         * Ideal for particle effects and UI elements where precision isn't critical.
         * @param position Center position of the billboard.
         * @param cameraPos Position of the viewing camera.
         * @param up World up vector (usually 0,1,0).
         * @return Billboard transformation matrix.
         */
        static Matrix43 CreateFastBillboard(
            const Vector3D& position,
            const Vector3D& cameraPos,
            const Vector3D& up = Vector3D(0, 1, 0))
        {
            const Vector3D look = (cameraPos - position).FastNormalize();
            const Vector3D right = up.Cross(look).FastNormalize();
            const Vector3D upVec = look.Cross(right);  // No need to normalize, will be perpendicular

            return Matrix43(
                right,
                upVec,
                look,
                position
            );
        }

        /**
         * @brief Creates transformation matrix using fast approximations.
         * Ideal for non-critical real-time updates.
         * @param translation Position offset.
         * @param rotation Rotation angles (X,Y,Z).
         * @param scale Scale factors (default: 1,1,1).
         * @return Combined transformation matrix.
         */
        static Matrix43 CreateFastTransform(
            const Vector3D& translation,
            const Vector3D& rotation,
            const Vector3D& scale = Vector3D(1.0, 1.0, 1.0))
        {
            // Use existing rotation methods since they're already optimized
            Matrix43 result = Matrix43::Identity();
            result.Scale(scale);
            result.RotateX(rotation.x);
            result.RotateY(rotation.y);
            result.RotateZ(rotation.z);
            result.Translate(translation);
            return result;
        }

        /**
         * @brief Creates matrix from individual transforms.
         * Applies in TRS order: Translation * Rotation * Scale
         * @param translation Position offset.
         * @param rotation Rotation angles (X,Y,Z).
         * @param scale Scale factors (default: 1,1,1).
         * @return Combined transformation matrix.
         */
        static Matrix43 CreateTransform(
            const Vector3D& translation,
            const Vector3D& rotation,
            const Vector3D& scale = Vector3D(1.0, 1.0, 1.0))
        {
            Matrix43 result = Matrix43::Identity();
            result.Scale(scale);
            result.RotateX(rotation.x);
            result.RotateY(rotation.y);
            result.RotateZ(rotation.z);
            result.Translate(translation);
            return result;
        }

        /**
         * @brief Creates identity matrix.
         * @return Identity transformation matrix.
         */
        static consteval Matrix43 Identity()
        {
            return Matrix43(
                Vector3D(1.0, 0.0, 0.0),
                Vector3D(0.0, 1.0, 0.0),
                Vector3D(0.0, 0.0, 1.0),
                Vector3D(0.0, 0.0, 0.0)
            );
        }
    };
}