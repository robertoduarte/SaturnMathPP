#pragma once

#include "mat33.hpp"

namespace SaturnMath
{
    struct Mat43 : public Mat33
    {
        Vector3D row3; /**< The fourth row (translation vector) of the 4x3 matrix. */

        /**
         * @brief Default constructor initializing a 4x3 matrix with zeros.
         */
        constexpr Mat43() : Mat33(), row3() {}

        /**
         * @brief Constructor initializing a 4x3 matrix with a given up vector, direction vector, and position vector.
         * @param up The up vector.
         * @param direction The direction vector.
         * @param position The position vector.
         */
        constexpr Mat43(const Vector3D& up, const Vector3D& direction, const Vector3D& position) : Mat33(up, direction), row3(position) {}

        /**
         * @brief Constructor initializing a 4x3 matrix with a given 3x3 matrix and a fourth row (translation vector).
         * @param mat33 The 3x3 matrix.
         * @param row3In The fourth row (translation vector) of the matrix.
         */
        constexpr Mat43(const Mat33& mat33, const Vector3D& row3In) : Mat33(mat33), row3(row3In) {}

        /**
         * @brief Constructor initializing a 4x3 matrix with specified rows, including the translation vector.
         * @param row0In The first row of the matrix.
         * @param row1In The second row of the matrix.
         * @param row2In The third row of the matrix.
         * @param row3In The fourth row (translation vector) of the matrix.
         */
        constexpr Mat43(const Vector3D& row0In, const Vector3D& row1In, const Vector3D& row2In, const Vector3D& row3In)
            : Mat33(row0In, row1In, row2In), row3(row3In)
        {
        }

        /**
         * @brief Multiply this matrix by another matrix.
         * @param other The matrix to multiply with.
         * @return The result of the matrix multiplication.
         */
        Mat43 operator*(const Mat43& other) const
        {
            // Transpose the 'other' 3x3 part to prepare for multiplication.
            Mat43 transposed = other;
            transposed.Transpose(); // This will call the 3x3 transpose function.

            return Mat43(

                // Calculate the first three rows of the result matrix by performing dot products.
                Vector3D(row0.Dot(transposed.row0), row0.Dot(transposed.row1), row0.Dot(transposed.row2)),
                Vector3D(row1.Dot(transposed.row0), row1.Dot(transposed.row1), row1.Dot(transposed.row2)),
                Vector3D(row2.Dot(transposed.row0), row2.Dot(transposed.row1), row2.Dot(transposed.row2)),

                // Calculate the fourth row (translation vector) by applying the 3x3 transformation
                // to the translation vector of 'other' and adding the translation of 'this'.
                transposed.row3 + Vector3D(row0.Dot(transposed.row3), row1.Dot(transposed.row3), row2.Dot(transposed.row3))
            );
        }

        /**
         * @brief Static function to create an identity 4x3 matrix.
         * @return The identity matrix.
         */
        static consteval Mat43 Identity()
        {
            return Mat43(
                Vector3D(1.0, 0.0, 0.0),
                Vector3D(0.0, 1.0, 0.0),
                Vector3D(0.0, 0.0, 1.0),
                Vector3D(0.0, 0.0, 0.0)
            );
        }
    };
}