#pragma once

#include "mat43.hpp"
#include <stdexcept>

namespace SaturnMath
{
    class MatrixStack
    {
    public:
        static constexpr size_t MAX_DEPTH = 16;  // Typical max depth for game scenes

    private:
        Matrix43 stack[MAX_DEPTH];
        uint8_t currentDepth;

    public:
        /**
         * @brief Default constructor.
         */
        MatrixStack() : currentDepth(0)
        {
            stack[0] = Matrix43::Identity();
        }

        /**
         * @brief Push matrix onto stack.
         * @param matrix Matrix to push.
         */
        void Push(const Matrix43& matrix)
        {
            if (currentDepth >= MAX_DEPTH - 1) return;
            stack[++currentDepth] = matrix;
        }

        /**
         * @brief Pop matrix from stack.
         */
        void Pop()
        {
            if (currentDepth > 0) --currentDepth;
        }

        /**
         * @brief Get reference to top matrix.
         * @return Reference to top matrix.
         */
        Matrix43& Top()
        {
            return stack[currentDepth];
        }

        /**
         * @brief Get const reference to top matrix.
         * @return Const reference to top matrix.
         */
        const Matrix43& Top() const
        {
            return stack[currentDepth];
        }

        /**
         * @brief Clear stack to identity matrix.
         */
        void Clear()
        {
            currentDepth = 0;
            stack[0] = Matrix43::Identity();
        }

        /**
         * @brief Check if stack is empty (only identity matrix).
         * @return True if empty.
         */
        bool IsEmpty() const
        {
            return currentDepth == 0;
        }

        /**
         * @brief Get current stack depth.
         * @return Current depth.
         */
        size_t GetDepth() const
        {
            return currentDepth;
        }

        /**
         * @brief Translate top matrix.
         * @param translation Translation vector.
         */
        void TranslateTop(const Vector3D& translation)
        {
            stack[currentDepth].Translate(translation);
        }

        /**
         * @brief Rotate top matrix.
         * @param rotation Rotation angles (X,Y,Z).
         */
        void RotateTop(const Vector3D& rotation)
        {
            stack[currentDepth] = stack[currentDepth] * Matrix33::CreateRotation(rotation);
        }

        /**
         * @brief Scale top matrix.
         * @param scale Scale factors.
         */
        void ScaleTop(const Vector3D& scale)
        {
            stack[currentDepth].Scale(scale);
        }

        /**
         * @brief Transform point by current matrix.
         * @param point Point to transform.
         * @return Transformed point.
         */
        Vector3D TransformPoint(const Vector3D& point) const
        {
            const Matrix43& m = stack[currentDepth];
            return Vector3D(
                m.Row0.Dot(point) + m.Row3.x,
                m.Row1.Dot(point) + m.Row3.y,
                m.Row2.Dot(point) + m.Row3.z
            );
        }

        /**
         * @brief Transform vector by current matrix (no translation).
         * @param vector Vector to transform.
         * @return Transformed vector.
         */
        Vector3D TransformVector(const Vector3D& vector) const
        {
            const Matrix43& m = stack[currentDepth];
            return Vector3D(
                m.Row0.Dot(vector),
                m.Row1.Dot(vector),
                m.Row2.Dot(vector)
            );
        }
    };
}