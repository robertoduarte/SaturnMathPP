#pragma once

#include "mat43.hpp"

namespace SaturnMath::Types
{
    /**
     * @brief A high-performance matrix stack implementation for hierarchical transformations.
     * 
     * @details MatrixStack implements a traditional matrix stack commonly used in 3D graphics
     * for managing hierarchical transformations. It allows pushing and popping matrices
     * to create parent-child relationships between transformations, which is essential
     * for scene graphs, skeletal animations, and other hierarchical structures.
     * 
     * Key features of this implementation:
     * - Fixed-size array-based stack to avoid dynamic memory allocation during rendering
     * - Automatic identity matrix initialization at the base of the stack
     * - Stack overflow protection to prevent crashes in deep hierarchies
     * - Convenience methods for common transformations (translate, rotate, scale)
     * - Direct point transformation using the current matrix state
     * 
     * Common usage patterns:
     * 1. Push a matrix before applying transformations to child objects
     * 2. Apply transformations (translate, rotate, scale) to the top matrix
     * 3. Use the transformed matrix to render objects
     * 4. Pop the matrix when returning to the parent level
     * 
     * This implementation is designed for performance-critical rendering paths where
     * predictable memory usage and cache-friendly operations are essential. The fixed-size
     * stack ensures that no dynamic memory allocation occurs during traversal of a scene
     * hierarchy, which is particularly important for real-time applications.
     * 
     * @note This class follows the traditional OpenGL-style matrix stack paradigm but
     * is implemented with modern C++ practices and optimized for embedded systems.
     */
    class MatrixStack
    {
    public:
        /**
         * @brief Maximum depth of the matrix stack.
         * 
         * @details Defines the maximum number of matrices that can be pushed onto the stack.
         * This value is chosen to be sufficient for typical game scene hierarchies while
         * avoiding excessive memory usage.
         * 
         * The value of 16 is selected based on the following considerations:
         * - Most game scene hierarchies rarely exceed 10-12 levels of nesting
         * - Each matrix consumes memory (typically 48-64 bytes for a 4x3 or 4x4 matrix)
         * - Embedded systems and performance-critical applications benefit from 
         *   predictable, fixed memory usage
         * 
         * If a push operation would exceed this depth, it is silently ignored to prevent
         * stack overflow, which is preferable to undefined behavior in a real-time system.
         * 
         * @note If your application requires deeper hierarchies, this constant can be
         * adjusted, but be aware of the increased memory footprint.
         */
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
         * @param angleX Angle of rotation around X axis.
         * @param angleY Angle of rotation around Y axis.
         * @param angleZ Angle of rotation around Z axis.
         */
        void RotateTop(const Angle& angleX, const Angle& angleY, const Angle& angleZ)
        {
            stack[currentDepth] = stack[currentDepth] * Matrix33::CreateRotation(angleX, angleY, angleZ);
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
                m.Row0.Dot(point) + m.Row3.X,
                m.Row1.Dot(point) + m.Row3.Y,
                m.Row2.Dot(point) + m.Row3.Z
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