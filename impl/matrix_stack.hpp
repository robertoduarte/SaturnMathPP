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
    template<int I = 16, int F = 16>
    class MatrixStackX
    {
        using T = FixedPoint<I, F>;
        using Vec3 = Vector3<I, F>;
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
         */
        static constexpr uint8_t MAX_DEPTH = 16;

    private:
        Matrix4x3<I, F> stack[MAX_DEPTH];
        uint8_t currentDepth = 0;
    public:
        /**
         * @brief Default constructor. Initializes stack with identity matrix at base.
         *
         * @details The stack starts at depth 0 with an identity matrix, ready for
         * immediate use in transformation chains.
         */
        constexpr MatrixStackX() : stack{}, currentDepth(0)
        {
            stack[0] = Matrix4x3<I, F>::Identity();
        }

        /**
         * @brief Pushes a matrix onto the stack.
         *
         * @param matrix The matrix to push onto the stack.
         *
         * @note If the stack is at MAX_DEPTH, the push is silently ignored to
         * prevent stack overflow.
         */
        constexpr void Push(const Matrix4x3<I, F>& matrix)
        {
            if (currentDepth >= MAX_DEPTH - 1) return;
            stack[++currentDepth] = matrix;
        }

        /**
         * @brief Pops the top matrix from the stack.
         *
         * @note If the stack is at depth 0 (only identity), the pop is ignored.
         */
        constexpr void Pop()
        {
            if (currentDepth > 0) --currentDepth;
        }

        /**
         * @brief Gets a mutable reference to the top matrix.
         * @return Reference to the matrix at the top of the stack.
         */
        constexpr Matrix4x3<I, F>& Top()
        {
            return stack[currentDepth];
        }

        /**
         * @brief Gets a const reference to the top matrix.
         * @return Const reference to the matrix at the top of the stack.
         */
        constexpr const Matrix4x3<I, F>& Top() const
        {
            return stack[currentDepth];
        }

        /**
         * @brief Clears the stack to a single identity matrix.
         *
         * @details Resets the stack to its initial state with depth 0 and an
         * identity matrix at the base.
         */
        constexpr void Clear()
        {
            currentDepth = 0;
            stack[0] = Matrix4x3<I, F>::Identity();
        }

        /**
         * @brief Checks if the stack is empty (at depth 0).
         * @return true if only the base identity matrix remains, false otherwise.
         */
        constexpr bool IsEmpty() const
        {
            return currentDepth == 0;
        }

        /**
         * @brief Gets the current stack depth.
         * @return The number of matrices pushed beyond the base (0 means only identity).
         */
        constexpr size_t GetDepth() const
        {
            return currentDepth;
        }

        /**
         * @brief Translates the top matrix by the given vector.
         * @param translation The translation vector to apply.
         *
         * @details Multiplies the top matrix by a translation matrix,
         * effectively moving the origin of the current transformation.
         */
        constexpr void TranslateTop(const Vec3& translation)
        {
            stack[currentDepth] = stack[currentDepth] * Matrix4x3<I, F>::CreateTranslation(translation);
        }

        /**
         * @brief Rotates the top matrix by the given Euler angles.
         * @param angleX Rotation angle around X-axis (pitch).
         * @param angleY Rotation angle around Y-axis (yaw).
         * @param angleZ Rotation angle around Z-axis (roll).
         *
         * @details Multiplies the top matrix by a rotation matrix created
         * from the specified Euler angles. Rotations are applied in Z, Y, X order.
         */
        constexpr void RotateTop(const Angle& angleX, const Angle& angleY, const Angle& angleZ)
        {
            stack[currentDepth] = stack[currentDepth] * Matrix3x3<I, F>::CreateRotation(angleX, angleY, angleZ);
        }

        /**
         * @brief Scales the top matrix by the given factors.
         * @param scale Per-axis scale factors.
         *
         * @details Multiplies the top matrix by a scaling matrix,
         * affecting the scale of subsequent transformations.
         */
        constexpr void ScaleTop(const Vec3& scale)
        {
            stack[currentDepth] = stack[currentDepth] * Matrix4x3<I, F>::CreateScale(scale);
        }

        /**
         * @brief Transforms a point by the current top matrix.
         * @param point The point to transform.
         * @return The transformed point with rotation and translation applied.
         */
        constexpr Vec3 TransformPoint(const Vec3& point) const
        {
            const Matrix4x3<I, F>& m = stack[currentDepth];
            return Vec3(
                m.Row0.Dot(point) + m.Row3.X,
                m.Row1.Dot(point) + m.Row3.Y,
                m.Row2.Dot(point) + m.Row3.Z
            );
        }

        /**
         * @brief Transforms a direction vector by the current top matrix.
         * @param vector The direction vector to transform.
         * @return The transformed vector with rotation only (no translation).
         */
        constexpr Vec3 TransformVector(const Vec3& vector) const
        {
            const Matrix4x3<I, F>& m = stack[currentDepth];
            return Vec3(
                m.Row0.Dot(vector),
                m.Row1.Dot(vector),
                m.Row2.Dot(vector)
            );
        }
    };

    using MatrixStack = MatrixStackX<>;  /**< Default instantiation alias */
}