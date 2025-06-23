#pragma once

/**
 * @file test_matrix_stack.hpp
 * @brief Compile-time unit tests for the MatrixStack class
 *
 * This file contains comprehensive tests for the MatrixStack class, covering:
 * - Stack operations (push, pop, top)
 * - Identity and clear
 * - Depth and empty checks
 * - Transformations (translate, rotate, scale)
 * - Point and vector transformation
 *
 * All tests are performed at compile-time using static_assert where possible.
 */

#include "../impl/matrix_stack.hpp"
#include "test_mat43.hpp" // For Matrix43-related helpers, if needed

namespace SaturnMath::Tests
{
    using namespace SaturnMath::Types;

    namespace
    {
        // Helper function to check if a matrix is identity
        constexpr bool IsIdentityMatrix43(const Matrix43& m)
        {
            return m.Row0 == Vector3D(1, 0, 0) &&
                   m.Row1 == Vector3D(0, 1, 0) &&
                   m.Row2 == Vector3D(0, 0, 1) &&
                   m.Row3 == Vector3D(0, 0, 0);
        }
    }

    struct MatrixStackTests
    {
        static constexpr void TestConstructionAndIdentity()
        {
            constexpr MatrixStack stack;
            static_assert(stack.IsEmpty(), "New stack should be empty (identity only)");
            static_assert(stack.GetDepth() == 0, "Initial stack depth should be 0");
            static_assert(IsIdentityMatrix43(stack.Top()), "Top of new stack should be identity");
        }

        static constexpr void TestPushPop()
        {
            // Use a lambda to work around constexpr limitations with mutable state
            constexpr auto test = []() {
                MatrixStack stack;
                Matrix43 m = Matrix43::Identity();
                m.Row3.X = 42;
                stack.Push(m);
                
                if (stack.GetDepth() != 1) return false;
                if (stack.Top().Row3.X != 42) return false;
                
                stack.Pop();
                
                return stack.GetDepth() == 0 && IsIdentityMatrix43(stack.Top());
            };
            
            static_assert(test(), "Push/Pop operations failed");
        }

        static constexpr void TestClear()
        {
            constexpr auto test = []() {
                MatrixStack stack;
                Matrix43 m = Matrix43::Identity();
                m.Row3.X = 1;
                stack.Push(m);
                stack.Clear();
                return stack.IsEmpty() && IsIdentityMatrix43(stack.Top());
            };
            
            static_assert(test(), "Clear operation failed");
        }

        static constexpr void TestTransformations()
        {
            // Test translation
            {
                constexpr auto test = []() {
                    MatrixStack stack;
                    stack.TranslateTop(Vector3D(1, 2, 3));
                    const auto& t = stack.Top();
                    return t.Row3 == Vector3D(1, 2, 3);
                };
                static_assert(test(), "Translation test failed");
            }
            
            // Test scale (using a separate lambda since we can't modify the same stack)
            {
                constexpr auto test = []() {
                    MatrixStack stack;
                    stack.ScaleTop(Vector3D(2, 3, 4));
                    const auto& t = stack.Top();
                    return t.Row0 == Vector3D(2, 0, 0) &&
                           t.Row1 == Vector3D(0, 3, 0) &&
                           t.Row2 == Vector3D(0, 0, 4);
                };
                static_assert(test(), "Scale test failed");
            }
        }

        static constexpr void TestTransformPointVector()
        {
            // Test point transformation
            {
                constexpr auto test = []() {
                    MatrixStack stack;
                    stack.TranslateTop(Vector3D(1, 0, 0));
                    Vector3D p = stack.TransformPoint(Vector3D(2, 0, 0));
                    return p == Vector3D(3, 0, 0);
                };
                static_assert(test(), "Point transformation failed");
            }
            
            // Test vector transformation (should not translate)
            {
                constexpr auto test = []() {
                    MatrixStack stack;
                    stack.TranslateTop(Vector3D(1, 0, 0));
                    Vector3D v = stack.TransformVector(Vector3D(2, 0, 0));
                    return v == Vector3D(2, 0, 0);
                };
                static_assert(test(), "Vector transformation failed");
            }
        }

        static constexpr void RunAll()
        {
            TestConstructionAndIdentity();
            TestPushPop();
            TestClear();
            TestTransformations();
            TestTransformPointVector();
        }
    };

    static_assert((MatrixStackTests::RunAll(), true), "MatrixStack tests failed");
}
