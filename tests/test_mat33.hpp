#pragma once

/**
 * @file test_mat33.hpp
 * @brief Compile-time unit tests for the Matrix33 class
 * 
 * This file contains comprehensive tests for the Matrix33 class, covering:
 * - Basic construction and factory methods
 * - Matrix arithmetic operations
 * - Matrix transformations (rotation, scaling, etc.)
 * - Determinant and inverse calculations
 * - Vector transformations
 * - Edge cases and special matrices
 * 
 * All tests are performed at compile-time using static_assert.
 */

#include "../impl/mat33.hpp"

namespace SaturnMath::Tests
{
    using namespace SaturnMath::Types;

    /**
     * @brief Test suite for Matrix33 class
     * 
     * This struct contains all test cases for the Matrix33 class, organized
     * into logical sections. Each test method is focused on a specific
     * aspect of the Matrix33 functionality.
     */
    struct Matrix33Tests
    {
        // ============================================
        // Construction and Factory Methods
        // ============================================
        
        /**
         * @brief Tests matrix construction and factory methods
         * 
         * Verifies:
         * - Default construction initializes to zero matrix
         * - Construction from row vectors works correctly
         * - Identity matrix factory creates correct matrix
         * - Scale matrix factory creates correct matrix
         * - Rotation matrix factories create correct matrices
         */
        static constexpr void TestConstruction()
        {
            // Test default constructor
            {
                constexpr Matrix33 m;
                static_assert(m.Row0.X == 0 && m.Row0.Y == 0 && m.Row0.Z == 0 &&
                              m.Row1.X == 0 && m.Row1.Y == 0 && m.Row1.Z == 0 &&
                              m.Row2.X == 0 && m.Row2.Y == 0 && m.Row2.Z == 0,
                              "Default constructor should initialize to zero matrix");
            }

            // Test row vector construction
            {
                constexpr Vector3D row0(1, 2, 3);
                constexpr Vector3D row1(4, 5, 6);
                constexpr Vector3D row2(7, 8, 9);
                
                constexpr Matrix33 m(row0, row1, row2);
                
                static_assert(m.Row0.X == 1 && m.Row0.Y == 2 && m.Row0.Z == 3 &&
                              m.Row1.X == 4 && m.Row1.Y == 5 && m.Row1.Z == 6 &&
                              m.Row2.X == 7 && m.Row2.Y == 8 && m.Row2.Z == 9,
                              "Construction from row vectors should work correctly");
            }

            // Test identity matrix factory
            {
                constexpr Matrix33 identity = Matrix33::Identity();
                
                static_assert(identity.Row0.X == 1 && identity.Row0.Y == 0 && identity.Row0.Z == 0 &&
                              identity.Row1.X == 0 && identity.Row1.Y == 1 && identity.Row1.Z == 0 &&
                              identity.Row2.X == 0 && identity.Row2.Y == 0 && identity.Row2.Z == 1,
                              "Identity matrix factory method should create correct identity matrix");
            }

            // Test scale matrix factory
            {
                constexpr Vector3D scale(2, 3, 4);
                constexpr Matrix33 scaleMatrix = Matrix33::CreateScale(scale);
                
                static_assert(scaleMatrix.Row0.X == 2 && scaleMatrix.Row0.Y == 0 && scaleMatrix.Row0.Z == 0 &&
                              scaleMatrix.Row1.X == 0 && scaleMatrix.Row1.Y == 3 && scaleMatrix.Row1.Z == 0 &&
                              scaleMatrix.Row2.X == 0 && scaleMatrix.Row2.Y == 0 && scaleMatrix.Row2.Z == 4,
                              "Scale matrix factory should create correct scaling matrix");
            }

            // Test rotation matrix factories
            {
                constexpr Angle angle90 = Angle::FromDegrees(90);
                constexpr Matrix33 rotX = Matrix33::CreateRotationX(angle90);
                constexpr Matrix33 rotY = Matrix33::CreateRotationY(angle90);
                constexpr Matrix33 rotZ = Matrix33::CreateRotationZ(angle90);
                
                // Verify X rotation matrix
                static_assert(rotX.Row0.X == 1 && rotX.Row0.Y == 0 && rotX.Row0.Z == 0 &&
                              rotX.Row1.X == 0 && rotX.Row1.Y == 0 && rotX.Row1.Z == -1 &&
                              rotX.Row2.X == 0 && rotX.Row2.Y == 1 && rotX.Row2.Z == 0,
                              "X-axis rotation matrix incorrect");
                
                // Verify Y rotation matrix
                static_assert(rotY.Row0.X == 0 && rotY.Row0.Y == 0 && rotY.Row0.Z == 1 &&
                              rotY.Row1.X == 0 && rotY.Row1.Y == 1 && rotY.Row1.Z == 0 &&
                              rotY.Row2.X == -1 && rotY.Row2.Y == 0 && rotY.Row2.Z == 0,
                              "Y-axis rotation matrix incorrect");
                
                // Verify Z rotation matrix
                static_assert(rotZ.Row0.X == 0 && rotZ.Row0.Y == -1 && rotZ.Row0.Z == 0 &&
                              rotZ.Row1.X == 1 && rotZ.Row1.Y == 0 && rotZ.Row1.Z == 0 &&
                              rotZ.Row2.X == 0 && rotZ.Row2.Y == 0 && rotZ.Row2.Z == 1,
                              "Z-axis rotation matrix incorrect");
            }
        }

        // ============================================
        // Basic Matrix Operations
        // ============================================
        
        /**
         * @brief Tests basic matrix operations
         * 
         * Verifies:
         * - Matrix addition and subtraction
         * - Scalar multiplication and division
         * - Matrix negation
         * - Compound assignment operations
         */
        static constexpr void TestBasicOperations()
        {
            constexpr Matrix33 a(
                Vector3D(1, 2, 3),
                Vector3D(4, 5, 6),
                Vector3D(7, 8, 9)
            );
            
            constexpr Matrix33 b(
                Vector3D(9, 8, 7),
                Vector3D(6, 5, 4),
                Vector3D(3, 2, 1)
            );

            // Test matrix addition
            {
                constexpr Matrix33 sum = a + b;
                static_assert(sum.Row0.X == 10 && sum.Row0.Y == 10 && sum.Row0.Z == 10 &&
                              sum.Row1.X == 10 && sum.Row1.Y == 10 && sum.Row1.Z == 10 &&
                              sum.Row2.X == 10 && sum.Row2.Y == 10 && sum.Row2.Z == 10,
                              "Matrix addition should add corresponding components");
            }

            // Test matrix subtraction
            {
                constexpr Matrix33 diff = a - b;
                static_assert(diff.Row0.X == -8 && diff.Row0.Y == -6 && diff.Row0.Z == -4 &&
                              diff.Row1.X == -2 && diff.Row1.Y == 0 && diff.Row1.Z == 2 &&
                              diff.Row2.X == 4 && diff.Row2.Y == 6 && diff.Row2.Z == 8,
                              "Matrix subtraction should subtract corresponding components");
            }

            // Test scalar multiplication
            {
                constexpr Matrix33 scaled = a * 2;
                static_assert(scaled.Row0.X == 2 && scaled.Row0.Y == 4 && scaled.Row0.Z == 6 &&
                              scaled.Row1.X == 8 && scaled.Row1.Y == 10 && scaled.Row1.Z == 12 &&
                              scaled.Row2.X == 14 && scaled.Row2.Y == 16 && scaled.Row2.Z == 18,
                              "Scalar multiplication should multiply each component by the scalar");
            }

            // Test scalar division
            {
                constexpr Matrix33 scaled = a * 2 / 2;
                static_assert(scaled.Row0.X == a.Row0.X && scaled.Row0.Y == a.Row0.Y && scaled.Row0.Z == a.Row0.Z &&
                              scaled.Row1.X == a.Row1.X && scaled.Row1.Y == a.Row1.Y && scaled.Row1.Z == a.Row1.Z &&
                              scaled.Row2.X == a.Row2.X && scaled.Row2.Y == a.Row2.Y && scaled.Row2.Z == a.Row2.Z,
                              "Scalar division should divide each component by the scalar");
            }

            // Test unary negation
            {
                constexpr Matrix33 neg = -a;
                static_assert(neg.Row0.X == -1 && neg.Row0.Y == -2 && neg.Row0.Z == -3 &&
                              neg.Row1.X == -4 && neg.Row1.Y == -5 && neg.Row1.Z == -6 &&
                              neg.Row2.X == -7 && neg.Row2.Y == -8 && neg.Row2.Z == -9,
                              "Unary negation should negate each component");
            }

            // Test compound assignment operators
            {
                constexpr auto TestCompound = []() {
                    Matrix33 m1 = a;
                    const Matrix33 m2 = b;
                    
                    // Test +=
                    m1 += m2;
                    if (!(m1.Row0.X == 10 && m1.Row0.Y == 10 && m1.Row0.Z == 10 &&
                          m1.Row1.X == 10 && m1.Row1.Y == 10 && m1.Row1.Z == 10 &&
                          m1.Row2.X == 10 && m1.Row2.Y == 10 && m1.Row2.Z == 10)) {
                        return false;
                    }
                    
                    // Test -=
                    m1 -= m2;
                    if (!(m1.Row0.X == a.Row0.X && m1.Row0.Y == a.Row0.Y && m1.Row0.Z == a.Row0.Z &&
                          m1.Row1.X == a.Row1.X && m1.Row1.Y == a.Row1.Y && m1.Row1.Z == a.Row1.Z &&
                          m1.Row2.X == a.Row2.X && m1.Row2.Y == a.Row2.Y && m1.Row2.Z == a.Row2.Z)) {
                        return false;
                    }
                    
                    // Test *=
                    m1 *= 2;
                    if (!(m1.Row0.X == 2 && m1.Row0.Y == 4 && m1.Row0.Z == 6 &&
                          m1.Row1.X == 8 && m1.Row1.Y == 10 && m1.Row1.Z == 12 &&
                          m1.Row2.X == 14 && m1.Row2.Y == 16 && m1.Row2.Z == 18)) {
                        return false;
                    }
                    
                    // Test /=
                    m1 /= 2;
                    if (!(m1.Row0.X == a.Row0.X && m1.Row0.Y == a.Row0.Y && m1.Row0.Z == a.Row0.Z &&
                          m1.Row1.X == a.Row1.X && m1.Row1.Y == a.Row1.Y && m1.Row1.Z == a.Row1.Z &&
                          m1.Row2.X == a.Row2.X && m1.Row2.Y == a.Row2.Y && m1.Row2.Z == a.Row2.Z)) {
                        return false;
                    }
                    
                    return true;
                };
                
                static_assert(TestCompound(), "Compound assignment operators should work correctly");
            }
        }

        // ============================================
        // Matrix-Vector Operations
        // ============================================
        
        /**
         * @brief Tests matrix-vector multiplication
         * 
         * Verifies:
         * - Matrix-vector multiplication produces correct results
         * - Transformations are applied correctly
         * - Edge cases with zero and identity matrices
         */
        static constexpr void TestMatrixVectorOperations()
        {
            // Test matrix-vector multiplication
            {
                constexpr Matrix33 m(
                    Vector3D(1, 2, 3),
                    Vector3D(4, 5, 6),
                    Vector3D(7, 8, 9)
                );
                
                constexpr Vector3D v(1, 2, 3);
                
                constexpr Vector3D result = m * v;
                
                static_assert(result.X == 14 && result.Y == 32 && result.Z == 50,
                              "Matrix-vector multiplication should produce correct result");
            }

            // Test with identity matrix
            {
                constexpr Matrix33 identity = Matrix33::Identity();
                constexpr Vector3D v(1, 2, 3);
                
                constexpr Vector3D result = identity * v;
                
                static_assert(result.X == v.X && result.Y == v.Y && result.Z == v.Z,
                              "Identity matrix should not change the vector");
            }

            // Test with zero matrix
            {
                constexpr Matrix33 zero;
                constexpr Vector3D v(1, 2, 3);
                
                constexpr Vector3D result = zero * v;
                
                static_assert(result.X == 0 && result.Y == 0 && result.Z == 0,
                              "Zero matrix should produce zero vector");
            }
        }

        // ============================================
        // Matrix-Matrix Operations
        // ============================================
        
        /**
         * @brief Tests matrix-matrix operations
         * 
         * Verifies:
         * - Matrix multiplication
         * - Matrix transposition
         * - Matrix inversion
         * - Determinant calculation
         */
        static constexpr void TestMatrixMatrixOperations()
        {
            // Test matrix multiplication
            {
                constexpr Matrix33 a(
                    Vector3D(1, 2, 3),
                    Vector3D(4, 5, 6),
                    Vector3D(7, 8, 9)
                );
                
                constexpr Matrix33 b(
                    Vector3D(9, 8, 7),
                    Vector3D(6, 5, 4),
                    Vector3D(3, 2, 1)
                );
                
                constexpr Matrix33 product = a * b;
                
                static_assert(product.Row0.X == 30 && product.Row0.Y == 24 && product.Row0.Z == 18 &&
                              product.Row1.X == 84 && product.Row1.Y == 69 && product.Row1.Z == 54 &&
                              product.Row2.X == 138 && product.Row2.Y == 114 && product.Row2.Z == 90,
                              "Matrix multiplication should produce correct result");
            }

            // Test matrix transpose
            {
                constexpr Matrix33 m(
                    Vector3D(1, 2, 3),
                    Vector3D(4, 5, 6),
                    Vector3D(7, 8, 9)
                );
                
                constexpr Matrix33 transposed = m.Transposed();
                
                static_assert(transposed.Row0.X == 1 && transposed.Row0.Y == 4 && transposed.Row0.Z == 7 &&
                              transposed.Row1.X == 2 && transposed.Row1.Y == 5 && transposed.Row1.Z == 8 &&
                              transposed.Row2.X == 3 && transposed.Row2.Y == 6 && transposed.Row2.Z == 9,
                              "Matrix transposition should swap rows and columns");
            }

            // Test matrix determinant
            {
                // Test with identity matrix (determinant = 1)
                {
                    constexpr Matrix33 identity = Matrix33::Identity();
                    static_assert(identity.Determinant() == 1,
                                  "Determinant of identity matrix should be 1");
                }

                // Test with singular matrix (determinant = 0)
                {
                    constexpr Matrix33 singular(
                        Vector3D(1, 2, 3),
                        Vector3D(4, 5, 6),
                        Vector3D(7, 8, 9)  // Linearly dependent rows
                    );
                    static_assert(singular.Determinant() == 0,
                                  "Determinant of singular matrix should be 0");
                }

                // Test with diagonal matrix
                {
                    constexpr Matrix33 diagonal(
                        Vector3D(2, 0, 0),
                        Vector3D(0, 3, 0),
                        Vector3D(0, 0, 4)
                    );
                    static_assert(diagonal.Determinant() == 24,  // 2 * 3 * 4
                                  "Determinant of diagonal matrix should be product of diagonal elements");
                }
            }

            // Test matrix inverse
            {
                // Test with invertible matrix
                {
                    constexpr Matrix33 m(
                        Vector3D(1, 0, 0),
                        Vector3D(0, 2, 0),
                        Vector3D(0, 0, 4)
                    );
                    
                    constexpr auto testInverse = []() {
                        Matrix33 inverse;
                        bool success = m.TryInverse(inverse);
                        if (!success) return false;
                        
                        // Check if inverse is correct by multiplying with original
                        Matrix33 product = m * inverse;
                        Matrix33 identity = Matrix33::Identity();
                        
                        return product == identity;
                    };
                    
                    static_assert(testInverse(), "Matrix inverse should be calculated correctly");
                }

                // Test with singular matrix (should not be invertible)
                {
                    constexpr Matrix33 singular(
                        Vector3D(1, 2, 3),
                        Vector3D(4, 5, 6),
                        Vector3D(7, 8, 9)  // Linearly dependent rows
                    );
                    
                    constexpr auto testSingularInverse = []() {
                        Matrix33 inverse;
                        bool success = singular.TryInverse(inverse);
                        return !success;  // Should fail to invert
                    };
                    
                    static_assert(testSingularInverse(), "Singular matrix should not be invertible");
                }
            }
        }

        // ============================================
        // Matrix Properties and Transformations
        // ============================================
        
        /**
         * @brief Tests matrix properties and transformation matrices
         * 
         * Verifies:
         * - Rotation matrices
         * - Scaling matrices
         * - Combined transformations
         * - Matrix properties (symmetric, orthogonal, etc.)
         */
        static constexpr void TestTransformations()
        {
            // Test rotation matrices
            {
                // X-axis rotation
                {
                    constexpr Angle angle90 = Angle::FromDegrees(90);
                    constexpr Matrix33 rotX = Matrix33::CreateRotationX(angle90);
                    constexpr Vector3D v(0, 1, 0);
                    constexpr Vector3D rotated = rotX * v;
                    
                    static_assert(rotated.X == 0 && rotated.Y == 0 && rotated.Z == 1,
                                  "X-axis rotation should rotate around X-axis correctly");
                }
                
                // Y-axis rotation
                {
                    constexpr Angle angle90 = Angle::FromDegrees(90);
                    constexpr Matrix33 rotY = Matrix33::CreateRotationY(angle90);
                    constexpr Vector3D v(0, 0, 1);
                    constexpr Vector3D rotated = rotY * v;
                    
                    static_assert(rotated.X == 1 && rotated.Y == 0 && rotated.Z == 0,
                                  "Y-axis rotation should rotate around Y-axis correctly");
                }
                
                // Z-axis rotation
                {
                    constexpr Angle angle90 = Angle::FromDegrees(90);
                    constexpr Matrix33 rotZ = Matrix33::CreateRotationZ(angle90);
                    constexpr Vector3D v(1, 0, 0);
                    constexpr Vector3D rotated = rotZ * v;
                    
                    static_assert(rotated.X == 0 && rotated.Y == 1 && rotated.Z == 0,
                                  "Z-axis rotation should rotate around Z-axis correctly");
                }
            }

            // Test compound rotations
            {
                constexpr Matrix33 rotX = Matrix33::CreateRotationX(Angle::FromDegrees(90));
                constexpr Matrix33 rotY = Matrix33::CreateRotationY(Angle::FromDegrees(90));
                constexpr Matrix33 m = rotY * rotX;  // Apply X rotation first, then Y
                
                constexpr Vector3D v(0, 0, 1);
                constexpr Vector3D rotated = m * v;
                
                static_assert(rotated.X == 0 && rotated.Y == -1 && rotated.Z == 0,
                              "Compound rotations should produce correct result");
            }

            // Test scaling
            {
                constexpr Vector3D scale(2, 3, 4);
                constexpr Matrix33 scaleMatrix = Matrix33::CreateScale(scale);
                constexpr Vector3D v(1, 1, 1);
                
                constexpr Vector3D scaled = scaleMatrix * v;
                
                static_assert(scaled.X == 2 && scaled.Y == 3 && scaled.Z == 4,
                              "Scaling should scale each component by the corresponding scale factor");
            }

            // Test combined transformation (scale, then rotate, then translate)
            {
                // Note: Translation is not part of Matrix33, but we can test scale and rotation
                constexpr Vector3D scale(2, 3, 4);
                constexpr Matrix33 scaleMatrix = Matrix33::CreateScale(scale);
                constexpr Matrix33 rotX = Matrix33::CreateRotationX(Angle::FromDegrees(90));
                
                // Apply scale first, then rotation
                constexpr Matrix33 transform = rotX * scaleMatrix;
                constexpr Vector3D v(1, 1, 0);
                
                constexpr Vector3D transformed = transform * v;
                
                // Expected: First scale (2, 3, 0), then rotate around X by 90 degrees
                // Rotation around X: (x, y, z) -> (x, -z, y)
                // So (2, 3, 0) -> (2, 0, 3)
                static_assert(transformed.X == 2 && transformed.Y == 0 && transformed.Z == 3,
                              "Combined transformation should apply scale then rotation");
            }
        }

        // ============================================
        // Edge Cases and Special Matrices
        // ============================================
        
        /**
         * @brief Tests edge cases and special matrices
         * 
         * Verifies:
         * - Zero matrix
         * - Identity matrix
         * - Singular matrices
         * - Matrices with extreme values
         */
        static constexpr void TestEdgeCases()
        {
            // Test zero matrix
            {
                constexpr Matrix33 zero;
                constexpr Matrix33 identity = Matrix33::Identity();
                
                // Zero matrix properties
                static_assert(zero.Determinant() == 0,
                              "Determinant of zero matrix should be 0");
                
                // Zero matrix multiplication
                constexpr Matrix33 product = zero * identity;
                static_assert(product.Row0.X == 0 && product.Row0.Y == 0 && product.Row0.Z == 0 &&
                              product.Row1.X == 0 && product.Row1.Y == 0 && product.Row1.Z == 0 &&
                              product.Row2.X == 0 && product.Row2.Y == 0 && product.Row2.Z == 0,
                              "Multiplying by zero matrix should produce zero matrix");
            }

            // Test identity matrix properties
            {
                constexpr Matrix33 identity = Matrix33::Identity();
                constexpr Matrix33 m(
                    Vector3D(1, 2, 3),
                    Vector3D(4, 5, 6),
                    Vector3D(7, 8, 9)
                );
                
                // Identity * M = M
                constexpr Matrix33 product1 = identity * m;
                static_assert(product1.Row0.X == m.Row0.X && product1.Row0.Y == m.Row0.Y && product1.Row0.Z == m.Row0.Z &&
                              product1.Row1.X == m.Row1.X && product1.Row1.Y == m.Row1.Y && product1.Row1.Z == m.Row1.Z &&
                              product1.Row2.X == m.Row2.X && product1.Row2.Y == m.Row2.Y && product1.Row2.Z == m.Row2.Z,
                              "Multiplying by identity matrix should not change the matrix");
                
                // M * Identity = M
                constexpr Matrix33 product2 = m * identity;
                static_assert(product2.Row0.X == m.Row0.X && product2.Row0.Y == m.Row0.Y && product2.Row0.Z == m.Row0.Z &&
                              product2.Row1.X == m.Row1.X && product2.Row1.Y == m.Row1.Y && product2.Row1.Z == m.Row1.Z &&
                              product2.Row2.X == m.Row2.X && product2.Row2.Y == m.Row2.Y && product2.Row2.Z == m.Row2.Z,
                              "Multiplying by identity matrix should not change the matrix");
                
                // Identity * Identity = Identity
                constexpr Matrix33 product3 = identity * identity;
                static_assert(product3.Row0.X == 1 && product3.Row0.Y == 0 && product3.Row0.Z == 0 &&
                              product3.Row1.X == 0 && product3.Row1.Y == 1 && product3.Row1.Z == 0 &&
                              product3.Row2.X == 0 && product3.Row2.Y == 0 && product3.Row2.Z == 1,
                              "Multiplying identity by itself should produce identity");
            }

            // Test with extreme values
            {
                constexpr Fxp maxVal = Fxp::MaxValue();
                constexpr Matrix33 m(
                    Vector3D(maxVal, 0, 0),
                    Vector3D(0, maxVal, 0),
                    Vector3D(0, 0, maxVal)
                );
                
                // Test that operations with extreme values don't overflow
                constexpr Matrix33 scaled = m * 0.5;
                static_assert(scaled.Row0.X == maxVal * 0.5 && scaled.Row1.Y == maxVal * 0.5 && scaled.Row2.Z == maxVal * 0.5,
                              "Scaling matrix with extreme values should work correctly");
            }
        }

        // ============================================
        // Test Suite Runner
        // ============================================
        
        /**
         * @brief Runs all matrix tests
         * 
         * This method executes all test cases in the correct order.
         * The tests are organized from basic to more complex functionality.
         */
        static constexpr void RunAll()
        {
            // Construction and Factory Methods
            TestConstruction();
            
            // Basic Matrix Operations
            TestBasicOperations();
            
            // Matrix-Vector Operations
            TestMatrixVectorOperations();
            
            // Matrix-Matrix Operations
            TestMatrixMatrixOperations();
            
            // Matrix Properties and Transformations
            TestTransformations();
            
            // Edge Cases and Special Matrices
            TestEdgeCases();
        }
    };
    
    // Execute all tests at compile time
    static_assert((Matrix33Tests::RunAll(), true), "Matrix33 tests failed");
}