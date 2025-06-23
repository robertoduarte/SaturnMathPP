#pragma once

/**
 * @file test_mat43.hpp
 * @brief Compile-time unit tests for the Matrix43 class
 * 
 * This file contains comprehensive tests for the Matrix43 class, covering:
 * - Basic construction and factory methods
 * - Matrix arithmetic operations
 * - Matrix transformations (translation, rotation, scaling)
 * - Vector transformations
 * - Matrix-matrix operations
 * - Edge cases and special matrices
 * 
 * All tests are performed at compile-time using static_assert.
 */

#include "../impl/mat43.hpp"

namespace SaturnMath::Tests
{
    using namespace SaturnMath::Types;

    /**
     * @brief Test suite for Matrix43 class
     * 
     * This struct contains all test cases for the Matrix43 class, organized
     * into logical sections. Each test method focuses on a specific
     * aspect of the Matrix43 functionality.
     */
    struct Matrix43Tests
    {
        // ============================================
        // Construction and Factory Methods
        // ============================================
        
        /**
         * @brief Tests matrix construction and factory methods
         * 
         * Verifies:
         * - Default construction initializes to identity matrix with zero translation
         * - Construction from individual elements works correctly
         * - Construction from row vectors works correctly
         * - Construction from rotation and translation works correctly
         * - Copy constructor works as expected
         * - Factory methods create correct matrices
         */
        static constexpr void TestConstruction()
        {
            // Default constructor should initialize to identity
            {
                constexpr Matrix43 identity = Matrix43::Identity();
                static_assert(identity.Row0.X == 1 && identity.Row0.Y == 0 && identity.Row0.Z == 0 &&
                              identity.Row1.X == 0 && identity.Row1.Y == 1 && identity.Row1.Z == 0 &&
                              identity.Row2.X == 0 && identity.Row2.Y == 0 && identity.Row2.Z == 1 &&
                              identity.Row3.X == 0 && identity.Row3.Y == 0 && identity.Row3.Z == 0,
                              "Default constructor should initialize to identity matrix with zero translation");
            }
            
            // Construction from individual elements
            {
                constexpr Matrix43 m(
                    {1, 2, 3},
                    {4, 5, 6},
                    {7, 8, 9},
                    {10, 11, 12}
                );
                static_assert(m.Row0.X == 1 && m.Row0.Y == 2 && m.Row0.Z == 3 &&
                              m.Row1.X == 4 && m.Row1.Y == 5 && m.Row1.Z == 6 &&
                              m.Row2.X == 7 && m.Row2.Y == 8 && m.Row2.Z == 9 &&
                              m.Row3.X == 10 && m.Row3.Y == 11 && m.Row3.Z == 12,
                              "Construction from individual elements should work");
            }
            
            // Construction from row vectors
            {
                constexpr Vector3D row1(1, 2, 3);
                constexpr Vector3D row2(4, 5, 6);
                constexpr Vector3D row3(7, 8, 9);
                constexpr Vector3D row4(10, 11, 12);
                constexpr Matrix43 mFromRows(row1, row2, row3, row4);
                static_assert(mFromRows.Row0.X == 1 && mFromRows.Row0.Y == 2 && mFromRows.Row0.Z == 3 &&
                              mFromRows.Row1.X == 4 && mFromRows.Row1.Y == 5 && mFromRows.Row1.Z == 6 &&
                              mFromRows.Row2.X == 7 && mFromRows.Row2.Y == 8 && mFromRows.Row2.Z == 9 &&
                              mFromRows.Row3.X == 10 && mFromRows.Row3.Y == 11 && mFromRows.Row3.Z == 12,
                              "Construction from row vectors should work");
            }
            
            // Construction from rotation and translation
            {
                constexpr Matrix33 rotation(
                    {1, 2, 3},
                    {4, 5, 6},
                    {7, 8, 9}
                );
                constexpr Vector3D translation(10, 11, 12);
                constexpr Matrix43 m(rotation, translation);
                
                static_assert(m.Row0.X == 1 && m.Row0.Y == 2 && m.Row0.Z == 3 &&
                              m.Row1.X == 4 && m.Row1.Y == 5 && m.Row1.Z == 6 &&
                              m.Row2.X == 7 && m.Row2.Y == 8 && m.Row2.Z == 9 &&
                              m.Row3.X == 10 && m.Row3.Y == 11 && m.Row3.Z == 12,
                              "Construction from rotation and translation should work");
            }
            
            // Test factory methods
            {
                // Test CreateTranslation
                constexpr Vector3D translation(1, 2, 3);
                constexpr Matrix43 transMat = Matrix43::CreateTranslation(translation);
                
                static_assert(transMat.Row0.X == 1 && transMat.Row0.Y == 0 && transMat.Row0.Z == 0 &&
                              transMat.Row1.X == 0 && transMat.Row1.Y == 1 && transMat.Row1.Z == 0 &&
                              transMat.Row2.X == 0 && transMat.Row2.Y == 0 && transMat.Row2.Z == 1 &&
                              transMat.Row3.X == 1 && transMat.Row3.Y == 2 && transMat.Row3.Z == 3,
                              "CreateTranslation should create correct matrix");
                              
                // Test CreateRotationX
                {
                    constexpr auto angle = Angle::FromDegrees(90);
                    constexpr Matrix43 rotXMat = Matrix43::CreateRotationX(angle);
                    
                    // For 90° rotation around X-axis:
                    // [1  0  0]
                    // [0  0  1]
                    // [0 -1  0]
                    static_assert(rotXMat.Row0.X == 1 && rotXMat.Row0.Y == 0 && rotXMat.Row0.Z == 0 &&
                                  rotXMat.Row1.X == 0 && rotXMat.Row1.Y == 0 && rotXMat.Row1.Z == 1 &&
                                  rotXMat.Row2.X == 0 && rotXMat.Row2.Y == -1 && rotXMat.Row2.Z == 0,
                                  "CreateRotationX should create correct rotation matrix");
                }
            }
            
            // Copy constructor
            {
                constexpr Matrix43 m(
                    {1, 2, 3},
                    {4, 5, 6},
                    {7, 8, 9},
                    {10, 11, 12}
                );
                constexpr Matrix43 copy = m;
                static_assert(copy.Row0.X == 1 && copy.Row0.Y == 2 && copy.Row0.Z == 3 &&
                              copy.Row1.X == 4 && copy.Row1.Y == 5 && copy.Row1.Z == 6 &&
                              copy.Row2.X == 7 && copy.Row2.Y == 8 && copy.Row2.Z == 9 &&
                              copy.Row3.X == 10 && copy.Row3.Y == 11 && copy.Row3.Z == 12,
                              "Copy constructor should work");
            }
        }
        
        /**
         * @brief Tests static factory methods
         * 
         * Verifies:
         * - Identity matrix creation
         * - Translation matrix creation (both from components and vector)
         * - Rotation matrix creation (X, Y, Z axes)
         * - Scale matrix creation
         */
        static constexpr void TestFactoryMethods()
        {
            // Identity matrix
            constexpr Matrix43 identity = Matrix43::Identity();
            static_assert(identity.Row0.X == 1 && identity.Row0.Y == 0 && identity.Row0.Z == 0 &&
                          identity.Row1.X == 0 && identity.Row1.Y == 1 && identity.Row1.Z == 0 &&
                          identity.Row2.X == 0 && identity.Row2.Y == 0 && identity.Row2.Z == 1 &&
                          identity.Row3.X == 0 && identity.Row3.Y == 0 && identity.Row3.Z == 0,
                          "Identity matrix factory method should work");
            
            // Translation matrix
            constexpr Matrix43 translation = Matrix43::Translation(1, 2, 3);
            static_assert(translation.Row0.X == 1 && translation.Row0.Y == 0 && translation.Row0.Z == 0 &&
                          translation.Row1.X == 0 && translation.Row1.Y == 1 && translation.Row1.Z == 0 &&
                          translation.Row2.X == 0 && translation.Row2.Y == 0 && translation.Row2.Z == 1 &&
                          translation.Row3.X == 1 && translation.Row3.Y == 2 && translation.Row3.Z == 3,
                          "Translation matrix factory method should work");
            
            // Translation matrix from vector
            constexpr Vector3D translationVec(1, 2, 3);
            constexpr Matrix43 translationFromVec = Matrix43::CreateTranslation(translationVec);
            static_assert(translationFromVec.Row0.X == 1 && translationFromVec.Row0.Y == 0 && translationFromVec.Row0.Z == 0 &&
                          translationFromVec.Row1.X == 0 && translationFromVec.Row1.Y == 1 && translationFromVec.Row1.Z == 0 &&
                          translationFromVec.Row2.X == 0 && translationFromVec.Row2.Y == 0 && translationFromVec.Row2.Z == 1 &&
                          translationFromVec.Row3.X == 1 && translationFromVec.Row3.Y == 2 && translationFromVec.Row3.Z == 3,
                          "Translation matrix from vector factory method should work");
            
            // Rotation matrix around X axis
            constexpr Angle angle90 = Angle::FromDegrees(90);
            constexpr Matrix43 rotX = Matrix43::CreateRotationX(angle90);
            static_assert(rotX.Row0.X == 1 && rotX.Row0.Y == 0 && rotX.Row0.Z == 0 &&
                          rotX.Row1.X == 0 && rotX.Row1.Y == 0 && rotX.Row1.Z == 1 &&
                          rotX.Row2.X == 0 && rotX.Row2.Y == -1 && rotX.Row2.Z == 0 &&
                          rotX.Row3.X == 0 && rotX.Row3.Y == 0 && rotX.Row3.Z == 0,
                          "Rotation matrix around X axis should work");
            
            // Scale matrix
            constexpr Matrix43 scale = Matrix43::Scale(2, 3, 4);
            static_assert(scale.Row0.X == 2 && scale.Row0.Y == 0 && scale.Row0.Z == 0 &&
                          scale.Row1.X == 0 && scale.Row1.Y == 3 && scale.Row1.Z == 0 &&
                          scale.Row2.X == 0 && scale.Row2.Y == 0 && scale.Row2.Z == 4 &&
                          scale.Row3.X == 0 && scale.Row3.Y == 0 && scale.Row3.Z == 0,
                          "Scale matrix factory method should work");
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
         * - Matrix equality and inequality
         */
        static constexpr void TestBasicOperations()
        {
            // Test matrix addition
            {
                constexpr Matrix43 a(
                    {1, 2, 3},
                    {4, 5, 6},
                    {7, 8, 9},
                    {10, 11, 12}
                );
                constexpr Matrix43 b(
                    {2, 3, 4},
                    {5, 6, 7},
                    {8, 9, 10},
                    {11, 12, 13}
                );
                constexpr auto result = a + b;
                
                static_assert(result.Row0.X == 3 && result.Row0.Y == 5 && result.Row0.Z == 7 &&
                              result.Row1.X == 9 && result.Row1.Y == 11 && result.Row1.Z == 13 &&
                              result.Row2.X == 15 && result.Row2.Y == 17 && result.Row2.Z == 19 &&
                              result.Row3.X == 21 && result.Row3.Y == 23 && result.Row3.Z == 25,
                              "Matrix addition should work correctly");
            }
            
            // Test scalar multiplication
            {
                constexpr Matrix43 m(
                    {1, 2, 3},
                    {4, 5, 6},
                    {7, 8, 9},
                    {10, 11, 12}
                );
                constexpr auto result = m * 2;
                
                static_assert(result.Row0.X == 2 && result.Row0.Y == 4 && result.Row0.Z == 6 &&
                              result.Row1.X == 8 && result.Row1.Y == 10 && result.Row1.Z == 12 &&
                              result.Row2.X == 14 && result.Row2.Y == 16 && result.Row2.Z == 18 &&
                              result.Row3.X == 20 && result.Row3.Y == 22 && result.Row3.Z == 24,
                              "Scalar multiplication should work correctly");
            }
            
            // Test matrix equality
            {
                constexpr Matrix43 a(
                    {1, 2, 3},
                    {4, 5, 6},
                    {7, 8, 9},
                    {10, 11, 12}
                );
                constexpr Matrix43 b(
                    {1, 2, 3},
                    {4, 5, 6},
                    {7, 8, 9},
                    {10, 11, 12}
                );
                constexpr Matrix43 c(
                    {2, 3, 4},
                    {5, 6, 7},
                    {8, 9, 10},
                    {11, 12, 13}
                );
                
                static_assert(a == b, "Identical matrices should compare equal");
                static_assert(a != c, "Different matrices should compare not equal");
            }
        }
        
        // ============================================
        // Matrix-Vector Operations
        // ============================================
        
        /**
         * @brief Tests matrix-vector operations
         * 
         * Verifies:
         * - Vector transformation (multiplication)
         * - Point transformation (with translation)
         */
        static constexpr void TestMatrixVectorOperations()
        {
            // Test vector transformation (without translation)
            {
                constexpr Matrix43 m = Matrix43::Identity();
                constexpr Vector3D v(1, 2, 3);
                constexpr auto result = m.TransformVector(v);
                
                static_assert(result.X == 1 && result.Y == 2 && result.Z == 3,
                              "Vector transformation should work correctly with identity matrix");
            }
            
            // Test point transformation (with translation)
            {
                constexpr Matrix43 m = Matrix43::CreateTranslation({1, 2, 3});
                constexpr Vector3D p(4, 5, 6);
                constexpr auto result = m.TransformPoint(p);
                
                static_assert(result.X == 5 && result.Y == 7 && result.Z == 9,
                              "Point transformation should include translation");
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
         * - Compound assignment operators
         * - Comparison operators
         */
        static constexpr void TestMatrixMatrixOperations()
        {
            // Test matrix multiplication with identity
            {
                constexpr Matrix43 m(
                    {1, 2, 3},
                    {4, 5, 6},
                    {7, 8, 9},
                    {10, 11, 12}
                );
                
                constexpr Matrix43 identity = Matrix43::Identity();
                constexpr auto result = m * identity;
                
                // Multiplying by identity should return the original matrix
                static_assert(result.Row0.X == 1 && result.Row0.Y == 2 && result.Row0.Z == 3 &&
                              result.Row1.X == 4 && result.Row1.Y == 5 && result.Row1.Z == 6 &&
                              result.Row2.X == 7 && result.Row2.Y == 8 && result.Row2.Z == 9 &&
                              result.Row3.X == 10 && result.Row3.Y == 11 && result.Row3.Z == 12,
                              "Matrix multiplication with identity should return original matrix");
            }
            
            // Test matrix multiplication with translation
            {
                constexpr Matrix43 a = Matrix43::CreateTranslation({1, 2, 3});
                constexpr Matrix43 b = Matrix43::CreateTranslation({4, 5, 6});
                constexpr auto result = a * b;
                
                // Combined translation should be (5, 7, 9)
                static_assert(result.Row3.X == 5 && result.Row3.Y == 7 && result.Row3.Z == 9,
                              "Combined translation should add translations");
            }
        }

        // ============================================
        // Transformation Operations
        // ============================================
        
        /**
         * @brief Tests transformation operations specific to Matrix43
         * 
         * Verifies:
         * - Translation operations
         * - Combined transformations
         * - Transformation application to points and vectors
         */
        static constexpr void TestTransformationOperations()
        {
            // Test translation
            {
                constexpr Vector3D translation(1, 2, 3);
                constexpr Matrix43 transMat = Matrix43::CreateTranslation(translation);
                
                // Test point transformation (should translate)
                constexpr Vector3D point(4, 5, 6);
                constexpr Vector3D transformedPoint = transMat.TransformPoint(point);
                
                static_assert(transformedPoint.X == 5 && 
                              transformedPoint.Y == 7 && 
                              transformedPoint.Z == 9,
                              "Point transformation should include translation");
                
                // Test vector transformation (should not translate)
                constexpr Vector3D vector(1, 0, 0);
                constexpr Vector3D transformedVector = transMat.TransformVector(vector);
                
                static_assert(transformedVector.X == 1 && 
                              transformedVector.Y == 0 && 
                              transformedVector.Z == 0,
                              "Vector transformation should not include translation");
            }
            
            // Test rotation
            {
                // 90 degree rotation around Z axis
                constexpr auto angle = Angle::FromDegrees(90);
                constexpr Matrix43 rotMat = Matrix43::CreateRotationZ(angle);
                
                // Test vector rotation
                constexpr Vector3D vector(1, 0, 0);
                constexpr Vector3D rotated = rotMat.TransformVector(vector);
                
                // Should rotate (1,0,0) to (0,-1,0) for a 90° counter-clockwise rotation around Z
                static_assert(rotated.X == 0 && 
                              rotated.Y == -1 && 
                              rotated.Z == 0,
                              "Vector rotation should work correctly");
            }
            
            // Test combined transformations
            {
                constexpr Vector3D translation(1, 0, 0);
                constexpr auto angle = Angle::FromDegrees(90);
                
                // Create translation matrix
                constexpr Matrix43 transMat = Matrix43::CreateTranslation(translation);
                
                // Create rotation matrix
                constexpr Matrix43 rotMat = Matrix43::CreateRotationZ(angle);
                
                // Combine transformations (translate then rotate)
                constexpr Matrix43 combined = rotMat * transMat;
                
                // Test point transformation
                constexpr Vector3D point(1, 0, 0);
                constexpr Vector3D transformed = combined.TransformPoint(point);
                
                // Should first translate to (2,0,0) then rotate to (0,-2,0)
                // 1. Translate (1,0,0) by (1,0,0) -> (2,0,0)
                // 2. Rotate (2,0,0) 90° counter-clockwise around Z -> (0,-2,0)
                static_assert(transformed.X == 0 && 
                              transformed.Y == -2 && 
                              transformed.Z == 0,
                              "Combined transformations should apply in correct order");
            }
            
            // Test rotation matrices
            {
                // Test CreateRotationX
                {
                    constexpr auto angle = Angle::FromDegrees(90);
                    constexpr auto rotMat = Matrix43::CreateRotationX(angle);
                    
                    // Rotate (1,0,0) 90° around X should be (1,0,0)
                    // Rotate (0,1,0) 90° around X should be (0,0,1)
                    // Rotate (0,0,1) 90° around X should be (0,-1,0)
                    constexpr Vector3D vx = rotMat.TransformVector(Vector3D(1, 0, 0));
                    constexpr Vector3D vy = rotMat.TransformVector(Vector3D(0, 1, 0));
                    constexpr Vector3D vz = rotMat.TransformVector(Vector3D(0, 0, 1));
                    
                    // Verify the rotation matrix structure
                    static_assert(rotMat.Row0.X == 1 && rotMat.Row0.Y == 0 && rotMat.Row0.Z == 0, "RotateX: First row should be (1,0,0)");
                    static_assert(rotMat.Row1.X == 0 && rotMat.Row1.Z == 1, "RotateX: Second row should have 0,cos,sin pattern");
                    static_assert(rotMat.Row2.X == 0 && rotMat.Row2.Y == -1, "RotateX: Third row should have 0,-sin,cos pattern");
                }
                
                // Test CreateRotationY
                {
                    constexpr auto angle = Angle::FromDegrees(90);
                    constexpr auto rotMat = Matrix43::CreateRotationY(angle);
                    
                    // Rotate (1,0,0) 90° around Y should be (0,0,-1)
                    // Rotate (0,1,0) 90° around Y should be (0,1,0)
                    // Rotate (0,0,1) 90° around Y should be (1,0,0)
                    constexpr Vector3D vx = rotMat.TransformVector(Vector3D(1, 0, 0));
                    constexpr Vector3D vy = rotMat.TransformVector(Vector3D(0, 1, 0));
                    constexpr Vector3D vz = rotMat.TransformVector(Vector3D(0, 0, 1));
                    
                    // Verify the rotation matrix structure
                    static_assert(rotMat.Row0.Z == -1, "RotateY: First row should have cos,0,-sin pattern");
                    static_assert(rotMat.Row1.X == 0 && rotMat.Row1.Y == 1 && rotMat.Row1.Z == 0, "RotateY: Second row should be (0,1,0)");
                    static_assert(rotMat.Row2.X == 1, "RotateY: Third row should have sin,0,cos pattern");
                }
                
                // Test CreateRotationZ
                {
                    constexpr auto angle = Angle::FromDegrees(90);
                    constexpr auto rotMat = Matrix43::CreateRotationZ(angle);
                    
                    // Rotate (1,0,0) 90° around Z should be (0,1,0)
                    // Rotate (0,1,0) 90° around Z should be (-1,0,0)
                    // Rotate (0,0,1) 90° around Z should be (0,0,1)
                    constexpr Vector3D vx = rotMat.TransformVector(Vector3D(1, 0, 0));
                    constexpr Vector3D vy = rotMat.TransformVector(Vector3D(0, 1, 0));
                    constexpr Vector3D vz = rotMat.TransformVector(Vector3D(0, 0, 1));
                    
                    // Verify the rotation matrix structure
                    static_assert(rotMat.Row0.Y == 1, "RotateZ: First row should have cos,sin,0 pattern");
                    static_assert(rotMat.Row1.X == -1, "RotateZ: Second row should have -sin,cos,0 pattern");
                    static_assert(rotMat.Row2.X == 0 && rotMat.Row2.Y == 0 && rotMat.Row2.Z == 1, "RotateZ: Third row should be (0,0,1)");
                }
                
                // Test that translation is preserved during matrix multiplication
                {
                    constexpr auto transMat = Matrix43::CreateTranslation(Vector3D(1, 2, 3));
                    constexpr auto rotMat = Matrix43::CreateRotationX(Angle::FromDegrees(90));
                    constexpr auto result = rotMat * transMat;
                    
                    // The translation part should be transformed by the rotation
                    static_assert(result.Row3.X == 1, "Translation X should be preserved");
                    static_assert(result.Row3.Y == 3, "Translation Y should become Z after 90° X rotation");
                    static_assert(result.Row3.Z == -2, "Translation Z should become -Y after 90° X rotation");
                }
            }
        }
        
        /**
         * @brief Tests matrix-matrix operations
         * 
         * Verifies:
         * - Matrix multiplication
         * - Compound assignment operators
         * - Comparison operators
         */
        static constexpr void TestMatrixMatrixOperations()
        {
            // Test matrix multiplication with identity
            {
                constexpr Matrix43 m(
                    {1, 2, 3},
                    {4, 5, 6},
                    {7, 8, 9},
                    {10, 11, 12}
                );
                
                constexpr Matrix43 identity = Matrix43::Identity();
                constexpr auto result = m * identity;
                
                // Multiplying by identity should return the original matrix
                static_assert(result.Row0.X == 1 && result.Row0.Y == 2 && result.Row0.Z == 3 &&
                              result.Row1.X == 4 && result.Row1.Y == 5 && result.Row1.Z == 6 &&
                              result.Row2.X == 7 && result.Row2.Y == 8 && result.Row2.Z == 9 &&
                              result.Row3.X == 10 && result.Row3.Y == 11 && result.Row3.Z == 12,
                              "Matrix multiplication with identity should return original matrix");
            }
            
            // Test matrix multiplication with translation
            {
                constexpr Matrix43 a = Matrix43::CreateTranslation({1, 2, 3});
                constexpr Matrix43 b = Matrix43::CreateTranslation({4, 5, 6});
                constexpr auto result = a * b;
                
                // Combined translation should be (5, 7, 9)
                static_assert(result.Row3.X == 5 && result.Row3.Y == 7 && result.Row3.Z == 9,
                              "Combined translation should add translations");
            }
            
            // Test compound multiplication assignment
            {
                constexpr auto testCompoundMultiply = []() {
                    constexpr Matrix43 a(
                        {1, 2, 3},
                        {4, 5, 6},
                        {7, 8, 9},
                        {10, 11, 12}
                    );
                    
                    constexpr Matrix43 b = Matrix43::Identity();
                    
                    // Test compound multiplication with identity (should be no change)
                    Matrix43 result = a;
                    result *= b;
                    
                    return result.Row0.X == 1 && result.Row0.Y == 2 && result.Row0.Z == 3 &&
                           result.Row1.X == 4 && result.Row1.Y == 5 && result.Row1.Z == 6 &&
                           result.Row2.X == 7 && result.Row2.Y == 8 && result.Row2.Z == 9 &&
                           result.Row3.X == 10 && result.Row3.Y == 11 && result.Row3.Z == 12;
                };
                static_assert(testCompoundMultiply(), 
                    "Compound multiplication with identity should not change matrix");
            }
            
            // Test equality operators
            {
                constexpr Matrix43 a = Matrix43::Identity();
                constexpr Matrix43 b = Matrix43::Identity();
                constexpr Matrix43 c(
                    {1, 0, 0},
                    {0, 1, 0},
                    {0, 0, 1},
                    {1, 1, 1}  // Different translation
                );
                
                static_assert(a == b, "Identical matrices should compare equal");
                static_assert(a != c, "Different matrices should compare not equal");
                
                // Test translation
                constexpr auto testTranslation = []() {
                    Matrix43 m = Matrix43::Identity();
                    m.Translate(Vector3D(1, 2, 3));
                    return m.Row3.X == 1 && m.Row3.Y == 2 && m.Row3.Z == 3;
                };
                static_assert(testTranslation(), "Translate should update the translation vector");
            }
            
            // Test scaling using lambda
            {
                constexpr auto testScaling = []() {
                    Matrix43 m = Matrix43::Scale(2, 3, 4);
                    return m.Row0.X == 2 && m.Row0.Y == 0 && m.Row0.Z == 0 &&
                           m.Row1.X == 0 && m.Row1.Y == 3 && m.Row1.Z == 0 &&
                           m.Row2.X == 0 && m.Row2.Y == 0 && m.Row2.Z == 4 &&
                           m.Row3.X == 0 && m.Row3.Y == 0 && m.Row3.Z == 0;  // Translation should remain zero
                };
                static_assert(testScaling(), "Scaling should work correctly");
            }
        }

        // ============================================
        // Transformation Operations
        // ============================================
        
        /**
         * @brief Tests transformation operations specific to Matrix43
         * 
         * Verifies:
         * - Translation operations
         * - Combined transformations
         * - Transformation application to points and vectors
         */
        static constexpr void TestTransformationOperations()
        {
            // Test identity matrix creation
            constexpr auto testIdentity = []() {
                constexpr auto identity = Matrix43::Identity();
                return identity.Row0.X == 1 && identity.Row0.Y == 0 && identity.Row0.Z == 0 &&
                       identity.Row1.X == 0 && identity.Row1.Y == 1 && identity.Row1.Z == 0 &&
                       identity.Row2.X == 0 && identity.Row2.Y == 0 && identity.Row2.Z == 1 &&
                       identity.Row3.X == 0 && identity.Row3.Y == 0 && identity.Row3.Z == 0;
            };
            static_assert(testIdentity(), "Identity matrix should be correctly constructed");
            
            // Test translation matrix creation
            constexpr auto testTranslation = []() {
                constexpr auto translation = Matrix43::CreateTranslation(Vector3D(10, 20, 30));
                return translation.Row0.X == 1 && translation.Row0.Y == 0 && translation.Row0.Z == 0 &&
                       translation.Row1.X == 0 && translation.Row1.Y == 1 && translation.Row1.Z == 0 &&
                       translation.Row2.X == 0 && translation.Row2.Y == 0 && translation.Row2.Z == 1 &&
                       translation.Row3.X == 10 && translation.Row3.Y == 20 && translation.Row3.Z == 30;
            };
            static_assert(testTranslation(), "Translation matrix should be correctly constructed");
            
            // Define a translated matrix for reuse in multiple tests
            constexpr auto translated = Matrix43::CreateTranslation(Vector3D(1, 2, 3));
            constexpr auto identity = Matrix43::Identity();
            
            // Test equality operators
            constexpr auto testEquality = []() {
                constexpr auto identity1 = Matrix43::Identity();
                constexpr auto identity2 = Matrix43::Identity();
                
                // Test equality using the new operator==
                const bool areEqual = (identity1 == identity2);
                
                // Test inequality using the new operator!=
                const bool areNotEqual = (identity1 != translated);
                
                return areEqual && areNotEqual;
            };
            static_assert(testEquality(), "Equality operators should work correctly");
            
            // Test unary negation
            constexpr auto testNegation = []() {
                constexpr auto m = Matrix43::CreateTranslation(Vector3D(1, 2, 3));
                constexpr auto neg = -m;
                return neg.Row0.X == -1 && neg.Row0.Y == 0 && neg.Row0.Z == 0 &&
                       neg.Row1.X == 0 && neg.Row1.Y == -1 && neg.Row1.Z == 0 &&
                       neg.Row2.X == 0 && neg.Row2.Y == 0 && neg.Row2.Z == -1 &&
                       neg.Row3.X == -1 && neg.Row3.Y == -2 && neg.Row3.Z == -3;
            };
            static_assert(testNegation(), "Unary negation should negate all components");
            
            // Test matrix-vector transformation
            constexpr auto testTransformPoint = []() {
                constexpr Vector3D point(1, 2, 3);
                constexpr auto transformedPoint = identity.TransformPoint(point);
                return transformedPoint == point;
            };
            static_assert(testTransformPoint(), 
                         "Transforming point with identity should return the same point");
            
            // Test vector transformation (rotation only)
            constexpr auto testTransformVector = []() {
                constexpr Vector3D vector(1, 0, 0);
                constexpr auto transformedVector = identity.TransformVector(vector);
                return transformedVector == vector;
            };
            static_assert(testTransformVector(), 
                         "Transforming vector with identity should return the same vector");
            
            // Test matrix-matrix multiplication with identity
            constexpr auto testMatrixMultiplication = []() {
                constexpr auto result = identity * translated;
                return result == translated; // Should be the same as multiplying by identity
            };
            static_assert(testMatrixMultiplication(), 
                         "Multiplying with identity should preserve the matrix");
            
            // Test compound multiplication assignment with identity
            constexpr auto testCompoundMultiply = []() {
                Matrix43 m = identity;
                m *= translated;
                return m == translated; // Should be the same as the translated matrix
            };
            static_assert(testCompoundMultiply(), 
                         "Compound multiplication assignment should work correctly");
            
            // Test matrix inversion
            constexpr auto testMatrixInversion = []() {
                constexpr auto inverted = translated.Invert();
                constexpr auto shouldBeIdentity = translated * inverted;
                
                // Check if it's close enough to identity (accounting for floating-point precision)
                constexpr auto isIdentity = [](const Matrix43& m) {
                    return (m.Row0 - Vector3D(1, 0, 0)).LengthSquared() < 0.001f &&
                           (m.Row1 - Vector3D(0, 1, 0)).LengthSquared() < 0.001f &&
                           (m.Row2 - Vector3D(0, 0, 1)).LengthSquared() < 0.001f &&
                           m.Row3.LengthSquared() < 0.001f;
                };
                
                return isIdentity(shouldBeIdentity);
            };
            static_assert(testMatrixInversion(), 
                         "Matrix inversion should produce the identity when multiplied by original");
        }

        // ============================================
        // Advanced Features and Edge Cases
        // ============================================
        
        /**
         * @brief Tests advanced features and edge cases
         * 
         * Verifies:
         * - Matrix inversion
         * - Edge cases (zero matrix, identity)
         * - Combination of transformations
         * - Floating-point precision handling
         */
        static constexpr void TestAdvancedFeatures()
        {
            // Test matrix inversion
            {
                constexpr Matrix43 transform = Matrix43::Translation(1, 2, 3);
                constexpr Matrix43 inverse = transform.Invert();
                constexpr Matrix43 identity = transform * inverse;
                
                // Check if inverse transformation brings points back to origin
                constexpr Vector3D point(1, 2, 3);
                constexpr Vector3D transformed = transform.TransformPoint(point);
                constexpr Vector3D inverted = inverse.TransformPoint(transformed);
                
                static_assert(inverted.X == point.X && 
                              inverted.Y == point.Y && 
                              inverted.Z == point.Z,
                              "Inverse should transform points back to original position");
            }

            // Test edge case: zero matrix
            {
                constexpr Matrix43 zero;
                constexpr Matrix43 identity = Matrix43::Identity();
                
                static_assert(zero * identity == zero, 
                    "Multiplying by zero matrix should result in zero matrix");
                static_assert(identity * zero == zero,
                    "Multiplying by zero matrix should result in zero matrix");
            }

            // Test edge case: identity matrix
            {
                constexpr Matrix43 identity = Matrix43::Identity();
                constexpr Matrix43 transform = Matrix43::Translation(1, 2, 3);
                
                static_assert(identity * transform == transform && 
                             transform * identity == transform,
                    "Multiplying by identity matrix should not change the matrix");
            }

            // Test combination of transformations
            {
                constexpr Matrix43 translate = Matrix43::Translation(1, 0, 0);
                constexpr Matrix43 scale = Matrix43::Scale(2, 2, 2);
                constexpr Matrix43 combined = translate * scale;
                
                // Verify combined transformation
                constexpr Vector3D point(1, 1, 1);
                constexpr Vector3D transformed = combined.TransformPoint(point);
                
                // Should scale first (2,2,2) then translate (3,2,2)
                static_assert(transformed == Vector3D(3, 2, 2),
                    "Combined transformations should apply in correct order");
            }
        }
        
        // ============================================
        // Advanced Construction
        // ============================================
        
        /**
         * @brief Tests advanced construction methods for Matrix43.
         * 
         * @details This test verifies the advanced construction capabilities of the Matrix43 class,
         * including creating a transformation matrix from up and direction vectors, and ensuring
         * proper orthonormal basis creation.
         * 
         * Test cases:
         * - Construction from up and direction vectors with position
         * - Verification of orthonormal basis creation
         * - Verification of correct position setting
         */
        static constexpr void TestAdvancedConstruction()
        {
            // Test construction from up/direction vectors
            constexpr Vector3D up(0, 1, 0);
            constexpr Vector3D direction(0, 0, 1);
            constexpr Vector3D position(10, 20, 30);
            constexpr Matrix43 m(up, direction, position);
            
            // Verify that the Matrix33 part is built correctly from up/direction
            // The right vector should be the cross product of up and direction
            static_assert(m.Row0.X == 1 && m.Row0.Y == 0 && m.Row0.Z == 0 &&  // right vector
                          m.Row1.X == 0 && m.Row1.Y == 1 && m.Row1.Z == 0 &&  // up vector
                          m.Row2.X == 0 && m.Row2.Y == 0 && m.Row2.Z == 1 &&  // direction vector
                          m.Row3.X == 10 && m.Row3.Y == 20 && m.Row3.Z == 30,
                          "Construction from up/direction vectors should create correct orthonormal basis");
        }
        
        /**
         * @brief Runs all Matrix43 tests.
         * 
         * @details Executes all test cases for the Matrix43 class and returns the overall result.
         * The test order is structured from basic to complex functionality.
         * 
         * @return true if all tests pass, false otherwise.
         */
        /**
         * @brief Runs all Matrix43 tests
         * 
         * This method executes all test cases in the correct order.
         * The tests are organized from basic to more complex functionality.
         * 
         * @return true if all tests pass, false otherwise
         */
        static constexpr bool RunAll()
        {
            // Construction
            TestConstruction();
            
            // Factory Methods
            TestFactoryMethods();
            
            // Basic Matrix Operations
            TestBasicOperations();
            
            // Matrix-Vector Operations
            TestMatrixVectorOperations();
            
            // Matrix-Matrix Operations
            TestMatrixMatrixOperations();
            
            // Transformation Operations
            TestTransformationOperations();
            
            // Advanced Features
            TestAdvancedFeatures();
            
            // Advanced Construction
            TestAdvancedConstruction();
            
            return true;
        }
    };
    
    // Execute all tests at compile time
    static_assert((Matrix43Tests::RunAll(), true), "Matrix43 tests failed");
}
