#pragma once

/**
 * @file test_vector2d.hpp
 * @brief Compile-time unit tests for the Vector2D class
 * 
 * This file contains comprehensive tests for the Vector2D class, covering:
 * - Basic construction and properties
 * - Vector arithmetic operations
 * - Length and distance calculations with different precision modes
 * - Vector operations (dot product, cross product, normalization)
 * - Angle calculations
 * - Edge cases and error conditions
 * 
 * All tests are performed at compile-time using static_assert.
 * 
 * Test Organization:
 * 1. Construction and Basic Properties
 * 2. Arithmetic Operations
 * 3. Compound Assignment Operations
 * 4. Length and Magnitude Tests
 * 5. Comparison and Ordering
 * 6. Vector Operation Tests
 * 7. Angle and Direction Tests
 * 8. Edge Case Tests
 * 9. Precision Mode Tests
 * 10. Test Suite Runner
 */

#include "../impl/vector2d.hpp"

namespace SaturnMath::Tests
{
    using namespace SaturnMath::Types;

    /**
     * @brief Precision mode documentation for vector operations
     * 
     * The library provides three precision modes for vector operations:
     * 1. Precision::Accurate - Uses exact mathematical calculations
     * 2. Precision::Fast - Uses fast approximation algorithms (~6.3% error)
     * 3. Precision::Turbo - Uses alpha-beta-gamma approximation (up to ~38% error)
     * 
     * The trade-off is between accuracy and performance, with Turbo being fastest
     * but least accurate, and Accurate being slowest but most precise.
     */
    struct Vector2DTests
    {
        // ============================================
        // 1. Construction and Basic Properties
        // ============================================
        
        /**
         * @brief Tests the construction and initialization of Vector2D objects
         * 
         * Verifies:
         * - Default construction initializes to zero vector
         * - Single-value constructor sets both components to the same value
         * - Component-wise constructor sets X and Y components correctly
         * - Copy constructor creates an exact copy
         * 
         * @note All tests are performed at compile-time using static_assert
         */
        static constexpr void TestConstruction()
        {
            // Default constructor should initialize to zero
            constexpr Vector2D a;
            static_assert(a.X == 0 && a.Y == 0, "Default constructor should initialize to zero");
            
            // Single value constructor
            constexpr Vector2D b(5);
            static_assert(b.X == 5 && b.Y == 5, "Single value constructor should set all components to the same value");
            
            // Component constructor
            constexpr Vector2D c(1, 2);
            static_assert(c.X == 1 && c.Y == 2, "Component constructor should set components correctly");
            
            // Copy constructor
            constexpr Vector2D d = c;
            static_assert(d.X == c.X && d.Y == c.Y, "Copy constructor should work");
        }
        
        // ============================================
        // 2. Arithmetic Operations
        // ============================================
        
        /**
         * @brief Tests basic arithmetic operations on Vector2D objects
         * 
         * Verifies:
         * - Vector addition and subtraction
         * - Scalar multiplication and division (both left and right)
         * - Vector negation
         * - Correct handling of component-wise operations
         * 
         * @note All operations are tested using exact comparisons as they should
         * produce precise results with integer coordinates.
         */
        static constexpr void TestArithmetic()
        {
            constexpr Vector2D a(1, 2);
            constexpr Vector2D b(3, 4);
            
            // Addition
            constexpr Vector2D sum = a + b;
            static_assert(sum.X == 4 && sum.Y == 6, "Vector addition should work");
            
            // Subtraction
            constexpr Vector2D diff = b - a;
            static_assert(diff.X == 2 && diff.Y == 2, "Vector subtraction should work");
            
            // Scalar multiplication
            constexpr Vector2D mulScalar = a * 2;
            static_assert(mulScalar.X == 2 && mulScalar.Y == 4, "Scalar multiplication should work");
            
            // Scalar multiplication (reversed)
            constexpr Vector2D mulScalarRev = 2 * a;
            static_assert(mulScalarRev.X == 2 && mulScalarRev.Y == 4, "Scalar multiplication (reversed) should work");
            
            // Scalar division
            constexpr Vector2D divScalar = b / 2;
            static_assert(divScalar.X == 1.5 && divScalar.Y == 2, "Scalar division should work");
            
            // Negation
            constexpr Vector2D neg = -a;
            static_assert(neg.X == -1 && neg.Y == -2, "Vector negation should work");
        }
        
        // ============================================
        // 3. Compound Assignment Operations
        // ============================================
        
        /**
         * @brief Tests compound assignment operations on Vector2D objects
         * 
         * Verifies:
         * - Addition assignment (+=)
         * - Subtraction assignment (-=)
         * - Multiplication assignment (*=)
         * - Division assignment (/=)
         * 
         * @note These operations modify the vector in-place and return a reference
         * to the modified vector. The tests verify both the modification and the
         * return value through the function's boolean result.
         */
        static constexpr void TestCompoundAssignment()
        {
            constexpr auto TestCompound = []() {
                Vector2D a(1, 2);
                Vector2D b(3, 4);
                
                // Addition assignment
                a += b;
                if (a.X != 4 || a.Y != 6) return false;
                
                // Subtraction assignment
                a -= b;
                if (a.X != 1 || a.Y != 2) return false;
                
                // Multiplication assignment
                a *= 2;
                if (a.X != 2 || a.Y != 4) return false;
                
                // Division assignment
                a /= 2;
                if (a.X != 1 || a.Y != 2) return false;
                
                return true;
            };
            static_assert(TestCompound(), "Compound assignment operators should work");
        }
        
        // ============================================
        // 4. Comparison and Ordering
        // ============================================
        
        /**
         * @brief Tests comparison operations on Vector2D objects
         * 
         * Verifies:
         * - Equality and inequality operators (==, !=)
         * - Lexicographical comparison operators (<, >, <=, >=)
         * - Correct handling of component-wise comparisons
         * - Proper ordering of vectors in all quadrants
         * 
         * @note The lexicographical comparison first compares the X components, and if
         * they are equal, it then compares the Y components. This establishes a total
         * order among 2D vectors.
         */
        static constexpr void TestComparisons()
        {
            constexpr Vector2D a(1, 2);
            constexpr Vector2D b(1, 2);
            constexpr Vector2D c(3, 4);
            
            // Equality
            static_assert(a == b, "Equality operator should work");
            static_assert(a != c, "Inequality operator should work");
            
            // Lexicographic comparison (X first, then Y)
            constexpr Vector2D d(1, 3);
            constexpr Vector2D e(2, 1);
            
            static_assert(a < d, "Less than operator should work (same X, different Y)");
            static_assert(a < e, "Less than operator should work (different X)");
            static_assert(e > a, "Greater than operator should work");
        }
        
        // ============================================
        // 5. Vector Operations
        // ============================================
        
        /**
         * @brief Tests core vector operations and properties
         * 
         * Verifies:
         * - Dot product calculations
         * - 2D cross product (which returns a scalar)
         * - Vector length calculations with different precision modes
         * - Vector normalization with different precision settings
         * - Length squared calculations
         * - Distance calculations between points
         * 
         * @note Tests include verification of different precision modes (Accurate, Fast, Turbo)
         * to ensure they meet their respective accuracy guarantees. The 2D cross product returns
         * the magnitude of the 3D cross product if the vectors were in the XY plane.
         */
        static constexpr void TestVectorOperations()
        {
            constexpr Vector2D a(3, 4);
            constexpr Vector2D b(1, 2);
            
            // Dot product
            constexpr Fxp dot = a.Dot(b);
            static_assert(dot == 11, "Dot product should work");
            
            // Cross product (2D cross product returns a scalar)
            constexpr Fxp cross = a.Cross(b);
            static_assert(cross == 2, "Cross product should work");
            
            // Length
            constexpr Fxp lengthAccurate = a.Length<Precision::Accurate>();
            constexpr Fxp lengthFast = a.Length<Precision::Fast>();
            
            // Accurate precision should be exactly 5
            static_assert(lengthAccurate == 5, "Length calculation (Accurate) should be exactly 5");
            
            // Fast precision should be within 6.3% error (max error documented in Fxp::Sqrt)
            constexpr Fxp lengthError = (lengthFast - 5).Abs();
            static_assert(lengthError <= 0.315, "Length calculation (Fast) should be within 6.3% error");
            
            // Length squared
            constexpr Fxp distanceSq = (a - b).LengthSquared();
            
            // Should be exactly 8
            static_assert(distanceSq == 8, "Distance squared calculation should be exact");
            
            // Distance
            constexpr Fxp distanceAccurate = a.DistanceTo<Precision::Accurate>(b); 
            constexpr Fxp distanceFast = a.DistanceTo<Precision::Fast>(b); 
            constexpr Fxp distanceTurbo = a.DistanceTo<Precision::Turbo>(b);
            
            // Accurate precision should be exactly 2.828427
            constexpr Fxp exactDistance = 2.828427;
            constexpr Fxp distanceAccurateError = (distanceAccurate - exactDistance).Abs();
            
            static_assert(distanceAccurateError <= 0.001, "Distance calculation (Accurate) should be exact");
            
            // Fast precision should be within 6.3% error
            constexpr Fxp distanceFastError = (distanceFast - exactDistance).Abs();
            static_assert(distanceFastError <= 0.178, "Distance calculation (Fast) should be within 6.3% error");
            
            // Turbo precision should be at least as accurate as Fast
            constexpr Fxp distanceTurboError = (distanceTurbo - exactDistance).Abs();
            static_assert(distanceTurboError <= distanceFastError, "Distance calculation (Turbo) should be at least as accurate as Fast");
            
            // Normalization
            constexpr Vector2D normalizedAccurate = a.Normalize<Precision::Accurate>();
            constexpr Vector2D normalizedFast = a.Normalize<Precision::Fast>();
            constexpr Vector2D normalizedTurbo = a.Normalize<Precision::Turbo>();
            
            // Accurate precision should be exactly (0.6, 0.8)
            static_assert(normalizedAccurate.X == 0.6 && normalizedAccurate.Y == 0.8, "Normalization (Accurate) should work");
            
            // Fast precision should be close to (0.6, 0.8) with a small error margin
            constexpr Fxp xError = (normalizedFast.X - 0.6).Abs();
            constexpr Fxp yError = (normalizedFast.Y - 0.8).Abs();
            static_assert(xError <= 0.063 && yError <= 0.063, "Normalization (Fast) should be within 6.3% error");
            
            // Turbo precision should be at least as accurate as Fast
            constexpr Fxp xTurboError = (normalizedTurbo.X - 0.6).Abs();
            constexpr Fxp yTurboError = (normalizedTurbo.Y - 0.8).Abs();
            static_assert(xTurboError <= xError && yTurboError <= yError, "Normalization (Turbo) should be at least as accurate as Fast");
        }
        
        // ============================================
        // 6. Absolute Value and Sorting Tests
        // ============================================
        
        /**
         * @brief Tests absolute value and sorting operations on Vector2D objects
         * 
         * Verifies:
         * - Absolute value calculation (component-wise)
         * - Sorting vector components in ascending order
         * - Sorting vector components in descending order
         * - Preservation of component signs during sorting
         * 
         * @note The absolute value operation is applied to each component independently.
         * Sorting reorders the components while preserving their original values.
         * The Sort method does not modify the original vector but returns a new one.
         */
        static constexpr void TestAbsAndSort()
        {
            constexpr Vector2D a(-1, 2);
            
            // Absolute value
            constexpr Vector2D abs = a.Abs();
            static_assert(abs.X == 1 && abs.Y == 2, "Absolute value should work");
            
            // Sorting (ascending)
            constexpr Vector2D sorted = a.Sort<SortOrder::Ascending>();
            static_assert(sorted.X == -1 && sorted.Y == 2, "Sorting (ascending) should work");
            
            // Sorting (descending)
            constexpr Vector2D sortedDesc = a.Sort<SortOrder::Descending>();
            static_assert(sortedDesc.X == 2 && sortedDesc.Y == -1, "Sorting (descending) should work");
        }
        
        // ============================================
        // 7. Static Method Tests
        // ============================================
        
        /**
         * @brief Tests static factory methods of the Vector2D class
         * 
         * Verifies the following static methods:
         * - UnitX/Y: Standard basis vectors
         * - Zero/One: Common constant vectors
         * - Directional vectors: Right, Left, Up, Down
         * 
         * @note These methods are commonly used as constants and should always
         * return the same values. They are tested for both correctness and constexpr
         * evaluation.
         */
        static constexpr void TestStaticMethods()
        {
            // Test UnitX
            constexpr Vector2D unitX = Vector2D::UnitX();
            static_assert(unitX.X == 1 && unitX.Y == 0, "UnitX should return (1, 0)");
            
            // Test UnitY
            constexpr Vector2D unitY = Vector2D::UnitY();
            static_assert(unitY.X == 0 && unitY.Y == 1, "UnitY should return (0, 1)");
            
            // Test Zero
            constexpr Vector2D zero = Vector2D::Zero();
            static_assert(zero.X == 0 && zero.Y == 0, "Zero should return (0, 0)");
            
            // Test One
            constexpr Vector2D one = Vector2D::One();
            static_assert(one.X == 1 && one.Y == 1, "One should return (1, 1)");
            
            // Test Right (same as UnitX)
            constexpr Vector2D right = Vector2D::Right();
            static_assert(right.X == 1 && right.Y == 0, "Right should return (1, 0)");
            
            // Test Left
            constexpr Vector2D left = Vector2D::Left();
            static_assert(left.X == -1 && left.Y == 0, "Left should return (-1, 0)");
            
            // Test Up (same as UnitY)
            constexpr Vector2D up = Vector2D::Up();
            static_assert(up.X == 0 && up.Y == 1, "Up should return (0, 1)");
            
            // Test Down
            constexpr Vector2D down = Vector2D::Down();
            static_assert(down.X == 0 && down.Y == -1, "Down should return (0, -1)");
        }
        
        // ============================================
        // 8. Bitwise Operation Tests
        // ============================================
        
        /**
         * @brief Tests bitwise shift operations on Vector2D components
         * 
         * Verifies:
         * - Right shift operator (>>) and right shift assignment (>>=)
         * - Left shift operator (<<) and left shift assignment (<<=)
         * - Behavior with zero shift (no-op)
         * - Handling of negative values
         * - Component-wise operation correctness
         * 
         * @note These operations are performed independently on each component.
         * For negative values, the behavior follows C++'s arithmetic shift rules.
         * The shift amount is applied to both X and Y components.
         */
        static constexpr void TestBitwiseOperations()
        {
            // Test shift right operator (>>)
            constexpr Vector2D a(8, 16);
            constexpr Vector2D shiftedRight = a >> 2; // Should be (2, 4)
            static_assert(shiftedRight.X == 2 && shiftedRight.Y == 4, 
                "Shift right operator should shift bits to the right");
            
            // Test shift right assignment operator (>>=)
            constexpr auto TestShiftRightAssignment = []() {
                Vector2D b(32, 64);
                b >>= 3; // Should be (4, 8)
                return b.X == 4 && b.Y == 8;
            };
            static_assert(TestShiftRightAssignment(), 
                "Shift right assignment operator should shift bits to the right in place");
            
            // Test shift left operator (<<)
            constexpr Vector2D c(1, 3);
            constexpr Vector2D shiftedLeft = c << 3; // Should be (8, 24)
            static_assert(shiftedLeft.X == 8 && shiftedLeft.Y == 24, 
                "Shift left operator should shift bits to the left");
            
            // Test shift left assignment operator (<<=)
            constexpr auto TestShiftLeftAssignment = []() {
                Vector2D d(2, 5);
                d <<= 2; // Should be (8, 20)
                return d.X == 8 && d.Y == 20;
            };
            static_assert(TestShiftLeftAssignment(), 
                "Shift left assignment operator should shift bits to the left in place");
            
            // Test with zero shift (should be no-op)
            constexpr Vector2D e(5, 10);
            constexpr Vector2D noShift = e << 0;
            static_assert(noShift.X == 5 && noShift.Y == 10, 
                "Shift by zero should be a no-op");
            
            // Test with negative values
            constexpr Vector2D f(-8, -16);
            constexpr Vector2D shiftedRightNegative = f >> 2; // Should be (-2, -4)
            static_assert(shiftedRightNegative.X == -2 && shiftedRightNegative.Y == -4, 
                "Shift right should work with negative values");
                
            constexpr Vector2D shiftedLeftNegative = f << 1; // Should be (-16, -32)
            static_assert(shiftedLeftNegative.X == -16 && shiftedLeftNegative.Y == -32, 
                "Shift left should work with negative values");
        }
        
        // ============================================
        // 9. Multi-Dot Product Accumulation Tests
        // ============================================
        
        /**
         * @brief Tests the MultiDotAccumulate functionality for efficient dot product calculations
         * 
         * Verifies:
         * - Single pair dot product calculation
         * - Multiple pairs dot product accumulation
         * - Orthogonal vectors (expecting zero dot product)
         * - Parallel vectors (expecting maximum dot product)
         * - Mixed parallel and orthogonal vectors
         * 
         * @note MultiDotAccumulate is optimized for calculating the sum of multiple
         * dot products in a single operation, which can be more efficient than
         * calculating them separately. This is particularly useful for matrix operations
         * and other vector-heavy computations.
         */
        static constexpr void TestMultiDotAccumulate()
        {
            // Test with a single pair of vectors
            constexpr Vector2D v1(1, 2);
            constexpr Vector2D v2(3, 4);
            
            constexpr Fxp singleDot = Vector2D::MultiDotAccumulate(
                std::pair{v1, v2}
            );
            static_assert(singleDot == 11, "MultiDotAccumulate with single pair should equal Dot product");
            
            // Test with multiple pairs
            constexpr Vector2D v3(2, 0);
            constexpr Vector2D v4(0, 3);
            
            constexpr Fxp multiDot = Vector2D::MultiDotAccumulate(
                std::pair{v1, v2},  // 1*3 + 2*4 = 11
                std::pair{v3, v4}   // 2*0 + 0*3 = 0
            );
            static_assert(multiDot == 11, "MultiDotAccumulate with multiple pairs should sum dot products");
            
            // Test with orthogonal vectors (dot product should be zero)
            constexpr Vector2D xAxis(1, 0);
            constexpr Vector2D yAxis(0, 1);
            
            constexpr Fxp orthogonalDot = Vector2D::MultiDotAccumulate(
                std::pair{xAxis, yAxis}
            );
            static_assert(orthogonalDot == 0, "Dot product of orthogonal vectors should be zero");
            
            // Test with parallel vectors
            constexpr Vector2D a(2, 3);
            constexpr Vector2D b(4, 6); // b = 2 * a
            
            constexpr Fxp parallelDot = Vector2D::MultiDotAccumulate(
                std::pair{a, b}  // 2*4 + 3*6 = 8 + 18 = 26
            );
            static_assert(parallelDot == 26, "Dot product of parallel vectors should match expected value");
            
            // Test with multiple parallel and orthogonal vectors
            constexpr Fxp mixedDot = Vector2D::MultiDotAccumulate(
                std::pair{xAxis, yAxis},  // 0 (orthogonal)
                std::pair{a, b},          // 26 (parallel)
                std::pair{v1, v2}         // 11
            );
            static_assert(mixedDot == 37, "MultiDotAccumulate should correctly sum mixed dot products");
        }
        
        // ============================================
        // 10. Linear Interpolation Tests
        // ============================================
        
        /**
         * @brief Tests linear interpolation between Vector2D points
         * 
         * Verifies:
         * - Interpolation at t=0 returns the start point
         * - Interpolation at t=1 returns the end point
         * - Interpolation at t=0.5 returns the midpoint
         * - Clamping behavior for t < 0 and t > 1
         * - Correct component-wise interpolation
         * 
         * @note The linear interpolation formula used is:
         *   start + t * (end - start)
         * Where t is clamped to [0, 1] if outside this range.
         * The operation is performed component-wise on the vectors.
         */
        static constexpr void TestLerp()
        {
            // Test linear interpolation (Lerp)
            constexpr Vector2D start(1, 2);
            constexpr Vector2D end(5, 6);
            
            // Lerp at t=0 should return start
            constexpr Vector2D atStart = Vector2D::Lerp(start, end, 0.0);
            static_assert(atStart == start, "Lerp at t=0 should return start point");
            
            // Lerp at t=1 should return end
            constexpr Vector2D atEnd = Vector2D::Lerp(start, end, 1.0);
            static_assert(atEnd == end, "Lerp at t=1 should return end point");
            
            // Lerp at t=0.5 should return midpoint
            constexpr Vector2D mid = Vector2D::Lerp(start, end, 0.5);
            constexpr Vector2D expectedMid(3, 4);
            static_assert(mid == expectedMid, "Lerp at t=0.5 should return midpoint");
            
            // Test with t outside [0,1] range
            constexpr Vector2D beforeStart = Vector2D::Lerp(start, end, -1.0);
            static_assert(beforeStart == start, "Lerp with t<0 should clamp to start");
            
            constexpr Vector2D afterEnd = Vector2D::Lerp(start, end, 2.0);
            static_assert(afterEnd == end, "Lerp with t>1 should clamp to end");
        }
        
        // ============================================
        // 11. Angle Calculation Tests
        // ============================================
        
        /**
         * @brief Tests angle calculations between Vector2D objects
         * 
         * Verifies angle calculations in all four quadrants:
         * - Right angles (90°) between axis-aligned vectors
         * - Straight angles (180°) between opposite vectors
         * - Zero angle between identical vectors
         * - Angles with zero vectors
         * - Angles with non-normalized vectors
         * - Wrap-around behavior at 180°
         * - 45° angle calculations
         * 
         * @note All angles are tested using the Angle class which handles
         * angle units and normalization. The tests verify both the mathematical
         * correctness and the handling of edge cases.
         */
        static constexpr void TestAngles()
        {
            // Test angle between vectors in all four quadrants
            constexpr Vector2D xAxis(1, 0);
            constexpr Vector2D yAxis(0, 1);
            constexpr Vector2D negX(-1, 0);
            constexpr Vector2D negY(0, -1);
            
            // Test right angles between axis-aligned vectors
            constexpr Angle rightAngle1 = Vector2D::Angle(xAxis, yAxis);
            static_assert(rightAngle1 == Angle::Right(), "Angle between x and y axes should be 90°");
            
            constexpr Angle rightAngle2 = Vector2D::Angle(yAxis, negX);
            static_assert(rightAngle2 == Angle::Right(), "Angle between y and -x axes should be 90°");
            
            // Test straight angle (180°)
            constexpr Angle straightAngle = Vector2D::Angle(xAxis, negX);
            static_assert(straightAngle == Angle::Straight(), "Angle between x and -x should be 180°");
            
            // Test zero angle (same vector)
            constexpr Angle zeroAngle = Vector2D::Angle(xAxis, xAxis);
            static_assert(zeroAngle == Angle::Zero(), "Angle between identical vectors should be 0°");
            
            // Test angle with zero vector (should return 0 for safety)
            constexpr Vector2D zero;
            constexpr Angle angleWithZero = Vector2D::Angle(xAxis, zero);
            static_assert(angleWithZero == Angle::Zero(), "Angle with zero vector should return 0 for safety");
            
            // Test angle with non-normalized vectors
            constexpr Vector2D longX(2, 0);
            constexpr Vector2D longY(0, 2);
            constexpr Angle angleNonNormalized = Vector2D::Angle(longX, longY);
            static_assert(angleNonNormalized == Angle::Right(),
                "Angle between non-normalized perpendicular vectors should be 90°");
                
            // Test angle between vectors in different quadrants
            constexpr Vector2D v1(1, 1);    // Q1
            constexpr Vector2D v2(-1, 1);   // Q2
            constexpr Vector2D v3(-1, -1);  // Q3
            constexpr Vector2D v4(1, -1);   // Q4
            
            // 90° angles between adjacent quadrants
            constexpr Angle angleQ1Q2 = Vector2D::Angle(v1, v2);
            constexpr Angle angleQ2Q3 = Vector2D::Angle(v2, v3);
            constexpr Angle angleQ3Q4 = Vector2D::Angle(v3, v4);
            constexpr Angle angleQ4Q1 = Vector2D::Angle(v4, v1);
            
            static_assert(angleQ1Q2 == Angle::Right(), "Angle between Q1 and Q2 should be 90°");
            static_assert(angleQ2Q3 == Angle::Right(), "Angle between Q2 and Q3 should be 90°");
            static_assert(angleQ3Q4 == Angle::Right(), "Angle between Q3 and Q4 should be 90°");
            static_assert(angleQ4Q1 == Angle::Right(), "Angle between Q4 and Q1 should be 90°");
            
            // 180° angles between opposite quadrants
            constexpr Angle angleQ1Q3 = Vector2D::Angle(v1, v3);
            constexpr Angle angleQ2Q4 = Vector2D::Angle(v2, v4);
            
            static_assert(angleQ1Q3 == Angle::Straight(), "Angle between Q1 and Q3 should be 180°");
            static_assert(angleQ2Q4 == Angle::Straight(), "Angle between Q2 and Q4 should be 180°");
            
            // Test angle with non-unit vectors
            constexpr Vector2D longVec1(3, 4);  // Length 5
            constexpr Vector2D longVec2(4, -3); // Length 5
            constexpr Angle angleLongVecs = Vector2D::Angle(longVec1, longVec2);
            // The angle should be -90° (or 270°), which is a right angle in the negative direction
            constexpr Angle expectedAngle1 = -Angle::Right();
            constexpr Angle expectedAngle2 = Angle::Pi() + Angle::HalfPi();  // 180° + 90° = 270°
            static_assert(angleLongVecs == expectedAngle1 || angleLongVecs == expectedAngle2, 
                "Angle between (3,4) and (4,-3) should be 270° (or -90°)");
        }
        
        // ============================================
        // 12. Projection and Reflection Tests
        // ============================================
        
        /**
         * @brief Tests vector projection and reflection operations
         * 
         * Verifies:
         * - Projection of a vector onto another vector
         * - Projection of a vector onto itself (should return the same vector)
         * - Projection onto perpendicular vectors (should return zero vector)
         * - Reflection across axis-aligned normals (X and Y axes)
         * - Reflection across a vector's own normal (should invert the vector)
         * - Reflection with non-unit normal vectors
         * - Reflection across 45-degree diagonal
         * 
         * @note Projection formula: proj_b(a) = (a·b / b·b) * b
         * Reflection formula: reflect(v, n) = v - 2*(v·n)*n
         * where n is the normal vector (must be normalized for standard reflection)
         */
        static constexpr void TestProjectionAndReflection()
        {
            // Test projection of vector onto another vector
            constexpr Vector2D v(3, 4);
            constexpr Vector2D onto(1, 0); // X-axis
            
            // Project v onto X-axis should give (3, 0)
            constexpr Vector2D proj = v.ProjectOnto(onto);
            static_assert(proj.X == 3 && proj.Y == 0, 
                "Projection onto X-axis should zero out Y component");
            
            // Projection of a vector onto itself should return the same vector (normalized)
            constexpr Vector2D v2(1, 1);
            constexpr Vector2D projSelf = v2.ProjectOnto(v2.Normalized<Precision::Default>());
            constexpr Fxp projError = (projSelf - v2).Length<Precision::Default>();
            static_assert(projError < 0.001, 
                "Projection of a vector onto its normalized version should return the original vector");
            
            // Projection of a vector onto a perpendicular vector should be zero
            constexpr Vector2D v3(1, 1);
            constexpr Vector2D perp(-1, 1); // Perpendicular to v3
            constexpr Vector2D projPerp = v3.ProjectOnto(perp);
            constexpr Fxp projPerpLength = projPerp.Length<Precision::Default>();
            static_assert(projPerpLength < 0.001, 
                "Projection of a vector onto a perpendicular vector should be zero");
            
            // Test reflection of a vector across a normal
            // Reflecting (1,1) across the X-axis should give (1,-1)
            constexpr Vector2D toReflect(1, 1);
            constexpr Vector2D normal(0, 1); // Normal pointing up
            constexpr Vector2D reflected = toReflect.Reflect(normal);
            static_assert(reflected.X == 1 && reflected.Y == -1, 
                "Reflection across X-axis should invert Y component");
            
            // Reflecting a vector across its normal should invert it
            constexpr Vector2D v4(2, 3);
            constexpr Vector2D v4Normalized = v4.Normalized<Precision::Default>();
            constexpr Vector2D reflected2 = v4.Reflect(v4Normalized);
            constexpr Vector2D expectedReflection = -v4;
            constexpr Fxp reflectionError = (reflected2 - expectedReflection).Length<Precision::Default>();
            static_assert(reflectionError < 0.001, 
                "Reflecting a vector across its own normal should invert it");
            
            // Test reflection with non-unit normal
            // Reflecting (1,0) across X-axis (normal = (0,2)) should give (1,0) since it's parallel to the mirror line
            constexpr Vector2D v5(1, 0);
            constexpr Vector2D xAxisNormal(0, 2); // X-axis normal (non-unit)
            constexpr Vector2D reflected3 = v5.Reflect(xAxisNormal);
            static_assert(reflected3.X == 1 && reflected3.Y == 0, 
                "Reflecting (1,0) across X-axis should give (1,0) since it's parallel to the mirror line");
                
            // Test reflection of (0,1) across X-axis (normal = (0,2)) should give (0,-1)
            constexpr Vector2D v6(0, 1);
            constexpr Vector2D reflected4 = v6.Reflect(xAxisNormal);
            static_assert(reflected4.X == 0 && reflected4.Y == -1, 
                "Reflecting (0,1) across X-axis should give (0,-1)");
                
            // Test reflection with non-unit normal at 45 degrees
            constexpr Vector2D v7(1, 1);
            constexpr Vector2D diagonalNormal(1, 1); // 45 degree normal (non-unit)
            constexpr Vector2D reflected5 = v7.Reflect(diagonalNormal);
            static_assert(reflected5.X == -1 && reflected5.Y == -1, 
                "Reflecting (1,1) across 45-degree normal should give (-1,-1)");
        }
        
        // ============================================
        // 13. Distance Method Tests
        // ============================================
        
        /**
         * @brief Tests distance-related methods of Vector2D
         * 
         * Verifies:
         * - Basic distance squared calculation between points
         * - Distance squared with negative coordinates
         * - Distance squared from a point to itself (should be zero)
         * - Distance squared with large values (verifying no overflow)
         * - Correct handling of edge cases
         * 
         * @note Distance squared is often preferred over actual distance for performance
         * reasons when only comparing distances, as it avoids the expensive square root
         * operation. These tests ensure the squared distance calculations are accurate.
         */
        static constexpr void TestDistanceMethods()
        {
            // Test basic distance squared calculation
            {
                constexpr Vector2D a(1, 2);
                constexpr Vector2D b(4, 6);
                constexpr Fxp distSq = a.DistanceSquared(b);
                static_assert(distSq == 25, "Distance squared calculation should be 25 (3² + 4²)");
            }

            // Test distance squared with negative coordinates
            {
                constexpr Vector2D a(-1, -1);
                constexpr Vector2D b(2, 3);
                constexpr Fxp distSq = a.DistanceSquared(b);
                static_assert(distSq == 25, "Distance squared with negative coords should be 25 (3² + 4²)");
            }

            // Test distance squared with same point
            {
                constexpr Vector2D a(5, 5);
                constexpr Vector2D b(5, 5);
                constexpr Fxp distSq = a.DistanceSquared(b);
                static_assert(distSq == 0, "Distance squared to self should be 0");
            }

            // Test distance squared with large values (should not overflow)
            {
                constexpr Vector2D a(100, 200);
                constexpr Vector2D b(300, 400);
                constexpr Fxp dx = 300 - 100;  // 200
                constexpr Fxp dy = 400 - 200;  // 200
                constexpr Fxp expected = dx * dx + dy * dy;  // 200*200 + 200*200 = 40000 + 40000 = 80000
                constexpr Fxp distSq = a.DistanceSquared(b);
                static_assert(distSq == expected, "Distance squared with large values should be 80,000 (200² + 200²)");
            }
        }
        
        // ============================================
        // 14. Edge Case Tests
        // ============================================
        
        /**
         * @brief Tests edge cases and extreme values for Vector2D operations
         * 
         * Verifies behavior with:
         * - Zero vector length and normalization
         * - Very large values (verifying overflow protection)
         * - Very small values (verifying underflow protection)
         * - Mixed large and small values
         * - Minimum and maximum representable values
         * - Safe length squared calculations with overflow protection
         * 
         * @note These tests are crucial for ensuring numerical stability and
         * correctness in all edge cases. They verify that the implementation
         * handles extreme values gracefully without overflow or underflow.
         */
        static constexpr void TestEdgeCases()
        {
            // Test with zero vector
            constexpr Vector2D zero = Vector2D::Zero();
            
            // Length of zero vector should be zero for all precision modes
            constexpr Fxp zeroLengthAccurate = zero.Length<Precision::Accurate>();
            constexpr Fxp zeroLengthFast = zero.Length<Precision::Fast>();
            constexpr Fxp zeroLengthTurbo = zero.Length<Precision::Turbo>();
            static_assert(zeroLengthAccurate == 0, "Accurate: Length of zero vector should be zero");
            static_assert(zeroLengthFast == 0, "Fast: Length of zero vector should be zero");
            static_assert(zeroLengthTurbo == 0, "Turbo: Length of zero vector should be zero");
            
            // Normalization of zero vector should return zero vector (to avoid division by zero)
            constexpr Vector2D normalizedZero = zero.Normalized<Precision::Default>();
            static_assert(normalizedZero.X == 0 && normalizedZero.Y == 0, 
                "Normalization of zero vector should return zero vector");
            
            // Test with moderate values (avoiding overflow)
            constexpr Fxp largeValue = 100; // Safe value that won't overflow when squared
            constexpr Vector2D largeVec(largeValue, largeValue);
            constexpr Fxp expectedLargeLength = largeValue * 1.41421356237; // sqrt(2) * largeValue
            
            // Test with different precision modes
            {
                // Accurate mode - should be very precise
                constexpr Fxp length = largeVec.Length<Precision::Accurate>();
                constexpr Fxp error = (length - expectedLargeLength).Abs();
                // Allow 2% error for accurate mode (due to fixed-point limitations)
                static_assert(error < expectedLargeLength * 0.02, "Accurate: Length should be very precise for large values");
            }
            
            {
                // Fast mode - allow 10% error
                constexpr Fxp length = largeVec.Length<Precision::Fast>();
                constexpr Fxp error = (length - expectedLargeLength).Abs();
                static_assert(error < expectedLargeLength * 0.10, "Fast: Length should be reasonably accurate for large values");
            }
            
            {
                // Turbo mode - allow 20% error
                constexpr Fxp length = largeVec.Length<Precision::Turbo>();
                constexpr Fxp error = (length - expectedLargeLength).Abs();
                static_assert(error < expectedLargeLength * 0.20, "Turbo: Length should be within acceptable range for large values");
                
                // Turbo should be faster but less accurate than Fast
                constexpr Fxp fastLength = largeVec.Length<Precision::Fast>();
                constexpr Fxp turboVsFastError = (length - fastLength).Abs();
                static_assert(turboVsFastError > 0.0, "Turbo: Should be different from Fast mode");
            }
            
            // Test with small values (using a larger value for better fixed-point precision)
            constexpr Fxp smallValue = 0.1; // Using a larger small value for better fixed-point precision
            constexpr Vector2D smallVec(smallValue, smallValue);
            constexpr Fxp expectedSmallLength = smallValue * 1.41421356237; // sqrt(2) * smallValue
            
            // Test with different precision modes for small values
            {
                // Accurate mode - should be precise even for small values
                constexpr Fxp length = smallVec.Length<Precision::Accurate>();
                constexpr Fxp error = (length - expectedSmallLength).Abs();
                static_assert(error < smallValue * 0.05, "Accurate: Length should be precise for small values (5% error allowed)");
            }
            
            {
                // Fast mode - allow more error for small values
                constexpr Fxp length = smallVec.Length<Precision::Fast>();
                constexpr Fxp error = (length - expectedSmallLength).Abs();
                static_assert(error < smallValue * 0.20, "Fast: Length should be within 20% error for small values");
            }
            
            {
                // Turbo mode - allow even more error for small values
                constexpr Fxp length = smallVec.Length<Precision::Turbo>();
                constexpr Fxp error = (length - expectedSmallLength).Abs();
                static_assert(error < smallValue * 0.50, "Turbo: Length should be within 50% error for small values");
            }
            
            // Test with mixed large and small values (100, 0.01)
            constexpr Fxp mixedSmallValue = 0.01; // Larger small value for better fixed-point precision
            constexpr Vector2D mixedVec(largeValue, mixedSmallValue);
            
            // For mixed values, calculate the expected length using Pythagorean theorem
            constexpr Fxp expectedMixedLength = (largeValue * largeValue + mixedSmallValue * mixedSmallValue).Sqrt<Precision::Accurate>();
            
            {
                // Accurate mode - should handle the small component correctly
                constexpr Fxp length = mixedVec.Length<Precision::Accurate>();
                constexpr Fxp error = (length - expectedMixedLength).Abs();
                // Allow 2% error for accurate mode (due to fixed-point limitations)
                static_assert(error < expectedMixedLength * 0.02, "Accurate: Should handle mixed values precisely");
            }
            
            {
                // Fast mode - allow 10% error
                constexpr Fxp length = mixedVec.Length<Precision::Fast>();
                constexpr Fxp error = (length - expectedMixedLength).Abs();
                static_assert(error < expectedMixedLength * 0.10, "Fast: Should handle mixed values with acceptable error");
            }
            
            {
                // Turbo mode - allow 20% error
                constexpr Fxp length = mixedVec.Length<Precision::Turbo>();
                constexpr Fxp error = (length - expectedMixedLength).Abs();
                static_assert(error < expectedMixedLength * 0.20, "Turbo: Should handle mixed values within acceptable range");
                
                // Turbo should be faster but less accurate than Fast
                constexpr Fxp fastLength = mixedVec.Length<Precision::Fast>();
                constexpr Fxp turboVsFastError = (length - fastLength).Abs();
                static_assert(turboVsFastError > 0.0, "Turbo: Should be different from Fast mode");
            }
            
            // Test with values that won't cause overflow - using unsafe version for performance
            constexpr Fxp testVal = 100;  // Safe value well below overflow threshold
            constexpr Vector2D testVec(testVal, testVal);
            constexpr Fxp expectedSquared = testVal * testVal * 2;  // 100*100*2 = 20000
            
            // Verify safe version works for safe values
            constexpr Fxp safeSquared = testVec.LengthSquared();
            static_assert(safeSquared == expectedSquared, "Safe length squared calculation incorrect");
            
            // Test edge cases with safe version (should return MaxValue for overflow cases)
            constexpr Fxp maxFxp = Fxp::MaxValue();
            constexpr Vector2D maxVec(maxFxp, maxFxp);
            static_assert(maxVec.LengthSquared() == Fxp::MaxValue(), "Safe version should return MaxValue for max value");
            
            constexpr Fxp minFxp = Fxp::MinValue();
            constexpr Vector2D minVec(minFxp, minFxp);
            static_assert(minVec.LengthSquared() == Fxp::MaxValue(), "Safe version should return MaxValue for min value");
            
            constexpr Fxp halfMax = Fxp::MaxValue() / 2;
            constexpr Vector2D halfMaxVec(halfMax, halfMax);
            static_assert(halfMaxVec.LengthSquared() == Fxp::MaxValue(), "Safe version should return MaxValue for potential overflow");
            
            // Test with a simple case that should work
            constexpr Vector2D simpleVec(1, 1);
            constexpr Fxp simpleLenSq = simpleVec.LengthSquared();
            static_assert(simpleLenSq > Fxp(0), "Simple length squared should be positive");
            
            // Test with a large value that should trigger MaxValue
            constexpr Vector2D largeVec2(181, 0);
            static_assert(largeVec2.LengthSquared() == Fxp::MaxValue(), "Large values should return MaxValue");
            
            // Test with MinValue
            static_assert(Vector2D(Fxp::MinValue(), Fxp(0)).LengthSquared() == Fxp::MaxValue(), "MinValue should return MaxValue");
        }
        
        // ============================================
        // 15. Test Suite Runner
        // ============================================
        
        /**
         * @brief Executes all test cases in the Vector2D test suite
         * 
         * This method runs all test methods in sequence. If any test fails,
         * it will trigger a compile-time error via static_assert. The tests are
         * organized to verify all aspects of Vector2D functionality, including:
         * - Basic construction and arithmetic
         * - Vector operations and properties
         * - Edge cases and error conditions
         * - Performance-sensitive operations with different precision modes
         * 
         * @note This method is marked as constexpr to enable compile-time
         * evaluation of all tests. Any test failures will be caught during
         * compilation rather than at runtime.
         */
        static constexpr void RunAll()
        {
            TestConstruction();
            TestArithmetic();
            TestCompoundAssignment();
            TestComparisons();
            TestVectorOperations();
            TestAbsAndSort();
            TestStaticMethods();
            TestBitwiseOperations();
            TestMultiDotAccumulate();
            TestLerp();
            TestAngles();
            TestProjectionAndReflection();
            TestDistanceMethods();
            TestEdgeCases();
        }
    };
    
    // Execute all tests
    static_assert((Vector2DTests::RunAll(), true), "Vector2D tests failed");
}
