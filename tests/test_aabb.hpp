#pragma once

/**
 * @file test_aabb.hpp
 * @brief Compile-time unit tests for the AABB (Axis-Aligned Bounding Box) class
 * 
 * This file contains comprehensive tests for the AABB class, covering:
 * - Basic construction and factory methods
 * - Geometric properties and calculations
 * - Transformations and spatial operations
 * - Containment and intersection tests
 * - Edge cases and special scenarios
 * 
 * All tests are performed at compile-time using static_assert.
 */

#include "../impl/aabb.hpp"

namespace SaturnMath::Tests
{
    using namespace SaturnMath::Types;

    /**
     * @brief Test suite for AABB class
     * 
     * This struct contains all test cases for the AABB class, organized
     * into logical sections. Each test method focuses on a specific
     * aspect of the AABB functionality.
     */
    struct AABBTests
    {
        // ============================================
        // Construction and Factory Methods
        // ============================================
        /**
         * @brief Tests AABB construction and factory methods
         * 
         * Verifies:
         * - Default construction creates a zero-sized box at origin
         * - Construction from center and size
         * - Construction from center and half-extents
         * - Factory method FromMinMax
         */
        static constexpr void TestConstruction()
        {
            // Default construction
            {
                constexpr AABB defaultBox;
                constexpr auto defaultBoxPos = defaultBox.GetPosition();
                static_assert(defaultBoxPos.X == 0 && defaultBoxPos.Y == 0 && 
                            defaultBoxPos.Z == 0 && defaultBox.GetHalfExtents().X == 0 && 
                            defaultBox.GetHalfExtents().Y == 0 && defaultBox.GetHalfExtents().Z == 0,
                            "Default constructor should create a zero-sized box at origin");
            }
            
            // Construction from center and uniform size
            {
                constexpr Vector3D center(0, 0, 0);
                constexpr Fxp halfSize(1);
                constexpr AABB box(center, halfSize);
                constexpr auto boxPos = box.GetPosition();
                static_assert(boxPos.X == 0 && boxPos.Y == 0 && boxPos.Z == 0 &&
                            box.GetHalfExtents().X == 1 && box.GetHalfExtents().Y == 1 && box.GetHalfExtents().Z == 1,
                            "Construction from center and uniform size should work");
            }
            
            // Construction from center and non-uniform half extents
            {
                constexpr Vector3D center(0, 0, 0);
                constexpr Vector3D halfExtents(1, 2, 3);
                constexpr AABB boxFromCenter(center, halfExtents);
                constexpr auto boxFromCenterPos = boxFromCenter.GetPosition();
                static_assert(boxFromCenterPos.X == 0 && boxFromCenterPos.Y == 0 && boxFromCenterPos.Z == 0 &&
                            boxFromCenter.GetHalfExtents().X == 1 && boxFromCenter.GetHalfExtents().Y == 2 && boxFromCenter.GetHalfExtents().Z == 3,
                            "Construction from center and non-uniform half extents should work");
            }
            
            // Construction from min/max points using factory method
            {
                constexpr Vector3D min(-1, -2, -3);
                constexpr Vector3D max(1, 2, 3);
                constexpr AABB boxFromMinMax = AABB::FromMinMax(min, max);
                constexpr auto boxFromMinMaxPos = boxFromMinMax.GetPosition();
                static_assert(boxFromMinMaxPos.X == 0 && boxFromMinMaxPos.Y == 0 && boxFromMinMaxPos.Z == 0 &&
                            boxFromMinMax.GetHalfExtents().X == 1 && boxFromMinMax.GetHalfExtents().Y == 2 && boxFromMinMax.GetHalfExtents().Z == 3,
                            "Construction from min/max points using factory method should work");
            }
            
            // Copy construction
            {
                constexpr Vector3D center(1, 2, 3);
                constexpr Fxp halfSize(4);
                constexpr AABB original(center, halfSize);
                constexpr AABB copy = original;
                
                constexpr auto originalPos = original.GetPosition();
                constexpr auto copyPos = copy.GetPosition();
                static_assert(copyPos.X == originalPos.X && copyPos.Y == originalPos.Y && copyPos.Z == originalPos.Z,
                            "Copy constructor should copy position correctly");
                static_assert(copy.GetHalfExtents().X == original.GetHalfExtents().X && 
                            copy.GetHalfExtents().Y == original.GetHalfExtents().Y && 
                            copy.GetHalfExtents().Z == original.GetHalfExtents().Z,
                            "Copy constructor should copy size correctly");
            }
        }
        
        // ============================================
        // Basic Properties Tests
        // ============================================
        
        /**
         * @brief Tests basic AABB properties
         * 
         * Verifies:
         * - GetMin/GetMax return correct extents
         * - GetHalfExtents returns correct dimensions
         * - GetVolume and GetSurfaceArea return correct values
         */
        static constexpr void TestProperties()
        {
            // Basic box properties
            {
                constexpr Vector3D min(-1, -2, -3);
                constexpr Vector3D max(1, 2, 3);
                constexpr AABB box = AABB::FromMinMax(min, max);
                
                // Verify center position
                {
                    constexpr Vector3D center = box.GetPosition();
                    static_assert(center.X == 0 && center.Y == 0 && center.Z == 0,
                                "GetPosition should return the center of the box");
                }
                
                // Verify half-extents (size)
                {
                    constexpr Vector3D halfExtents = box.GetHalfExtents();
                    static_assert(halfExtents.X == 1 && halfExtents.Y == 2 && halfExtents.Z == 3,
                                "GetHalfExtents should return the half-extents of the box");
                }
                
                // Verify minimum point calculation
                {
                    constexpr Vector3D minPoint = box.GetMin();
                    static_assert(minPoint.X == -1 && minPoint.Y == -2 && minPoint.Z == -3,
                                "GetMin should return the minimum point of the box (center - half-extents)");
                }
                
                // Verify maximum point calculation
                {
                    constexpr Vector3D maxPoint = box.GetMax();
                    static_assert(maxPoint.X == 1 && maxPoint.Y == 2 && maxPoint.Z == 3,
                                "GetMax should return the maximum point of the box (center + half-extents)");
                }
            }
            
            // Volume calculation
            {
                constexpr AABB box(Vector3D(0, 0, 0), Vector3D(1, 2, 3));
                constexpr Fxp volume = box.GetVolume();
                // Volume = (2*1) * (2*2) * (2*3) = 2 * 4 * 6 = 48
                static_assert(volume == 48, 
                            "GetVolume should return the volume of the box (2*4*6 = 48)");
            }
            
            // Surface area calculation
            {
                constexpr AABB box(Vector3D(0, 0, 0), Vector3D(1, 2, 3));
                constexpr Fxp surfaceArea = box.GetSurfaceArea();
                // Surface area = 8*(xy + yz + zx) where x,y,z are half-extents
                // For (1,2,3): 8*(1*2 + 2*3 + 3*1) = 8*(2 + 6 + 3) = 8*11 = 88
                static_assert(surfaceArea == 88, 
                            "GetSurfaceArea should return 88 for a box with half-extents (1,2,3)");
            }
        }
        
        // ============================================
        // Transformations Tests
        // ============================================
        
        /**
         * @brief Tests AABB transformation methods
         * 
         * Verifies:
         * - Translate moves the AABB correctly
         * - Expand increases the size correctly
         * - Scale changes the size correctly
         */
        static constexpr void TestTransformation()
        {
            // Expand operation
            {
                // Create a test box from (-1,-2,-3) to (1,2,3)
                constexpr Vector3D min(-1, -2, -3);
                constexpr Vector3D max(1, 2, 3);
                constexpr AABB box = AABB::FromMinMax(min, max);
                
                // Expand by 0.5 units in all directions
                constexpr Fxp margin(0.5);
                constexpr AABB expanded = box.Expand(margin);
                
                // Verify half-extents increased by margin in each direction
                // Original half-extents: (1,2,3)
                // After expand: (1.5, 2.5, 3.5)
                static_assert(expanded.GetHalfExtents().X == 1.5 && 
                             expanded.GetHalfExtents().Y == 2.5 && 
                             expanded.GetHalfExtents().Z == 3.5,
                            "Expand should increase half-extents by margin in all directions");
                
                // Verify the center remains the same
                static_assert(expanded.GetPosition().X == 0 &&
                             expanded.GetPosition().Y == 0 &&
                             expanded.GetPosition().Z == 0,
                            "Expand should not change the center position");
            }
            
            // Uniform scaling
            {
                // Create a test box from (-1,-2,-3) to (1,2,3)
                constexpr AABB box(Vector3D(0, 0, 0), Vector3D(1, 2, 3));
                
                // Scale by factor of 2
                constexpr Fxp scale(2);
                constexpr AABB scaled = box.Scale(scale);
                
                // Verify half-extents are scaled correctly
                // Original half-extents: (1,2,3)
                // After scale: (2,4,6)
                static_assert(scaled.GetHalfExtents().X == 2 && 
                             scaled.GetHalfExtents().Y == 4 && 
                             scaled.GetHalfExtents().Z == 6,
                            "Scale should multiply half-extents by the scale factor");
                
                // Verify the center remains the same
                static_assert(scaled.GetPosition().X == 0 &&
                             scaled.GetPosition().Y == 0 &&
                             scaled.GetPosition().Z == 0,
                            "Scale should not change the center position");
            }
            
            // Translation
            {
                // Create a test box at origin
                constexpr Vector3D originalCenter(0, 0, 0);
                constexpr Vector3D size(1, 1, 1);
                constexpr AABB box(originalCenter, size);
                
                // Translate by (1,2,3)
                constexpr Vector3D translation(1, 2, 3);
                constexpr Vector3D newCenter = originalCenter + translation;
                constexpr AABB translated(newCenter, size);
                
                // Verify the center is at the new position
                static_assert(translated.GetPosition().X == 1 &&
                             translated.GetPosition().Y == 2 &&
                             translated.GetPosition().Z == 3,
                            "AABB should be created at the translated position");
                
                // Verify the size remains the same
                static_assert(translated.GetHalfExtents().X == 1 &&
                             translated.GetHalfExtents().Y == 1 &&
                             translated.GetHalfExtents().Z == 1,
                            "Translation should not affect the box size");
            }
        }
        
        // ============================================
        // Closest Point Calculation Tests
        // ============================================
        
        /**
         * @brief Tests finding the closest point on AABB to a given point
         * 
         * Verifies:
         * - Points inside the AABB return the point itself
         * - Points outside return the closest point on the AABB surface
         * - Edge and corner cases are handled correctly
         */
        static constexpr void TestClosestPoint()
        {
            // Point inside the AABB
            {
                // Create a test box from (-1,-2,-3) to (1,2,3)
                constexpr AABB box = AABB::FromMinMax(Vector3D(-1, -2, -3), Vector3D(1, 2, 3));
                
                // Point at the center of the box
                constexpr Vector3D inside(0, 0, 0);
                constexpr Vector3D closest = box.GetClosestPoint(inside);
                
                // Should return the point itself
                static_assert(closest.X == 0 && closest.Y == 0 && closest.Z == 0,
                            "GetClosestPoint should return the point itself when inside the box");
            }
            
            // Point outside the AABB
            {
                // Create a test box from (-1,-2,-3) to (1,2,3)
                constexpr AABB box = AABB::FromMinMax(Vector3D(-1, -2, -3), Vector3D(1, 2, 3));
                
                // Point outside the box in the positive octant
                constexpr Vector3D outside(2, 3, 4);
                constexpr Vector3D closest = box.GetClosestPoint(outside);
                
                // Should clamp to the nearest point on the box surface (1,2,3)
                static_assert(closest.X == 1 && closest.Y == 2 && closest.Z == 3,
                            "GetClosestPoint should clamp to the nearest surface point when outside");
            }
            
            // Point on the AABB boundary
            {
                // Create a test box from (-1,-2,-3) to (1,2,3)
                constexpr AABB box = AABB::FromMinMax(Vector3D(-1, -2, -3), Vector3D(1, 2, 3));
                
                // Point exactly on the right face of the box
                constexpr Vector3D onBoundary(1, 0, 0);
                constexpr Vector3D closest = box.GetClosestPoint(onBoundary);
                
                // Should return the point itself since it's already on the boundary
                static_assert(closest.X == 1 && closest.Y == 0 && closest.Z == 0,
                            "GetClosestPoint should return the point itself when on the boundary");
            }
            
            // Point outside in the negative direction
            {
                // Create a test box from (-1,-2,-3) to (1,2,3)
                constexpr AABB box = AABB::FromMinMax(Vector3D(-1, -2, -3), Vector3D(1, 2, 3));
                
                // Point outside in the negative octant
                constexpr Vector3D outside(-2, -3, -4);
                constexpr Vector3D closest = box.GetClosestPoint(outside);
                
                // Should clamp to the nearest point on the box surface (-1,-2,-3)
                static_assert(closest.X == -1 && closest.Y == -2 && closest.Z == -3,
                            "GetClosestPoint should handle negative directions correctly");
            }
            
            // Point aligned with an edge
            {
                // Create a test box from (-1,-2,-3) to (1,2,3)
                constexpr AABB box = AABB::FromMinMax(Vector3D(-1, -2, -3), Vector3D(1, 2, 3));
                
                // Point outside aligned with the top-right edge (x=2, y=3, z=0)
                constexpr Vector3D edgePoint(2, 3, 0);
                constexpr Vector3D closest = box.GetClosestPoint(edgePoint);
                
                // Should clamp to the nearest point on the edge (1,2,0)
                static_assert(closest.X == 1 && closest.Y == 2 && closest.Z == 0,
                            "GetClosestPoint should handle edge cases correctly");
            }
        }
        
        // ============================================
        // Merge Operations Tests
        // ============================================
        
        /**
         * @brief Tests AABB merge operation
         * 
         * Verifies:
         * - Merging with self returns the same AABB
         * - Merging with a contained AABB returns the outer AABB
         * - Merging with a separate AABB returns the minimal enclosing AABB
         */
        static constexpr void TestMerge()
        {
            // Merge two separate boxes
            {
                constexpr AABB box1(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                constexpr AABB box2(Vector3D(2, 2, 2), Vector3D(1, 1, 1));
                
                // Merge the two boxes
                constexpr AABB merged = box1.Merge(box2);
                
                // Verify the merged box contains both original boxes
                static_assert(merged.GetMin().X == -1 && merged.GetMin().Y == -1 && merged.GetMin().Z == -1,
                            "Merge should contain the minimum bounds of both boxes");
                static_assert(merged.GetMax().X == 3 && merged.GetMax().Y == 3 && merged.GetMax().Z == 3,
                            "Merge should contain the maximum bounds of both boxes");
            }
            
            // Merge a box with itself
            {
                constexpr AABB box1(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                
                // Merge box with itself
                constexpr AABB selfMerge = box1.Merge(box1);
                
                // Verify the result is equivalent to the original box
                static_assert(selfMerge.GetMin().X == -1 && 
                            selfMerge.GetMin().Y == -1 && 
                            selfMerge.GetMin().Z == -1,
                            "Merging with itself should return an equivalent box");
                static_assert(selfMerge.GetMax().X == 1 && 
                            selfMerge.GetMax().Y == 1 && 
                            selfMerge.GetMax().Z == 1,
                            "Merging with itself should return an equivalent box");
            }
            
            // Merge with contained box
            {
                constexpr AABB outerBox(Vector3D(0, 0, 0), Vector3D(2, 2, 2));
                constexpr AABB innerBox(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                
                // Merge outer box with inner box
                constexpr AABB merged = outerBox.Merge(innerBox);
                
                // Result should be the same as the outer box
                static_assert(merged.GetMin().X == -2 && 
                            merged.GetMin().Y == -2 && 
                            merged.GetMin().Z == -2,
                            "Merging with a contained box should return the outer box");
                static_assert(merged.GetMax().X == 2 && 
                            merged.GetMax().Y == 2 && 
                            merged.GetMax().Z == 2,
                            "Merging with a contained box should return the outer box");
            }
        }
        
        // ============================================
        // GetVertices Tests
        // ============================================
        
        /**
         * @brief Tests AABB vertex retrieval
         * 
         * Verifies:
         * - Correct vertices are returned for a simple AABB
         * - Vertices are returned in the correct order
         */
        static constexpr void TestGetVertices()
        {
            // Basic vertex retrieval
            {
                // Create a box with center at (1,2,3) and half-extents (1,2,3)
                // This gives us a box from (0,0,0) to (2,4,6)
                constexpr AABB box(Vector3D(1, 2, 3), Vector3D(1, 2, 3));
                constexpr auto vertices = box.GetVertices();
                
                // Verify all 8 vertices are in the correct order
                // Vertex 0: (minX, minY, minZ) = (0, 0, 0)
                static_assert(vertices[0].X == 0 && vertices[0].Y == 0 && vertices[0].Z == 0, 
                            "Vertex 0 (min corner) is incorrect");
                
                // Vertex 1: (maxX, minY, minZ) = (2, 0, 0)
                static_assert(vertices[1].X == 2 && vertices[1].Y == 0 && vertices[1].Z == 0, 
                            "Vertex 1 (min corner + x) is incorrect");
                
                // Vertex 2: (maxX, maxY, minZ) = (2, 4, 0)
                static_assert(vertices[2].X == 2 && vertices[2].Y == 4 && vertices[2].Z == 0, 
                            "Vertex 2 (min corner + x + y) is incorrect");
                
                // Vertex 3: (minX, maxY, minZ) = (0, 4, 0)
                static_assert(vertices[3].X == 0 && vertices[3].Y == 4 && vertices[3].Z == 0, 
                            "Vertex 3 (min corner + y) is incorrect");
                
                // Vertex 4: (minX, minY, maxZ) = (0, 0, 6)
                static_assert(vertices[4].X == 0 && vertices[4].Y == 0 && vertices[4].Z == 6, 
                            "Vertex 4 (min corner + z) is incorrect");
                
                // Vertex 5: (maxX, minY, maxZ) = (2, 0, 6)
                static_assert(vertices[5].X == 2 && vertices[5].Y == 0 && vertices[5].Z == 6, 
                            "Vertex 5 (min corner + x + z) is incorrect");
                
                // Vertex 6: (maxX, maxY, maxZ) = (2, 4, 6)
                static_assert(vertices[6].X == 2 && vertices[6].Y == 4 && vertices[6].Z == 6, 
                            "Vertex 6 (max corner) is incorrect");
                
                // Vertex 7: (minX, maxY, maxZ) = (0, 4, 6)
                static_assert(vertices[7].X == 0 && vertices[7].Y == 4 && vertices[7].Z == 6, 
                            "Vertex 7 (min corner + y + z) is incorrect");
            }
            
            // Zero-sized box
            {
                constexpr AABB zeroBox(Vector3D(0, 0, 0), Vector3D(0, 0, 0));
                constexpr auto vertices = zeroBox.GetVertices();
                
                // All vertices should be at the origin for a zero-sized box
                static_assert(vertices[0].X == 0 && vertices[0].Y == 0 && vertices[0].Z == 0, "Vertex 0 should be at origin");
                static_assert(vertices[1].X == 0 && vertices[1].Y == 0 && vertices[1].Z == 0, "Vertex 1 should be at origin");
                static_assert(vertices[2].X == 0 && vertices[2].Y == 0 && vertices[2].Z == 0, "Vertex 2 should be at origin");
                static_assert(vertices[3].X == 0 && vertices[3].Y == 0 && vertices[3].Z == 0, "Vertex 3 should be at origin");
                static_assert(vertices[4].X == 0 && vertices[4].Y == 0 && vertices[4].Z == 0, "Vertex 4 should be at origin");
                static_assert(vertices[5].X == 0 && vertices[5].Y == 0 && vertices[5].Z == 0, "Vertex 5 should be at origin");
                static_assert(vertices[6].X == 0 && vertices[6].Y == 0 && vertices[6].Z == 0, "Vertex 6 should be at origin");
                static_assert(vertices[7].X == 0 && vertices[7].Y == 0 && vertices[7].Z == 0, "Vertex 7 should be at origin");
            }
            
            // Unit box at origin
            {
                constexpr AABB unitBox(Vector3D(0.5, 0.5, 0.5), Vector3D(0.5, 0.5, 0.5));
                constexpr auto vertices = unitBox.GetVertices();
                
                // Expected vertices for a unit cube from (0,0,0) to (1,1,1)
                constexpr Vector3D expectedVertices[8] = {
                    {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                    {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}
                };
                
                // Verify each vertex
                static_assert(vertices[0].X == 0 && vertices[0].Y == 0 && vertices[0].Z == 0, "Vertex 0 of unit box incorrect");
                static_assert(vertices[1].X == 1 && vertices[1].Y == 0 && vertices[1].Z == 0, "Vertex 1 of unit box incorrect");
                static_assert(vertices[2].X == 1 && vertices[2].Y == 1 && vertices[2].Z == 0, "Vertex 2 of unit box incorrect");
                static_assert(vertices[3].X == 0 && vertices[3].Y == 1 && vertices[3].Z == 0, "Vertex 3 of unit box incorrect");
                static_assert(vertices[4].X == 0 && vertices[4].Y == 0 && vertices[4].Z == 1, "Vertex 4 of unit box incorrect");
                static_assert(vertices[5].X == 1 && vertices[5].Y == 0 && vertices[5].Z == 1, "Vertex 5 of unit box incorrect");
                static_assert(vertices[6].X == 1 && vertices[6].Y == 1 && vertices[6].Z == 1, "Vertex 6 of unit box incorrect");
                static_assert(vertices[7].X == 0 && vertices[7].Y == 1 && vertices[7].Z == 1, "Vertex 7 of unit box incorrect");
            }
        }
        
        // ============================================
        // Point Encapsulation Tests
        // ============================================
        
        /**
         * @brief Tests the Encapsulate method with points
         * 
         * Verifies that:
         * 1. Encapsulating a point inside the AABB returns the same AABB
         * 2. Encapsulating a point outside the AABB returns a larger AABB
         * 3. Encapsulating multiple points works correctly
         * 4. Edge cases with zero-size AABBs and points on the boundary
         */
        static constexpr void TestEncapsulatePoint()
        {
            // Encapsulate point inside AABB
            {
                constexpr AABB box(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                constexpr Vector3D insidePoint(0.5, 0.5, 0.5);
                constexpr AABB result = box.Encapsulate(insidePoint);
                
                // Should be the same as original box since point is inside
                static_assert(result.GetMin() == box.GetMin(), "Encapsulate with inside point changed min");
                static_assert(result.GetMax() == box.GetMax(), "Encapsulate with inside point changed max");
            }
            
            // Encapsulate point outside AABB
            {
                constexpr AABB box(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                constexpr Vector3D outsidePoint(2, 2, 2);
                constexpr AABB result = box.Encapsulate(outsidePoint);
                
                // Should expand to include the new point
                static_assert(result.GetMin() == Vector3D(-1, -1, -1), "Encapsulate with outside point min incorrect");
                static_assert(result.GetMax() == Vector3D(2, 2, 2), "Encapsulate with outside point max incorrect");
            }
            
            // Multiple encapsulations
            {
                constexpr AABB box(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                constexpr AABB result = box
                    .Encapsulate(Vector3D(-2, 0, 0))  // Expand left
                    .Encapsulate(Vector3D(0, 3, 0))   // Expand up
                    .Encapsulate(Vector3D(0, 0, -1));  // Expand back
                
                static_assert(result.GetMin() == Vector3D(-2, -1, -1), "Multiple encapsulations min incorrect");
                static_assert(result.GetMax() == Vector3D(1, 3, 1), "Multiple encapsulations max incorrect");
            }
            
            // Edge cases
            {
                // Zero-size AABB
                constexpr AABB zeroBox(Vector3D(0, 0, 0), Vector3D(0, 0, 0));
                constexpr AABB result = zeroBox.Encapsulate(Vector3D(1, 2, 3));
                
                // Should create a box from (0,0,0) to (1,2,3)
                static_assert(result.GetMin() == Vector3D(0, 0, 0), "Zero-size AABB encapsulate min incorrect");
                static_assert(result.GetMax() == Vector3D(1, 2, 3), "Zero-size AABB encapsulate max incorrect");
                
                // Point exactly on the boundary
                constexpr AABB box2(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                constexpr AABB result2 = box2.Encapsulate(Vector3D(1, 1, 1));  // On max corner
                static_assert(result2.GetMin() == box2.GetMin() && result2.GetMax() == box2.GetMax(), 
                            "Point on boundary should not change AABB");
            }
        }
        
        // ============================================
        // AABB Encapsulation Tests
        // ============================================
        
        /**
         * @brief Tests the Encapsulate method with another AABB
         * 
         * Verifies that:
         * 1. Encapsulating an AABB already contained returns the original AABB
         * 2. Encapsulating a larger AABB returns the larger AABB
         * 3. Encapsulating overlapping AABBs returns the correct combined AABB
         * 4. Encapsulating non-overlapping AABBs returns the minimal containing AABB
         * 5. Edge cases with zero-size AABBs
         */
        static constexpr void TestEncapsulateAABB()
        {
            // Encapsulate contained AABB
            {
                constexpr AABB outer(Vector3D(0, 0, 0), Vector3D(4, 4, 4));
                constexpr AABB inner(Vector3D(1, 1, 1), Vector3D(1, 1, 1));
                constexpr AABB result = outer.Encapsulate(inner);
                
                // Should be the same as outer AABB since inner is contained
                static_assert(result.GetMin() == outer.GetMin(), "Encapsulate with contained AABB changed min");
                static_assert(result.GetMax() == outer.GetMax(), "Encapsulate with contained AABB changed max");
            }
            
            // Encapsulate larger AABB
            {
                constexpr AABB smaller(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                constexpr AABB larger(Vector3D(0, 0, 0), Vector3D(2, 2, 2));
                constexpr AABB result = smaller.Encapsulate(larger);
                
                // Should be the same as larger AABB
                static_assert(result.GetMin() == larger.GetMin(), "Encapsulate with larger AABB min incorrect");
                static_assert(result.GetMax() == larger.GetMax(), "Encapsulate with larger AABB max incorrect");
            }
            
            // Encapsulate overlapping AABBs
            {
                // Box1: center (0,0,0), half-extent (2,2,2) -> from (-2,-2,-2) to (2,2,2)
                // Box2: center (1,1,1), half-extent (2,2,2) -> from (-1,-1,-1) to (3,3,3)
                constexpr AABB box1(Vector3D(0, 0, 0), Vector3D(2, 2, 2));
                constexpr AABB box2(Vector3D(1, 1, 1), Vector3D(2, 2, 2));
                constexpr AABB result = box1.Encapsulate(box2);
                
                // The minimal AABB containing both boxes should be from (-2,-2,-2) to (3,3,3)
                static_assert(result.GetMin() == Vector3D(-2, -2, -2), "Overlapping AABBs min incorrect");
                static_assert(result.GetMax() == Vector3D(3, 3, 3), "Overlapping AABBs max incorrect");
            }
            
            // Encapsulate non-overlapping AABBs
            {
                // Box1: center (0,0,0), half-extent (1,1,1) -> from (-1,-1,-1) to (1,1,1)
                // Box2: center (3,3,3), half-extent (1,1,1) -> from (2,2,2) to (4,4,4)
                constexpr AABB box1(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                constexpr AABB box2(Vector3D(3, 3, 3), Vector3D(1, 1, 1));
                constexpr AABB result = box1.Encapsulate(box2);
                
                // The minimal AABB containing both boxes should be from (-1,-1,-1) to (4,4,4)
                static_assert(result.GetMin() == Vector3D(-1, -1, -1), "Non-overlapping AABBs min incorrect");
                static_assert(result.GetMax() == Vector3D(4, 4, 4), "Non-overlapping AABBs max incorrect");
            }
            
            // Edge cases with zero-size AABBs
            {
                // Zero-size AABB with regular AABB
                constexpr AABB regularBox(Vector3D(1, 2, 3), Vector3D(1, 1, 1));  // Box from (0,1,2) to (2,3,4)
                constexpr Vector3D pointInside(1.5, 2.5, 3.5);  // Point inside the box
                constexpr AABB zeroBox(pointInside, Vector3D(0, 0, 0));  // Zero-size AABB at (1.5, 2.5, 3.5)
                
                constexpr AABB result1 = zeroBox.Encapsulate(regularBox);
                constexpr AABB result2 = regularBox.Encapsulate(zeroBox);
                
                // Both results should be the same as regularBox since it already contains zeroBox
                static_assert(result1.GetMin() == regularBox.GetMin() && 
                              result1.GetMax() == regularBox.GetMax(),
                            "Zero-size AABB encapsulation 1 failed");
                static_assert(result2.GetMin() == regularBox.GetMin() && 
                              result2.GetMax() == regularBox.GetMax(),
                            "Zero-size AABB encapsulation 2 failed");
                
                // Two zero-size AABBs (points)
                constexpr AABB point1(Vector3D(1, 2, 3), Vector3D(0, 0, 0));  // Point at (1,2,3)
                constexpr AABB point2(Vector3D(0, 0, 0), Vector3D(0, 0, 0));  // Point at (0,0,0)
                constexpr AABB result3 = point1.Encapsulate(point2);
                
                // Should create a box from (0,0,0) to (1,2,3)
                static_assert(result3.GetMin() == Vector3D(0, 0, 0), 
                            "Two zero-size AABBs min incorrect");
                static_assert(result3.GetMax() == Vector3D(1, 2, 3), 
                            "Two zero-size AABBs max incorrect");
            }
        }
        
        // ============================================
        // 13. Edge Cases
        // ============================================
        
        /**
         * @brief Tests edge cases and special AABBs
         * 
         * Verifies:
         * - Zero-size AABBs behave correctly
         * - Non-centered AABBs work as expected
         * - Degenerate AABBs (flattened in one or more dimensions)
         * - Edge cases for all operations
         * - Boundary conditions
         */
        static constexpr void TestEdgeCases()
        {
            // ============================================
            // Zero-size AABB tests
            // ============================================
            {
                // Zero-size AABB at origin
                {
                    constexpr AABB zeroBoxAtOrigin(Vector3D(0, 0, 0), Vector3D(0, 0, 0));
                    static_assert(zeroBoxAtOrigin.GetVolume() == 0, 
                                "Zero-size box at origin should have zero volume");
                    static_assert(zeroBoxAtOrigin.GetSurfaceArea() == 0, 
                                "Zero-size box at origin should have zero surface area");
                    static_assert(zeroBoxAtOrigin.GetMin() == Vector3D(0, 0, 0), 
                                "Zero-size box at origin should have min at origin");
                    static_assert(zeroBoxAtOrigin.GetMax() == Vector3D(0, 0, 0), 
                                "Zero-size box at origin should have max at origin");
                }
                
                // Zero-size AABB at offset position
                {
                    constexpr AABB zeroBoxOffset(Vector3D(1, 2, 3), Vector3D(0, 0, 0));
                    static_assert(zeroBoxOffset.GetVolume() == 0, 
                                "Zero-size offset box should have zero volume");
                    static_assert(zeroBoxOffset.GetSurfaceArea() == 0, 
                                "Zero-size offset box should have zero surface area");
                    static_assert(zeroBoxOffset.GetMin() == Vector3D(1, 2, 3), 
                                "Zero-size offset box min should be at position");
                    static_assert(zeroBoxOffset.GetMax() == Vector3D(1, 2, 3), 
                                "Zero-size offset box max should be at position");
                }
            }
            
            // ============================================
            // Non-centered AABB tests
            // ============================================
            {
                // Simple offset box
                {
                    constexpr AABB offsetBox(Vector3D(1, 2, 3), Vector3D(1, 1, 1));
                    static_assert(offsetBox.GetMin().X == 0, "Offset box min X should be 0");
                    static_assert(offsetBox.GetMin().Y == 1, "Offset box min Y should be 1");
                    static_assert(offsetBox.GetMin().Z == 2, "Offset box min Z should be 2");
                    static_assert(offsetBox.GetMax().X == 2, "Offset box max X should be 2");
                    static_assert(offsetBox.GetMax().Y == 3, "Offset box max Y should be 3");
                    static_assert(offsetBox.GetMax().Z == 4, "Offset box max Z should be 4");
                }
                
                // Large offset box
                {
                    constexpr AABB largeOffsetBox(Vector3D(100, 200, 300), Vector3D(50, 60, 70));
                    static_assert(largeOffsetBox.GetMin().X == 50, "Large offset box min X incorrect");
                    static_assert(largeOffsetBox.GetMax().X == 150, "Large offset box max X incorrect");
                }
            }
            
            // ============================================
            // Degenerate AABB tests (flattened dimensions)
            // ============================================
            {
                // Flat AABB (2D)
                {
                    constexpr AABB flatBox(Vector3D(0, 0, 0), Vector3D(1, 1, 0));
                    static_assert(flatBox.GetVolume() == 0, "Flat box should have zero volume");
                    // For a flat box (2x2x0), the surface area is 8 (2*2*2 + 2*0*2 + 2*2*0 = 8)
                    static_assert(flatBox.GetSurfaceArea() == 8, 
                                "Flat box should have surface area of 8 (2*2*2 + 2*0*2 + 2*2*0 = 8)");
                }
                
                // Line AABB (1D)
                {
                    constexpr AABB lineBox(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
                    static_assert(lineBox.GetVolume() == 0, "Line box should have zero volume");
                    static_assert(lineBox.GetSurfaceArea() == 0, 
                                "Line box should have zero surface area");
                }
            }
            
            // ============================================
            // Boundary condition tests
            // ============================================
            {
                // AABB with large extents
                {
                    // Using values that won't cause overflow in calculations
                    constexpr int32_t largeVal = 1000;  // Large but safe value
                    constexpr AABB largeBox(Vector3D(0, 0, 0), Vector3D(largeVal, largeVal, largeVal));
                    
                    // Verify the box has the expected extents
                    static_assert(largeBox.GetMin().X == -largeVal, "Large box min X incorrect");
                    static_assert(largeBox.GetMax().X == largeVal, "Large box max X incorrect");
                }
                
                // AABB with very small extents using Fxp::Epsilon()
                {
                    constexpr Fxp epsilon = Fxp::Epsilon();
                    constexpr AABB tinyBox(Vector3D(0, 0, 0), Vector3D(epsilon, epsilon, epsilon));
                    
                    // Verify the box has the expected extents
                    constexpr Fxp negEpsilon = Fxp(0) - epsilon;
                    static_assert(tinyBox.GetMin().X == negEpsilon, "Tiny box min X incorrect");
                    static_assert(tinyBox.GetMax().X == epsilon, "Tiny box max X incorrect");
                }
            }
        }
        
        // ============================================
        // Test Runner
        // ============================================
        
        /**
         * @brief Runs all AABB test cases
         * @return true if all tests pass (compile-time constant)
         * 
         * @note This method is called by the static_assert at file scope
         * to verify all tests pass at compile time.
         */
        static constexpr bool RunAll()
        {
            // Execute each 
            TestConstruction();      // 1. Verify construction and factory methods
            TestProperties();        // 2. Test geometric properties
            TestTransformation();    // 3. Test transformations
            TestClosestPoint();      // 4. Test closest point calculations
            TestMerge();             // 5. Test merge operations
            TestGetVertices();       // 6. Test vertex generation
            TestEncapsulatePoint();  // 7. Test point encapsulation
            TestEncapsulateAABB();   // 8. Test AABB encapsulation
            TestEdgeCases();         // 9. Test edge cases and special scenarios
            
            return true;
        }
    };
    
    // Execute all tests at compile time
    static_assert((AABBTests::RunAll(), true), "AABB tests failed");
}
