#pragma once

/**
 * @file test_collision.hpp
 * @brief Compile-time unit tests for collision detection system
 * 
 * This file contains comprehensive tests for the collision detection system,
 * covering intersections and containment tests between different shape types.
 * 
 * All tests are performed at compile-time using static_assert.
 */

#include "../impl/collision.hpp"

namespace SaturnMath::Tests
{
    using namespace SaturnMath::Types;
    using namespace SaturnMath::Collision;

    /**
     * @brief Test suite for collision detection system
     * 
     * This struct contains all test cases for the collision detection system,
     * organized by shape pairs and test categories.
     */
    struct CollisionTests
    {
        // ============================================
        // Classification Tests
        // ============================================
        /**
         * @brief Tests the classification of geometric primitives against planes
         *
         * Verifies:
         * - Point-Plane classification (Front, Back, Intersects)
         * - Sphere-Plane classification
         * - AABB-Plane classification
         * - Edge cases and special configurations
         */
        static constexpr void TestClassification()
        {
            // Point-Plane Classification
            {
                constexpr Plane plane(Vector3D::UnitY(), 1.0f);
                constexpr Vector3D pointAbove(0.0f, 2.0f, 0.0f);
                constexpr Vector3D pointBelow(0.0f, 0.0f, 0.0f);
                constexpr Vector3D pointOn(0.0f, 1.0f, 0.0f);

                static_assert(Classify(pointAbove, plane) == PlaneRelationship::Front, "Point should be in front of plane");
                static_assert(Classify(pointBelow, plane) == PlaneRelationship::Back, "Point should be behind plane");
                static_assert(Classify(pointOn, plane) == PlaneRelationship::Intersects, "Point should intersect plane");
            }

            // Sphere-Plane Classification
            {
                constexpr Plane plane(Vector3D::UnitY(), 0.0f);
                constexpr Sphere aboveSphere(Vector3D(0.0f, 2.0f, 0.0f), 1.0f);
                constexpr Sphere belowSphere(Vector3D(0.0f, -2.0f, 0.0f), 1.0f);
                constexpr Sphere intersectSphere(Vector3D(0.0f, 0.5f, 0.0f), 1.0f);

                static_assert(Classify(aboveSphere, plane) == PlaneRelationship::Front, "Sphere should be in front of plane");
                static_assert(Classify(belowSphere, plane) == PlaneRelationship::Back, "Sphere should be behind plane");
                static_assert(Classify(intersectSphere, plane) == PlaneRelationship::Intersects, "Sphere should intersect plane");
            }

            // AABB-Plane Classification
            {
                constexpr Plane plane(Vector3D::UnitY(), 0.0f);
                // AABB completely above plane
                constexpr AABB aboveAABB(Vector3D(0.0f, 1.0f, 0.0f), Vector3D(0.5f, 0.5f, 0.5f));
                static_assert(Classify(aboveAABB, plane) == PlaneRelationship::Front, "AABB should be in front of plane");
                
                // AABB completely below plane
                constexpr AABB belowAABB(Vector3D(0.0f, -2.0f, 0.0f), Vector3D(0.5f, 0.5f, 0.5f));
                static_assert(Classify(belowAABB, plane) == PlaneRelationship::Back, "AABB should be behind plane");
                
                // Edge case: AABB exactly on plane (should be intersecting due to epsilon)
                // Using a small Y-extent to simulate being exactly on the plane
                constexpr AABB onPlaneAABB(Vector3D(0.0f, 0.0f, 0.0f), Vector3D(1.0f, 0.1f, 1.0f));
                static_assert(Classify(onPlaneAABB, plane) == PlaneRelationship::Intersects, "AABB on plane should intersect");
            }
        }

        // ============================================
        // AABB-AABB Tests
        // ============================================
        /**
         * @brief Tests intersection between two Axis-Aligned Bounding Boxes
         *
         * Verifies:
         * - Simple intersection cases
         * - Non-intersection cases
         * - Edge case: touching edges
         * - Intersection commutativity
         */
        static constexpr void TestAABBvsAABB()
        {
            // Simple intersection
            {
                constexpr AABB a(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                constexpr AABB b(Vector3D(0.5f, 0.5f, 0.5f), Vector3D(1, 1, 1));
                static_assert(Intersects(a, b), "AABBs should intersect");
                static_assert(Intersects(b, a), "Intersection should be commutative");
            }

            // No intersection
            {
                constexpr AABB a(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                constexpr AABB b(Vector3D(2.1f, 2.1f, 2.1f), Vector3D(1, 1, 1));
                static_assert(!Intersects(a, b), "AABBs should not intersect");
            }

            // Touching edges
            {
                constexpr AABB a(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                constexpr AABB b(Vector3D(1, 0, 0), Vector3D(1, 1, 1));
                static_assert(Intersects(a, b), "Touching AABBs should intersect");
            }
        }

        // ============================================
        // Sphere-Sphere Tests
        // ============================================
        /**
         * @brief Tests intersection between two spheres
         *
         * Verifies:
         * - Simple intersection cases
         * - Non-intersection cases
         * - Edge case: touching spheres
         * - Intersection commutativity
         */
        static constexpr void TestSphereVsSphere()
        {
            // Simple intersection
            {
                constexpr Sphere a(Vector3D(0, 0, 0), 1.0f);
                constexpr Sphere b(Vector3D(1, 0, 0), 1.0f);
                static_assert(Intersects(a, b), "Spheres should intersect");
                static_assert(Intersects(b, a), "Intersection should be commutative");
            }

            // No intersection
            {
                constexpr Sphere a(Vector3D(0, 0, 0), 1.0f);
                constexpr Sphere b(Vector3D(3, 0, 0), 1.0f);
                static_assert(!Intersects(a, b), "Spheres should not intersect");
            }

            // Touching spheres
            {
                constexpr Sphere a(Vector3D(0, 0, 0), 1.0f);
                constexpr Sphere b(Vector3D(2, 0, 0), 1.0f);
                static_assert(Intersects(a, b), "Touching spheres should intersect");
            }
        }

        // ============================================
        // AABB-Sphere Tests
        // ============================================
        /**
         * @brief Tests intersection between an AABB and a Sphere
         *
         * Verifies:
         * - Sphere inside AABB
         * - Sphere outside AABB
         * - Sphere intersecting AABB face
         * - Intersection commutativity
         */
        static constexpr void TestAABBvsSphere()
        {
            // Sphere inside AABB
            {
                constexpr AABB aabb(Vector3D(0, 0, 0), Vector3D(2, 2, 2));
                constexpr Sphere sphere(Vector3D(1, 1, 1), 0.5f);
                static_assert(Intersects(aabb, sphere), "Sphere inside AABB should intersect");
                static_assert(Intersects(sphere, aabb), "Intersection should be commutative");
            }

            // Sphere outside AABB
            {
                constexpr AABB aabb(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                constexpr Sphere sphere(Vector3D(3, 3, 3), 1.0f);
                static_assert(!Intersects(aabb, sphere), "Sphere outside AABB should not intersect");
            }

            // Sphere intersecting AABB face
            {
                constexpr AABB aabb(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                constexpr Sphere sphere(Vector3D(1.5f, 0.5f, 0.5f), 0.6f);
                static_assert(Intersects(aabb, sphere), "Sphere intersecting AABB face should intersect");
            }
        }

        // ============================================
        // Plane Intersection Tests
        // ============================================
        /**
         * @brief Tests intersection with Planes against various primitives
         *
         * Verifies:
         * - Point-Plane intersection
         * - AABB-Plane intersection
         * - Sphere-Plane intersection
         * - Intersection commutativity
         */
        static constexpr void TestPlaneIntersections()
        {
            // Point on plane
            {
                constexpr Plane plane(Vector3D(0, 1, 0), 2.0f);  // y = 2.0
                constexpr Vector3D point(1.0f, 2.0f, 3.0f);
                static_assert(Intersects(plane, point), "Point on plane should intersect");
                static_assert(Intersects(point, plane), "Intersection should be commutative");
            }

            // Point above plane
            {
                constexpr Plane plane(Vector3D(0, 1, 0), 2.0f);  // y = 2.0
                constexpr Vector3D point(1.0f, 3.0f, 3.0f);
                static_assert(!Intersects(plane, point), "Point above plane should not intersect");
            }

            // Point below plane
            {
                constexpr Plane plane(Vector3D(0, 1, 0), 2.0f);  // y = 2.0
                constexpr Vector3D point(1.0f, 1.0f, 3.0f);
                static_assert(!Intersects(plane, point), "Point below plane should not intersect");
            }

            // Point on diagonal plane
            {
                constexpr Plane plane(Vector3D(1, 1, 1).Normalized(), 0.0f);  // x + y + z = 0
                constexpr Vector3D point(1.0f, -1.0f, 0.0f);
                static_assert(Intersects(plane, point), "Point on diagonal plane should intersect");
            }
        }

        // ============================================
        // Sphere-Plane Tests
        // ============================================
        /**
         * @brief Tests intersection between Spheres and Planes
         *
         * Verifies:
         * - Sphere partially intersecting plane
         * - Sphere not intersecting plane
         * - Sphere tangent to plane (touching at one point)
         * - Intersection commutativity
         */
        static constexpr void TestSphereVsPlane()
        {
            // Sphere intersecting plane
            {
                constexpr Sphere sphere(Vector3D(1, 0, 0), 1.0f);
                constexpr Plane plane(Vector3D(1, 0, 0), 1.5f);  // x = 1.5
                static_assert(Intersects(sphere, plane), "Sphere should intersect plane");
                static_assert(Intersects(plane, sphere), "Intersection should be commutative");
            }

            // Sphere not intersecting plane
            {
                constexpr Sphere sphere(Vector3D(0, 0, 0), 1.0f);
                constexpr Plane plane(Vector3D(0, 1, 0), 2.0f);  // y = 2.0
                static_assert(!Intersects(sphere, plane), "Sphere should not intersect plane");
            }

            // Sphere tangent to plane
            {
                constexpr Sphere sphere(Vector3D(0, 1, 0), 1.0f);
                constexpr Plane plane(Vector3D(0, 1, 0), 0.0f);  // y = 0.0 (tangent at y=0)
                static_assert(Intersects(sphere, plane), "Sphere tangent to plane should intersect");
            }
        }

        // ============================================
        // Containment Tests
        // ============================================
        /**
         * @brief Tests containment relationships between different primitives
         *
         * Verifies:
         * - AABB containing another AABB
         * - Sphere containing AABB
         * - AABB containing Sphere
         * - Sphere containing Point
         * - AABB containing Point
         * - Edge cases for containment tests
         */
        static constexpr void TestContainment()
        {
            // AABB contains AABB
            {
                constexpr AABB container(Vector3D(0, 0, 0), Vector3D(2, 2, 2));
                constexpr AABB contained(Vector3D(0.5f, 0.5f, 0.5f), Vector3D(0.5f, 0.5f, 0.5f));
                static_assert(Contains(container, contained), "Larger AABB should contain smaller AABB");
                static_assert(!Contains(contained, container), "Smaller AABB should not contain larger AABB");
            }

            // Sphere contains AABB
            {
                constexpr Sphere sphere(Vector3D(0, 0, 0), 2.0f);
                constexpr AABB aabb(Vector3D(0.5f, 0.5f, 0.5f), Vector3D(0.5f, 0.5f, 0.5f));
                static_assert(Contains(sphere, aabb), "Sphere should contain AABB");
            }

            // AABB contains Sphere
            {
                constexpr AABB aabb(Vector3D(0, 0, 0), Vector3D(2, 2, 2));
                constexpr Sphere sphere(Vector3D(0.5f, 0.5f, 0.5f), 0.5f);
                static_assert(Contains(aabb, sphere), "AABB should contain sphere");
            }

            // Sphere contains point
            {
                constexpr Sphere sphere(Vector3D(0, 0, 0), 2.0f);
                constexpr Vector3D point(1.0f, 0.5f, 0.5f);
                static_assert(Contains(sphere, point), "Sphere should contain point");
            }

            // AABB contains point
            {
                constexpr AABB aabb(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                constexpr Vector3D point(0.5f, 0.5f, 0.5f);
                static_assert(Contains(aabb, point), "AABB should contain point");
            }
        }

        // ============================================
        // Edge Cases
        // ============================================
        /**
         * @brief Tests special edge cases for collision detection
         *
         * Verifies:
         * - Zero-radius sphere intersection and containment
         * - Zero-size AABB intersection and containment
         * - Degenerate primitive behavior
         */
        static constexpr void TestEdgeCases()
        {
            // Zero-radius sphere
            {
                constexpr Sphere zeroSphere(Vector3D(0, 0, 0), 0.0f);
                constexpr AABB aabb(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
                constexpr Plane plane(Vector3D(1, 0, 0), 0.0f);
                
                // Should intersect if the center is on the plane
                static_assert(Intersects(zeroSphere, plane), "Zero-radius sphere on plane should intersect");
                
                // Should be contained if the center is inside
                static_assert(Contains(aabb, zeroSphere), "AABB should contain zero-radius sphere at center");
            }

            // Zero-size AABB
            {
                constexpr AABB zeroAABB(Vector3D(0, 0, 0), Vector3D(0, 0, 0));
                constexpr Sphere sphere(Vector3D(0, 0, 0), 1.0f);
                constexpr Plane plane(Vector3D(1, 0, 0), 0.0f);
                
                // Zero-size AABB at plane should intersect
                static_assert(Intersects(zeroAABB, plane), "Zero-size AABB on plane should intersect");
                
                // Sphere should contain zero-size AABB at center
                static_assert(Contains(sphere, zeroAABB), "Sphere should contain zero-size AABB at center");
            }
        }

        // ============================================
        // 9. Point-AABB Tests
        // ============================================
        /**
         * @brief Tests intersection between a point and an AABB
         *
         * Verifies:
         * - Point inside AABB
         * - Point on AABB face
         * - Point on AABB edge
         * - Point on AABB corner
         * - Point outside AABB
         * - Intersection commutativity
         */
        static constexpr void TestPointVsAABB()
        {
            constexpr AABB box(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
            
            // Point inside AABB
            {
                constexpr Vector3D point(0.5f, 0.5f, 0.5f);
                static_assert(Intersects(point, box), "Point inside AABB should intersect");
                static_assert(Intersects(box, point), "Intersection should be commutative");
            }
            
            // Point on AABB face
            {
                constexpr Vector3D point(0.0f, 0.5f, 0.5f);
                static_assert(Intersects(point, box), "Point on AABB face should intersect");
            }
            
            // Point on AABB edge
            {
                constexpr Vector3D point(0.0f, 0.0f, 0.5f);
                static_assert(Intersects(point, box), "Point on AABB edge should intersect");
            }
            
            // Point on AABB corner
            {
                constexpr Vector3D point(1.0f, 1.0f, 1.0f);
                static_assert(Intersects(point, box), "Point on AABB corner should intersect");
            }
            
            // Point outside AABB
            {
                constexpr Vector3D point(2.0f, 2.0f, 2.0f);
                static_assert(!Intersects(point, box), "Point outside AABB should not intersect");
            }
        }

        // ============================================
        // Point-Sphere Tests
        // ============================================
        /**
         * @brief Tests intersection between a point and a sphere
         *
         * Verifies:
         * - Point at sphere center
         * - Point inside sphere
         * - Point on sphere surface
         * - Point outside sphere
         * - Intersection commutativity
         */
        static constexpr void TestPointVsSphere()
        {
            constexpr Sphere sphere(Vector3D(0, 0, 0), 1.0f);
            
            //  Point at sphere center
            {
                constexpr Vector3D point(0.0f, 0.0f, 0.0f);
                static_assert(Intersects(point, sphere), "Point at sphere center should intersect");
                static_assert(Intersects(sphere, point), "Intersection should be commutative");
            }
            
            //  Point on sphere surface
            {
                constexpr Vector3D point(1.0f, 0.0f, 0.0f);
                static_assert(Intersects(point, sphere), "Point on sphere surface should intersect");
            }
            
            //  Point inside sphere
            {
                constexpr Vector3D point(0.5f, 0.0f, 0.0f);
                static_assert(Intersects(point, sphere), "Point inside sphere should intersect");
            }
            
            //  Point outside sphere
            {
                constexpr Vector3D point(2.0f, 0.0f, 0.0f);
                static_assert(!Intersects(point, sphere), "Point outside sphere should not intersect");
            }
            
            //  Point on sphere surface (non-axis-aligned)
            {
                constexpr Vector3D point(0.57735f, 0.57735f, 0.57735f); // ~(1/√3, 1/√3, 1/√3)
                static_assert(Intersects(point, sphere), "Point on sphere surface (non-axis) should intersect");
            }
        }

        // ============================================
        // Test Runner
        // ============================================
        /**
         * @brief Runs all test cases in the collision test suite
         * @return true if all tests pass (which they must at compile-time)
         */
        static constexpr bool RunAll()
        {
            TestClassification();
            TestAABBvsAABB();
            TestSphereVsSphere();
            TestAABBvsSphere();
            TestPlaneIntersections();
            TestSphereVsPlane();
            TestContainment();
            TestEdgeCases();
            TestPointVsAABB();
            TestPointVsSphere();
            return true;
        }
    };

    // Execute all tests at compile time
    static_assert(CollisionTests::RunAll(), "Collision tests failed");
}