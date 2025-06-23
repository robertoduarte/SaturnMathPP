#pragma once

#include "../impl/frustum.hpp"
#include "../impl/mat43.hpp"
#include "../impl/vector3d.hpp"
#include "../impl/aabb.hpp"
#include "../impl/sphere.hpp"
#include <cassert>

namespace SaturnMath::Tests
{
    using namespace SaturnMath::Types;
    using namespace SaturnMath::Collision;

    /**
 * @file test_frustum.hpp
 * @brief Compile-time unit tests for the Frustum class
 *
 * This file contains comprehensive, constexpr-based tests for the Frustum class, covering:
 * - Construction and basic properties
 * - Plane generation and orientation
 * - Point, sphere, and AABB containment and intersection
 * - Edge cases and rotated views
 *
 * All tests are performed at compile time using static_assert for maximum reliability.
 */
    struct FrustumTests
    {
        // ============================================
        // Construction & Properties
        // ============================================

        /**
         * @brief Helper function to create a test frustum with default parameters
         *
         * Returns a frustum with 90-degree FOV, 4:3 aspect, near=1, far=1000.
         */
        static constexpr Frustum CreateTestFrustum()
        {
            constexpr Angle fov = Angle::FromDegrees(90);
            constexpr Fxp aspect = 4.0 / 3.0;  // 4:3 aspect ratio
            constexpr Fxp nearDist(1);
            constexpr Fxp farDist(30000);
            return Frustum(fov, aspect, nearDist, farDist);
        }

        /**
         * @brief Tests frustum construction and basic properties
         *
         * Verifies:
         * - Correct initialization of distances and dimensions
         * - Aspect ratio calculations for near/far planes
         */
        static constexpr void TestConstruction()
        {
            // Test default construction with typical FOV and distances
            constexpr Frustum frustum = CreateTestFrustum();

            // Verify basic properties after construction
            static_assert(frustum.NearDist > 0, "Near distance should be positive");
            static_assert(frustum.FarDist > frustum.NearDist,
                "Far distance should be greater than near distance");
            static_assert(frustum.FarWidth > 0, "Far width should be positive");
            static_assert(frustum.FarHeight > 0, "Far height should be positive");
            static_assert(frustum.NearWidth > 0, "Near width should be positive");
            static_assert(frustum.NearHeight > 0, "Near height should be positive");

            // Verify aspect ratio is maintained for both near and far planes
            constexpr Fxp farAspectRatio = frustum.FarWidth / frustum.FarHeight;
            constexpr Fxp nearAspectRatio = frustum.NearWidth / frustum.NearHeight;
            constexpr Fxp expectedAspect = 4.0 / 3.0;
            constexpr Fxp aspectTolerance(0.1);

            static_assert((farAspectRatio - expectedAspect).Abs() < aspectTolerance,
                "Far plane aspect ratio should be approximately 4:3");
            static_assert((nearAspectRatio - expectedAspect).Abs() < aspectTolerance,
                "Near plane aspect ratio should be approximately 4:3");
        }

        // ============================================
        // Plane Generation & Orientation
        // ============================================

        /**
         * @brief Tests frustum plane generation from a view matrix
         *
         * Verifies:
         * - Plane normals are oriented inward as expected
         * - Plane normals match the coordinate system conventions
         */
        static constexpr void TestPlaneGeneration()
        {
            constexpr Vector3D position(0, 0, 0);
            constexpr Vector3D target(0, 0, -1);
            constexpr Vector3D up(0, 1, 0);
            constexpr Matrix43 viewMatrix = Matrix43::CreateLookAt(position, target, up);

            // Verify view matrix values
            static_assert(viewMatrix.Row0.X == -1, "X axis X should be -1");
            static_assert(viewMatrix.Row0.Y == 0, "X axis Y should be 0");
            static_assert(viewMatrix.Row0.Z == 0, "X axis Z should be 0");

            static_assert(viewMatrix.Row1.X == 0, "Y axis X should be 0");
            static_assert(viewMatrix.Row1.Y == 1, "Y axis Y should be 1");
            static_assert(viewMatrix.Row1.Z == 0, "Y axis Z should be 0");

            static_assert(viewMatrix.Row2.X == 0, "Z axis X should be 0");
            static_assert(viewMatrix.Row2.Y == 0, "Z axis Y should be 0");
            static_assert(viewMatrix.Row2.Z == 1, "Z axis Z should be 1");

            static_assert(viewMatrix.Row3.X == 0, "Translation X should be 0");
            static_assert(viewMatrix.Row3.Y == 0, "Translation Y should be 0");
            static_assert(viewMatrix.Row3.Z == 0, "Translation Z should be 0");

            // Create an updated frustum with the view matrix
            constexpr Frustum frustum = CreateTestFrustum().Updated(viewMatrix);

            // Get all plane references
            constexpr auto nearPlane = frustum.GetPlane(Frustum::PLANE_NEAR);
            constexpr auto farPlane = frustum.GetPlane(Frustum::PLANE_FAR);
            constexpr auto leftPlane = frustum.GetPlane(Frustum::PLANE_LEFT);
            constexpr auto rightPlane = frustum.GetPlane(Frustum::PLANE_RIGHT);
            constexpr auto topPlane = frustum.GetPlane(Frustum::PLANE_TOP);
            constexpr auto bottomPlane = frustum.GetPlane(Frustum::PLANE_BOTTOM);


            // Near plane validation (should face away from camera, negative Z in view space)
            static_assert(nearPlane.Normal.X == 0, "Near plane X should be 0");
            static_assert(nearPlane.Normal.Y == 0, "Near plane Y should be 0");
            static_assert(nearPlane.Normal.Z < -0.9, "Near plane Z should be negative (facing away from camera)");

            // Far plane validation (should face toward camera, positive Z in view space)
            static_assert(farPlane.Normal.X == 0, "Far plane X should be 0");
            static_assert(farPlane.Normal.Y == 0, "Far plane Y should be 0");
            static_assert(farPlane.Normal.Z > 0.9, "Far plane Z should be positive (facing toward camera)");

            // Left plane validation (facing right)
            static_assert(leftPlane.Normal.X > 0, "Left plane X should be positive (facing right)");
            static_assert(leftPlane.Normal.Y == 0, "Left plane Y should be 0");
            static_assert(leftPlane.Normal.Z < 0, "Left plane Z should be negative (tilted toward camera)");

            // Right plane validation (facing left)
            static_assert(rightPlane.Normal.X < 0, "Right plane X should be negative (facing left)");
            static_assert(rightPlane.Normal.Y == 0, "Right plane Y should be 0");
            static_assert(rightPlane.Normal.Z < 0, "Right plane Z should be negative (tilted toward camera)");

            // Top plane validation (facing down)
            static_assert(topPlane.Normal.X == 0, "Top plane X should be 0");
            static_assert(topPlane.Normal.Y < 0, "Top plane Y should be negative (facing down)");
            static_assert(topPlane.Normal.Z < 0, "Top plane Z should be negative (tilted toward camera)");

            // Bottom plane validation (facing up)
            static_assert(bottomPlane.Normal.X == 0, "Bottom plane X should be 0");
            static_assert(bottomPlane.Normal.Y > 0, "Bottom plane Y should be positive (facing up)");
            static_assert(bottomPlane.Normal.Z < 0, "Bottom plane Z should be negative (tilted toward camera)");
        }

        // ============================================
        // Point Containment
        // ============================================

        /**
         * @brief Tests point containment and classification
         *
         * Verifies:
         * - Points inside, outside, and on the boundary of the frustum
         * - Correct FrustumRelationship returned for each case
         */
        static constexpr void TestPointContainment()
        {
            // Create a simple view matrix (camera at origin looking down -Z)
            constexpr Vector3D position(0, 0, 0);
            constexpr Vector3D target(0, 0, -1);
            constexpr Vector3D up(0, 1, 0);
            constexpr Matrix43 viewMatrix = Matrix43::CreateLookAt(position, target, up);

            // Create and update frustum
            constexpr auto TestFrustum = []()
            {
                Frustum frustum = CreateTestFrustum();
                frustum.Update(viewMatrix);
                return frustum;
            };

            constexpr Frustum frustum = TestFrustum();

            // Get frustum dimensions for test points
            constexpr Fxp nearZ = -frustum.NearDist;
            constexpr Fxp farZ = -frustum.FarDist;
            constexpr Fxp midZ = (nearZ + farZ) / 2;

            // Points that should be inside the frustum
            constexpr Vector3D origin(0, 0, 0);
            constexpr Vector3D insidePoint(0, 0, midZ);
            constexpr Vector3D nearPlaneCenter(0, 0, nearZ);

            // Points inside the frustum
            static_assert(frustum.Classify(insidePoint) == Frustum::FrustumRelationship::Inside,
                "Point inside frustum should be detected");
            static_assert(frustum.Classify(nearPlaneCenter) == Frustum::FrustumRelationship::Intersects,
                "Point on near plane should be considered intersecting");

            // Points outside the frustum
            constexpr Vector3D behindNear(0, 0, 1);  // Behind camera
            constexpr Vector3D beyondFar(0, 0, farZ - 1);  // Beyond far plane

            // Calculate points outside each plane
            constexpr Fxp largeOffset(30001);
            constexpr Vector3D outsideLeft(-largeOffset, 0, midZ);
            constexpr Vector3D outsideRight(largeOffset, 0, midZ);
            constexpr Vector3D outsideTop(0, largeOffset, midZ);
            constexpr Vector3D outsideBottom(0, -largeOffset, midZ);

            // Test containment
            static_assert(frustum.Classify(origin) == Frustum::FrustumRelationship::Outside,
                "Point at camera position should be outside");
            static_assert(frustum.Classify(behindNear) == Frustum::FrustumRelationship::Outside,
                "Point behind near plane should be outside");
            static_assert(frustum.Classify(beyondFar) == Frustum::FrustumRelationship::Outside,
                "Point beyond far plane should be outside");
            static_assert(frustum.Classify(outsideLeft) == Frustum::FrustumRelationship::Outside,
                "Point outside left plane should be outside");
            static_assert(frustum.Classify(outsideRight) == Frustum::FrustumRelationship::Outside,
                "Point outside right plane should be outside");
            static_assert(frustum.Classify(outsideTop) == Frustum::FrustumRelationship::Outside,
                "Point outside top plane should be outside");
            static_assert(frustum.Classify(outsideBottom) == Frustum::FrustumRelationship::Outside,
                "Point outside bottom plane should be outside");
        }


        // Helper function to check AABB against each frustum plane
        static constexpr void CheckAABBAgainstPlanes(const Frustum& frustum, const AABB& box, const char* testName)
        {
            // Check each plane with assertions
            auto relation = Collision::Classify(box, frustum.Planes[Frustum::PLANE_NEAR]);
            assert((relation == Collision::PlaneRelationship::Front ||
                relation == Collision::PlaneRelationship::Intersects) &&
                "AABB is outside near plane");

            relation = Collision::Classify(box, frustum.Planes[Frustum::PLANE_FAR]);
            assert((relation == Collision::PlaneRelationship::Front ||
                relation == Collision::PlaneRelationship::Intersects) &&
                "AABB is outside far plane");

            relation = Collision::Classify(box, frustum.Planes[Frustum::PLANE_TOP]);
            assert((relation == Collision::PlaneRelationship::Front ||
                relation == Collision::PlaneRelationship::Intersects) &&
                "AABB is outside top plane");

            relation = Collision::Classify(box, frustum.Planes[Frustum::PLANE_BOTTOM]);
            assert((relation == Collision::PlaneRelationship::Front ||
                relation == Collision::PlaneRelationship::Intersects) &&
                "AABB is outside bottom plane");

            relation = Collision::Classify(box, frustum.Planes[Frustum::PLANE_LEFT]);
            assert((relation == Collision::PlaneRelationship::Front ||
                relation == Collision::PlaneRelationship::Intersects) &&
                "AABB is outside left plane");

            relation = Collision::Classify(box, frustum.Planes[Frustum::PLANE_RIGHT]);
            assert((relation == Collision::PlaneRelationship::Front ||
                relation == Collision::PlaneRelationship::Intersects) &&
                "AABB is outside right plane");
        }

        // ============================================
        // AABB Intersection
        // ============================================

        /**
         * @brief Tests AABB intersection and classification
         *
         * Verifies:
         * - AABBs inside, outside, and intersecting the frustum
         * - Correct FrustumRelationship and per-plane checks
         */
        static constexpr void TestAABBIntersection()
        {
            constexpr Angle fov = Angle::FromDegrees(90);
            constexpr Fxp aspect(1);
            constexpr Fxp nearDist(1);
            constexpr Fxp farDist(10);

            // Create a view matrix looking down -Z
            constexpr Vector3D position(0, 0, 0);
            constexpr Vector3D target(0, 0, -1);
            constexpr Vector3D up(0, 1, 0);
            constexpr Matrix43 viewMatrix = Matrix43::CreateLookAt(position, target, up);

            constexpr Frustum initialFrustum(fov, aspect, nearDist, farDist);
            constexpr Frustum frustum = initialFrustum.Updated(viewMatrix);

            // AABB completely inside frustum
            {
                constexpr Vector3D center(0, 0, -5);
                constexpr Vector3D extents(0.5, 0.5, 0.5);
                constexpr AABB box(center, extents);

                // First check each plane individually
                CheckAABBAgainstPlanes(frustum, box, "AABB inside frustum");

                // Then test the full classification
                constexpr auto relation = frustum.Classify(box);
                static_assert(relation == Frustum::FrustumRelationship::Inside,
                    "AABB inside frustum should be detected as Inside");
                static_assert(frustum.Intersects(box),
                    "AABB inside frustum should be detected by Intersects");
            }
            // AABB intersecting frustum
            {
                constexpr Vector3D center(0, 0, -nearDist);
                constexpr Vector3D extents(1, 1, 1);
                constexpr AABB box(center, extents);
                static_assert(frustum.Classify(box) == Frustum::FrustumRelationship::Intersects,
                    "AABB intersecting frustum should be detected as Intersects");
                static_assert(frustum.Intersects(box),
                    "AABB intersecting frustum should be detected by Intersects");
            }

            // AABB completely outside frustum (behind)
            {
                constexpr Vector3D center(0, 0, 1);  // Behind camera
                constexpr Vector3D extents(0.5, 0.5, 0.5);
                constexpr AABB box(center, extents);
                static_assert(frustum.Classify(box) == Frustum::FrustumRelationship::Outside,
                    "AABB outside frustum should be detected as Outside");
                static_assert(!frustum.Intersects(box),
                    "AABB outside frustum should not be detected by Intersects");
            }

            // AABB intersecting multiple planes
            {
                // This AABB is large enough to intersect multiple frustum planes
                constexpr Vector3D center(0, 0, -5);
                constexpr Vector3D extents(10, 10, 10);
                constexpr AABB box(center, extents);
                static_assert(frustum.Classify(box) == Frustum::FrustumRelationship::Intersects,
                    "Large AABB should be detected as intersecting");
                static_assert(frustum.Intersects(box),
                    "Large AABB should be detected by Intersects");
            }

            // AABB completely outside frustum (behind near plane)
            {
                constexpr Vector3D center(0, 0, 1);  // Behind camera
                constexpr Vector3D extents(0.5, 0.5, 0.5);
                constexpr AABB box(center, extents);
                static_assert(!frustum.Intersects(box), "AABB behind frustum should not intersect");
            }

            // AABB completely outside frustum (beyond far plane)
            {
                constexpr Vector3D center(0, 0, -farDist - 1);
                constexpr Vector3D extents(0.5, 0.5, 0.5);
                constexpr AABB box(center, extents);
                static_assert(!frustum.Intersects(box), "AABB beyond far plane should not intersect");
            }

            // Large AABB containing frustum
            {
                constexpr Vector3D center(0, 0, -farDist / 2);
                constexpr Vector3D extents(farDist, farDist, farDist);
                constexpr AABB box(center, extents);
                static_assert(frustum.Intersects(box), "AABB containing frustum should intersect");
            }
        }

        static constexpr Collision::PlaneRelationship ClassifyAABB(const AABB& aabb, const Plane& plane, Fxp epsilon = Fxp::Epsilon())
        {
            // Get the AABB's center and half-extents
            const Vector3D center = aabb.GetPosition();
            const Vector3D halfExtents = aabb.GetHalfExtents();

            // Project the half-extents onto the plane normal (manhattan distance)
            // This gives us the effective radius of the AABB in the plane's normal direction
            const Fxp radius = halfExtents.Dot(plane.Normal.Abs());

            // Calculate the signed distance from the AABB's center to the plane
            const Fxp distance = plane.GetSignedDistance(center);

            return (distance > radius + epsilon) ? Collision::PlaneRelationship::Front :
                (distance < -radius - epsilon) ? Collision::PlaneRelationship::Back :
                Collision::PlaneRelationship::Intersects;
        }

        // ============================================
        // Sphere Intersection
        // ============================================

        /**
         * @brief Tests sphere intersection and classification
         *
         * Verifies:
         * - Spheres inside, outside, and intersecting the frustum
         * - Correct FrustumRelationship for each case
         */
        static constexpr void TestSphereIntersection()
        {
            constexpr Angle fov = Angle::FromDegrees(90);
            constexpr Fxp aspect(1);
            constexpr Fxp nearDist(1);
            constexpr Fxp farDist(10);

            // Create a view matrix looking down -Z
            constexpr Vector3D position(0, 0, 0);
            constexpr Vector3D target(0, 0, -1);
            constexpr Vector3D up(0, 1, 0);
            constexpr Matrix43 viewMatrix = Matrix43::CreateLookAt(position, target, up);

            constexpr Frustum initialFrustum(fov, aspect, nearDist, farDist);
            constexpr Frustum frustum = initialFrustum.Updated(viewMatrix);
            
            // Sphere completely inside frustum
            {
                constexpr Vector3D center(0, 0, -5);
                constexpr Fxp radius(0.5);
                constexpr Sphere sphere(center, radius);
                static_assert(frustum.Intersects(sphere), "Sphere inside frustum should be detected");
            }

            // Sphere intersecting near plane
            {
                constexpr Vector3D center(0, 0, -nearDist + 0.5);
                constexpr Fxp radius(1);
                constexpr Sphere sphere(center, radius);
                static_assert(frustum.Intersects(sphere), "Sphere intersecting frustum should be detected");
            }

            // Sphere completely outside frustum (behind)
            {
                constexpr Vector3D center(0, 0, 1);  // Behind camera
                constexpr Fxp radius(0.5);
                constexpr Sphere sphere(center, radius);
                static_assert(!frustum.Intersects(sphere), "Sphere behind frustum should not intersect");
            }

            // Large sphere containing frustum
            {
                constexpr Vector3D center(0, 0, -farDist / 2);
                constexpr Fxp radius(farDist);
                constexpr Sphere sphere(center, radius);
                static_assert(frustum.Intersects(sphere), "Large sphere containing frustum should intersect");
            }
        }

        // Test frustum with a rotated camera view
        static constexpr void TestRotatedView()
        {
            constexpr Angle fov = Angle::FromDegrees(90);
            constexpr Fxp aspect(1);
            constexpr Fxp nearDist(1);
            constexpr Fxp farDist(10);

            // Create a view matrix with camera rotated 45 degrees around Y axis
            constexpr Vector3D position(0, 0, 0);
            constexpr Vector3D target(1, 0, -1);  // Looking diagonally to the right
            constexpr Vector3D up(0, 1, 0);
            constexpr Matrix43 viewMatrix = Matrix43::CreateLookAt(position, target, up);
            constexpr Frustum initialFrustum(fov, aspect, nearDist, farDist);
            constexpr Frustum frustum = initialFrustum.Updated(viewMatrix);

            // Point in front of rotated camera (should be inside)
            {
                constexpr Vector3D point(5, 0, -5);
                static_assert(frustum.Classify(point) == Frustum::FrustumRelationship::Inside,
                    "Point in front of rotated camera should be inside");
            }

            // Point behind rotated camera (should be outside)
            {
                constexpr Vector3D point(-1, 0, 1);
                static_assert(frustum.Classify(point) == Frustum::FrustumRelationship::Outside,
                    "Point behind rotated camera should be outside");
            }

            // AABB intersecting the rotated frustum
            {
                constexpr Vector3D minPoint(4, -1, -6);
                constexpr Vector3D maxPoint(6, 1, -4);
                constexpr AABB box = AABB::FromMinMax(minPoint, maxPoint);

                // Check each plane individually to find which one fails
                constexpr auto nearPlane = frustum.GetPlane(Frustum::PLANE_NEAR);
                constexpr auto farPlane = frustum.GetPlane(Frustum::PLANE_FAR);
                constexpr auto topPlane = frustum.GetPlane(Frustum::PLANE_TOP);
                constexpr auto bottomPlane = frustum.GetPlane(Frustum::PLANE_BOTTOM);
                constexpr auto leftPlane = frustum.GetPlane(Frustum::PLANE_LEFT);
                constexpr auto rightPlane = frustum.GetPlane(Frustum::PLANE_RIGHT);

                constexpr auto result = ClassifyAABB(box, nearPlane);
                constexpr auto result2 = Collision::Classify(box, nearPlane);

                // Check classification against each plane
                static_assert(Collision::Classify(box, nearPlane) == Collision::PlaneRelationship::Front, "AABB should be in front of near plane");
                static_assert(Collision::Classify(box, farPlane) == Collision::PlaneRelationship::Front, "AABB should be in front of far plane");
                static_assert(Collision::Classify(box, topPlane) == Collision::PlaneRelationship::Front, "AABB should be in front of top plane");
                static_assert(Collision::Classify(box, bottomPlane) == Collision::PlaneRelationship::Front, "AABB should be in front of bottom plane");
                static_assert(Collision::Classify(box, leftPlane) == Collision::PlaneRelationship::Front, "AABB should be in front of left plane");
                static_assert(Collision::Classify(box, rightPlane) == Collision::PlaneRelationship::Front, "AABB should be in front of right plane");

                // Now check the full classification
                static_assert(frustum.Classify(box) == Frustum::FrustumRelationship::Inside,
                    "AABB in front of rotated camera should be inside");
            }

            // AABB completely outside the rotated frustum
            {
                constexpr Vector3D minPoint(-6, -1, 4);
                constexpr Vector3D maxPoint(-4, 1, 6);
                constexpr AABB box = AABB::FromMinMax(minPoint, maxPoint);
                static_assert(frustum.Classify(box) == Frustum::FrustumRelationship::Outside,
                    "AABB behind rotated camera should be outside");
            }

            // Sphere intersecting the rotated frustum
            {
                constexpr Vector3D center(5, 0, -5);
                constexpr Fxp radius(2);
                constexpr Sphere sphere(center, radius);
                static_assert(frustum.Classify(sphere) == Frustum::FrustumRelationship::Inside,
                    "Sphere in front of rotated camera should be inside");
            }

            // Sphere completely outside the rotated frustum
            {
                constexpr Vector3D center(-5, 0, 5);
                constexpr Fxp radius(1);
                constexpr Sphere sphere(center, radius);
                static_assert(frustum.Classify(sphere) == Frustum::FrustumRelationship::Outside,
                    "Sphere behind rotated camera should be outside");
            }
        }

        // ============================================
        // Test Runner
        // ============================================

        /**
         * @brief Runs all frustum tests
         */
        static constexpr bool RunAll()
        {
            TestConstruction();
            TestPlaneGeneration();
            TestPointContainment();
            TestAABBIntersection();
            TestSphereIntersection();
            TestRotatedView();
            return true;
        }
    };

    // Execute all tests at compile time
    static_assert((FrustumTests::RunAll(), true), "Frustum tests failed");
}
