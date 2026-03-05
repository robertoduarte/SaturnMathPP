#pragma once

#include "mat43.hpp"
#include "collision.hpp"

namespace SaturnMath::Types
{
    /**
     * @brief High-performance view frustum implementation for efficient 3D culling operations.
     * 
     * @details The Frustum class represents a truncated pyramid defined by six planes,
     * commonly used for view frustum culling in 3D rendering pipelines. It provides
     * optimized methods for testing whether geometric primitives are inside, outside,
     * or intersecting the frustum.
     * 
     * Key features:
     * - Memory-efficient representation (six planes)
     * - Optimized intersection tests with common primitives (points, AABBs, spheres)
     * - Multiple construction methods (from projection matrix, from individual planes)
     * - Fixed-point arithmetic for consistent behavior across platforms
     * - Specialized fast-rejection tests for performance-critical rendering paths
     * 
     * Frustum planes:
     * - Near: Close clipping plane (minimum Z distance)
     * - Far: Distance clipping plane (maximum Z distance)
     * - Left/Right: Horizontal bounds of the view
     * - Top/Bottom: Vertical bounds of the view
     * 
     * Coordinate system:
     * - Uses a right-handed coordinate system where:
     *   - +X is right
     *   - +Y is up
     *   - -Z is forward (into the screen)
     * 
     * All plane normals point inward toward the frustum interior, so an object is inside
     * the frustum if it's on the positive side of all planes. This convention simplifies
     * containment tests and is consistent with standard graphics literature.
     * 
     * Performance optimizations:
     * - Tests are ordered to maximize early rejection
     * - AABB tests use optimized corner selection based on plane normals
     * - Sphere tests use squared distances to avoid square root calculations
     * 
     * Common applications:
     * - View frustum culling in rendering pipelines
     * - Portal culling for indoor environments
     * - Occlusion culling pre-processing
     * - Visibility determination for LOD systems
     * 
     * @note For optimal culling performance, consider using a spatial partitioning
     * structure (octree, BVH) in conjunction with frustum culling to minimize the
     * number of individual object tests.
     * 
     * @see Plane For the underlying plane implementation
     * @see AABB For axis-aligned bounding box intersection tests
     * @see Sphere For sphere intersection tests
     */
    struct Frustum
    {
        // Named constants for plane indices
        static constexpr size_t PLANE_NEAR = 0;
        static constexpr size_t PLANE_FAR = 1;
        static constexpr size_t PLANE_TOP = 2;
        static constexpr size_t PLANE_BOTTOM = 3;
        static constexpr size_t PLANE_LEFT = 4;
        static constexpr size_t PLANE_RIGHT = 5;
        static constexpr size_t PLANE_COUNT = 6;
        static constexpr Fxp REFERENCE_MAX_DISTANCE = 10;
        static constexpr Fxp REFERENCE_NEAR_DISTANCE = 1;

        Plane Planes[PLANE_COUNT];  /**< Frustum boundary planes with inward-facing normals */

        Fxp NearDist;   /**< Near clipping plane distance (always positive) */
        Fxp FarDist;    /**< Far clipping plane distance (always > nearDist) */
        Fxp NearHeight; /**< Height of the near plane (half-height * 2) */
        Fxp NearWidth;  /**< Width of the near plane (half-width * 2) */
        Fxp FarHeight;  /**< Height of the far plane (half-height * 2) */
        Fxp FarWidth;   /**< Width of the far plane (half-width * 2) */

    
        enum class FrustumRelationship
        {
            Inside,
            Intersects,
            Outside
        };

        /**
         * @brief Constructs a view frustum.
         * 
         * @param verticalFov Vertical field of view in radians (use Angle class)
         * @param aspectRatio Width/height ratio of the viewport
         * @param nearDist Distance to near clipping plane (must be > 0)
         * @param farDist Distance to far clipping plane (must be > nearDist)
         * 
         * @note FOV is the full angle, so tan(fov/2) is used for calculations
         */
        constexpr Frustum(const Angle& verticalFov, const Fxp& aspectRatio, const Fxp& nearDist, const Fxp& farDist)
            : NearDist(nearDist)
            , FarDist(farDist)
            , NearHeight(Trigonometry::Tan(verticalFov / 2) * REFERENCE_NEAR_DISTANCE)
            , NearWidth(NearHeight * aspectRatio)
            , FarHeight(Trigonometry::Tan(verticalFov / 2) * REFERENCE_MAX_DISTANCE)
            , FarWidth(FarHeight * aspectRatio)
        {
            // Planes will be initialized in Update()
        }

        /**
         * @brief Creates a plane from three points, ensuring the normal faces inward.
         * 
         * Builds a plane via FromPoints, then flips the normal if it does not
         * point toward the given interior reference point.
         * 
         * @param a First point on the plane
         * @param b Second point on the plane
         * @param c Third point on the plane
         * @param interiorPoint A point known to be inside the frustum
         * @return Plane with inward-facing normal
         */
        static constexpr Plane MakeInwardPlane(
            const Vector3D& a, const Vector3D& b, const Vector3D& c,
            const Vector3D& interiorPoint)
        {
            Plane p = Plane::FromPoints(a, b, c);
            // If the interior point is behind the plane, the normal faces outward — flip it.
            if (p.GetSignedDistance(interiorPoint) < 0)
            {
                p.Normal = -p.Normal;
                p.SignedDistance = -p.SignedDistance;
            }
            return p;
        }

        /**
         * @brief Updates frustum planes based on view matrix.
         * 
         * Recalculates all six frustum planes using the camera's:
         * - Position (recovered from viewMatrix rotation and translation)
         * - Forward direction (-viewMatrix.Row2)
         * - Up direction (viewMatrix.Row1)
         * - Right direction (viewMatrix.Row0)
         * 
         * @param viewMatrix Camera's view transformation
         */
        constexpr void Update(const Matrix43& viewMatrix)
        {
            // Recover world-space camera position from the view matrix.
            // Row3 of a view matrix stores (-right·eye, -up·eye, -viewZ·eye),
            // so we reconstruct the eye position by inverting the transform.
            const Vector3D pos = -(viewMatrix.Row0 * viewMatrix.Row3.X
                                 + viewMatrix.Row1 * viewMatrix.Row3.Y
                                 + viewMatrix.Row2 * viewMatrix.Row3.Z);

            // Camera basis vectors
            const Vector3D right = viewMatrix.Row0;   // camera right
            const Vector3D up    = viewMatrix.Row1;    // camera up
            const Vector3D forward = -viewMatrix.Row2; // camera forward (into the scene)

            // Near and far plane centers at actual clipping distances
            const Vector3D nearPlaneCenter = pos + forward * NearDist;
            const Vector3D farPlaneCenter  = pos + forward * FarDist;

            // Reference near/far centers used for side plane geometry
            const Vector3D nearCenter = pos + forward * REFERENCE_NEAR_DISTANCE;
            const Vector3D farCenter  = pos + forward * REFERENCE_MAX_DISTANCE;

            // Near plane corners (camera-space naming: tl/tr/bl/br)
            const Vector3D nearUp    = up * NearHeight;
            const Vector3D nearRight = right * NearWidth;

            const Vector3D ntl = nearCenter + nearUp - nearRight; // camera top-left
            const Vector3D ntr = nearCenter + nearUp + nearRight; // camera top-right
            const Vector3D nbl = nearCenter - nearUp - nearRight; // camera bottom-left
            const Vector3D nbr = nearCenter - nearUp + nearRight; // camera bottom-right

            // Far plane corners
            const Vector3D farUp    = up * FarHeight;
            const Vector3D farRight = right * FarWidth;

            const Vector3D ftl = farCenter + farUp - farRight;
            const Vector3D ftr = farCenter + farUp + farRight;
            const Vector3D fbl = farCenter - farUp - farRight;
            const Vector3D fbr = farCenter - farUp + farRight;

            // Frustum center point for inward-normal validation
            const Vector3D frustumCenter = pos + forward * ((REFERENCE_NEAR_DISTANCE + REFERENCE_MAX_DISTANCE) / Fxp(int16_t{2}));

            // Near plane: normal points into the frustum (same as forward)
            Planes[PLANE_NEAR] = Plane(forward, nearPlaneCenter);

            // Far plane: normal points back toward camera (opposite of forward)
            Planes[PLANE_FAR] = Plane(-forward, farPlaneCenter);

            // Side planes: build from three coplanar points, then
            // MakeInwardPlane ensures the normal always faces inward.

            // Top plane: uses camera top-left, top-right, and far top-right
            Planes[PLANE_TOP] = MakeInwardPlane(ntl, ntr, ftr, frustumCenter);

            // Bottom plane: uses camera bottom-right, bottom-left, and far bottom-left
            Planes[PLANE_BOTTOM] = MakeInwardPlane(nbr, nbl, fbl, frustumCenter);

            // Left plane: uses camera bottom-left, top-left, and far top-left
            Planes[PLANE_LEFT] = MakeInwardPlane(nbl, ntl, ftl, frustumCenter);

            // Right plane: uses camera top-right, bottom-right, and far bottom-right
            Planes[PLANE_RIGHT] = MakeInwardPlane(ntr, nbr, fbr, frustumCenter);
        }

        constexpr Frustum Updated(const Matrix43& viewMatrix) const
        {
            Frustum result = *this;
            result.Update(viewMatrix);
            return result;
        }

         /**
         * @brief Classifies a point's relationship with the frustum
         * @param point The point to test
         * @return FrustumRelationship::Inside if the point is inside all frustum planes,
         *         FrustumRelationship::Intersects if the point lies exactly on a frustum plane,
         *         FrustumRelationship::Outside if the point is outside any frustum plane
         */
        constexpr FrustumRelationship Classify(const Vector3D& point) const
        {
            bool onPlane = false;
            
            for (const Plane& plane : Planes)
            {
                auto relation = Collision::Classify(point, plane);
                if (relation == Collision::PlaneRelationship::Back)
                    return FrustumRelationship::Outside;
                if (relation == Collision::PlaneRelationship::Intersects)
                    onPlane = true;
            }
            
            return onPlane ? FrustumRelationship::Intersects : FrustumRelationship::Inside;
        }

        /**
         * @brief Classifies a sphere's relationship with the frustum
         * @param sphere The sphere to test
         * @return FrustumRelationship::Inside if the sphere is completely inside all frustum planes,
         *         FrustumRelationship::Intersects if the sphere intersects any frustum plane,
         *         FrustumRelationship::Outside if the sphere is completely outside any frustum plane
         */
        constexpr FrustumRelationship Classify(const Sphere& sphere) const
        {
            bool intersects = false;
            
            for (const Plane& plane : Planes)
            {
                auto relation = Collision::Classify(sphere, plane);
                if (relation == Collision::PlaneRelationship::Back)
                    return FrustumRelationship::Outside;
                if (relation == Collision::PlaneRelationship::Intersects)
                    intersects = true;
            }
            
            return intersects ? FrustumRelationship::Intersects : FrustumRelationship::Inside;
        }

        /**
         * @brief Classifies an AABB's relationship with the frustum
         * @param aabb The axis-aligned bounding box to test
         * @return FrustumRelationship::Inside if the AABB is completely inside all frustum planes,
         *         FrustumRelationship::Intersects if the AABB intersects any frustum plane,
         *         FrustumRelationship::Outside if the AABB is completely outside any frustum plane
         */
        constexpr FrustumRelationship Classify(const AABB& aabb) const
        {
            bool intersects = false;
            
            // Use index-based loop for better constexpr compatibility
            for (size_t i = 0; i < 6; ++i)
            {
                auto relation = Collision::Classify(aabb, Planes[i]);
                if (relation == Collision::PlaneRelationship::Back)
                    return FrustumRelationship::Outside;
                if (relation == Collision::PlaneRelationship::Intersects)
                    intersects = true;
            }
            
            return intersects ? FrustumRelationship::Intersects : FrustumRelationship::Inside;
        }

        /**
         * @brief Gets a frustum plane by index
         * @param index Index of the plane (0-5)
         * @return The requested frustum plane
         */
        constexpr const Plane& GetPlane(size_t index) const { return Planes[index]; }

        /**
         * @brief Checks if a point is inside or intersecting the frustum
         * @param point The point to test
         * @return true if the point is inside or intersecting the frustum, false otherwise
         * 
         * @note This is more efficient than Classify() when you only need to know if the point is visible
         */
        constexpr bool Intersects(const Vector3D& point) const
        {
            for (const Plane& p : Planes)
            {
                if (Collision::Classify(point, p) == Collision::PlaneRelationship::Back)
                    return false;
            }
            return true;
        }

        /**
         * @brief Checks if a sphere is inside or intersecting the frustum
         * @param sphere The sphere to test
         * @return true if the sphere is inside or intersecting the frustum, false otherwise
         * 
         * @note This is more efficient than Classify() when you only need to know if the sphere is visible
         */
        constexpr bool Intersects(const Sphere& sphere) const
        {
            for (const Plane& p : Planes)
            {
                if (Collision::Classify(sphere, p) == Collision::PlaneRelationship::Back)
                    return false;
            }
            return true;
        }

        /**
         * @brief Checks if an AABB is inside or intersecting the frustum
         * @param aabb The axis-aligned bounding box to test
         * @return true if the AABB is inside or intersecting the frustum, false otherwise
         * 
         * @note This is more efficient than Classify() when you only need to know if the AABB is visible
         */
        constexpr bool Intersects(const AABB& aabb) const
        {
            // Use index-based loop for better constexpr compatibility
            for (size_t i = 0; i < 6; ++i)
            {
                if (Collision::Classify(aabb, Planes[i]) == Collision::PlaneRelationship::Back)
                    return false;
            }
            return true;
        }
    };
}
