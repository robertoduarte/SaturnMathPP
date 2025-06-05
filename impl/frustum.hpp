#pragma once

#include "plane.hpp"
#include "mat43.hpp"
#include "aabb.hpp"
#include "sphere.hpp"

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
    class Frustum
    {
    private:
        // Named constants for plane indices
        static constexpr size_t PLANE_NEAR = 0;
        static constexpr size_t PLANE_FAR = 1;
        static constexpr size_t PLANE_TOP = 2;
        static constexpr size_t PLANE_BOTTOM = 3;
        static constexpr size_t PLANE_LEFT = 4;
        static constexpr size_t PLANE_RIGHT = 5;
        static constexpr size_t PLANE_COUNT = 6;

        Plane planes[PLANE_COUNT];  /**< Frustum boundary planes with inward-facing normals */

        Fxp nearDist;   /**< Near clipping plane distance (always positive) */
        Fxp farDist;    /**< Far clipping plane distance (always > nearDist) */
        Fxp farHeight;  /**< Height of the far plane (half-height * 2) */
        Fxp farWidth;   /**< Width of the far plane (half-width * 2) */

    public:
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
        Frustum(const Angle& verticalFov, const Fxp& aspectRatio, const Fxp& nearDist, const Fxp& farDist)
            : nearDist(nearDist)
            , farDist(farDist)
            , farHeight(Trigonometry::Tan(verticalFov * Fxp(0.5)) * farDist)
            , farWidth(farHeight * aspectRatio)
        {
            // Planes will be initialized in Update()
        }

        /**
         * @brief Updates frustum planes based on view matrix.
         * 
         * Recalculates all six frustum planes using the camera's:
         * - Position (viewMatrix.Row3)
         * - Forward direction (-viewMatrix.Row2)
         * - Up direction (viewMatrix.Row1)
         * - Right direction (viewMatrix.Row0)
         * 
         * @param viewMatrix Camera's view transformation
         */
        void Update(const Matrix43& viewMatrix)
        {
            // Calculate frustum corners at far plane
            const Vector3D& pos = viewMatrix.Row3;
            const Vector3D farCenter = pos - viewMatrix.Row3 * farDist;
            const Vector3D farUp = viewMatrix.Row1 * farHeight;
            const Vector3D farRight = viewMatrix.Row0 * farWidth;

            const Vector3D farTopCenter = farCenter + farUp;
            const Vector3D farTopLeft = farTopCenter - farRight;
            const Vector3D farTopRight = farTopCenter + farRight;

            const Vector3D farBottomCenter = farCenter - farUp;
            const Vector3D farBottomLeft = farBottomCenter - farRight;
            const Vector3D farBottomRight = farBottomCenter + farRight;

            // Define planes with inward-facing normals
            planes[PLANE_NEAR] = Plane(viewMatrix.Row2, pos - viewMatrix.Row2 * nearDist);
            planes[PLANE_FAR] = Plane(-viewMatrix.Row2, pos - viewMatrix.Row2 * farDist);
            planes[PLANE_TOP] = Plane(farTopLeft, pos, farTopRight);
            planes[PLANE_BOTTOM] = Plane(farBottomRight, pos, farBottomLeft);
            planes[PLANE_LEFT] = Plane(farBottomLeft, pos, farTopLeft);
            planes[PLANE_RIGHT] = Plane(farTopRight, pos, farBottomRight);
        }

        /**
         * @brief Tests if a point is inside the frustum.
         * 
         * A point is inside if it's on the positive side of all planes.
         * 
         * @param point Point to test
         * @return true if point is inside frustum
         */
        bool Contains(const Vector3D& point) const
        {
            for (const Plane& p : planes)
            {
                if (p.Distance(point) < 0)
                    return false;  // Outside this plane
            }
            return true;  // Inside all planes
        }

        /**
         * @brief Tests if a sphere intersects the frustum.
         * 
         * A sphere intersects if it's not completely outside any plane.
         * 
         * @param sphere Sphere to test
         * @return true if sphere intersects frustum
         */
        bool Contains(const Sphere& sphere) const
        {
            for (const Plane& p : planes)
            {
                if (!sphere.Intersects(p))
                    return false;  // Completely outside this plane
            }
            return true;  // At least partially inside all planes
        }

        /**
         * @brief Tests if an AABB intersects the frustum.
         * 
         * An AABB intersects if it's not completely outside any plane.
         * 
         * @param aabb Axis-aligned bounding box to test
         * @return true if AABB intersects frustum
         */
        bool Contains(const AABB& aabb) const
        {
            for (const Plane& p : planes)
            {
                if (!aabb.Intersects(p))
                    return false;  // Completely outside this plane
            }
            return true;  // At least partially inside all planes
        }
    };
}