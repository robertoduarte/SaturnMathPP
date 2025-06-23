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
        constexpr void Update(const Matrix43& viewMatrix)
        {
            const Vector3D& pos = viewMatrix.Row3;
            
            // Calculate near plane corners
            const Vector3D originalNearCenter = pos - viewMatrix.Row2 * NearDist;
            const Vector3D nearCenter = pos - viewMatrix.Row2 * REFERENCE_NEAR_DISTANCE;
            const Vector3D nearUp = viewMatrix.Row1 * NearHeight;
            const Vector3D nearRight = viewMatrix.Row0 * NearWidth;
            
            const Vector3D nearTopCenter = nearCenter + nearUp;
            const Vector3D nearTopRight = nearTopCenter - nearRight; 
            const Vector3D nearTopLeft = nearTopCenter + nearRight;  
            const Vector3D nearBottomCenter = nearCenter - nearUp;
            const Vector3D nearBottomRight = nearBottomCenter - nearRight; 
            const Vector3D nearBottomLeft = nearBottomCenter + nearRight;

            // Use original far distance only for the far plane center
            const Vector3D originalFarCenter = pos - viewMatrix.Row2 * FarDist;
            const Vector3D farCenter = pos - viewMatrix.Row2 * REFERENCE_MAX_DISTANCE;
            const Vector3D farUp = viewMatrix.Row1 * FarHeight;
            const Vector3D farRight = viewMatrix.Row0 * FarWidth;

            const Vector3D farTopRight = farCenter + farUp - farRight; 
            const Vector3D farTopLeft = farCenter + farUp + farRight;  
            const Vector3D farBottomRight = farCenter - farUp - farRight; 
            const Vector3D farBottomLeft = farCenter - farUp + farRight;   

            // Define planes with inward-facing normals in view space
            // In view space, the camera looks down the negative Z axis
            // So the near plane normal should be (0,0,-1) and far plane normal (0,0,1)
            
            // Near plane (facing away from camera, negative Z in view space)
            Planes[PLANE_NEAR] = Plane(-viewMatrix.Row2, originalNearCenter);
            
            // Far plane (facing toward camera, positive Z in view space)
            Planes[PLANE_FAR] = Plane(viewMatrix.Row2, originalFarCenter);
            
            // Calculate plane normals using camera position and view direction vectors
            // This ensures the planes are correctly oriented in view space
            
            // Top plane (facing down, negative Y in view space)
            // Using three points in counter-clockwise order when viewed from inside the frustum
            Planes[PLANE_TOP] = Plane::FromPoints(
                nearTopRight,   // top-right near
                nearTopLeft,    // top-left near
                farTopLeft      // top-left far
            );
            
            // Bottom plane (facing up, positive Y in view space)
            // Using three points in counter-clockwise order when viewed from inside the frustum
            Planes[PLANE_BOTTOM] = Plane::FromPoints(
                nearBottomLeft,    // bottom-left near
                nearBottomRight,  // bottom-right near
                farBottomRight     // bottom-right far
            );
            
            // Left plane (facing right, positive X in view space)
            Planes[PLANE_LEFT] = Plane::FromPoints(
                nearTopLeft,     // top-left near
                nearBottomLeft,  // bottom-left near
                farBottomLeft    // bottom-left far
            );

            // Right plane (facing left, negative X in view space)
            Planes[PLANE_RIGHT] = Plane::FromPoints(
                nearBottomRight, // bottom-right near
                nearTopRight,    // top-right near
                farTopRight      // top-right far
            );
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
