#pragma once

#include "shape.hpp"
#include "plane.hpp"
#include "aabb.hpp"

namespace SaturnMath
{
    /**
     * @brief Perfect sphere defined by center and radius.
     * 
     * Provides intersection tests against:
     * - Planes (one-sided test)
     * - AABBs (exact test)
     * - Other spheres (exact test)
     * - Points (exact test)
     */
    class Sphere : public Shape
    {
    public:
        /**
         * @brief Creates sphere from center and radius.
         * @param center Center point of sphere
         * @param radius Radius of sphere (must be >= 0)
         */
        constexpr Sphere(const Vector3D& center, const Fxp& radius)
            : Shape(center)
            , radius(radius)
        {}

        /** @brief Gets sphere radius. */
        constexpr Fxp GetRadius() const { return radius; }

        /**
         * @brief Tests intersection with plane.
         * 
         * A sphere intersects a plane if any part of it is on
         * the positive side (same side as plane normal).
         * 
         * @param plane Plane to test against
         * @return true if sphere intersects positive half-space
         */
        bool Intersects(const Plane& plane) const override
        {
            return plane.Distance(position) >= -radius;
        }

        /**
         * @brief Tests if point is inside sphere.
         * @param point Point to test
         * @return true if point is inside or on surface
         */
        constexpr bool Contains(const Vector3D& point) const
        {
            return (point - position).LengthSquared() <= radius * radius;
        }

        /**
         * @brief Tests intersection with another sphere.
         * @param other Sphere to test against
         * @return true if spheres overlap
         */
        constexpr bool Intersects(const Sphere& other) const
        {
            Fxp radiusSum = radius + other.radius;
            return (other.position - position).LengthSquared() <= radiusSum * radiusSum;
        }

        /**
         * @brief Tests intersection with AABB.
         * @param box Box to test against
         * @return true if sphere and box overlap
         */
        bool Intersects(const AABB& box) const
        {
            // Find closest point on box to sphere center
            Vector3D closest = box.GetClosestPoint(position);
            
            // Test if this point is within sphere radius
            return (closest - position).LengthSquared() <= radius * radius;
        }

        /**
         * @brief Calculates bounding box containing sphere.
         * @return Axis-aligned box that exactly contains sphere
         */
        constexpr AABB GetBoundingBox() const
        {
            Vector3D offset(radius, radius, radius);
            return AABB::FromMinMax(position - offset, position + offset);
        }

    private:
        Fxp radius;  /**< Sphere radius (always >= 0) */
    };
}
