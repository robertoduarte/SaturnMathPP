#pragma once

#include "vector3d.hpp"

namespace SaturnMath::Types
{
    /**
     * @brief Infinite plane in 3D space defined by normal and distance.
     * 
     * Uses the equation: normal·X + d = 0, where:
     * - normal: Unit vector perpendicular to the plane
     * - d: Signed distance from origin to plane along normal
     * - X: Any point on the plane
     * 
     * Distance to point P is calculated as: normal·P + d
     * - Positive: Point is on same side as normal
     * - Zero: Point is on plane
     * - Negative: Point is on opposite side from normal
     */
    class Plane
    {
    public:
        Vector3D normal; /**< Unit normal vector (should be normalized) */
        Fxp d;          /**< Signed distance from origin to plane */

        /** @brief Default constructor. Creates XY plane at origin. */
        constexpr Plane() : normal(Vector3D::UnitZ()), d((int16_t)0) {}

        /**
         * @brief Creates plane from normal and distance.
         * @param normal Direction perpendicular to plane (should be normalized)
         * @param d Signed distance from origin to plane along normal
         */
        constexpr Plane(const Vector3D& normal, Fxp d) : normal(normal), d(d) {}

        /**
         * @brief Creates plane from normal and point.
         * @param normal Direction perpendicular to plane (should be normalized)
         * @param point Any point that lies on the plane
         */
        constexpr Plane(const Vector3D& normal, const Vector3D& point)
            : normal(normal)
            , d(-normal.Dot(point))  // Negative because we want normal·X + d = 0
        {}

        /**
         * @brief Creates plane from three non-collinear points.
         * 
         * Points should be specified in counter-clockwise order when
         * viewed from the side the normal points toward.
         * 
         * @param a First point on plane
         * @param b Second point on plane
         * @param c Third point on plane
         */
        Plane(const Vector3D& a, const Vector3D& b, const Vector3D& c)
        {
            Vector3D cross = (b - a).Cross(c - a);
            normal = cross.Normalize();
            d = -normal.Dot(a);  // Negative because we want normal·X + d = 0
        }

        /**
         * @brief Calculates signed distance from point to plane.
         * 
         * @param point Point to test
         * @return Signed distance where:
         *         > 0: Point is on same side as normal
         *         = 0: Point is on plane
         *         < 0: Point is on opposite side from normal
         */
        constexpr Fxp Distance(const Vector3D& point) const
        {
            return normal.Dot(point) + d;  // normal·X + d
        }

        /**
         * @brief Projects point onto plane.
         * @param point Point to project
         * @return Closest point on plane to given point
         */
        constexpr Vector3D Project(const Vector3D& point) const
        {
            return point - normal * Distance(point);
        }

        /**
         * @brief Normalizes the plane equation.
         * 
         * Ensures normal is unit length while maintaining
         * the same plane equation.
         * 
         * @return Reference to this plane
         */
        constexpr Plane& Normalize()
        {
            Fxp len = normal.Length();
            if (len > 0)
            {
                normal /= len;
                d /= len;
            }
            return *this;
        }
    };
}