#pragma once

#include "vector3d.hpp"

namespace SaturnMath::Types
{
    /**
     * @brief Infinite plane in 3D space defined by normal and distance for efficient geometric calculations.
     * 
     * @details The Plane class represents an infinite mathematical plane in 3D space
     * using the standard form of the plane equation: normal·X - d = 0, where:
     * - normal: Unit vector perpendicular to the plane (defines orientation)
     * - d: Signed distance from origin to plane along normal direction
     * - X: Any point on the plane
     * 
     * Key features:
     * - Memory-efficient representation (normal vector + distance scalar)
     * - Constant-time distance calculations to points
     * - Efficient intersection tests with rays, lines, and other geometric primitives
     * - Multiple construction methods (3 points, normal+point, normal+distance)
     * - Fixed-point arithmetic for consistent behavior across platforms
     * 
     * Mathematical properties:
     * - Distance to point P is calculated as: normal·P - d
     *   - Positive: Point is on same side as normal (in front of plane)
     *   - Zero: Point is exactly on the plane
     *   - Negative: Point is on opposite side from normal (behind plane)
     * - The normal vector should always be normalized (unit length)
     *   for correct distance calculations
     * - Planes can be used to define half-spaces for constructive solid geometry
     * 
     * Common applications:
     * - Collision detection and response
     * - View frustum culling (camera planes)
     * - Constructive solid geometry (CSG)
     * - Reflections and shadow calculations
     * - Clipping algorithms
     * 
     * @note For optimal performance, ensure the normal vector is normalized.
     * Non-normalized normals will result in incorrect distance calculations.
     * 
     * @see Frustum For usage of planes in camera view frustums
     * @see AABB For intersection tests between planes and bounding boxes
     */
    template<int I = 16, int F = 16>
    class PlaneX
    {
        using T = FixedPoint<I, F>;
        using Vec3 = Vector3<I, F>;
    public:
        Vec3 Normal; /**< Unit normal vector (should be normalized) */
        T SignedDistance;          /**< Signed distance from origin to plane */

        /** @brief Default constructor. Creates XZ plane at origin. */
        constexpr PlaneX() : Normal(Vec3::UnitY()), SignedDistance((int16_t)0) {}

        /**
         * @brief Creates plane from normal and distance.
         * @param normal Direction perpendicular to plane (should be normalized)
         * @param signedDistance Signed distance from origin to plane along normal
         */
        constexpr PlaneX(const Vec3& normal, T signedDistance) : Normal(normal), SignedDistance(signedDistance) {}

        /**
         * @brief Creates plane from normal and point.
         * @param normal Direction perpendicular to plane (should be normalized)
         * @param point Any point that lies on the plane
         */
        static constexpr PlaneX FromNormalAndPoint(const Vec3& normal, const Vec3& point)
        {
            return PlaneX(normal, normal.Dot(point));
        }
        
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
        static constexpr PlaneX FromPoints(const Vec3& a, const Vec3& b, const Vec3& c)
        {
            // Calculate normal using cross product
            Vec3 cross = (b - a).Cross(c - a);

            // If the cross product is zero (points are collinear), return an invalid plane
            if (cross.LengthSquared() < T::Epsilon())
            {
                return PlaneX(); // Or handle error appropriately
            }

            Vec3 normal = cross.Normalize();
            return PlaneX(normal, normal.Dot(a));
        }
        
        /**
         * @brief Creates plane from normal and point.
         * @param normal Direction perpendicular to plane (should be normalized)
         * @param point Any point that lies on the plane
         */
        constexpr PlaneX(const Vec3& normal, const Vec3& point)
            : Normal(normal)
            , SignedDistance(normal.Dot(point))
        {}
        
        /**
         * @brief Calculates signed distance from point to plane.
         * 
         * @param point Point to test
         * @return Signed distance where:
         *         > 0: Point is on same side as normal
         *         = 0: Point is on plane
         *         < 0: Point is on opposite side from normal
         */
        constexpr T GetSignedDistance(const Vec3& point) const
        {
            return Normal.Dot(point) - SignedDistance;  // normal·X - d
        }
        
        /**
         * @brief Calculates absolute distance from point to plane.
         * 
         * @param point Point to test
         * @return Absolute distance (always positive or zero)
         */
        constexpr T GetDistance(const Vec3& point) const
        {
            return GetSignedDistance(point).Abs();
        }

        /**
         * @brief Projects point onto plane.
         * @param point Point to project
         * @return Closest point on plane to given point
         */
        constexpr Vec3 ProjectPoint(const Vec3& point) const
        {
            return point - Normal * GetSignedDistance(point);
        }
        
        /**
         * @brief Reflects a point across the plane.
         * @param point Point to reflect
         * @return Reflected point
         */
        constexpr Vec3 ReflectPoint(const Vec3& point) const
        {
            return point - Normal * (GetSignedDistance(point) * 2);
        }
        
        /**
         * @brief Reflects a direction vector across the plane's normal.
         * @param direction Direction vector to reflect
         * @return Reflected direction
         */
        constexpr Vec3 ReflectVector(const Vec3& direction) const
        {
            return direction - Normal * (Normal.Dot(direction) * 2);
        }
        
        /**
         * @brief Checks if the plane is valid (has a non-zero normal).
         * @return true if the plane is valid, false otherwise
         */
        constexpr bool IsValid() const
        {
            // Check if normal is not a zero vector
            constexpr T epsilon = T(0.0001f);
            return Normal.LengthSquared() > (epsilon * epsilon);
        }
        


        /**
         * @brief Returns a normalized copy of this plane.
         * @return New normalized plane
         */
        constexpr PlaneX Normalized() const
        {
            PlaneX result = *this;
            result.Normalize();
            return result;
        }

        /**
         * @brief Normalizes the plane equation.
         * 
         * Ensures normal is unit length while maintaining
         * the same plane equation.
         * 
         * @return Reference to this plane
         */
        constexpr PlaneX& Normalize()
        {
            T len = Normal.Length();
            if (len > 0)
            {
                Normal /= len;
                SignedDistance /= len;
            }
            return *this;
        }
    };

    using Plane = PlaneX<>;  /**< Default instantiation alias */
}