#pragma once

#include "vector3d.hpp"
#include "fxp.hpp"
#include "shape.hpp"
#include <array>

namespace SaturnMath
{
    class AABB : public Shape
    {
    public:
        /**
         * @brief Creates AABB at origin with zero size.
         */
        constexpr AABB() : Shape(), size() {}

        /**
         * @brief Creates AABB from center and size.
         * @param center Center point
         * @param size Half-extents in each axis
         */
        constexpr AABB(const Vector3D& center, const Fxp& size)
            : Shape(center), size(size) {}

        /**
         * @brief Creates AABB from center and half-extents.
         * @param center Center point
         * @param halfSize Half-extents for each axis
         */
        constexpr AABB(const Vector3D& center, const Vector3D& halfSize)
            : Shape(center), size(halfSize) {}

        /**
         * @brief Creates AABB from min/max points.
         * @param minPoint Minimum point (x,y,z)
         * @param maxPoint Maximum point (x,y,z)
         * @return AABB containing the points
         */
        static constexpr AABB FromMinMax(const Vector3D& minPoint, const Vector3D& maxPoint)
        {
            Vector3D center = (minPoint + maxPoint) / 2;
            Vector3D size = (maxPoint - minPoint) / 2;
            return AABB(center, size);
        }

        /**
         * @brief Gets box half-size.
         */
        constexpr Vector3D GetSize() const { return size; }

        /**
         * @brief Gets minimum corner point.
         */
        constexpr Vector3D GetMin() const { return position - size; }

        /**
         * @brief Gets maximum corner point.
         */
        constexpr Vector3D GetMax() const { return position + size; }

        /**
         * @brief Calculate the volume of the AABB.
         * @return The volume as an Fxp value.
         */
        constexpr Fxp GetVolume() const
        {
            return 8 * size.x * size.y * size.z;
        }

        /**
         * @brief Calculate the surface area of the AABB.
         * @return The surface area as an Fxp value.
         */
        constexpr Fxp GetSurfaceArea() const
        {
            return 6 * size * size;
        }

        /**
         * @brief Expand the AABB by a margin.
         * @param margin The margin to expand by.
         * @return A new expanded AABB.
         */
        constexpr AABB Expand(const Fxp& margin) const
        {
            return AABB(position, size + (margin * 2));
        }

        /**
         * @brief Scale the AABB by a factor.
         * @param scale The scale factor.
         * @return A new scaled AABB.
         */
        constexpr AABB Scale(const Fxp& scale) const
        {
            return AABB(position, size * scale);
        }

        /**
         * @brief Gets closest point on AABB to target point.
         * @param point Target point
         * @return Closest point on AABB surface or inside
         */
        constexpr Vector3D GetClosestPoint(const Vector3D& point) const
        {
            // Clamp point to AABB bounds
            Vector3D min = GetMin();
            Vector3D max = GetMax();
            
            return Vector3D(
                Fxp::Max(min.x, Fxp::Min(point.x, max.x)),
                Fxp::Max(min.y, Fxp::Min(point.y, max.y)),
                Fxp::Max(min.z, Fxp::Min(point.z, max.z))
            );
        }

        /**
         * @brief Get all 8 vertices of the AABB.
         * @return Array of 8 vertices in counter-clockwise order.
         */
        std::array<Vector3D, 8> GetVertices() const
        {
            // More efficient to calculate min/max once and reuse
            Vector3D min = GetMin();
            Vector3D max = GetMax();
            
            return {
                Vector3D(min.x, min.y, min.z), // 0: left bottom back
                Vector3D(max.x, min.y, min.z), // 1: right bottom back
                Vector3D(max.x, max.y, min.z), // 2: right top back
                Vector3D(min.x, max.y, min.z), // 3: left top back
                Vector3D(min.x, min.y, max.z), // 4: left bottom front
                Vector3D(max.x, min.y, max.z), // 5: right bottom front
                Vector3D(max.x, max.y, max.z), // 6: right top front
                Vector3D(min.x, max.y, max.z)  // 7: left top front
            };
        }

        /**
         * @brief Check if the AABB intersects with a plane.
         * @param plane The plane to check intersection with.
         * @return True if the AABB intersects with the plane, false otherwise.
         */
        bool Intersects(const Plane& plane) const override
        {
            // Find the vertex that's furthest in the opposite direction of the plane normal
            Vector3D min = GetMin();
            Vector3D max = GetMax();
            Vector3D vertexN(
                (plane.normal.x >= 0) ? min.x : max.x,
                (plane.normal.y >= 0) ? min.y : max.y,
                (plane.normal.z >= 0) ? min.z : max.z
            );
            return plane.Distance(vertexN) >= 0;
        }

        /**
         * @brief Check if a point is inside the AABB.
         * @param point The point to check.
         * @return True if the point is inside the AABB, false otherwise.
         */
        bool ContainsPoint(const Vector3D& point) const
        {
            return (point.x >= GetMin().x && point.x <= GetMax().x) &&
                   (point.y >= GetMin().y && point.y <= GetMax().y) &&
                   (point.z >= GetMin().z && point.z <= GetMax().z);
        }

        /**
         * @brief Merge this AABB with another AABB to create a new AABB that encompasses both.
         * @param other The other AABB to merge with.
         * @return A new AABB that contains both input AABBs.
         */
        AABB Merge(const AABB& other) const
        {
            Vector3D min(
                std::min(GetMin().x, other.GetMin().x),
                std::min(GetMin().y, other.GetMin().y),
                std::min(GetMin().z, other.GetMin().z)
            );
            
            Vector3D max(
                std::max(GetMax().x, other.GetMax().x),
                std::max(GetMax().y, other.GetMax().y),
                std::max(GetMax().z, other.GetMax().z)
            );
            
            return FromMinMax(min, max);
        }

        /**
         * @brief Check if another AABB is inside this AABB.
         * @param other The other AABB to check.
         * @return True if the other AABB is inside this AABB, false otherwise.
         */
        bool ContainsAABB(const AABB& other) const
        {
            return (other.GetMin().x >= GetMin().x && other.GetMax().x <= GetMax().x) &&
                   (other.GetMin().y >= GetMin().y && other.GetMax().y <= GetMax().y) &&
                   (other.GetMin().z >= GetMin().z && other.GetMax().z <= GetMax().z);
        }

        /**
         * @brief Check if this AABB intersects with another AABB.
         * @param other The other AABB to check intersection with.
         * @return True if the AABBs intersect, false otherwise.
         */
        bool IntersectsAABB(const AABB& other) const
        {
            return !(other.GetMin().x > GetMax().x || 
                    other.GetMax().x < GetMin().x ||
                    other.GetMin().y > GetMax().y ||
                    other.GetMax().y < GetMin().y ||
                    other.GetMin().z > GetMax().z ||
                    other.GetMax().z < GetMin().z);
        }

        /**
         * @brief Calculate the intersection of two AABBs.
         * @param other The other AABB to intersect with.
         * @return A new AABB representing the intersection, or a zero-sized AABB if no intersection.
         */
        AABB Intersection(const AABB& other) const
        {
            if (!IntersectsAABB(other))
                return AABB();

            Vector3D min(
                std::max(GetMin().x, other.GetMin().x),
                std::max(GetMin().y, other.GetMin().y),
                std::max(GetMin().z, other.GetMin().z)
            );
            
            Vector3D max(
                std::min(GetMax().x, other.GetMax().x),
                std::min(GetMax().y, other.GetMax().y),
                std::min(GetMax().z, other.GetMax().z)
            );
            
            return FromMinMax(min, max);
        }

        /**
         * @brief Get the position of the AABB.
         * @return The position of the AABB.
         */
        Vector3D GetPosition() const
        {
            return position;
        }

    private:
        Vector3D size;
    };
}
