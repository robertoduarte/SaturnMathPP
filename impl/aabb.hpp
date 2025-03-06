#pragma once

#include "vector3d.hpp"
#include "fxp.hpp"
#include "shape.hpp"
#include <array>

namespace SaturnMath::Types
{
    /**
     * @brief Axis-Aligned Bounding Box (AABB) representation for efficient collision detection.
     * 
     * @details An AABB is a rectangular cuboid whose faces are aligned with the coordinate axes.
     * It is defined by a center point and half-extents (size) along each axis.
     * 
     * Key features of this implementation:
     * - Memory-efficient representation using only a center point and half-extents
     * - Constant-time intersection tests with other AABBs, rays, and points
     * - Optimized for performance-critical collision detection pipelines
     * - Supports both construction from min/max points and from center/size
     * - Provides methods for volume and surface area calculations
     * 
     * AABBs are primarily used for:
     * - Broad-phase collision detection in physics simulations
     * - Spatial partitioning structures (octrees, BVH trees)
     * - View frustum culling in rendering pipelines
     * - Ray-casting acceleration
     * 
     * This implementation inherits from the Shape base class, allowing it to be used
     * polymorphically with other geometric primitives in the library.
     * 
     * @note While AABBs are very efficient for collision detection, they can become
     * inefficient for rotated objects. Consider using oriented bounding boxes (OBBs)
     * for objects that undergo significant rotation.
     */
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
            : Shape(center), size(size)
        {
        }

        /**
         * @brief Creates AABB from center and half-extents.
         * @param center Center point
         * @param halfSize Half-extents for each axis
         */
        constexpr AABB(const Vector3D& center, const Vector3D& halfSize)
            : Shape(center), size(halfSize)
        {
        }

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
         *
         * This function computes the volume of the Axis-Aligned Bounding Box (AABB) based on its half-extents.
         * The volume is calculated using the formula:
         * 
         *     Volume = width * height * depth
         * 
         * Since the size represents half-extents, the actual dimensions of the AABB are:
         * - Width = size.X * 2
         * - Height = size.Y * 2
         * - Depth = size.Z * 2
         * 
         * Therefore, the volume is computed as:
         * 
         *     Volume = (size.X * 2) * (size.Y * 2) * (size.Z * 2) = size.X * size.Y * size.Z * 8
         * 
         * @return The volume as an Fxp value, representing the total volume of the AABB.
         */
        constexpr Fxp GetVolume() const
        {
            return size.X * size.Y * size.Z * 8;
        }

        /**
         * @brief Calculate the surface area of the AABB.
         *
         * This function computes the surface area of the Axis-Aligned Bounding Box (AABB).
         * The surface area is calculated using the formula:
         * 
         *     Surface Area = 2 * (width * height + height * depth + depth * width)
         * 
         * Since the size represents half-extents, the actual dimensions of the AABB are:
         * - Width = size.X * 2
         * - Height = size.Y * 2
         * - Depth = size.Z * 2
         * 
         * Therefore, the surface area is computed as:
         * 
         *     Surface Area = 2 * ((size.X * 2) * (size.Y * 2) + (size.Y * 2) * (size.Z * 2) + (size.Z * 2) * (size.X * 2))
         *     = (size.X * size.Y + size.Y * size.Z + size.Z * size.X) * 2
         * 
         * @return The surface area as an Fxp value, representing the total surface area of the AABB.
         */
        constexpr Fxp GetSurfaceArea() const
        {
            return (size.X * size.Y + size.Y * size.Z + size.Z * size.X) * 2;
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
                Fxp::Max(min.X, Fxp::Min(point.X, max.X)),
                Fxp::Max(min.Y, Fxp::Min(point.Y, max.Y)),
                Fxp::Max(min.Z, Fxp::Min(point.Z, max.Z))
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
                Vector3D(min.X, min.Y, min.Z), // 0: left bottom back
                Vector3D(max.X, min.Y, min.Z), // 1: right bottom back
                Vector3D(max.X, max.Y, min.Z), // 2: right top back
                Vector3D(min.X, max.Y, min.Z), // 3: left top back
                Vector3D(min.X, min.Y, max.Z), // 4: left bottom front
                Vector3D(max.X, min.Y, max.Z), // 5: right bottom front
                Vector3D(max.X, max.Y, max.Z), // 6: right top front
                Vector3D(min.X, max.Y, max.Z)  // 7: left top front
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
                (plane.normal.X >= 0) ? min.X : max.X,
                (plane.normal.Y >= 0) ? min.Y : max.Y,
                (plane.normal.Z >= 0) ? min.Z : max.Z
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
            return (point.X >= GetMin().X && point.X <= GetMax().X) &&
                (point.Y >= GetMin().Y && point.Y <= GetMax().Y) &&
                (point.Z >= GetMin().Z && point.Z <= GetMax().Z);
        }

        /**
         * @brief Check if a point is inside the AABB.
         * @param point The point to check.
         * @return True if the point is inside the AABB, false otherwise.
         */
        bool Contains(const Vector3D& point) const override
        {
            return ContainsPoint(point);
        }

        /**
         * @brief Merge this AABB with another AABB to create a new AABB that encompasses both.
         * @param other The other AABB to merge with.
         * @return A new AABB that contains both input AABBs.
         */
        AABB Merge(const AABB& other) const
        {
            Vector3D min = GetMin();
            Vector3D max = GetMax();

            Vector3D otherMin = other.GetMin();
            Vector3D otherMax = other.GetMax();

            Vector3D mergedMin(
                std::min(min.X, otherMin.X),
                std::min(min.Y, otherMin.Y),
                std::min(min.Z, otherMin.Z)
            );

            Vector3D mergedMax(
                std::max(max.X, otherMax.X),
                std::max(max.Y, otherMax.Y),
                std::max(max.Z, otherMax.Z)
            );

            return FromMinMax(mergedMin, mergedMax);
        }

        /**
         * @brief Check if another AABB is inside this AABB.
         * @param other The other AABB to check.
         * @return True if the other AABB is inside this AABB, false otherwise.
         */
        bool ContainsAABB(const AABB& other) const
        {
            Vector3D min = GetMin();
            Vector3D max = GetMax();
            Vector3D otherMin = other.GetMin();
            Vector3D otherMax = other.GetMax();

            return (otherMin.X >= min.X && otherMax.X <= max.X) &&
                (otherMin.Y >= min.Y && otherMax.Y <= max.Y) &&
                (otherMin.Z >= min.Z && otherMax.Z <= max.Z);
        }

        /**
         * @brief Check if this AABB intersects with another AABB.
         * @param other The other AABB to check intersection with.
         * @return True if the AABBs intersect, false otherwise.
         */
        bool IntersectsAABB(const AABB& other) const
        {
            Vector3D min = GetMin();
            Vector3D max = GetMax();
            Vector3D otherMin = other.GetMin();
            Vector3D otherMax = other.GetMax();

            return !(otherMin.X > max.X ||
                otherMax.X < min.X ||
                otherMin.Y > max.Y ||
                otherMax.Y < min.Y ||
                otherMin.Z > max.Z ||
                otherMax.Z < min.Z);
        }

        /**
         * @brief Calculate the intersection of two AABBs.
         * @param other The other AABB to intersect with.
         * @return A new AABB representing the intersection, or a zero-sized AABB if no intersection.
         */
        AABB Intersection(const AABB& other) const
        {
            Vector3D min = GetMin();
            Vector3D max = GetMax();

            Vector3D otherMin = other.GetMin();
            Vector3D otherMax = other.GetMax();

            if (!(otherMin.X > max.X ||
                otherMax.X < min.X ||
                otherMin.Y > max.Y ||
                otherMax.Y < min.Y ||
                otherMin.Z > max.Z ||
                otherMax.Z < min.Z)) {

                Vector3D intersectionMin(
                    std::max(min.X, otherMin.X),
                    std::max(min.Y, otherMin.Y),
                    std::max(min.Z, otherMin.Z)
                );

                Vector3D intersectionMax(
                    std::min(max.X, otherMax.X),
                    std::min(max.Y, otherMax.Y),
                    std::min(max.Z, otherMax.Z)
                );

                return FromMinMax(intersectionMin, intersectionMax);
            }
            return AABB();
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
        Vector3D size; /**< Half-extents in each axis */
    };
}
