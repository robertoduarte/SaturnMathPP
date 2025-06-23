#pragma once

#include "vector3d.hpp"
#include "collision.hpp"
#include "fxp.hpp"
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
    class AABB
    {
    public:
        /**
         * @brief Creates AABB at origin with zero size.
         */
        constexpr AABB() : position(), halfExtents() {}

        /**
         * @brief Creates AABB from center and size.
         * @param center Center point
         * @param size Half-extents in each axis
         */
        constexpr AABB(const Vector3D& center, const Fxp& size)
            : position(center), halfExtents(size, size, size)
        {
        }

        /**
         * @brief Creates AABB from center and half-extents.
         * @param center Center point
         * @param halfExtents Half-extents for each axis
         */
        constexpr AABB(const Vector3D& center, const Vector3D& halfExtents)
            : position(center), halfExtents(halfExtents)
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
            Vector3D halfExtents = (maxPoint - minPoint) / 2;
            return AABB(center, halfExtents);
        }

        /**
         * @brief Gets box half-extents.
         * @return The half-extents of the AABB in each axis.
         */
        constexpr Vector3D GetHalfExtents() const { return halfExtents; }

        /**
         * @brief Check if the AABB is degenerate (has zero size in any dimension)
         * @return True if the AABB has zero size in any dimension
         */
        constexpr bool IsDegenerate() const {
            return halfExtents.X == 0 && halfExtents.Y == 0 && halfExtents.Z == 0;
        }

        /**
         * @brief Gets minimum corner point.
         */
        constexpr Vector3D GetMin() const { return position - halfExtents; }

        /**
         * @brief Gets maximum corner point.
         */
        constexpr Vector3D GetMax() const { return position + halfExtents; }

        /**
         * @brief Calculate the volume of the AABB.
         *
         * This function computes the volume of the Axis-Aligned Bounding Box (AABB) based on its half-extents.
         * The volume is calculated using the formula:
         * 
         *     Volume = width * height * depth
         * 
         * Since the half-extents represent half the dimensions of the AABB, the actual dimensions of the AABB are:
         * - Width = halfExtents.X * 2
         * - Height = halfExtents.Y * 2
         * - Depth = halfExtents.Z * 2
         * 
         * Therefore, the volume is computed as:
         * 
         *     Volume = (halfExtents.X * 2) * (halfExtents.Y * 2) * (halfExtents.Z * 2) = halfExtents.X * halfExtents.Y * halfExtents.Z * 8
         * 
         * @return The volume as an Fxp value, representing the total volume of the AABB.
         */
        constexpr Fxp GetVolume() const
        {
            return halfExtents.X * halfExtents.Y * halfExtents.Z * 8;
        }

        /**
         * @brief Calculate the surface area of the AABB.
         *
         * This function computes the surface area of the Axis-Aligned Bounding Box (AABB).
         * The surface area is calculated using the formula:
         * 
         *     Surface Area = 2 * (width * height + height * depth + depth * width)
         * 
         * Since the half-extents represent half the dimensions of the AABB, the actual dimensions of the AABB are:
         * - Width = halfExtents.X * 2
         * - Height = halfExtents.Y * 2
         * - Depth = halfExtents.Z * 2
         * 
         * Therefore, the surface area is computed as:
         * 
         *     Surface Area = 2 * ((halfExtents.X * 2) * (halfExtents.Y * 2) + (halfExtents.Y * 2) * (halfExtents.Z * 2) + (halfExtents.Z * 2) * (halfExtents.X * 2))
         *     = 8 * (halfExtents.X * halfExtents.Y + halfExtents.Y * halfExtents.Z + halfExtents.Z * halfExtents.X)
         * 
         * @return The surface area as an Fxp value, representing the total surface area of the AABB.
         */
        constexpr Fxp GetSurfaceArea() const
        {
            return (halfExtents.X * halfExtents.Y + halfExtents.Y * halfExtents.Z + halfExtents.Z * halfExtents.X) * 8;
        }

        /**
         * @brief Expand the AABB by a margin.
         * @param margin The margin to expand by (added to each half-extent).
         * @return A new expanded AABB.
         */
        constexpr AABB Expand(const Fxp& margin) const
        {
            return AABB(position, halfExtents + margin);
        }

        /**
         * @brief Create a new AABB that includes the given point.
         * @param point The point to encapsulate.
         * @return A new AABB that contains both the original AABB and the point.
         * 
         * @details If the point is already inside the AABB, returns a copy of the original AABB.
         * Otherwise, returns a new AABB that is the minimal AABB containing both the original AABB and the point.
         * 
         * Example usage:
         * @code
         * AABB box(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
         * AABB expanded = box.Encapsulate(Vector3D(2, 0, 0));
         * // expanded now goes from (-1,-1,-1) to (2,1,1)
         * @endcode
         */
        constexpr AABB Encapsulate(const Vector3D& point) const
        {
            Vector3D min = GetMin();
            Vector3D max = GetMax();

            Vector3D newMin(
                Fxp::Min(min.X, point.X),
                Fxp::Min(min.Y, point.Y),
                Fxp::Min(min.Z, point.Z)
            );

            Vector3D newMax(
                Fxp::Max(max.X, point.X),
                Fxp::Max(max.Y, point.Y),
                Fxp::Max(max.Z, point.Z)
            );

            return FromMinMax(newMin, newMax);
        }

        /**
         * @brief Create a new AABB that fully contains both this AABB and another AABB.
         * @param other The other AABB to encapsulate.
         * @return A new AABB that contains both AABBs.
         * 
         * @details If the other AABB is already fully contained within this AABB,
         * returns a copy of this AABB. Otherwise, returns the minimal AABB
         * that contains both AABBs.
         * 
         * Example usage:
         * @code
         * AABB box1(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
         * AABB box2(Vector3D(1, 1, 1), Vector3D(1, 1, 1));
         * AABB result = box1.Encapsulate(box2);
         * // result now goes from (-1,-1,-1) to (2,2,2)
         * @endcode
         */
        /**
         * @brief Create a new AABB that fully contains both this AABB and another AABB.
         * @param other The other AABB to encapsulate.
         * @return A new AABB that contains both AABBs.
         * 
         * @details Returns the minimal AABB that contains both this AABB and the other AABB.
         * This is done by computing the component-wise minimum of the minimum points
         * and the component-wise maximum of the maximum points.
         * 
         * Example usage:
         * @code
         * AABB box1(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
         * AABB box2(Vector3D(1, 1, 1), Vector3D(1, 1, 1));
         * AABB result = box1.Encapsulate(box2);
         * // result now goes from (-1,-1,-1) to (2,2,2)
         * @endcode
         */
        /**
         * @brief Compute the minimal AABB that contains both this AABB and another AABB.
         * @param other The other AABB to encapsulate.
         * @return A new AABB that is the minimal AABB containing both input AABBs.
         * 
         * @details This method computes the minimal axis-aligned bounding box that
         * contains both this AABB and the other AABB. The result is always the
         * smallest AABB that fully contains both input AABBs, regardless of whether
         * they overlap or not.
         * 
         * The algorithm works by taking the component-wise minimum of the min points
         * and the component-wise maximum of the max points of both AABBs.
         * 
         * Example usage:
         * @code
         * AABB box1(Vector3D(0, 0, 0), Vector3D(1, 1, 1));  // Box from (-1,-1,-1) to (1,1,1)
         * AABB box2(Vector3D(1, 1, 1), Vector3D(1, 1, 1));  // Box from (0,0,0) to (2,2,2)
         * AABB result = box1.Encapsulate(box2);  // Result is box from (-1,-1,-1) to (2,2,2)
         * @endcode
         */
        constexpr AABB Encapsulate(const AABB& other) const
        {
            // Get the min and max points of both AABBs
            const Vector3D thisMin = GetMin();
            const Vector3D thisMax = GetMax();
            const Vector3D otherMin = other.GetMin();
            const Vector3D otherMax = other.GetMax();
            
            // Compute the minimal AABB that contains both AABBs by taking the
            // component-wise min of the min points and component-wise max of the max points
            return AABB::FromMinMax(
                Vector3D(Fxp::Min(thisMin.X, otherMin.X),
                        Fxp::Min(thisMin.Y, otherMin.Y),
                        Fxp::Min(thisMin.Z, otherMin.Z)),
                Vector3D(Fxp::Max(thisMax.X, otherMax.X),
                        Fxp::Max(thisMax.Y, otherMax.Y),
                        Fxp::Max(thisMax.Z, otherMax.Z))
            );
        }

        /**
         * @brief Scale the AABB by a factor.
         * @param scale The scale factor.
         * @return A new scaled AABB.
         */
        constexpr AABB Scale(const Fxp& scale) const
        {
            return AABB(position, halfExtents * scale);
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
        constexpr std::array<Vector3D, 8> GetVertices() const
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
         * @brief Get the position of the AABB.
         * @return The position of the AABB.
         */
        constexpr Vector3D GetPosition() const { return position; }

        /**
         * @brief Set the position of the AABB.
         * @param pos The new position of the AABB.
         */
        constexpr void SetPosition(const Vector3D& pos) { position = pos; }

        /**
         * @brief Merge this AABB with another AABB to create a new AABB that encompasses both.
         * @param other The other AABB to merge with.
         * @return A new AABB that contains both input AABBs.
         */
        constexpr AABB Merge(const AABB& other) const
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
         * @brief Get the position of the AABB.
         * @return The position of the AABB.
         */
        constexpr Vector3D GetPosition() const
        {
            return position;
        }

        /**
         * @brief Creates an AABB that contains all points in space
         * @return AABB An infinite AABB
         */
        static constexpr AABB Infinite() {
            // Using max Fxp value that can be represented
            const Fxp maxExtent = Fxp::MaxValue();
            return AABB(Vector3D::Zero(), maxExtent);
        }

    private:
        Vector3D position;     /**< Center position of the AABB */
        Vector3D halfExtents;  /**< Half-extents in each axis (distance from center to each face) */
    };
}
