#pragma once

#include "vector3d.hpp"
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
    template<int I = 16, int F = 16>
    class AABBX
    {
        using T = FixedPoint<I, F>;
    public:
        /**
         * @brief Creates AABB at origin with zero size.
         */
        constexpr AABBX() : position(), halfExtents() {}

        /**
         * @brief Creates AABB from center and size.
         * @param center Center point
         * @param size Half-extents in each axis
         */
        constexpr AABBX(const Vector3<I, F>& center, const T& size)
            : position(center), halfExtents(size.Abs(), size.Abs(), size.Abs())
        {
        }

        /**
         * @brief Creates AABB from center and half-extents.
         * @param center Center point
         * @param halfExtents Half-extents for each axis
         */
        constexpr AABBX(const Vector3<I, F>& center, const Vector3<I, F>& halfExtents)
            : position(center), halfExtents(halfExtents.X.Abs(), halfExtents.Y.Abs(), halfExtents.Z.Abs())
        {
        }

        /**
         * @brief Creates AABB from min/max points.
         * @param minPoint Minimum point (x,y,z)
         * @param maxPoint Maximum point (x,y,z)
         * @return AABB containing the points
         */
        static constexpr AABBX FromMinMax(const Vector3<I, F>& minPoint, const Vector3<I, F>& maxPoint)
        {
            Vector3<I, F> actualMin(
                T::Min(minPoint.X, maxPoint.X),
                T::Min(minPoint.Y, maxPoint.Y),
                T::Min(minPoint.Z, maxPoint.Z)
            );
            Vector3<I, F> actualMax(
                T::Max(minPoint.X, maxPoint.X),
                T::Max(minPoint.Y, maxPoint.Y),
                T::Max(minPoint.Z, maxPoint.Z)
            );
            Vector3<I, F> center = (actualMin + actualMax) / 2;
            Vector3<I, F> halfExtents = (actualMax - actualMin) / 2;
            return AABBX(center, halfExtents);
        }

        /**
         * @brief Gets box half-extents.
         * @return The half-extents of the AABB in each axis.
         */
        constexpr Vector3<I, F> GetHalfExtents() const { return halfExtents; }

        /**
         * @brief Check if the AABB is degenerate (has zero size in any dimension)
         * @return True if the AABB has zero size in any dimension
         */
        constexpr bool IsDegenerate() const {
            return halfExtents.X == 0 || halfExtents.Y == 0 || halfExtents.Z == 0;
        }

        /**
         * @brief Gets minimum corner point.
         * @return The minimum corner as a Vector3<I, F>.
         */
        constexpr Vector3<I, F> GetMin() const { return position - halfExtents; }

        /**
         * @brief Gets maximum corner point.
         * @return The maximum corner as a Vector3<I, F>.
         */
        constexpr Vector3<I, F> GetMax() const { return position + halfExtents; }

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
        constexpr T GetVolume() const
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
        constexpr T GetSurfaceArea() const
        {
            return (halfExtents.X * halfExtents.Y + halfExtents.Y * halfExtents.Z + halfExtents.Z * halfExtents.X) * 8;
        }

        /**
         * @brief Expand the AABB by a margin.
         * @param margin The margin to expand by (added to each half-extent).
         * @return A new expanded AABB.
         */
        constexpr AABBX Expand(const T& margin) const
        {
            Vector3<I, F> newHalfExtents(
                T::Max(T(0), halfExtents.X + margin),
                T::Max(T(0), halfExtents.Y + margin),
                T::Max(T(0), halfExtents.Z + margin)
            );
            return AABBX(position, newHalfExtents);
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
        constexpr AABBX Encapsulate(const Vector3<I, F>& point) const
        {
            Vector3<I, F> min = GetMin();
            Vector3<I, F> max = GetMax();

            Vector3<I, F> newMin(
                T::Min(min.X, point.X),
                T::Min(min.Y, point.Y),
                T::Min(min.Z, point.Z)
            );

            Vector3<I, F> newMax(
                T::Max(max.X, point.X),
                T::Max(max.Y, point.Y),
                T::Max(max.Z, point.Z)
            );

            return FromMinMax(newMin, newMax);
        }

        /**
         * @brief Compute the minimal AABB that contains both this AABB and another AABB.
         * @param other The other AABB to encapsulate.
         * @return A new AABB that is the minimal AABB containing both input AABBs.
         *
         * @details Takes the component-wise minimum of the min points
         * and the component-wise maximum of the max points of both AABBs.
         *
         * @code
         * AABB box1(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
         * AABB box2(Vector3D(1, 1, 1), Vector3D(1, 1, 1));
         * AABB result = box1.Encapsulate(box2);
         * // result goes from (-1,-1,-1) to (2,2,2)
         * @endcode
         */
        constexpr AABBX Encapsulate(const AABBX& other) const
        {
            // Get the min and max points of both AABBs
            const Vector3<I, F> thisMin = GetMin();
            const Vector3<I, F> thisMax = GetMax();
            const Vector3<I, F> otherMin = other.GetMin();
            const Vector3<I, F> otherMax = other.GetMax();
            
            // Compute the minimal AABB that contains both AABBs by taking the
            // component-wise min of the min points and component-wise max of the max points
            return AABBX::FromMinMax(
                Vector3<I, F>(T::Min(thisMin.X, otherMin.X),
                        T::Min(thisMin.Y, otherMin.Y),
                        T::Min(thisMin.Z, otherMin.Z)),
                Vector3<I, F>(T::Max(thisMax.X, otherMax.X),
                        T::Max(thisMax.Y, otherMax.Y),
                        T::Max(thisMax.Z, otherMax.Z))
            );
        }

        /**
         * @brief Scale the AABB by a factor.
         * @param scale The scale factor.
         * @return A new scaled AABB.
         */
        constexpr AABBX Scale(const T& scale) const
        {
            return AABBX(position, halfExtents * scale.Abs());
        }

        /**
         * @brief Gets closest point on AABB to target point.
         * @param point Target point
         * @return Closest point on AABB surface or inside
         */
        constexpr Vector3<I, F> GetClosestPoint(const Vector3<I, F>& point) const
        {
            // Clamp point to AABB bounds
            Vector3<I, F> min = GetMin();
            Vector3<I, F> max = GetMax();

            return Vector3<I, F>(
                T::Max(min.X, T::Min(point.X, max.X)),
                T::Max(min.Y, T::Min(point.Y, max.Y)),
                T::Max(min.Z, T::Min(point.Z, max.Z))
            );
        }

        /**
         * @brief Get all 8 vertices of the AABB.
         * @return Array of 8 vertices in counter-clockwise order.
         */
        constexpr std::array<Vector3<I, F>, 8> GetVertices() const
        {
            // More efficient to calculate min/max once and reuse
            Vector3<I, F> min = GetMin();
            Vector3<I, F> max = GetMax();

            return {
                Vector3<I, F>(min.X, min.Y, min.Z), // 0: left bottom back
                Vector3<I, F>(max.X, min.Y, min.Z), // 1: right bottom back
                Vector3<I, F>(max.X, max.Y, min.Z), // 2: right top back
                Vector3<I, F>(min.X, max.Y, min.Z), // 3: left top back
                Vector3<I, F>(min.X, min.Y, max.Z), // 4: left bottom front
                Vector3<I, F>(max.X, min.Y, max.Z), // 5: right bottom front
                Vector3<I, F>(max.X, max.Y, max.Z), // 6: right top front
                Vector3<I, F>(min.X, max.Y, max.Z)  // 7: left top front
            };
        }

        /**
         * @brief Set the position of the AABB.
         * @param pos The new position of the AABB.
         */
        constexpr void SetPosition(const Vector3<I, F>& pos) { position = pos; }

        /**
         * @brief Merge this AABB with another AABB to create a new AABB that encompasses both.
         * @param other The other AABB to merge with.
         * @return A new AABB that contains both input AABBs.
         */
        constexpr AABBX Merge(const AABBX& other) const
        {
            Vector3<I, F> min = GetMin();
            Vector3<I, F> max = GetMax();

            Vector3<I, F> otherMin = other.GetMin();
            Vector3<I, F> otherMax = other.GetMax();

            Vector3<I, F> mergedMin(
                T::Min(min.X, otherMin.X),
                T::Min(min.Y, otherMin.Y),
                T::Min(min.Z, otherMin.Z)
            );

            Vector3<I, F> mergedMax(
                T::Max(max.X, otherMax.X),
                T::Max(max.Y, otherMax.Y),
                T::Max(max.Z, otherMax.Z)
            );

            return FromMinMax(mergedMin, mergedMax);
        }

        /**
         * @brief Get the position of the AABB.
         * @return The position of the AABB.
         */
        constexpr Vector3<I, F> GetPosition() const
        {
            return position;
        }

        /**
         * @brief Creates an AABB that contains all points in space
         * @return AABB An infinite AABB
         */
        static constexpr AABBX Infinite() {
            // Using max value that can be represented
            const T maxExtent = T::MaxValue();
            return AABBX(Vector3<I, F>::Zero(), maxExtent);
        }

    private:
        Vector3<I, F> position;     /**< Center position of the AABB */
        Vector3<I, F> halfExtents;  /**< Half-extents in each axis (distance from center to each face) */
    };

    using AABB = AABBX<>;  /**< Default instantiation alias */
}
