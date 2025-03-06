#pragma once

#include "vector3d.hpp"
#include "plane.hpp"

namespace SaturnMath::Types
{
    /**
     * @brief Base class for geometric shapes in 3D space with common functionality.
     * 
     * @details The Shape class serves as the foundation for the geometric primitive hierarchy
     * in the SaturnMath library. It provides core functionality that all shapes share:
     * - Positional data (center point)
     * - Intersection testing framework
     * - Containment testing framework
     * 
     * This abstract base class enables polymorphic usage of different shape types
     * in collision detection systems, spatial partitioning structures, and
     * physics simulations. By inheriting from Shape, derived classes gain
     * consistent interfaces for position manipulation and testing operations.
     * 
     * Key design features:
     * - Minimal memory footprint (only stores position vector)
     * - Virtual methods for shape-specific operations
     * - Consistent coordinate system across all shape types
     * - Optimized for performance-critical collision detection pipelines
     * 
     * Common derived classes include:
     * - AABB (Axis-Aligned Bounding Box)
     * - Sphere
     * - Plane
     * 
     * @note This class uses fixed-point arithmetic for all operations to ensure
     * consistent behavior across platforms and to optimize for Saturn hardware.
     */
    class Shape
    {
    public:
        /**
         * @brief Creates shape at specified position.
         * @param pos Center point
         */
        constexpr Shape(const Vector3D& pos) : position(pos) {}

        /**
         * @brief Creates shape at origin (0,0,0).
         */
        constexpr Shape() : position() {}

        /** @brief Virtual destructor for proper cleanup. */
        virtual ~Shape() = default;

        /** @brief Gets shape center position. */
        constexpr Vector3D GetPosition() const { return position; }

        /** @brief Sets shape center position. */
        constexpr void SetPosition(const Vector3D& pos) { position = pos; }

        /**
         * @brief Tests intersection with plane.
         * 
         * Each shape type defines its own intersection semantics.
         * See derived class documentation for details.
         * 
         * @param plane Plane to test against
         * @return true if shape intersects plane according to shape-specific rules
         */
        virtual bool Intersects(const Plane& plane) const = 0;

        /**
         * @brief Tests if point is inside shape.
         * 
         * Each shape defines its own containment rules.
         * See derived class documentation for details.
         * 
         * @param point Point to test
         * @return true if point is inside shape according to shape-specific rules
         */
        virtual bool Contains(const Vector3D& point) const = 0;

    protected:
        Vector3D position; /**< Center point of shape */
    };
}