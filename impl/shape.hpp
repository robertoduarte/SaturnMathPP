#pragma once

#include "vector3d.hpp"
#include "plane.hpp"

namespace SaturnMath::Types
{
    /**
     * @brief Base class for geometric shapes.
     * 
     * Common functionality:
     * - Position (center point)
     * - Intersection tests
     * - Containment tests
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