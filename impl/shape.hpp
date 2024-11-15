#pragma once

#include "vector3d.hpp"
#include "plane.hpp"

namespace SaturnMath
{
    /**
     * @brief Abstract base class for 3D geometric shapes.
     * 
     * Provides common functionality for all shapes:
     * - Position in 3D space
     * - Core intersection tests
     * 
     * All derived shapes must implement:
     * - Intersection with planes
     * - Point containment test
     */
    class Shape
    {
    public:
        /**
         * @brief Creates shape at specified position.
         * @param center Center point of shape
         */
        constexpr Shape(const Vector3D& center) noexcept : position(center) {}

        /** @brief Virtual destructor for proper cleanup. */
        virtual ~Shape() = default;

        /** @brief Gets shape's center position. */
        constexpr const Vector3D& GetPosition() const noexcept { return position; }

        /**
         * @brief Sets shape's center position.
         * @param newPos New center position
         */
        constexpr void SetPosition(const Vector3D& newPos) noexcept { position = newPos; }

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
        Vector3D position;  /**< Center point of shape */
    };
}