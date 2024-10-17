#pragma once

#include "vec3.hpp"
#include "plane.hpp"

namespace SaturnMath
{
    /**
     * @brief Represents a shape in 3D space.
     */
    class Shape {
    public:
    /**
     * @brief Constructor for Shape.
     * @param positionIn The position of the shape.
     */
    Shape(const Vec3& positionIn) : position(positionIn) {}

    /**
     * @brief Get the position of the shape.
     * @return The position of the shape.
     */
    Vec3 GetPosition() const { return position; }

    /**
     * @brief Check if the shape intersects with a plane.
     * @param plane The plane to check intersection with.
     * @return True if the shape intersects with the plane, false otherwise.
     */
    virtual bool Intersects(const Plane& plane) const = 0;

    protected:
        Vec3 position;
    };
}   