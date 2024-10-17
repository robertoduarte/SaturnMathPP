#pragma once

#include "shape.hpp"

namespace SaturnMath
{
    class Sphere : public Shape
    {
    public:
        /**
         * @brief Constructor for Sphere.
         * @param positionIn The position of the sphere.
         * @param radiusIn The radius of the sphere.
         */
        Sphere(const Vec3& positionIn, const Fxp& radiusIn)
            : Shape(positionIn), radius(radiusIn)
        {
        }

        /**
         * @brief Get the radius of the sphere.
         * @return The radius of the sphere.
         */
        Fxp GetRadius() const { return radius; }

        /**
         * @brief Check if the sphere intersects with a plane.
         * @param plane The plane to check intersection with.
         * @return True if the sphere intersects with the plane, false otherwise.
         */
        bool Intersects(const Plane& plane) const override
        {
            Fxp distance = plane.Distance(position);
            return distance <= radius;
        }

    private:
        Fxp radius;
    };
}
