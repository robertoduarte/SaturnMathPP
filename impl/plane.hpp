#pragma once

#include "vector3d.hpp"

namespace SaturnMath
{
    /**
     * @brief Represents a 3D plane.
     */
    class Plane
    {
    public:
        Vector3D normal; /**< Normal vector of the plane. */
        Fxp d;     /**< Offset or signed distance of the plane. */

        /**
         * @brief Default constructor.
         */
        Plane() : normal(), d() {}

        /**
         * @brief Constructor with a given normal vector and offset.
         */
        Plane(const Vector3D& normal, Fxp d) : normal(normal), d(d) {}

        /**
         * @brief Constructor with a given normal vector and a point on the plane.
         */
        Plane(const Vector3D& normal, const Vector3D& position) : normal(normal), d(normal.Dot(position)) {}

        /**
         * @brief Constructor with three vertices defining the plane.
         */
        Plane(const Vector3D& vertexA, const Vector3D& vertexB, const Vector3D& vertexC)
        {
            normal = Vector3D::CalcNormal(vertexA, vertexB, vertexC);
            d = normal.Dot(vertexB);
        }

        /**
         * @brief Calculate the signed distance from a point to the plane.
         * @param point The point in 3D space.
         * @return The signed distance from the point to the plane.
         */
        Fxp Distance(const Vector3D& point) const
        {
            return d - normal.Dot(point);
        }
    };
}