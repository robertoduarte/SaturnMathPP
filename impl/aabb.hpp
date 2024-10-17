#pragma once

#include "vector3d.hpp"
#include "fxp.hpp"
#include "shape.hpp"

namespace SaturnMath
{
    class AABB : public Shape
    {
    public:
        /**
       * @brief Constructor for AABB.
       * @param positionIn The position of the AABB.
       * @param sizeIn The size of the AABB.
       */
        AABB(const Vector3D& positionIn, const Fxp& sizeIn)
            : Shape(positionIn), size(sizeIn)
        {
        }

        /**
         * @brief Get the minimum point of the AABB.
         * @return The minimum point of the AABB.
         */
        Vector3D GetMin() const
        {
            return position - Vector3D(size / 2.0f);
        }

        /**
         * @brief Get the maximum point of the AABB.
         * @return The maximum point of the AABB.
         */
        Vector3D GetMax() const
        {
            return position + Vector3D(size / 2.0f);
        }

        /**
         * @brief Get a vertex of the AABB.
         * @tparam getPositiveVector Set to true for the positive direction, false for the negative direction.
         * @param normal The normal vector of the plane.
         * @return The calculated vertex of the AABB.
         */
        template <bool getPositiveVector>
        Vector3D GetVertex(const Vector3D& normal) const
        {
            Fxp halfExtent = size / 2.0;

            if constexpr (!getPositiveVector)
                halfExtent = -halfExtent;

            Vector3D result;
            result.x = (normal.x >= 0.0) ? position.x + halfExtent : position.x - halfExtent;
            result.y = (normal.y >= 0.0) ? position.y + halfExtent : position.y - halfExtent;
            result.z = (normal.z >= 0.0) ? position.z + halfExtent : position.z - halfExtent;

            return result;
        }

        /**
         * @brief Check if the AABB intersects with a plane.
         * @param plane The plane to check intersection with.
         * @return True if the AABB intersects with the plane, false otherwise.
         */
        bool Intersects(const Plane& plane) const override
        {
            Vector3D vertexN = this->GetVertex<false>(plane.normal);
            return plane.Distance(vertexN) >= 0.0;
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
         * @brief Get the sizeIn of the AABB.
         * @return The sizeIn of the AABB.
         */
        Fxp GetSize() const
        {
            return size;
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
        Fxp size;
    };
}
