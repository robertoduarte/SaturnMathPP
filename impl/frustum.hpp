#pragma once

#include "plane.hpp"
#include "mat43.hpp"
#include "aabb.hpp"
#include "sphere.hpp"

namespace SaturnMath
{
    /**
     * @brief Represents a view frustum in 3D space.
     */
    class Frustum
    {
    private:
        // Named constants for plane indices
        static constexpr size_t PLANE_NEAR = 0;
        static constexpr size_t PLANE_FAR = 1;
        static constexpr size_t PLANE_TOP = 2;
        static constexpr size_t PLANE_BOTTOM = 3;
        static constexpr size_t PLANE_LEFT = 4;
        static constexpr size_t PLANE_RIGHT = 5;
        static constexpr size_t PLANE_COUNT = 6;

        Plane plane[PLANE_COUNT];

        Fxp nearDistance;   /**< Near clipping plane distance. */
        Fxp farDistance;    /**< Far clipping plane distance. */
        Fxp farWidth;       /**< Width of the far plane. */
        Fxp farHeight;      /**< Height of the far plane. */

    public:
        /**
         * @brief Constructor to initialize the frustum.
         * @param verticalFov Vertical field of view.
         * @param ratio Aspect ratio.
         * @param nearDistance Near clipping plane distance.
         * @param farDistance Far clipping plane distance.
         */
        Frustum(const Fxp& verticalFov, const Fxp& ratio, const Fxp& nearDistance, const Fxp& farDistance)
            : farHeight(Trigonometry::Tan(verticalFov)),
            farWidth(farHeight* ratio),
            nearDistance(nearDistance),
            farDistance(farDistance)
        {
        }


        /**
         * @brief Update the frustum based on the view matrix.
         * @param viewMatrix The 4x3 view matrix representing the camera's transformation.
         */
        void Update(const Mat43& viewMatrix)
        {
            Vector3D xAxis = viewMatrix.row0;
            Vector3D yAxis = viewMatrix.row1;
            Vector3D zAxis = viewMatrix.row2;
            Vector3D position = viewMatrix.row3;

            Vector3D farCentre(position + zAxis);
            Vector3D farHalfHeight(yAxis * farHeight);
            Vector3D farHalfWidth(yAxis * farWidth);

            Vector3D farTop(farCentre + farHalfHeight);
            Vector3D farTopLeft(farTop - farHalfWidth);
            Vector3D farTopRight(farTop + farHalfWidth);

            Vector3D farBottom(farCentre - farHalfHeight);
            Vector3D farBottomRight(farBottom + farHalfWidth);
            Vector3D farBottomLeft(farBottom - farHalfWidth);

            plane[PLANE_NEAR] = Plane(-zAxis, position + zAxis * nearDistance);
            plane[PLANE_FAR] = Plane(zAxis, position + zAxis * farDistance);
            plane[PLANE_TOP] = Plane(farTopRight, position, farTopLeft);
            plane[PLANE_BOTTOM] = Plane(farBottomLeft, position, farBottomRight);
            plane[PLANE_LEFT] = Plane(farTopLeft, position, farBottomLeft);
            plane[PLANE_RIGHT] = Plane(farBottomRight, position, farTopRight);
        }

        /**
         * @brief Check if a point is inside the frustum.
         * @param point The point to check.
         * @return True if the point is inside the frustum, false otherwise.
         */
        bool Contains(const Vector3D& point) const
        {
            for (size_t i = 0; i < PLANE_COUNT; i++)
            {
                if (plane[i].Distance(point) < 0.0)
                    return false;
            }
            return true;
        }

        /**
       * @brief Check if an sphere is inside the frustum.
       * @param sphere The Sphere to check.
       * @return True if the Sphere is inside the frustum, false otherwise.
       */
        bool Contains(const Sphere& sphere) const
        {
            for (size_t i = 0; i < PLANE_COUNT; i++)
            {
                if (sphere.Intersects(plane[i]) == false)
                    return false;
            }

            return true;
        }

        /**
         * @brief Check if an axis-aligned box is inside the frustum.
         * @param other The AABB to check.
         * @return True if the AABB is inside the frustum, false otherwise.
         */
        bool Contains(const AABB& aabb) const
        {
            Vector3D vertexN;
            for (size_t i = 0; i < PLANE_COUNT; i++)
            {
                if (aabb.Intersects(plane[i]) == false)
                    return false;
            }
            return true;
        }
    };
}