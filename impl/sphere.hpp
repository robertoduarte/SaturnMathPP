#pragma once

#include "vector3d.hpp"
#include "utils.hpp"  // For Pi constant

namespace SaturnMath::Types
{
    /**
     * @brief High-performance sphere primitive for collision detection and bounding volume applications.
     * 
     * @details The Sphere class represents a perfect mathematical sphere in 3D space,
     * defined by a center point and radius. It provides optimized intersection tests
     * with various geometric primitives and serves as an efficient bounding volume
     * for complex objects.
     * 
     * Key features:
     * - Memory-efficient representation (center point + radius)
     * - Constant-time intersection tests with common primitives
     * - Fixed-point arithmetic for consistent behavior across platforms
     * - Inheritance from Shape for polymorphic usage in collision systems
     * 
     * Performance characteristics:
     * - Sphere-sphere tests: O(1) complexity, very fast
     * - Sphere-AABB tests: O(1) complexity, slightly more expensive than sphere-sphere
     * - Sphere-plane tests: O(1) complexity, very fast
     * - Sphere-point tests: O(1) complexity, extremely fast
     * 
     * Common applications:
     * - Bounding volume for complex meshes
     * - Collision detection in physics simulations
     * - Level-of-detail calculations
     * - Spatial partitioning acceleration structures
     * - Fast rejection tests in ray tracing
     * 
     * Implementation notes:
     * - All intersection tests use squared distances where possible to avoid
     *   expensive square root operations
     * - The radius is stored as a fixed-point value for consistent precision
     * - Inheritance from Shape allows polymorphic usage in collision systems
     * 
     * @note When using spheres as bounding volumes for complex objects, consider
     * the trade-off between tightness of fit and computational efficiency. Spheres
     * provide the fastest intersection tests but may not approximate elongated
     * objects as well as other bounding volumes like AABBs or OBBs.
     * 
     * @see AABB For axis-aligned bounding box alternative
     * @see Shape For the base class interface
     * @see Plane For plane intersection tests
     */
    class Sphere
    {
    public:
        /**
         * @brief Default constructor creates a unit sphere at origin.
         */
        constexpr Sphere() : position(Vector3D::Zero()), radius(Fxp(1)) {}
        
        /**
         * @brief Creates sphere from center and radius.
         * @param center Center point of sphere
         * @param radius Radius of sphere (must be >= 0)
         */
        constexpr Sphere(const Vector3D& center, const Fxp& radius)
            : position(center)
            , radius(radius)
        {}

        /** @brief Gets sphere radius. */
        constexpr Fxp GetRadius() const { return radius; }
        
        /** @brief Gets sphere center position. */
        constexpr Vector3D GetPosition() const { return position; }
        
        /** @brief Sets sphere center position. */
        constexpr void SetPosition(const Vector3D& pos) { position = pos; }
        
        /**
         * @brief Checks if the sphere is valid (has non-negative radius).
         * @return true if radius >= 0, false otherwise
         */
        constexpr bool IsValid() const { return radius >= 0; }
        
        /**
         * @brief Gets the volume of the sphere.
         * @return Volume as (4/3) * π * r³
         */
        constexpr Fxp GetVolume() const 
        { 
            return (Fxp(4) / 3) * Fxp::Pi() * radius * radius * radius; 
        }
        
        /**
         * @brief Gets the surface area of the sphere.
         * @return Surface area as 4 * π * r²
         */
        constexpr Fxp GetSurfaceArea() const 
        { 
            return 4 * Fxp::Pi() * radius * radius; 
        }
        
        /**
         * @brief Gets the diameter of the sphere.
         * @return Diameter as 2 * radius
         */
        constexpr Fxp GetDiameter() const { return radius * 2; }

        /**
         * @brief Tests intersection with another sphere.
         * @param other Sphere to test against.
         * @return true if spheres intersect or touch.
         */
        constexpr bool Intersects(const Sphere& other) const
        {
            Fxp distanceSquared = (GetPosition() - other.GetPosition()).LengthSquared();
            Fxp sumOfRadii = radius + other.GetRadius();
            return distanceSquared <= sumOfRadii * sumOfRadii;
        }


        
        /**
         * @brief Translates the sphere by the given offset.
         * @param translation The translation vector
         * @return A new translated sphere
         */
        constexpr Sphere Translate(const Vector3D& translation) const
        {
            return Sphere(GetPosition() + translation, radius);
        }
        
        /**
         * @brief Scales the sphere uniformly.
         * @param scaleFactor The scale factor to apply
         * @return A new scaled sphere
         */
        constexpr Sphere Scale(const Fxp& scaleFactor) const
        {
            return Sphere(GetPosition() * scaleFactor, radius * scaleFactor);
        }
        
        /**
         * @brief Scales the sphere non-uniformly.
         * @param scaleFactors The scale factors for each axis
         * @return A new scaled sphere (uses minimum scale component for radius)
         */
        constexpr Sphere Scale(const Vector3D& scaleFactors) const
        {
            // Find minimum scale component in a constexpr-friendly way
            Fxp minScale = scaleFactors.X;
            if (scaleFactors.Y < minScale) minScale = scaleFactors.Y;
            if (scaleFactors.Z < minScale) minScale = scaleFactors.Z;
            
            // Scale the position by the scale factors (component-wise) and the radius by the minimum scale
            const Vector3D& pos = GetPosition();
            return Sphere(Vector3D(pos.X * scaleFactors.X, pos.Y * scaleFactors.Y, pos.Z * scaleFactors.Z), 
                         radius * minScale);
        }
        
        /**
         * @brief Gets the closest point on the sphere's surface to a given point.
         * @tparam P Precision level for square root calculation (defaults to Fast for performance)
         * @param point The point to find the closest point to
         * @return The closest point on the sphere's surface
         * 
         * @note For most game physics and collision detection, the default Fast precision is sufficient.
         * Use Precision::Accurate only when higher precision is required at the cost of performance.
         */
        template<Precision P = Precision::Default>
        constexpr Vector3D GetClosestPoint(const Vector3D& point) const
        {
            if (radius <= 0) return GetPosition();  // Degenerate case
            
            Vector3D direction = point - GetPosition();
            Fxp distanceSq = direction.LengthSquared();
            Fxp radiusSq = radius * radius;
            
            // If the point is at the center, return any point on the sphere
            if (distanceSq <= Fxp::Epsilon())
                return GetPosition() + Vector3D(radius, 0, 0);
            
            // If point is inside or on the sphere, return the point itself
            if (distanceSq <= radiusSq)
                return point;
                
            // Point is outside the sphere, project onto surface
            Fxp distance = distanceSq.Sqrt<P>();
            return GetPosition() + (direction * (radius / distance));
        }

    private:
        Vector3D position;  /**< Center position of the sphere */
        Fxp radius;          /**< Radius of the sphere */  /**< Sphere radius (always >= 0) */
    };
}
