#pragma once

#include "aabb.hpp"
#include "sphere.hpp"
#include "plane.hpp"

namespace SaturnMath::Collision
{
    /**
     * @brief Describes the spatial relationship between an object and a plane
     */
    enum class PlaneRelationship
    {
        Front,      // Object is completely in front of the plane
        Back,       // Object is completely behind the plane
        Intersects  // Object intersects or is coincident with the plane
    };

    /**
     * @brief Classifies the relationship between a point and a plane
     * @param point The point to test
     * @param plane The plane to test against
     * @param epsilon Tolerance for considering a point on the plane
     * @return PlaneRelationship indicating the spatial relationship
     */
    constexpr PlaneRelationship Classify(const Vector3D& point, const Plane& plane, Fxp epsilon = Fxp::Epsilon())
    {
        Fxp distance = plane.GetSignedDistance(point);
        if (distance > epsilon) return PlaneRelationship::Front;
        if (distance < -epsilon) return PlaneRelationship::Back;
        return PlaneRelationship::Intersects;
    }

    /**
     * @brief Classifies the relationship between a sphere and a plane
     * @param sphere The sphere to test
     * @param plane The plane to test against
     * @param epsilon Tolerance for considering a point on the plane
     * @return PlaneRelationship indicating the spatial relationship
     */
    constexpr PlaneRelationship Classify(const Sphere& sphere, const Plane& plane, Fxp epsilon = Fxp::Epsilon())
    {
        Fxp distance = plane.GetSignedDistance(sphere.GetPosition());
        Fxp radius = sphere.GetRadius();
        
        if (distance > radius + epsilon) return PlaneRelationship::Front;
        if (distance < -radius - epsilon) return PlaneRelationship::Back;
        return PlaneRelationship::Intersects;
    }

    /**
     * @brief Classifies the relationship between an AABB and a plane
     * @param aabb The AABB to test
     * @param plane The plane to test against
     * @param epsilon Tolerance for considering a point on the plane
     * @return PlaneRelationship indicating the spatial relationship
     * 
     * This function uses the AABB's center and half-extents to efficiently
     * determine the spatial relationship with the plane by projecting the
     * effective radius of the AABB onto the plane normal.
     */
    constexpr PlaneRelationship Classify(const AABB& aabb, const Plane& plane, Fxp epsilon = Fxp::Epsilon())
    {
        // Get the AABB's center and half-extents
        const Vector3D center = aabb.GetPosition();
        const Vector3D halfExtents = aabb.GetHalfExtents();
        
        // Project the half-extents onto the plane normal (manhattan distance)
        // This gives us the effective radius of the AABB in the plane's normal direction
        const Fxp radius = halfExtents.Dot(plane.Normal.Abs());
        
        // Calculate the signed distance from the AABB's center to the plane
        const Fxp distance = plane.GetSignedDistance(center);
        
        // Classify based on the distance and effective radius
        return (distance > radius + epsilon) ? PlaneRelationship::Front :
               (distance < -radius - epsilon) ? PlaneRelationship::Back :
               PlaneRelationship::Intersects;
    }

    /**
     * @brief Check if two AABBs intersect
     * @param a First AABB
     * @param b Second AABB
     * @return True if the AABBs intersect, false otherwise
     * 
     * This function performs an axis-aligned bounding box intersection test.
     * It's faster than OBB (Oriented Bounding Box) tests but less precise.
     */
    constexpr bool Intersects(const AABB& a, const AABB& b)
    {
        Vector3D aMin = a.GetMin();
        Vector3D aMax = a.GetMax();
        Vector3D bMin = b.GetMin();
        Vector3D bMax = b.GetMax();
        
        return (aMin.X <= bMax.X && aMax.X >= bMin.X) &&
               (aMin.Y <= bMax.Y && aMax.Y >= bMin.Y) &&
               (aMin.Z <= bMax.Z && aMax.Z >= bMin.Z);
    }

    /**
     * @brief Check if two spheres intersect
     * @param a First sphere
     * @param b Second sphere
     * @return True if the spheres intersect, false otherwise
     * 
     * This function checks if the distance between sphere centers is less than
     * the sum of their radii.
     */
    constexpr bool Intersects(const Sphere& a, const Sphere& b)
    {
        Vector3D delta = a.GetPosition() - b.GetPosition();
        Fxp radiusSum = a.GetRadius() + b.GetRadius();
        return delta.LengthSquared() <= (radiusSum * radiusSum);
    }

    /**
     * @brief Check if an AABB and a sphere intersect
     * @param aabb The axis-aligned bounding box
     * @param sphere The sphere
     * @return True if the AABB and sphere intersect, false otherwise
     * 
     * This function finds the closest point on the AABB to the sphere's center
     * and checks if it's within the sphere's radius.
     */
    constexpr bool Intersects(const AABB& aabb, const Sphere& sphere)
    {
        // Find the closest point on AABB to sphere center
        Vector3D closest = sphere.GetPosition();
        Vector3D min = aabb.GetMin();
        Vector3D max = aabb.GetMax();
        
        // Clamp sphere center to AABB bounds
        closest.X = (closest.X < min.X) ? min.X : (closest.X > max.X) ? max.X : closest.X;
        closest.Y = (closest.Y < min.Y) ? min.Y : (closest.Y > max.Y) ? max.Y : closest.Y;
        closest.Z = (closest.Z < min.Z) ? min.Z : (closest.Z > max.Z) ? max.Z : closest.Z;
        
        // Check if closest point is within sphere radius
        Vector3D delta = sphere.GetPosition() - closest;
        return delta.LengthSquared() <= (sphere.GetRadius() * sphere.GetRadius());
    }

    // Overload for Sphere-AABB case
    constexpr bool Intersects(const Sphere& sphere, const AABB& aabb)
    {
        return Intersects(aabb, sphere);
    }

    /**
     * @brief Check if an AABB intersects with a plane
     * @param aabb The axis-aligned bounding box
     * @param plane The plane
     * @return True if the AABB intersects the plane, false otherwise
     * 
     * This function uses the separating axis theorem to check for intersection
     * between an AABB and a plane.
     */
    constexpr bool Intersects(const AABB& aabb, const Plane& plane)
    {
        Vector3D min = aabb.GetMin();
        Vector3D max = aabb.GetMax();
        
        // Project the AABB center onto the plane normal
        Vector3D center = (min + max) * Fxp(0.5f);
        Fxp extentX = (max.X - min.X) * Fxp(0.5f);
        Fxp extentY = (max.Y - min.Y) * Fxp(0.5f);
        Fxp extentZ = (max.Z - min.Z) * Fxp(0.5f);
        
        // Calculate the radius of the AABB when projected onto the plane normal
        Fxp radius = extentX * plane.Normal.X.Abs() +
                   extentY * plane.Normal.Y.Abs() +
                    extentZ * plane.Normal.Z.Abs();
        
        // Calculate the distance from the AABB center to the plane
        Fxp distance = plane.GetSignedDistance(center);
        
        // Check for intersection
        return distance.Abs() <= radius;
    }

    /**
     * @brief Check if a sphere intersects with a plane
     * @param sphere The sphere
     * @param plane The plane
     * @return True if the sphere intersects the plane, false otherwise
     * 
     * This function checks if the distance from the sphere's center to the plane
     * is less than or equal to the sphere's radius.
     */
    constexpr bool Intersects(const Sphere& sphere, const Plane& plane)
    {
        Fxp distance = plane.GetSignedDistance(sphere.GetPosition());
        return distance.Abs()<= sphere.GetRadius();
    }

    // Overload for Plane-Sphere case
    constexpr bool Intersects(const Plane& plane, const Sphere& sphere)
    {
        return Intersects(sphere, plane);
    }

    // Overload for Plane-AABB case
    constexpr bool Intersects(const Plane& plane, const AABB& aabb)
    {
        return Intersects(aabb, plane);
    }

    /**
     * @brief Check if one AABB completely contains another
     * @param container The containing AABB
     * @param contained The AABB to check for containment
     * @return True if container completely contains contained, false otherwise
     */
    constexpr bool Contains(const AABB& container, const AABB& contained)
    {
        Vector3D containerMin = container.GetMin();
        Vector3D containerMax = container.GetMax();
        Vector3D containedMin = contained.GetMin();
        Vector3D containedMax = contained.GetMax();
        
        return (containedMin.X >= containerMin.X) && (containedMax.X <= containerMax.X) &&
               (containedMin.Y >= containerMin.Y) && (containedMax.Y <= containerMax.Y) &&
               (containedMin.Z >= containerMin.Z) && (containedMax.Z <= containerMax.Z);
    }

    /**
     * @brief Check if an AABB contains a sphere
     * @param aabb The containing AABB
     * @param sphere The sphere to check for containment
     * @return True if the AABB completely contains the sphere, false otherwise
     */
    constexpr bool Contains(const AABB& aabb, const Sphere& sphere)
    {
        Vector3D min = aabb.GetMin();
        Vector3D max = aabb.GetMax();
        Vector3D center = sphere.GetPosition();
        Fxp radius = sphere.GetRadius();
        
        return (center.X - radius >= min.X) && (center.X + radius <= max.X) &&
               (center.Y - radius >= min.Y) && (center.Y + radius <= max.Y) &&
               (center.Z - radius >= min.Z) && (center.Z + radius <= max.Z);
    }

    /**
     * @brief Check if a sphere contains an AABB
     * @param sphere The containing sphere
     * @param aabb The AABB to check for containment
     * @return True if the sphere completely contains the AABB, false otherwise
     */
    constexpr bool Contains(const Sphere& sphere, const AABB& aabb)
    {
        // Find the point on the AABB that's farthest from the sphere's center
        Vector3D farthestPoint = aabb.GetMin();
        Vector3D center = sphere.GetPosition();
        
        if (center.X - aabb.GetMin().X < aabb.GetMax().X - center.X)
            farthestPoint.X = aabb.GetMax().X;
        if (center.Y - aabb.GetMin().Y < aabb.GetMax().Y - center.Y)
            farthestPoint.Y = aabb.GetMax().Y;
        if (center.Z - aabb.GetMin().Z < aabb.GetMax().Z - center.Z)
            farthestPoint.Z = aabb.GetMax().Z;
            
        // Check if the farthest point is inside the sphere
        return (farthestPoint - center).LengthSquared() <= (sphere.GetRadius() * sphere.GetRadius());
    }

    /**
     * @brief Check if one sphere contains another
     * @param container The containing sphere
     * @param contained The sphere to check for containment
     * @return True if container completely contains contained, false otherwise
     */
    constexpr bool Contains(const Sphere& container, const Sphere& contained)
    {
        Fxp centerDistance = (container.GetPosition() - contained.GetPosition()).Length();
        return centerDistance + contained.GetRadius() <= container.GetRadius();
    }

    /**
     * @brief Check if a point lies on a plane
     * @param point The point to check
     * @param plane The plane to check against
     * @return True if the point lies on the plane (within floating-point epsilon), false otherwise
     * 
     * This function checks if the signed distance from the point to the plane is within
     * the floating-point epsilon threshold, meaning the point is effectively on the plane.
     */
    constexpr bool Intersects(const Vector3D& point, const Plane& plane)
    {
        // A point is on the plane if its distance to the plane is within epsilon
        Fxp distance = plane.GetSignedDistance(point);
        return distance.Abs() <= Fxp::Epsilon();
    }

    /**
     * @brief Check if a point lies on a plane (overload for plane-point order)
     * @param plane The plane to check against
     * @param point The point to check
     * @return True if the point lies on the plane (within floating-point epsilon), false otherwise
     */
    constexpr bool Intersects(const Plane& plane, const Vector3D& point)
    {
        return Intersects(point, plane);
    }

    /**
     * @brief Check if a point is inside or on an AABB
     * @param point The point to check
     * @param aabb The AABB to check against
     * @return True if the point is inside or on the AABB, false otherwise
     * 
     * This function checks if the point's coordinates are within the AABB's bounds,
     * including points exactly on the AABB's faces.
     */
    constexpr bool Intersects(const Vector3D& point, const AABB& aabb)
    {
        Vector3D min = aabb.GetMin();
        Vector3D max = aabb.GetMax();
        
        return (point.X >= min.X) && (point.X <= max.X) &&
               (point.Y >= min.Y) && (point.Y <= max.Y) &&
               (point.Z >= min.Z) && (point.Z <= max.Z);
    }

    /**
     * @brief Check if a point is inside or on an AABB (overload for AABB-point order)
     * @param aabb The AABB to check against
     * @param point The point to check
     * @return True if the point is inside or on the AABB, false otherwise
     */
    constexpr bool Intersects(const AABB& aabb, const Vector3D& point)
    {
        return Intersects(point, aabb);
    }

    /**
     * @brief Check if a point is inside or on a sphere
     * @param point The point to check
     * @param sphere The sphere to check against
     * @return True if the point is inside or on the sphere, false otherwise
     * 
     * This function checks if the squared distance from the point to the sphere's center
     * is less than or equal to the square of the sphere's radius.
     */
    constexpr bool Intersects(const Vector3D& point, const Sphere& sphere)
    {
        Vector3D delta = point - sphere.GetPosition();
        Fxp distanceSq = delta.LengthSquared();
        Fxp radiusSq = sphere.GetRadius() * sphere.GetRadius();
        return distanceSq <= radiusSq;
    }

    /**
     * @brief Check if a point is inside or on a sphere (overload for sphere-point order)
     * @param sphere The sphere to check against
     * @param point The point to check
     * @return True if the point is inside or on the sphere, false otherwise
     */
    constexpr bool Intersects(const Sphere& sphere, const Vector3D& point)
    {
        return Intersects(point, sphere);
    }

    /**
     * @brief Check if a point is inside or on an AABB
     * @param aabb The AABB to check against
     * @param point The point to check
     * @return True if the point is inside or on the AABB, false otherwise
     * 
     * This is an alias for Intersects(point, aabb) for consistency.
     */
    constexpr bool Contains(const AABB& aabb, const Vector3D& point)
    {
        return Intersects(point, aabb);
    }

    /**
     * @brief Check if a point is inside or on a sphere
     * @param sphere The sphere to check against
     * @param point The point to check
     * @return True if the point is inside or on the sphere, false otherwise
     * 
     * This is an alias for Intersects(point, sphere) for consistency.
     */
    constexpr bool Contains(const Sphere& sphere, const Vector3D& point)
    {
        return Intersects(point, sphere);
    }

    // Add more collision functions as needed...
}
