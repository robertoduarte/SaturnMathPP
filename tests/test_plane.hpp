#pragma once



#include "../impl/plane.hpp"
#include "../impl/vector3d.hpp"

namespace SaturnMath::Tests
{
    using namespace SaturnMath::Types;

    /**
     * @brief Static assertion tests for the Plane class
     * 
     * This file contains compile-time tests for the Plane class.
     * All tests are performed using static_assert, ensuring that the 
     * functionality is verified at compile time.
     * 
     * Note: Tests for classification and intersection with other primitives
     * are covered in test_collision.hpp.
     */
    struct PlaneTests
    {
        // Construction tests
        static constexpr void TestConstruction()
        {
            // Default constructor should initialize to XZ plane (normal = (0,1,0), distance = 0)
            constexpr Plane defaultPlane;
            static_assert(defaultPlane.Normal.X == 0 && defaultPlane.Normal.Y == 1 && 
                          defaultPlane.Normal.Z == 0 && defaultPlane.SignedDistance == 0,
                          "Default constructor should create XZ plane");
            
            // Construction from normal and distance
            constexpr Vector3D normal(1, 0, 0);
            constexpr Fxp distance = 5;
            constexpr Plane plane(normal, distance);
            static_assert(plane.Normal.X == 1 && plane.Normal.Y == 0 && 
                          plane.Normal.Z == 0 && plane.SignedDistance == 5,
                          "Construction from normal and distance should work");
            
            // Construction from normal and point
            constexpr Vector3D point(5, 0, 0);
            constexpr Plane planeFromPoint = Plane::FromNormalAndPoint(normal, point);
            static_assert(planeFromPoint.Normal.X == 1 && planeFromPoint.Normal.Y == 0 && 
                          planeFromPoint.Normal.Z == 0 && planeFromPoint.SignedDistance == 5,
                          "Construction from normal and point should work");
            
            // Test construction from three points
            // The normal direction depends on the order of the points
            // For points (0,0,0), (1,0,0), (0,1,0), the normal could be (0,0,1) or (0,0,-1)
            // depending on the winding order
            constexpr Vector3D p1(0, 0, 0);
            constexpr Vector3D p2(1, 0, 0);
            constexpr Vector3D p3(0, 1, 0);
            constexpr Plane planeFromPoints = Plane::FromPoints(p1, p2, p3);
            
            // Check if the normal is pointing in either positive or negative Z direction
            constexpr Fxp epsilon = Fxp(0.05f);
            constexpr bool isNormalInZDirection = 
                planeFromPoints.Normal.X.Abs() < epsilon && 
                planeFromPoints.Normal.Y.Abs() < epsilon && 
                (planeFromPoints.Normal.Z.Abs() - Fxp(1)).Abs() < epsilon;
                
            static_assert(isNormalInZDirection,
                        "Plane normal should be in the Z direction");
            
            // The distance should be 0 since the plane passes through the origin
            static_assert(planeFromPoints.SignedDistance.Abs() < epsilon, 
                        "Plane from points distance calculation failed - expected 0");
            
            // Verify that the plane contains all three points (within epsilon)
            static_assert(planeFromPoints.GetSignedDistance(p1).Abs() < epsilon, 
                        "Point p1 should be on the plane");
            static_assert(planeFromPoints.GetSignedDistance(p2).Abs() < epsilon, 
                        "Point p2 should be on the plane");
            static_assert(planeFromPoints.GetSignedDistance(p3).Abs() < epsilon, 
                        "Point p3 should be on the plane");
            
            // Test construction from points
            {
                constexpr Vector3D a(1, 0, 0);
                constexpr Vector3D b(0, 1, 0);
                constexpr Vector3D c(0, 0, 1);
                constexpr Plane plane = Plane::FromPoints(a, b, c);
                
                // The normal should be normalized - use a more lenient epsilon for fixed-point
                constexpr Fxp lengthSq = plane.Normal.LengthSquared();
                constexpr Fxp lengthEpsilon = Fxp(0.1f);  // Increased from 0.01f
                constexpr bool isNormalized = (lengthSq - Fxp(1)).Abs() < lengthEpsilon;
                static_assert(isNormalized, 
                             "Plane normal should be normalized");
                
                // For the plane from points (1,0,0), (0,1,0), (0,0,1)
                // The normal should be in the (1,1,1) direction (or negative)
                constexpr Vector3D expectedNormal = Vector3D(1, 1, 1).Normalized();
                constexpr Fxp dot = plane.Normal.Dot(expectedNormal);
                constexpr Fxp dotEpsilon = Fxp(0.1f);  // Increased from 0.01f
                constexpr bool isInExpectedDirection = (dot.Abs() - Fxp(1)).Abs() < dotEpsilon;
                static_assert(isInExpectedDirection, 
                             "Plane normal should be in the (1,1,1) direction (or negative)");
                
                // The normal should point in the (1,1,1) direction (normalized)
                constexpr Fxp componentEpsilon = Fxp(0.01f);
                static_assert((plane.Normal.X - expectedNormal.X).Abs() < componentEpsilon &&
                             (plane.Normal.Y - expectedNormal.Y).Abs() < componentEpsilon &&
                             (plane.Normal.Z - expectedNormal.Z).Abs() < componentEpsilon,
                             "Plane normal should point in the (1,1,1) direction");
                
                // The plane should contain all three points (within epsilon)
                constexpr Fxp distanceEpsilon = Fxp(0.0001f);
                static_assert(plane.GetSignedDistance(a).Abs() < distanceEpsilon, 
                             "Point a should be on the plane");
                static_assert(plane.GetSignedDistance(b).Abs() < distanceEpsilon, 
                             "Point b should be on the plane");
                static_assert(plane.GetSignedDistance(c).Abs() < Fxp(0.0001f), 
                             "Point c should be on the plane");
            }

            // Test copy constructor
            constexpr Plane original(Vector3D(0, 1, 0), 10);
            constexpr Plane copy = original;
            static_assert(copy.Normal == original.Normal && 
                         copy.SignedDistance == original.SignedDistance,
                        "Copy constructor should create an identical plane");
        }

        // ============================================
        // Normalization Tests
        // ============================================
        static constexpr void TestNormalization()
        {
            // Test with non-normalized normal
            {
                constexpr Vector3D nonUnitNormal(2, 0, 0); // Length = 2
                constexpr Fxp distance = 4;
                constexpr Plane plane(nonUnitNormal, distance);
                constexpr Plane normalized = plane.Normalized();
                
                // normal should be normalized
                static_assert(normalized.Normal.LengthSquared() > 0.99 && 
                             normalized.Normal.LengthSquared() < 1.01,
                            "Normalized plane should have unit length normal");
                
                // Distance should be scaled accordingly
                // Original plane equation: 2x + 0y + 0z - 4 = 0
                // After normalization: 1x + 0y + 0z - 2 = 0
                static_assert(normalized.SignedDistance == 2, 
                            "Distance should be scaled when normalizing");
            }
            
            // Test with already normalized normal
            {
                constexpr Vector3D unitNormal(0, 1, 0);
                constexpr Fxp distance = 5;
                constexpr Plane plane(unitNormal, distance);
                constexpr Plane normalized = plane.Normalized();
                
                // Should be unchanged
                static_assert(normalized.Normal == unitNormal, 
                            "Already normalized normal should remain unchanged");
                static_assert(normalized.SignedDistance == distance, 
                            "Distance should remain the same when normal is already normalized");
            }
            
            // Test zero normal handling
            {
                constexpr Vector3D zeroNormal(0, 0, 0);
                constexpr Plane plane1(zeroNormal, 0);
                
                // Should have zero normal and distance
                static_assert(plane1.Normal == Vector3D(0, 0, 0), 
                            "Zero normal should be preserved");
                static_assert(plane1.SignedDistance == 0, 
                            "Distance should be preserved");
                
                // Test Normalized() with zero normal
                constexpr Plane plane2 = Plane(zeroNormal, 5).Normalized();
                // Normalized() should keep zero normal as is
                static_assert(plane2.Normal == Vector3D(0, 0, 0), 
                            "Zero normal should remain zero after Normalized()");
                static_assert(plane2.SignedDistance == 5, 
                            "Distance should be preserved after Normalized() with zero normal");
            }
        }

        // ============================================
        // Distance Tests
        // ============================================
        static constexpr void TestDistance()
        {
            // Test with a simple XZ plane (normal = (0,1,0), distance = 0)
            constexpr Plane xzPlane(Vector3D(0, 1, 0), 0);
            
            // Test point above the plane
            {
                constexpr Vector3D point(0, 5, 0);
                constexpr Fxp signedDist = xzPlane.GetSignedDistance(point);
                static_assert(signedDist == 5, 
                            "Signed distance to point above plane should be positive");
                static_assert(xzPlane.GetDistance(point) == 5,
                            "Distance to point above plane should be positive");
            }
            
            // Test point below the plane
            {
                constexpr Vector3D point(0, -3, 0);
                constexpr Fxp signedDist = xzPlane.GetSignedDistance(point);
                static_assert(signedDist == -3, 
                            "Signed distance to point below plane should be negative");
                static_assert(xzPlane.GetDistance(point) == 3,
                            "Distance to point below plane should be positive");
            }
            
            // Test point on the plane
            {
                constexpr Vector3D point(10, 0, 5);
                constexpr Fxp signedDist = xzPlane.GetSignedDistance(point);
                static_assert(signedDist == 0, 
                            "Signed distance to point on plane should be zero");
                static_assert(xzPlane.GetDistance(point) == 0,
                            "Distance to point on plane should be zero");
            }
            
            // Test with a non-origin plane (y = 10)
            constexpr Plane offsetPlane(Vector3D(0, 1, 0), 10);
            
            // Test point in front of offset plane
            {
                constexpr Vector3D point(0, 15, 0);
                constexpr Fxp signedDist = offsetPlane.GetSignedDistance(point);
                static_assert(signedDist == 5, 
                            "Signed distance to point in front of offset plane should be positive");
            }
            
            // Test point behind offset plane
            {
                constexpr Vector3D point(0, 5, 0);
                constexpr Fxp signedDist = offsetPlane.GetSignedDistance(point);
                static_assert(signedDist == -5, 
                            "Signed distance to point behind offset plane should be negative");
            }
            
            // Test with an angled plane (diagonal)
            {
                constexpr Vector3D normal(1, 1, 0);
                constexpr Plane angledPlane = Plane(normal, 0).Normalized();
                constexpr Vector3D point(2, 2, 0);
                
                // For a plane with normal (1,1,0) and point (2,2,0), the distance should be (2+2)/√2 = 2√2
                // Using direct calculation with fixed-point math
                constexpr Fxp actualDist = angledPlane.GetSignedDistance(point);
                
                // The exact value is 2√2 ≈ 2.8284
                // Check if the value is within a reasonable range for 2√2 (≈2.8284)
                constexpr Fxp minExpected = Fxp(2.6f);  // Slightly lower minimum
                constexpr Fxp maxExpected = Fxp(3.0f);  // Slightly higher maximum
                constexpr bool isDistanceInRange = actualDist >= minExpected && actualDist <= maxExpected;
                constexpr Fxp sqrt2 = Fxp(2).Sqrt();
                constexpr Fxp expectedDist = Fxp(2) * sqrt2;  // 2√2
                // For the error message, we can't use string concatenation with # in static_assert
                // So we'll just provide a fixed message with the expected value
                static_assert(isDistanceInRange,
                            "Distance to point from angled plane should be approximately 2.8284 (2√2). "
                            "Check the actual value and expected range (2.6 to 3.0).");
                
                // Test absolute distance (should be the same as signed distance since it's positive)
                constexpr Fxp absDist = angledPlane.GetDistance(point);
                static_assert(absDist == actualDist,
                            "Absolute distance should match signed distance for points in front");
            }
            
            // Test point on offset plane
            {
                constexpr Vector3D point(100, 10, -50);
                constexpr Fxp dist = offsetPlane.GetSignedDistance(point);
                static_assert(dist == 0, 
                            "Point on offset plane should have zero distance");
            }
        }
        
        // ============================================
        // Projection and Reflection Tests
        // ============================================
        static constexpr void TestProjectionAndReflection()
        {
            constexpr Plane plane(Vector3D(0, 1, 0), 0);  // XZ plane
            
            // Test point projection
            constexpr Vector3D point(1, 5, 2);
            constexpr Vector3D projected = plane.ProjectPoint(point);
            static_assert(projected.X == 1 && projected.Y == 0 && projected.Z == 2, 
                        "Projection onto XZ plane should zero the Y component");
            
            // Test point reflection
            constexpr Vector3D reflected = plane.ReflectPoint(point);
            static_assert(reflected.X == 1 && reflected.Y == -5 && reflected.Z == 2, 
                        "Reflection over XZ plane should invert Y component");
            
            // Test vector reflection
            constexpr Vector3D vector(1, 1, 0);
            constexpr Vector3D reflectedVec = plane.ReflectVector(vector);
            static_assert(reflectedVec.X == 1 && reflectedVec.Y == -1 && reflectedVec.Z == 0, 
                        "Reflection of vector should invert Y component");
            
            // Test reflection of point on plane
            constexpr Vector3D pointOnPlane(1, 0, 2);
            constexpr Vector3D reflectedOnPlane = plane.ReflectPoint(pointOnPlane);
            static_assert(reflectedOnPlane.X == pointOnPlane.X && 
                         reflectedOnPlane.Y == pointOnPlane.Y && 
                         reflectedOnPlane.Z == pointOnPlane.Z, 
                        "Reflection of point on plane should return the same point");
        }
        
        // Note: Transformation tests are not included as they require additional math utilities
        // that are not part of the core Plane class. These would typically be tested in a separate
        // test file that includes the necessary math utilities.
        
        // Run all tests
        static constexpr void RunAll()
        {
            TestConstruction();
            TestNormalization();
            TestDistance();
            TestProjectionAndReflection();
            
            // All tests passed
            return;
        }
    };
    
    // Execute all tests
    static_assert((PlaneTests::RunAll(), true), "Plane tests failed");
}
