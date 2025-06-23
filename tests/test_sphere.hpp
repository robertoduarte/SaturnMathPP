#pragma once

/**
 * @file test_sphere.hpp
 * @brief Compile-time unit tests for the Sphere class
 *
 * This file contains comprehensive tests for the Sphere class, covering:
 * - Basic construction and factory methods
 * - Validity checks and edge cases
 * - Containment and intersection tests
 * - Transformation operations (translation, scaling)
 * - Conversions to and from other shapes (AABB)
 *
 * All tests are performed at compile-time using static_assert.
 */

#include "../impl/sphere.hpp"

namespace SaturnMath::Tests
{
    using namespace SaturnMath::Types;

    /**
     * @brief Test suite for Sphere class
     *
     * This struct contains all test cases for the Sphere class, organized
     * into logical sections. Each test method focuses on a specific
     * aspect of the Sphere functionality.
     */
    struct SphereTests
    {
        // ============================================
        // Construction and Factory Methods
        // ============================================

        /**
         * @brief Tests sphere construction and factory methods
         *
         * Verifies:
         * - Default construction creates a unit sphere at origin
         * - Construction from center and radius works correctly
         * - Copy constructor performs a deep copy
         */
        static constexpr void TestConstruction()
        {
            // Default constructor
            {
                constexpr Sphere defaultSphere;
                constexpr Vector3D defaultPosition = defaultSphere.GetPosition();
                constexpr Fxp defaultRadius = defaultSphere.GetRadius();
                static_assert(defaultPosition.X == 0 && defaultPosition.Y == 0 &&
                    defaultPosition.Z == 0 && defaultRadius == 1,
                    "Default constructor should create a unit sphere at origin");
            }


            // Parameterized constructor
            {
                constexpr Vector3D center(1, 2, 3);
                constexpr Fxp radius = 2;
                constexpr Sphere sphere(center, radius);
                constexpr Vector3D spherePosition = sphere.GetPosition();
                constexpr Fxp sphereRadius = sphere.GetRadius();
                static_assert(spherePosition.X == 1 && spherePosition.Y == 2 &&
                    spherePosition.Z == 3 && sphereRadius == 2,
                    "Construction from center and radius should work");
            }


            // Copy constructor
            {
                constexpr Vector3D center(1, 2, 3);
                constexpr Fxp radius = 2;
                constexpr Sphere sphere(center, radius);
                constexpr Vector3D spherePosition = sphere.GetPosition();
                constexpr Fxp sphereRadius = sphere.GetRadius();
                
                constexpr Sphere copy = sphere;
                constexpr Vector3D copyPosition = copy.GetPosition();
                constexpr Fxp copyRadius = copy.GetRadius();
                static_assert(copyPosition.X == spherePosition.X && copyPosition.Y == spherePosition.Y &&
                    copyPosition.Z == spherePosition.Z && copyRadius == sphereRadius,
                    "Copy constructor should perform a deep copy of the sphere");
            }
        }

        // ============================================
        // Validity Tests
        // ============================================

        /**
         * @brief Tests sphere validity checks
         *
         * Verifies:
         * - Spheres with positive radius are valid
         * - Zero-radius spheres (point spheres) are valid
         * - Spheres with negative radius are invalid
         * - Edge cases with Fxp::Epsilon() and large values
         */
        static constexpr void TestValidity()
        {
            constexpr Vector3D center(1, 2, 3);

            // Valid sphere with positive radius
            {
                constexpr Fxp radius = 2;
                constexpr Sphere validSphere(center, radius);
                static_assert(validSphere.IsValid() == true,
                    "Sphere with positive radius should be valid");
            }

            // Minimum positive radius (using Fxp(1) as smallest non-zero value)
            {
                constexpr Fxp minRadius = Fxp(1);
                constexpr Sphere minRadiusSphere(center, minRadius);
                static_assert(minRadiusSphere.IsValid() == true,
                    "Sphere with smallest positive radius should be valid");
            }

            // Large radius (using a reasonable large value)
            {
                constexpr Fxp largeRadius = Fxp(1000);
                constexpr Sphere largeSphere(center, largeRadius);
                static_assert(largeSphere.IsValid() == true,
                    "Sphere with large radius should be valid");
            }

            // Zero radius (point sphere)
            {
                constexpr Sphere zeroRadiusSphere(center, 0);
                static_assert(zeroRadiusSphere.IsValid() == true,
                    "Sphere with zero radius (point sphere) should be valid");
            }
            // Negative radius (invalid case)
            {
                constexpr Sphere negativeRadiusSphere(center, -1);
                static_assert(negativeRadiusSphere.IsValid() == false,
                    "Sphere with negative radius should be invalid");
            }
            // Epsilon radius (smallest positive value)
            {
                constexpr Sphere epsilonSphere(center, Fxp::Epsilon());
                static_assert(epsilonSphere.IsValid() == true,
                    "Sphere with Fxp::Epsilon() radius should be valid");
            }
            // Negative epsilon radius (invalid case)
            {
                constexpr Sphere negativeEpsilonSphere(center, -Fxp::Epsilon());
                static_assert(negativeEpsilonSphere.IsValid() == false,
                    "Sphere with negative Fxp::Epsilon() radius should be invalid");
            }
        }

        // ============================================
        // Basic Properties Tests
        // ============================================

        /**
         * @brief Tests basic geometric properties of spheres
         *
         * Verifies:
         * - Volume calculation is correct (4/3 * π * r³)
         * - Surface area calculation is correct (4 * π * r²)
         * - Diameter is exactly twice the radius
         */
        static constexpr void TestProperties()
        {
            constexpr Vector3D center(1, 2, 3);
            constexpr Fxp radius = 2;
            constexpr Sphere sphere(center, radius);

            // Volume calculation
            {
                // Volume = (4/3) * π * r³
                // For r = 2, volume ≈ 33.5103
                constexpr Fxp volume = sphere.GetVolume();
                static_assert(volume > 33.4 && volume < 33.6,
                    "GetVolume should return the volume of the sphere");
            }

            // Surface area calculation
            {
                // Surface area = 4 * π * r²
                // For r = 2, surface area ≈ 50.2655
                constexpr Fxp surfaceArea = sphere.GetSurfaceArea();
                static_assert(surfaceArea > 50.2 && surfaceArea < 50.4,
                    "GetSurfaceArea should return the surface area of the sphere");
            }

            // Diameter calculation
            {
                constexpr Fxp diameter = sphere.GetDiameter();
                static_assert(diameter == 4,
                    "GetDiameter should return exactly twice the radius");
            }
        }

        // ============================================
        // Transformation Tests
        // ============================================

        /**
         * @brief Tests sphere transformations
         * 
         * Verifies:
         * - Translation moves the center without changing the radius
         * - Uniform scaling affects both center and radius proportionally
         * - Non-uniform scaling uses minimum scale component for radius
         * - Transformations maintain sphere validity
         */
        static constexpr void TestTransformation()
        {
            // ============================================
            // Test Setup - Create test sphere
            // ============================================
            // Create a test sphere at (1, 2, 3) with radius 2
            constexpr Vector3D center(1, 2, 3);
            constexpr Fxp radius = 2;
            constexpr Sphere sphere(center, radius);

            // ============================================
            // Test Cases
            // ============================================
            
            // Translation
            // Verifies that translation moves the center without affecting the radius
            {
                constexpr Vector3D translation(10, 20, 30);
                constexpr Sphere translated = sphere.Translate(translation);
                constexpr Vector3D translatedCenter = translated.GetPosition();
                constexpr Fxp translatedRadius = translated.GetRadius();

                // Verify center is translated by the translation vector
                static_assert(translatedCenter.X == 11 &&
                            translatedCenter.Y == 22 &&
                            translatedCenter.Z == 33,
                            "Translated sphere center should be offset by the translation vector");
                
                // Verify radius remains unchanged
                static_assert(translatedRadius == 2,
                            "Translation should not affect the sphere's radius");
                
                // Verify the sphere remains valid after translation
                static_assert(translated.IsValid() == true,
                            "Translated sphere should remain valid");
            }

            // Uniform scaling
            // Verifies that uniform scaling affects both center and radius proportionally
            {
                constexpr Fxp scale = 3;
                constexpr Sphere scaled = sphere.Scale(scale);
                constexpr Vector3D scaledCenter = scaled.GetPosition();
                constexpr Fxp scaledRadius = scaled.GetRadius();

                // Verify center is scaled by the scale factor
                static_assert(scaledCenter.X == 3 &&
                            scaledCenter.Y == 6 &&
                            scaledCenter.Z == 9,
                            "Scaling should multiply the center coordinates by the scale factor");
                
                // Verify radius is scaled by the scale factor
                static_assert(scaledRadius == 6,
                            "Scaling should multiply the radius by the scale factor");
                
                // Verify the sphere remains valid after scaling
                static_assert(scaled.IsValid() == true,
                            "Scaled sphere should remain valid");
            }

            // Non-uniform scaling
            // Verifies that non-uniform scaling uses the minimum scale component for radius
            {
                constexpr Vector3D nonUniformScale(2, 3, 4);
                constexpr Sphere nonUniformScaled = sphere.Scale(nonUniformScale);
                constexpr Vector3D nonUniformScaledPos = nonUniformScaled.GetPosition();
                constexpr Fxp nonUniformScaledRad = nonUniformScaled.GetRadius();

                // Verify center is scaled by each component
                static_assert(nonUniformScaledPos.X == 2 &&
                            nonUniformScaledPos.Y == 6 &&
                            nonUniformScaledPos.Z == 12,
                            "Non-uniform scaling should scale each center coordinate by the corresponding scale component");
                
                // Verify radius is scaled by the minimum scale component (2 in this case)
                static_assert(nonUniformScaledRad == 4,
                            "Non-uniform scaling should scale the radius by the minimum scale component (2 * 2 = 4)");
                
                // Verify the sphere remains valid after non-uniform scaling
                static_assert(nonUniformScaled.IsValid() == true,
                            "Non-uniformly scaled sphere should remain valid");
            }
            
            // Zero scaling (edge case)
            // Verifies behavior when scaling by zero
            {
                constexpr Fxp zeroScale = 0;
                constexpr Sphere zeroScaled = sphere.Scale(zeroScale);
                
                // The sphere should become a point at the origin with zero radius
                static_assert(zeroScaled.GetPosition() == Vector3D(0, 0, 0),
                            "Scaling by zero should move the center to the origin");
                static_assert(zeroScaled.GetRadius() == 0,
                            "Scaling by zero should result in zero radius");
                static_assert(zeroScaled.IsValid() == true,
                            "Sphere scaled to zero radius should still be valid (point sphere)");
            }
        }

        // ============================================
        // Closest Point Tests
        // ============================================

        /**
         * @brief Tests finding the closest point on a sphere to a given point
         *
         * Verifies behavior for points:
         * - Inside the sphere (returns the point itself)
         * - Outside the sphere (returns the nearest point on the surface)
         * - On the sphere's surface (returns the point itself)
         *
         * Uses Precision::Accurate for distance calculations to ensure correctness.
         */
        static constexpr void TestClosestPoint()
        {
            // Create a test sphere centered at origin with radius 2
            constexpr Vector3D center(0, 0, 0);
            constexpr Fxp radius = 2;
            constexpr Sphere sphere(center, radius);

            // Point inside the sphere
            {
                constexpr Vector3D inside(1, 0, 0);
                constexpr Vector3D closestInside = sphere.GetClosestPoint<Precision::Accurate>(inside);
                static_assert(closestInside.X == 1 && closestInside.Y == 0 && closestInside.Z == 0,
                    "Closest point to a point inside should be the point itself");
            }

            // Point outside the sphere
            {
                constexpr Vector3D outside(4, 0, 0);
                constexpr Vector3D closestOutside = sphere.GetClosestPoint<Precision::Accurate>(outside);
                static_assert(closestOutside.X == 2 && closestOutside.Y == 0 && closestOutside.Z == 0,
                    "Closest point to a point outside should be on the sphere surface in the direction of the point");
            }

            // Point on the sphere's surface
            {
                constexpr Vector3D onSurface(0, 2, 0);
                constexpr Vector3D closestOnSurface = sphere.GetClosestPoint<Precision::Accurate>(onSurface);
                static_assert(closestOnSurface.X == 0 && closestOnSurface.Y == 2 && closestOnSurface.Z == 0,
                    "Closest point to a point on the surface should be the point itself");
            }

            // Point outside in diagonal direction
            {
                constexpr Vector3D outsideDiagonal(3, 3, 3);
                constexpr Vector3D closestDiagonal = sphere.GetClosestPoint<Precision::Accurate>(outsideDiagonal);

                // The point should be on the sphere surface in the direction of the diagonal
                // For a unit vector in the direction (1,1,1), the length is sqrt(3)
                // So the point on the sphere should be (2/sqrt(3), 2/sqrt(3), 2/sqrt(3))
                // Check that all components are equal (within a small epsilon)
                constexpr Fxp componentDiffXY = (closestDiagonal.X - closestDiagonal.Y).Abs();
                constexpr Fxp componentDiffYZ = (closestDiagonal.Y - closestDiagonal.Z).Abs();
                static_assert(componentDiffXY < Fxp::Epsilon() * 10 &&
                    componentDiffYZ < Fxp::Epsilon() * 10,
                    "Closest point to a diagonal point should have approximately equal components");
            }
        }

        // ============================================
        // AABB Conversion Tests
        // ============================================

        /**
         * @brief Tests conversion between Sphere and AABB representations
         * 
         * Verifies:
         * - Sphere to AABB conversion produces a properly sized bounding box
         * - AABB to Sphere conversion correctly computes center and radius
         * - The resulting sphere from AABB conversion bounds the original AABB
         */
        static constexpr void TestAABBConversion()
        {
            // Convert sphere to AABB
            {
                constexpr Vector3D center(0, 0, 0);
                constexpr Fxp radius = 2;
                constexpr Sphere sphere(center, radius);

                // Convert sphere to AABB
                constexpr AABB aabb = sphere.ToAABB();
                constexpr Vector3D aabbMin = aabb.GetMin();
                constexpr Vector3D aabbMax = aabb.GetMax();
                
                // Verify the AABB properly bounds the sphere
                static_assert(aabbMin.X == -2 && aabbMin.Y == -2 && aabbMin.Z == -2 &&
                            aabbMax.X == 2 && aabbMax.Y == 2 && aabbMax.Z == 2,
                            "ToAABB should return an AABB that bounds the sphere");
            }

            // Convert AABB to sphere
            {
                // Create an AABB with min (-1,-2,-3) and max (1,2,3)
                constexpr Vector3D minPoint(-1, -2, -3);
                constexpr Vector3D maxPoint(1, 2, 3);
                constexpr AABB box = AABB::FromMinMax(minPoint, maxPoint);
                
                // Convert AABB to sphere
                constexpr Sphere fromAABB = Sphere::FromAABB(box);
                constexpr Vector3D aabbCenter = fromAABB.GetPosition();
                constexpr Fxp aabbRadius = fromAABB.GetRadius();

                // Verify the center is at the AABB's center
                constexpr Vector3D expectedCenter = (minPoint + maxPoint) * Fxp(0.5f);
                static_assert(aabbCenter.X == expectedCenter.X &&
                            aabbCenter.Y == expectedCenter.Y &&
                            aabbCenter.Z == expectedCenter.Z,
                            "FromAABB should create a sphere with center at the center of the AABB");

                // Verify the radius is the distance to the furthest corner
                // For the box with extents (-1,-2,-3) to (1,2,3), the furthest corner is (1,2,3)
                // Distance from center (0,0,0) to (1,2,3) is sqrt(1²+2²+3²) = sqrt(14) ≈ 3.742
                constexpr Fxp expectedRadius = Vector3D(1, 2, 3).Length<Precision::Accurate>();
                constexpr Fxp radiusDiff = (aabbRadius - expectedRadius).Abs();
                static_assert(radiusDiff < Fxp::Epsilon() * 10,
                            "FromAABB should create a sphere with radius equal to the distance to the furthest corner");
            }
        }

        // ============================================
        // Test Runner
        // ============================================
        
        /**
         * @brief Runs all sphere test cases
         * 
         * @return constexpr bool True if all tests pass, false otherwise
         * 
         * This method serves as the entry point for all sphere tests.
         * It executes each test case in sequence and will fail at compile-time
         * if any test fails due to the use of static_assert.
         */
        static constexpr bool RunAll()
        {
            // Execute each test case in sequence
            TestConstruction();      // 1. Verify construction and factory methods
            TestValidity();          // 2. Test sphere validity checks
            TestProperties();        // 3. Test geometric properties
            TestTransformation();    // 4. Test transformations (translate, scale)
            TestClosestPoint();      // 5. Test closest point calculations
            TestAABBConversion();    // 6. Test conversions to/from AABB
            
            // If we reach this point, all tests passed
            return true;
        }
    };

    // Execute all tests
    static_assert(SphereTests::RunAll(), "Sphere tests failed");
}
