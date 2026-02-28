/**
 * @brief Standalone test for AABB edge case fixes
 * 
 * This file tests the specific AABB edge cases that were fixed:
 * - Negative inputs in constructors
 * - Swapped min/max in FromMinMax
 * - IsDegenerate checking ANY axis (not ALL)
 * - Expand with negative margin clamping
 * - Scale with negative factor using absolute value
 * 
 * Compile and run with:
 *   g++ -std=c++23 -I.. -o test_aabb_edge_cases test_aabb_edge_cases.cpp
 *   ./test_aabb_edge_cases
 * 
 * If compilation succeeds and the program exits with code 0,
 * all static_assert tests have passed.
 */

#include "../impl/aabb.hpp"

using namespace SaturnMath::Types;

// Test negative uniform size
static_assert([]() {
    constexpr AABB box(Vector3D(0, 0, 0), Fxp(-2));
    return box.GetHalfExtents().X == 2 && 
           box.GetHalfExtents().Y == 2 && 
           box.GetHalfExtents().Z == 2;
}(), "Negative uniform size should use absolute value");

// Test negative half-extents
static_assert([]() {
    constexpr AABB box(Vector3D(0, 0, 0), Vector3D(-1, -2, -3));
    return box.GetHalfExtents().X == 1 && 
           box.GetHalfExtents().Y == 2 && 
           box.GetHalfExtents().Z == 3;
}(), "Negative half-extents should use absolute values");

// Test swapped min/max
static_assert([]() {
    constexpr AABB box = AABB::FromMinMax(Vector3D(1, 2, 3), Vector3D(-1, -2, -3));
    return box.GetMin() == Vector3D(-1, -2, -3) && 
           box.GetMax() == Vector3D(1, 2, 3);
}(), "FromMinMax should handle swapped min/max");

// Test IsDegenerate with X=0
static_assert(AABB(Vector3D(0, 0, 0), Vector3D(0, 1, 1)).IsDegenerate(), 
              "AABB with X=0 should be degenerate");

// Test IsDegenerate with Y=0
static_assert(AABB(Vector3D(0, 0, 0), Vector3D(1, 0, 1)).IsDegenerate(), 
              "AABB with Y=0 should be degenerate");

// Test IsDegenerate with Z=0
static_assert(AABB(Vector3D(0, 0, 0), Vector3D(1, 1, 0)).IsDegenerate(), 
              "AABB with Z=0 should be degenerate");

// Test IsDegenerate with all non-zero
static_assert(!AABB(Vector3D(0, 0, 0), Vector3D(1, 1, 1)).IsDegenerate(), 
              "AABB with all non-zero should not be degenerate");

// Test Expand with negative margin clamping
static_assert([]() {
    constexpr AABB box(Vector3D(0, 0, 0), Vector3D(1, 2, 3));
    constexpr AABB shrunk = box.Expand(Fxp(-1));
    return shrunk.GetHalfExtents().X == 0 && 
           shrunk.GetHalfExtents().Y == 1 && 
           shrunk.GetHalfExtents().Z == 2 &&
           shrunk.IsDegenerate();
}(), "Expand with negative margin should clamp and create degenerate box");

// Test Expand with large negative margin
static_assert([]() {
    constexpr AABB box(Vector3D(0, 0, 0), Vector3D(1, 2, 3));
    constexpr AABB collapsed = box.Expand(Fxp(-100));
    return collapsed.GetHalfExtents() == Vector3D(0, 0, 0) &&
           collapsed.GetMin() == Vector3D(0, 0, 0) &&
           collapsed.GetMax() == Vector3D(0, 0, 0);
}(), "Expand with large negative should collapse to point");

// Test Scale with negative factor
static_assert([]() {
    constexpr AABB box(Vector3D(0, 0, 0), Vector3D(1, 2, 3));
    constexpr AABB scaled = box.Scale(Fxp(-2));
    return scaled.GetHalfExtents().X == 2 && 
           scaled.GetHalfExtents().Y == 4 && 
           scaled.GetHalfExtents().Z == 6;
}(), "Scale with negative factor should use absolute value");

int main()
{
    // If we reach here, all compile-time tests passed
    return 0;
}
