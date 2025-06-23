#pragma once

/**
 * @brief Main test header for SaturnMathPP library
 * 
 * This file includes all test files for the SaturnMathPP library.
 * Each test file contains static_assert tests for a specific component.
 * 
 * To run the tests, simply include this header in a cpp file and compile.
 * If compilation succeeds, all tests have passed.
 */

// Include all test files
#include "test_aabb.hpp"
#include "test_angle.hpp"
#include "test_collision.hpp"
#include "test_frustum.hpp"
#include "test_fxp.hpp"
#include "test_main.hpp"
#include "test_mat33.hpp"
#include "test_mat43.hpp"
#include "test_plane.hpp"
#include "test_sphere.hpp"
#include "test_trigonometry.hpp"
#include "test_vector2d.hpp"
#include "test_vector3d.hpp"

namespace SaturnMath::Tests
{
    // Compile-time verification that all tests are included
    constexpr bool AllTestsIncluded = true;
    static_assert(AllTestsIncluded, "All tests are included");
}
