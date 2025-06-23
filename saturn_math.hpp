#pragma once

/**
 * @file saturn_math.hpp
 * @brief Main include file for SaturnMath++ library
 * 
 * This header includes all public headers from the implementation directory.
 * For optimal compile times, consider including only the specific headers you need.
 */

// Core math types
#include <impl/angle.hpp>
#include <impl/fxp.hpp>
#include <impl/precision.hpp>

// Vector and matrix types
#include <impl/vector2d.hpp>
#include <impl/vector3d.hpp>
#include <impl/mat33.hpp>
#include <impl/mat43.hpp>
#include <impl/matrix_stack.hpp>

// Geometric primitives
#include <impl/plane.hpp>
#include <impl/aabb.hpp>
#include <impl/sphere.hpp>
#include <impl/frustum.hpp>

// Math utilities
#include <impl/trigonometry.hpp>
#include <impl/collision.hpp>
#include <impl/random.hpp>
#include <impl/sort_order.hpp>
#include <impl/utils.hpp>