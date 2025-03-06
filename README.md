<p align="center">
<img src="https://github.com/robertoduarte/SaturnMathPP/blob/main/documentation/resources/smpp_sqrt_pi_logo.svg" >
</p>

# SaturnMath++

<p align="center">
  <img src="https://img.shields.io/badge/C%2B%2B-23-blue" alt="C++23">
  <img src="https://img.shields.io/badge/Platform-Sega%20Saturn-yellow" alt="Platform">
</p>

SaturnMath++ is a high-performance mathematical library specifically engineered for Sega Saturn game development. It provides a comprehensive suite of fixed-point arithmetic operations, vector/matrix transformations, and geometric calculations optimized for the Saturn's SH-2 processors.

## Overview

Developed with the Saturn's unique hardware architecture in mind, SaturnMath++ addresses the platform's key constraints while maximizing performance:

- **Fixed-Point Precision**: Replaces costly floating-point operations with optimized 16.16 fixed-point arithmetic
- **Hardware-Aware Design**: Takes advantage of the SH-2's 32-bit operations and instruction set
- **Performance-First Philosophy**: Offers multiple precision levels to balance accuracy and speed
- **Modern C++ Features**: Leverages C++23 capabilities for compile-time optimizations
- **Zero Overhead**: No dynamic memory allocation, minimal branching, and cache-friendly data structures

Whether you're developing a 3D racing game, a 2D sprite-based platformer, or a complex simulation, SaturnMath++ provides the mathematical foundation you need without sacrificing precious CPU cycles.

## Architecture

SaturnMath++ is organized into two main namespaces:

### SaturnMath::Types
Contains all fundamental mathematical types and structures:
- Vector arithmetic (`Vector2D`, `Vector3D`)
- Matrix operations (`Mat33`, `Matrix43`)
- Geometric primitives (`AABB`, `Sphere`, `Plane`, `Frustum`)
- Fixed-point numbers (`Fxp`)
- Angle representation optimized for Saturn hardware

### SaturnMath
Provides mathematical operations and utilities:
- Trigonometric functions
- Interpolation methods
- Integer-specific optimizations
- Template-based utility functions for common operations (Min, Max, Abs, Clamp)

## Features

### Core Components
- **Fixed-Point Arithmetic**: High-performance `Fxp` class with precise fixed-point operations
  - Power function for integer exponents
  - Value clamping between bounds
  - Comprehensive arithmetic operations
  - MinValue and MaxValue constants for range boundaries
  - Modulo operations for both Fxp and integer types
  - Flexible value conversion:
    ```cpp
    // Compile-time conversion (preferred)
    constexpr Fxp a = 3.14159f;   // Exact conversion at compile-time (float)
    constexpr Fxp b = 42;         // Integer conversion at compile-time (int16_t)

    // Runtime conversion with safety checks
    int32_t someInt = GetRuntimeValue();
    Fxp c = Fxp::Convert(someInt);   // Range checking (converts to int16_t first)

    // Runtime floating-point conversion (with performance warning)
    float someFloat = GetRuntimeFloat();
    Fxp d = Fxp::Convert(someFloat); // Warning: heavy operation on Saturn hardware
    
    // Disable performance warnings for specific sections if needed
    #define DISABLE_PERFORMANCE_WARNINGS
    #include "saturn_math.hpp"
    // Now conversion warnings are suppressed in this compilation unit

    // Converting back to other types
    int16_t i = a.As<int16_t>();     // To integer
    float f = a.As<float>();         // To float (with performance warning)
    ```
  - **Advanced Usage**: For experts who understand the 16.16 fixed-point format, direct raw value manipulation is available:
    ```cpp
    // Create from raw 16.16 value (for advanced users only)
    Fxp raw = Fxp::BuildRaw(0x00010000);  // 1.0 in 16.16 format
    
    // Hardware-optimized asynchronous division
    Fxp dividend = 10;
    Fxp divisor = 3;
    Fxp::AsyncDivSet(dividend, divisor);
    Fxp quotient = Fxp::AsyncDivGetResult();    // Get division result
    Fxp remainder = Fxp::AsyncDivGetRemainder(); // Get remainder
    ```

- **Angle Handling**: Type-safe `Angle` class for angular calculations
  - Raw value construction and access
  - Scalar multiplication and division
  - Automatic wrap-around handling
  - Comprehensive comparison operators with wrap-around considerations
  - Performance warnings for angle conversions (can be disabled)
  ```cpp
  // Angle operations with comparison
  Angle angle1 = Angle::FromDegrees(45);
  Angle angle2 = Angle::FromDegrees(90);

  if (angle1 < angle2) {
      // This works, but be cautious near wrap-around boundaries
  }

  // Disable performance warnings when needed
  #define DISABLE_PERFORMANCE_WARNINGS
  #include "saturn_math.hpp"
  // Now conversion warnings are suppressed
  ```

- **Euler Angles**: Type-safe representation of 3D rotations
  - Intrinsic Tait-Bryan angles (pitch-yaw-roll)
  - X-Y-Z rotation order
  - Zero-initialization support

### Vectors and Matrices
- **2D Vectors**: `Vector2D` class with optimized operations
  - Unit vectors (UnitX, UnitY)
  - Directional vectors (Left, Right, Up, Down)
  - Multiple precision levels for normalization and length calculations
- **3D Vectors**: `Vector3D` class extending `Vector2D` functionality
  - Unit vectors (UnitX, UnitY, UnitZ)
  - Optimized cross/dot products
  - Template-based precision control for geometric operations
- **Matrix Operations**: Efficient `Matrix33` and `Matrix43` implementations
  - Common transformations (scale, rotate, translate)
  - Optimized multiplication with detailed documentation
  - Identity/zero matrix constants
  - Row-based layout with clear geometric meaning
  - Orthonormal basis in right-handed coordinate system
  - Billboard matrix creation
  - Look-at matrix for camera positioning
  - Transform decomposition into scale/rotation/translation
  - EulerAngles support for rotations
- **Matrix Stack**: Fixed-size stack for transform hierarchies
  - No dynamic allocation
  - Depth checking
  - Direct transformation methods

### Geometric Primitives
- **Shapes**: Abstract base class for geometric primitives
- **AABB**: Axis-aligned bounding box with comprehensive intersection tests
  - Fast min/max calculations
  - Volume and surface area computation
  - Merging and expansion operations
- **Sphere**: Perfect sphere with exact collision detection
  - Point containment tests
  - Sphere-sphere intersection
  - Sphere-AABB intersection
- **Plane**: Infinite plane with normal and distance representation
  - Point-plane distance calculation
  - Construction from points/normal
  - Normalization utilities
- **Frustum**: View frustum for efficient visibility culling
  - Fast plane extraction from matrices
  - Comprehensive intersection tests
  - View space utilities

### Trigonometry
- **Basic Functions**: Efficient implementations using lookup tables
  - Sine, cosine, and tangent
  - Arctangent2 with full quadrant support
- **Draft Functions**: Work-in-progress implementations
  - Inverse trigonometric (asin, acos)
  - Hyperbolic functions (sinh, cosh, tanh)
- **Interpolation**: Smooth angle transitions
  - Spherical linear interpolation (SLerp)
  - Automatic shortest-path selection

## Installation

### Requirements
- C++23 compliant compiler
- No external dependencies

### Integration
Just include the library in your Sega Saturn project:
```cpp
#include "saturn_math.hpp"
```

## Usage Examples

### Vector and Matrix Operations

```cpp
#include "saturn_math.hpp"
using namespace SaturnMath::Types;

// 3D vector operations with different precision levels
Vector3D position(1, 2, 3);
Vector3D direction = Vector3D::UnitZ();  // Forward direction (0,0,1)

// Standard precision - highest accuracy
Vector3D normal = direction.Normalize<Precision::Standard>();  // Explicit
Vector3D same = direction.Normalize();                        // Implicit (defaults to Standard)

// Fast precision - balanced performance
Vector3D approxNormal = direction.Normalize<Precision::Fast>();

// Turbo precision - fastest calculation
Vector3D quickNormal = direction.Normalize<Precision::Turbo>();

// 2D vector operations
Vector2D screenPos = Vector2D::Zero();
screenPos += Vector2D::Right() * 10;  // Move 10 units right
screenPos += Vector2D::Up() * 5;      // Move 5 units up

// Matrix operations with precision control
Matrix43 transform = Matrix43::Identity();
transform.Translate(Vector3D::UnitY() * 5);  // Move 5 units up
transform.Rotate(Vector3D(0, Angle::FromDegrees(90), 0));

// Matrix decomposition with specified precision
Vector3D scale, rotation, translation;
transform.Decompose<Precision::Standard>(scale, rotation, translation);

// Transform the position
Vector3D transformed = transform * position;
```

### Geometric Operations

```cpp
// Create and manipulate geometric primitives
Vector3D center(0, 0, 0);
Vector3D size(2, 2, 2);
AABB box(center, size);

// Calculate normals with different precision levels
Vector3D v1(0, 0, 0), v2(1, 0, 0), v3(0, 1, 0);
Vector3D normal = Vector3D::CalcNormal<Precision::Standard>(v1, v2, v3);
Vector3D fastNormal = Vector3D::CalcNormal<Precision::Fast>(v1, v2, v3);

// Collision detection
Sphere sphere(center, 2.0_fxp);
bool collision = box.Intersects(sphere);

// Create view matrix with precision control
Vector3D eye(0, 5, -10);
Vector3D target(0, 0, 0);
Vector3D up = Vector3D::UnitY();
Matrix43 view = Matrix43::CreateLookAt<Precision::Standard>(eye, target, up);

// Frustum culling
Frustum viewFrustum;
bool isVisible = viewFrustum.Contains(box);
```

### Fixed-Point and Angle Operations

```cpp
#include "saturn_math.hpp"
using namespace SaturnMath::Types;

// Fixed-point arithmetic
Fxp a(5);                     // Create from int16_t (works at compile-time or runtime)
Fxp b(2.5f);                  // Create from float (compile-time only)
Fxp c = a * b;                // 12.5 (0x000C8000)
int16_t i = c.As<int16_t>();  // Convert back to int16_t

// Power and clamping operations
Fxp squared = a.Pow(2);    // 25
Fxp clamped = b.Clamp(0, 2); // 2

// Fixed-point conversion methods
// 1. COMPILE-TIME construction (preferred when values are known at compile time)
constexpr Fxp compile_time = 3.14159f;         // Exact conversion at compile-time (float)
constexpr Fxp compile_time_int = 42;           // Integer conversion at compile-time (int16_t)

// 2. RUNTIME conversion with static Convert() (for values not known at compile time)
int32_t runtime_int = 12345;
float runtime_float = 67.89f;
Fxp int_conversion = Fxp::Convert(runtime_int);   // With range checking
Fxp float_conversion = Fxp::Convert(runtime_float); // Warning: heavy operation

// 3. Converting FROM Fxp back to other types
int16_t as_int = compile_time.As<int16_t>();    // Convert to int16_t
float as_float = compile_time.As<float>();      // Convert to float (warning: heavy)

// Angle calculations
Angle rotation = Angle::FromDegrees(45);
Fxp sine = SaturnMath::Sin(rotation);
Fxp cosine = SaturnMath::Cos(rotation);

// Example of angle arithmetic
Angle doubled = rotation * Fxp(2);    // Double the angle
Angle halved = rotation / Fxp(2);     // Half the angle

// Euler angles for 3D rotation
Vector3D orientation(
    Angle::FromDegrees(30),  // Pitch
    Angle::FromDegrees(45),  // Yaw
    Angle::FromDegrees(0)    // Roll
);

// Create transformation matrix from angles
Matrix43 transform = Matrix43::CreateTransform(
    Vector3D(1, 2, 3),      // Translation
    orientation,             // Rotation
    Vector3D(1, 1, 1)       // Scale
);

// Create billboard matrix
Matrix43 billboard = Matrix43::CreateBillboard(
    Vector3D(0, 0, 0),      // Billboard position
    Vector3D(0, 0, 5),      // Camera position
    Vector3D::UnitY()       // Up vector
);

// Smooth angle interpolation
Angle start = Angle::Zero();
Angle end = Angle::HalfPi();
Angle interpolated = SaturnMath::Interpolation::SLerp(start, end, Fxp(0.5));

// Utility functions
using namespace SaturnMath;
auto maxVal = Max(5_fxp, 3_fxp);
auto absVal = Abs(-5_fxp);
auto clampedVal = Clamp(7_fxp, 0_fxp, 5_fxp);
```

## Performance Considerations

### Performance Features
- Template-based precision control for performance-critical operations
- Cache-friendly data layouts
- Fixed-size containers to avoid dynamic allocation
- Lookup table-based trigonometry
- Compile-time constant evaluation

### Precision Control

SaturnMath++ provides a template-based precision control system that allows you to balance between accuracy and performance. Each precision level is optimized for different use cases. Operations that support precision control will use `Standard` precision by default when no template parameter is specified.

```cpp
using namespace SaturnMath;

// Standard precision - highest accuracy
Vector3D normal = direction.Normalize<Precision::Standard>();  // Explicit
Vector3D same = direction.Normalize();                        // Implicit (defaults to Standard)

// Fast precision - balanced performance
Vector3D approxNormal = direction.Normalize<Precision::Fast>();

// Turbo precision - fastest calculation
Vector3D quickNormal = direction.Normalize<Precision::Turbo>();
```

Supported operations with precision control:
- Vector normalization and length calculations
- Matrix decomposition and transformations
- Square root calculations
- Geometric calculations (normals, distances)

### Best Practices
- Use `Fast` or `Turbo` precision for non-critical calculations where performance is important
- Keep `Standard` precision (default) for calculations requiring high accuracy
- Prefer fixed-size containers (like `MatrixStack`) over dynamic allocation
- Take advantage of lookup-based trig functions for better performance
- Leverage compile-time constants with `consteval` methods
- Structure data for optimal cache usage on SH-2

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
