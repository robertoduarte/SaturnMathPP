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

- **Fixed-Point Precision**: Replaces costly floating-point operations with configurable fixed-point arithmetic (default 16.16, also 24.8 and 8.24) templated across all math types
- **Hardware-Aware Design**: Takes advantage of the SH-2's 32-bit operations and instruction set
- **Hardware-Optimized**: Hand-tuned SH-2 assembly in critical paths (64-bit MAC multiplication, hardware divider unit, `xtrct` for fixed-point alignment)
- **Modern C++ Features**: Leverages C++23 capabilities for compile-time optimizations
- **Zero Overhead**: No dynamic memory allocation, minimal branching, and cache-friendly data structures

Whether you're developing a 3D racing game, a 2D sprite-based platformer, or a complex simulation, SaturnMath++ provides the mathematical foundation you need without sacrificing precious CPU cycles.

## Architecture

SaturnMath++ is organized into two main namespaces:

### SaturnMath::Types
Contains all fundamental mathematical types and structures:
- Vector arithmetic (`Vector2D`, `Vector3D`)
- Matrix operations (`Matrix33`, `Matrix43`)
- Geometric primitives (`AABB`, `Sphere`, `Plane`, `Frustum`) — all templated with `X` suffix (e.g. `AABBX<I, F>`) and bare aliases for Q16.16
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
- **Fixed-Point Arithmetic**: High-performance template-based `FixedPoint<I, F>` class with precise fixed-point operations
  - Configurable integer (`I`) and fractional (`F`) bits at compile-time (constraint: `I + F == 32`, `I >= 2`, `F >= 8`)
  - Built-in aliases for common formats:
    - `Fxp` / `Fxp16_16` — `FixedPoint<16, 16>` (default, balanced range/precision)
    - `Fxp24_8` — `FixedPoint<24, 8>` (large-world coordinates)
    - `Fxp8_24` — `FixedPoint<8, 24>` (high-precision normalized values, rotations)
    - **Note**: The `Fxp` alias is kept as the general-purpose default and for legacy compatibility; it is identical to `Fxp16_16` and both can be used interchangeably. Use `Fxp16_16` in new code for clarity, or continue using `Fxp` if you prefer the shorter name.
  - Power function for integer exponents
  - Value clamping between bounds
  - Comprehensive arithmetic operations
  - MinValue and MaxValue constants for range boundaries
  - Modulo operations for both Fxp and integer types
  - **Operations between different FixedPoint formats**:
    - Multiplication and division are supported (result uses left operand's format)
    - Addition, subtraction, comparison, and modulo are NOT supported between different formats by design
    - Use `Convert()` for explicit conversion when needed; the safe overload is silent, while the lossy overload (precision loss or overflow risk) is `[[deprecated]]` to warn the developer
    ```cpp
    FixedPoint<16, 16> a = 10;
    FixedPoint<24, 8> b = 5;
    // auto sum = a + b;                      // ERROR: not allowed
    auto sum = a + FixedPoint<16, 16>::Convert(b); // OK (deprecation warning if lossy)
    auto product = a * b;                          // OK: result is FixedPoint<16, 16>
    ```
  - Flexible value conversion:
    ```cpp
    // Compile-time conversion (preferred)
    constexpr Fxp a = 3.14159;   // Exact conversion at compile-time (float)
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
  - **Advanced Usage**: For experts who understand the fixed-point format, direct raw value manipulation and hardware-level parallel division are available:
    ```cpp
    // Create from raw value (for advanced users only)
    Fxp raw = Fxp::BuildRaw(0x00010000);  // 1.0 in 16.16 format

    // Hardware-optimized parallel division (overlaps CPU work with DIVU)
    Fxp a(10), b(3), c(4), d(5), e(2);
    Fxp cd;
    Fxp r = (a / ParallelDiv(b, [&]{ cd = c * d; })) * e;
    // a/b runs on the DIVU hardware while c*d executes on the CPU
    ```
  - **Comparison Operators**: Comprehensive comparison support with both compile-time and runtime operation:
    ```cpp
    // All of these work at both compile-time and runtime
    Fxp a(5);
    Fxp b(3);
    
    if (a > b) { /* works fine */ }
    if (a == 5) { /* works fine */ }
    if (a > 40.0) { /* works fine */ }
    
    // Reversed operand order also works (int/float on left side)
    int value = GetRuntimeValue();
    Fxp d(10);
    if (value < d) { /* works fine */ }
    if (40.0 < d) { /* works fine */ }
    if (5 == a) { /* works fine */ }
    ```
  - **Conversion Best Practices**:
    ```cpp
    // For COMPILE-TIME literals, use direct constructor:
    constexpr Fxp a(5);      // Integer literal - efficient
    constexpr Fxp b(3.14);   // Float literal - efficient at compile-time
    
    // For RUNTIME integer values, use the Convert method:
    int runtimeInt = GetValue();
    Fxp c = Fxp::Convert(runtimeInt); // Convert method - safer with range checking
    
    // For RUNTIME float values, avoid if possible (CPU intensive):
    float runtimeFloat = GetValue();
    // Fxp d(runtimeFloat);          // ERROR: Won't compile, constructor only works with compile-time floats
    Fxp d = Fxp::Convert(runtimeFloat); // Works but VERY expensive on Saturn hardware
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

All vector, matrix, and geometric types are templated with `template<int I = 16, int F = 16>`, using `FixedPoint<I, F>` internally. Aliases are provided for the default Q16.16 precision:

| Template | Alias (Q16.16) | Custom Precision Example |
|---|---|---|
| `Vector2<I, F>` | `Vector2D` | `Vector2<24, 8>` |
| `Vector3<I, F>` | `Vector3D` | `Vector3<8, 24>` |
| `Matrix3x3<I, F>` | `Matrix33` | `Matrix3x3<24, 8>` |
| `Matrix4x3<I, F>` | `Matrix43` | `Matrix4x3<24, 8>` |
| `AABBX<I, F>` | `AABB` | `AABBX<24, 8>` |
| `SphereX<I, F>` | `Sphere` | `SphereX<24, 8>` |
| `PlaneX<I, F>` | `Plane` | `PlaneX<24, 8>` |
| `FrustumX<I, F>` | `Frustum` | `FrustumX<24, 8>` |
| `MatrixStackX<I, F>` | `MatrixStack` | `MatrixStackX<24, 8>` |

> **Design note**: For vector/matrix types, the template name is descriptive (e.g. `Vector3`, `Matrix4x3`) and the alias is a shorter legacy name (e.g. `Vector3D`, `Matrix43`). For geometric primitives and MatrixStack, the template name uses an `X` suffix (e.g. `AABBX`, `SphereX`) since the base name is already maximally descriptive — the `X` suffix distinguishes the template from the alias. For non-default precisions, use the template directly (e.g. `AABBX<24, 8>`). If a shorter name is needed in user code, a local `using` declaration is recommended.

- **2D Vectors**: `Vector2<I, F>` with optimized operations
  - Unit vectors (UnitX, UnitY)
  - Directional vectors (Left, Right, Up, Down)
  - `TurboLength()` for fast alpha-beta-gamma length approximation
- **3D Vectors**: `Vector3<I, F>` extending `Vector2<I, F>` functionality
  - Unit vectors (UnitX, UnitY, UnitZ)
  - Optimized cross/dot products
  - `TurboLength()` for fast alpha-beta-gamma length approximation
  - Optimized operators for integral types
- **Matrix Operations**: Efficient `Matrix3x3<I, F>` and `Matrix4x3<I, F>` implementations
  - Common transformations (scale, rotate, translate)
  - Optimized multiplication with detailed documentation
  - Identity/zero matrix constants
  - Row-based layout with clear geometric meaning
  - Orthonormal basis in right-handed coordinate system
  - Billboard matrix creation
  - Look-at matrix for camera positioning
  - Transform decomposition into scale/rotation/translation
  - EulerAngles support for rotations
- **Matrix Stack**: `MatrixStackX<I, F>` (alias: `MatrixStack`) — Fixed-size stack for transform hierarchies
  - No dynamic allocation
  - Depth checking
  - Direct transformation methods

### Geometric Primitives
- **AABB**: `AABBX<I, F>` (alias: `AABB`) — Axis-aligned bounding box with comprehensive intersection tests
  - Fast min/max calculations
  - Volume and surface area computation
  - Merging and expansion operations
- **Sphere**: `SphereX<I, F>` (alias: `Sphere`) — Perfect sphere with exact collision detection
  - Point containment tests
  - Sphere-sphere intersection
  - Sphere-AABB intersection
- **Plane**: `PlaneX<I, F>` (alias: `Plane`) — Infinite plane with normal and distance representation
  - Point-plane distance calculation
  - Construction from points/normal
  - Normalization utilities
- **Frustum**: `FrustumX<I, F>` (alias: `Frustum`) — View frustum for efficient visibility culling
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
- **Class-Based Interpolation**: Each type provides its own specialized interpolation methods
  - Angle class
    - SLerp (spherical linear interpolation) for smooth angle transitions
    - Automatic shortest-path selection
  - Fxp class
    - Lerp (linear interpolation)
    - Smoothstep for smooth acceleration and deceleration (3t² - 2t³)
    - Smootherstep for even smoother transitions (6t⁵ - 15t⁴ + 10t³)
    - EaseIn/Out (quadratic) for accelerating/decelerating motion
    - CubicEaseIn/Out for stronger acceleration/deceleration
    - ElasticEaseIn for spring-like motion
    - BounceEaseIn/Out for bouncing ball effects
    - CubicBezier for custom easing curves
  - Vector types
    - Lerp for component-wise linear interpolation
    - Additional vector-specific interpolation

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

// Default precision (Q16.16) — use legacy aliases
Vector3D position(1, 2, 3);
Vector3D direction = Vector3D::UnitZ();  // Forward direction (0,0,1)
Vector3D normal = direction.Normalize();

// Custom precision — use template directly
Vector3<24, 8> worldPos(1000, 2000, 3000);       // Large-world coordinates
Matrix4x3<24, 8> worldTransform = Matrix4x3<24, 8>::Identity();

// 2D vector operations
Vector2D screenPos = Vector2D::Zero();
screenPos += Vector2D::Right() * 10;  // Move 10 units right
screenPos += Vector2D::Up() * 5;      // Move 5 units up

// Matrix operations
Matrix43 transform = Matrix43::Identity();
transform.Translate(Vector3D::UnitY() * 5);  // Move 5 units up
transform.RotateY(Angle::FromDegrees(90));   // Rotate 90° around Y

// Matrix decomposition
Vector3D scale, rotation, translation;
transform.Decompose(scale, rotation, translation);

// Transform the position
Vector3D transformed = transform * position;
```

### Geometric Operations

```cpp
// Create and manipulate geometric primitives (default Q16.16 via aliases)
Vector3D center(0, 0, 0);
Vector3D size(2, 2, 2);
AABB box(center, size);

// Or use custom precision with the X-suffix template
AABBX<24, 8> largeBox(Vector3<24, 8>(1000, 2000, 3000), Vector3<24, 8>(10, 10, 10));

// Calculate normal vector for a triangle
Vector3D v1(0, 0, 0), v2(1, 0, 0), v3(0, 1, 0);
Vector3D normal = Vector3D::CalcNormal(v1, v2, v3);

// Collision detection using the Collision namespace
Sphere sphere(center, Fxp(2));
bool collision = SaturnMath::Collision::Intersects(box, sphere);

// Create view matrix
Vector3D eye(0, 5, -10);
Vector3D target(0, 0, 0);
Vector3D up = Vector3D::UnitY();
Matrix43 view = Matrix43::CreateLookAt(eye, target, up);

// Frustum culling
Frustum viewFrustum(fov, aspect, nearDist, farDist);
bool isVisible = viewFrustum.Intersects(box);
```

### Fixed-Point and Angle Operations

```cpp
#include "saturn_math.hpp"
using namespace SaturnMath::Types;

// Fixed-point arithmetic
Fxp a(5);                     // Create from int16_t (works at compile-time or runtime)
Fxp b(2.5);                  // Create from float (compile-time only)
Fxp c = a * b;                // 12.5 (0x000C8000)
int16_t i = c.As<int16_t>();  // Convert back to int16_t

// Power and clamping operations
Fxp squared = a.Pow(2);    // 25
Fxp clamped = b.Clamp(0, 2); // 2

// Interpolation and easing functions
Fxp interpolated = Fxp::Lerp(a, b, Fxp(0.5));         // Linear interpolation
Fxp smooth = Fxp::Smoothstep(a, b, Fxp(0.5));        // Smooth acceleration/deceleration
Fxp smoother = Fxp::Smootherstep(a, b, Fxp(0.5));     // Even smoother transition
Fxp easeIn = Fxp::EaseIn(a, b, Fxp(0.5));           // Quadratic acceleration
Fxp easeOut = Fxp::EaseOut(a, b, Fxp(0.5));         // Quadratic deceleration
Fxp bounce = Fxp::BounceEaseOut(a, b, Fxp(0.5));    // Bouncing ball effect

// Fixed-point conversion methods
// 1. COMPILE-TIME construction (preferred when values are known at compile time)
constexpr Fxp compile_time = 3.14159;         // Exact conversion at compile-time (float)
constexpr Fxp compile_time_int = 42;           // Integer conversion at compile-time (int16_t)

// 2. RUNTIME conversion with static Convert() (for values not known at compile time)
int32_t runtime_int = 12345;
float runtime_float = 67.89;
Fxp int_conversion = Fxp::Convert(runtime_int);   // With range checking
Fxp float_conversion = Fxp::Convert(runtime_float); // Warning: heavy operation

// 3. Converting FROM Fxp back to other types
int16_t as_int = compile_time.As<int16_t>();    // Convert to int16_t
float as_float = compile_time.As<float>();      // Convert to float (warning: heavy)

// Angle calculations
Angle rotation = Angle::FromDegrees(45);
Fxp sine = SaturnMath::Trigonometry::Sin(rotation);
Fxp cosine = SaturnMath::Trigonometry::Cos(rotation);

// Example of angle arithmetic
Angle doubled = rotation * Fxp(2);    // Double the angle
Angle halved = rotation / Fxp(2);     // Half the angle

// Euler angles for 3D rotation
EulerAngles orientation(
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
Angle interpolated = Angle::SLerp(start, end, Fxp(0.5));

// Utility functions
using namespace SaturnMath;
auto maxInt = Max(5, 3);                      // Works with integers
auto absInt = Abs(-42);                       // Works with integers
auto clampedInt = Clamp(7, 0, 5);             // Works with integers
auto maxVec = Max(Vector2D(1, 5), Vector2D(3, 2));  // Works with vectors (component-wise)
```

## Performance Considerations

### Performance Features
- Template-based `FixedPoint<I, F>` and `Vector/Matrix<I, F>` allowing per-use-case format selection (range vs precision)
- Cache-friendly data layouts
- Fixed-size containers to avoid dynamic allocation
- Lookup table-based trigonometry
- Compile-time constant evaluation
- Optimized operations for integral types with specialized implementations
- Hardware-optimized multiplication (`dmuls.l`) and division (hardware divider unit) on SH-2
### Precision Control (Deprecated)

> ⚠️ **Deprecation Notice**: The template-based precision modes (`Precision::Accurate`, `Precision::Fast`, `Precision::Turbo`, `Precision::Default`) are **deprecated**. In practice they have proven to add significant template complexity and API surface area without delivering meaningful real-world performance benefits to justify the trade-off. The precision template parameter is preserved on existing methods only to avoid breaking call sites — it is ignored at runtime. New code should not use precision modes; future versions will remove them entirely.

Historically, SaturnMath++ provided template-based precision control via a `Precision` enum template parameter on methods like `Normalize()`, `Length()`, and `CalcNormal()`. These have been replaced by single optimized implementations. For fast length approximation, use `TurboLength()` explicitly.

### Testing

SaturnMath++ includes a comprehensive suite of compile-time tests that verify the correctness of all mathematical operations. These tests ensure that:

- All operations produce expected results within appropriate tolerance ranges
- Edge cases (zero values, perfect squares, very small values) are handled correctly
- Cross-format FixedPoint conversions preserve values correctly

All tests are implemented as static assertions that run at compile-time, ensuring zero runtime overhead while providing strong correctness guarantees.

### Optimized Integer Operations

SaturnMath++ provides specialized operators for working with integral types, offering better performance on Saturn hardware:

```cpp
// Optimized vector operations with integers
Vector3D position(1, 2, 3);
Vector3D doubled = position * 2;     // Uses optimized integral multiplication
Vector3D halved = position / 2;      // Uses optimized integral division

// Component-wise utility functions work with both Fxp and integers
auto maxVal = Max(5, 3);             // Works with integers
auto absVal = Abs(-42);              // Works with integers
auto clampedVal = Clamp(7, 0, 5);    // Works with integers

// Component-wise operations on vectors
auto maxVec = Max(Vector2D(1, 5), Vector2D(3, 2));  // Returns Vector2D(3, 5)
```

### Best Practices
- ~~Use `Fast` or `Turbo` precision for non-critical calculations where performance is important~~ *(precision modes are deprecated; rely on the default implementation)*
- Choose the appropriate `FixedPoint<I, F>` format for the job (`Fxp16_16` for general use, `Fxp24_8` for large worlds, `Fxp8_24` for normalized/precision-sensitive values)
- Use the same `I, F` precision for vectors and matrices as for the `FixedPoint` values they interact with (e.g. `Vector3<24, 8>` with `Fxp24_8`)
- Legacy aliases (`Vector2D`, `Vector3D`, `Matrix33`, `Matrix43`, `AABB`, `Sphere`, `Plane`, `Frustum`, `MatrixStack`) default to Q16.16 — use `X` suffix templates (e.g. `AABBX<24, 8>`) for other precisions
- If a shorter name for a custom precision is needed, use a local `using` declaration rather than relying on library-provided aliases
- Avoid implicit conversions between FixedPoint formats — be explicit with `Convert()` and pay attention to the deprecation warning that flags lossy conversions
- Prefer fixed-size containers (like `MatrixStack`) over dynamic allocation
- Take advantage of lookup-based trig functions for better performance
- Leverage compile-time constants with `consteval` methods
- Structure data for optimal cache usage on SH-2

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
