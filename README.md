<p align="center">
<img src="https://github.com/robertoduarte/SaturnMathPP/blob/main/documentation/resources/smpp_sqrt_pi_logo.svg" >
</p>

# SaturnMath++ (C++23)
SaturnMath++ is a C++23 library dedicated to Sega Saturn hardware, offering essential mathematical operations tailored for fixed-point arithmetic and geometric calculations.
## Overview

SaturnMath++ is designed specifically for the Sega Saturn's hardware constraints and capabilities:
- Uses fixed-point arithmetic instead of floating-point
- Optimized for SH-2's 32-bit operations
- Cache-friendly data layouts
- Compile-time optimizations with constexpr/consteval
- Zero dynamic memory allocation

## Features

### Core Components
- **Fixed-Point Arithmetic**: High-performance `Fxp` class with precise fixed-point operations
- **Angle Handling**: Type-safe `Angle` class for angular calculations
- **Integer Math**: Efficient integer operations via `IntegerUtils`

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
  - Optimized multiplication
  - Identity/zero matrix constants
  - Precision-controlled decomposition and view matrix creation
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
using namespace SaturnMath;

// 3D vector operations with different precision levels
Vector3D position(1, 2, 3);
Vector3D direction = Vector3D::UnitZ();  // Forward direction (0,0,1)

// Standard precision for accurate results
Vector3D normalizedDir = direction.Normalize<Precision::Standard>();

// Fast precision for better performance
float length = position.Length<Precision::Fast>();

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

// Create view matrix with precision control
Vector3D eye(0, 5, -10);
Vector3D target(0, 0, 0);
Vector3D up = Vector3D::UnitY();
Matrix43 view = Matrix43::CreateLookAt<Precision::Standard>(eye, target, up);
```

### Fixed-Point and Angle Operations

```cpp
#include "saturn_math.hpp"
using namespace SaturnMath;

// Fixed-point arithmetic
Fxp a(5);                   // 5 (0x00050000)
Fxp b(2.5);                // 2.5 (0x00028000)
Fxp c = a * b;             // 12.5 (0x000C8000)
int16_t i = c.ToInt();     // 12

// Angle calculations
Angle rotation = Angle::FromDegrees(45);
Fxp sine = Sin(rotation);
Fxp cosine = Cos(rotation);

// Efficient simultaneous calculation
auto [sin, cos] = SinCos(rotation);

// Integer math utilities
uint32_t value = 100;
uint32_t sqrt = IntegerUtils::Sqrt<Precision::Fast>(value);  // Quick square root calculation
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
