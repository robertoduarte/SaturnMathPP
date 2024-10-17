# SaturnMath++ (C++23)

SaturnMath++ is a C++23 library dedicated to Sega Saturn hardware, offering essential mathematical operations tailored for fixed-point arithmetic and geometric calculations.

## Features

- **Matrices and Vectors**: Efficient implementations for 3x3 matrices (`Mat33`) and 4x3 matrices (`Mat43`), along with 3D vector operations (`Vec3`).
- **Frustum**: A robust implementation of a view frustum (`Frustum`) for determining visible objects within a 3D scene using plane equations.
- **Fixed-Point Arithmetic**: Includes the `Fxp` class for precise fixed-point arithmetic operations, crucial for Sega Saturn's hardware limitations.
- **Geometry Calculations**: Provides methods for matrix multiplication, vector transformations, and geometric tests such as point-in-frustum and sphere-in-frustum checks.

## Usage

To use SaturnMath++ in your C++23 project for Sega Saturn:
1. Clone the repository or download the source files.
2. Include the main header file (`saturn_math.hpp`) in your project.
3. Instantiate objects of `Mat33`, `Mat43`, `Vec3`, `Frustum`, and `Fxp` to perform matrix operations, vector transformations, fixed-point arithmetic, or frustum visibility checks.

## Example usage of matrices and vectors

```cpp
#include "saturn_math.hpp"

using namespace SaturnMath;

int main()
{
    // Example usage of matrices and vectors
    Vec3 up(0.0, 1.0, 0.0);
    Vec3 direction(1.0, 0.0, 0.0);
    Mat33 matrix(up, direction);
    Vec3 transformedVector = matrix * Vec3(1.0, 0.0, 0.0);

    return 0;
}

## Example usage of frustum

```cpp
#include "saturn_math.hpp"

using namespace SaturnMath;

int main()
{
    // Example usage of frustum
    Frustum frustum(60.0, 1.333, 0.1, 100.0);
    Vec3 position(0.0, 0.0, 0.0);
    Vec3 xAxis(1.0, 0.0, 0.0);
    Vec3 yAxis(0.0, 1.0, 0.0);
    Vec3 zAxis(0.0, 0.0, 1.0);
    frustum.Update(position, xAxis, yAxis, zAxis);
    bool isInFrustum = frustum.PointInFrustum(Vec3(1.0, 1.0, 1.0));

    return 0;
}

## Example usage of fixed-point arithmetic

```cpp
#include "saturn_math.hpp"

using namespace SaturnMath;

int main()
{
    // Example usage of fixed-point arithmetic
    Fxp fixedPointValue(1.5); // Represents 1.5 in fixed-point format
    Fxp fixedPointSquare = fixedPointValue.Square(); // Computes the square of the value

    return 0;
}