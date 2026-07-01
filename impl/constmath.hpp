#pragma once

#include <cstdint>

namespace SaturnMath
{
    /**
     * @brief Compile-time math functions using pure C++ constexpr.
     *
     * Provides constexpr-compatible sin, cos, tan, atan, and sqrt
     * for use in consteval/constexpr table generation.
     * Uses Taylor series with argument reduction for full double precision.
     *
     * @warning These functions use double arithmetic and should only be called
     * in compile-time (constexpr/consteval) contexts. Runtime calls on SH-2
     * would require floating-point support that is not available.
     */
    class ConstexprMath final
    {
    private:
        static constexpr double pi = 3.14159265358979323846;
        static constexpr double twoPi = 2.0 * pi;
        static constexpr double halfPi = pi / 2.0;
        static constexpr double quarterPi = pi / 4.0;

        static constexpr double abs(double x) { return x < 0 ? -x : x; }

        /**
         * @brief Reduce angle to [-π, π].
         *
         * @param x Angle in radians.
         * @return Equivalent angle in range [-π, π].
         */
        static constexpr double reduceToPi(double x)
        {
            while (x > pi) x -= twoPi;
            while (x < -pi) x += twoPi;
            return x;
        }

    public:
        /**
         * @brief constexpr square root via Newton's method.
         *
         * @param x Value to compute square root of (must be >= 0).
         * @param guess Initial guess for the iterative method (default: 1.0).
         * @return Approximate square root of x.
         */
        static constexpr double Sqrt(double x, double guess = 1.0)
        {
            if (x < 0) return 0;
            if (x == 0) return 0;
            double next = (guess + x / guess) * 0.5;
            if (abs(next - guess) < 1e-15 * guess) return next;
            return Sqrt(x, next);
        }

        /**
         * @brief constexpr sine via Taylor series with argument reduction.
         *
         * @details Reduces to [-π/2, π/2] then uses 16-term Taylor series.
         *
         * @param x Angle in radians.
         * @return Sine of x.
         */
        static constexpr double Sin(double x)
        {
            // Reduce to [-π, π]
            x = reduceToPi(x);
            // Reduce to [-π/2, π/2] using sin(π - x) = sin(x)
            if (x > halfPi) x = pi - x;
            if (x < -halfPi) x = -pi - x;

            // Taylor series: sin(x) = x - x³/3! + x⁵/5! - ...
            double x2 = x * x;
            double term = x;
            double sum = x;
            for (int n = 1; n <= 16; n++)
            {
                term *= -x2 / static_cast<double>((2 * n) * (2 * n + 1));
                sum += term;
            }
            return sum;
        }

        /**
         * @brief constexpr cosine: cos(x) = sin(x + π/2).
         *
         * @param x Angle in radians.
         * @return Cosine of x.
         */
        static constexpr double Cos(double x)
        {
            return Sin(x + halfPi);
        }

        /**
         * @brief constexpr tangent: tan(x) = sin(x) / cos(x).
         *
         * @details Returns large value when cos(x) ≈ 0 (at π/2).
         *
         * @param x Angle in radians.
         * @return Tangent of x.
         */
        static constexpr double Tan(double x)
        {
            double c = Cos(x);
            if (abs(c) < 1e-15) return (Sin(x) >= 0) ? 1e15 : -1e15;
            return Sin(x) / c;
        }

        /**
         * @brief constexpr arctangent via argument reduction + Taylor series.
         *
         * @details For |x| > 1:  atan(x) = π/2 - atan(1/x)
         * For |x| > 0.5: atan(x) = π/4 + atan((x-1)/(x+1))
         * For |x| ≤ 0.5: Taylor series with 30 terms
         *
         * @param x Value to compute arctangent of.
         * @return Arctangent of x in radians, range [-π/2, π/2].
         */
        static constexpr double Atan(double x)
        {
            // Reduce large arguments
            if (x > 1.0)  return halfPi - Atan(1.0 / x);
            if (x < -1.0) return -halfPi - Atan(1.0 / x);

            // Reduce near-unity arguments
            if (x > 0.5)  return quarterPi + Atan((x - 1.0) / (x + 1.0));
            if (x < -0.5) return -quarterPi + Atan((x - 1.0) / (x + 1.0));

            // Taylor series for |x| ≤ 0.5
            // atan(x) = x - x³/3 + x⁵/5 - x⁷/7 + ...
            double x2 = x * x;
            double term = x;
            double sum = x;
            for (int n = 1; n <= 30; n++)
            {
                term *= -x2;
                sum += term / static_cast<double>(2 * n + 1);
            }
            return sum;
        }

        /**
         * @brief constexpr atan2: full-quadrant arctangent of y/x.
         *
         * @param y Y-coordinate.
         * @param x X-coordinate.
         * @return Angle in radians in range [-π, π], correct for all quadrants.
         */
        static constexpr double Atan2(double y, double x)
        {
            if (x == 0.0)
            {
                if (y > 0.0) return halfPi;
                if (y < 0.0) return -halfPi;
                return 0.0;
            }
            double ratio = y / x;
            if (x > 0.0) return Atan(ratio);
            if (y >= 0.0) return Atan(ratio) + pi;
            return Atan(ratio) - pi;
        }
    };
}
