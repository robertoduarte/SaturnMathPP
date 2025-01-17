#pragma once

#include "fxp.hpp"
#include "angle.hpp"

namespace SaturnMath
{
    /**
     * @brief Class containing interpolation and easing functions optimized for fixed-point arithmetic.
     *
     * This class provides a comprehensive set of interpolation and easing functions designed for game development.
     * All functions are implemented using fixed-point arithmetic for consistent behavior across platforms.
     *
     * Key features:
     * - Compile-time evaluation with constexpr
     * - No floating-point operations
     * - Optimized for Saturn hardware
     * - Memory-efficient implementations
     */
    class Interpolation
    {
    public:
        /**
         * @brief Linear interpolation between two fixed-point values.
         *
         * Performs linear interpolation (lerp) between start and end values using the formula:
         * result = start + (end - start) * t
         *
         * @details The interpolation follows this pattern:
         * - When t = 0, returns start
         * - When t = 1, returns end
         * - When t = 0.5, returns the midpoint
         *
         * Common uses:
         * - Camera movement
         * - UI element transitions
         * - Color blending
         * - Position interpolation
         *
         * @param start Starting value of the interpolation
         * @param end Ending value of the interpolation
         * @param t Interpolation factor in range [0,1]
         * @return Interpolated value
         *
         * @note For best performance, ensure t is pre-clamped to [0,1]
         */
        static constexpr Fxp Lerp(const Fxp& start, const Fxp& end, const Fxp& t)
        {
            return start + (end - start) * t;
        }

        /**
         * @brief Smoothstep interpolation for smooth acceleration and deceleration.
         *
         * Implements Ken Perlin's smoothstep function using the formula:
         * 3t² - 2t³
         *
         * @details The smoothstep produces this behavior:
         * - Smooth acceleration from start
         * - Constant velocity at midpoint
         * - Smooth deceleration to end
         *
         * Common uses:
         * - Camera transitions
         * - Smooth UI animations
         * - Particle system parameters
         * - Fade effects
         *
         * @param start Starting value of the interpolation
         * @param end Ending value of the interpolation
         * @param t Interpolation factor, automatically clamped to [0,1]
         * @return Smoothly interpolated value
         *
         * @note Automatically clamps input t to [0,1] for safety
         */
        static constexpr Fxp Smoothstep(const Fxp& start, const Fxp& end, const Fxp& t)
        {
            Fxp x = (t < 0) ? 0 : ((t > 1) ? 1 : t);
            Fxp factor = x * x * (Fxp(3) - Fxp(2) * x);
            return Lerp(start, end, factor);
        }

        /**
         * @brief Clamps a value between minimum and maximum bounds.
         *
         * Ensures a value stays within specified bounds using the formula:
         * result = min(max(value, min), max)
         *
         * @details Clamping behavior:
         * - If value < min, returns min
         * - If value > max, returns max
         * - Otherwise, returns value unchanged
         *
         * Common uses:
         * - Constraining player movement
         * - Limiting camera angles
         * - Bounding UI element positions
         * - Normalizing input values
         *
         * @param value Value to be clamped
         * @param min Minimum allowed value
         * @param max Maximum allowed value
         * @return Clamped value
         *
         * @note Assumes min <= max
         */
        static constexpr Fxp Clamp(const Fxp& value, const Fxp& min, const Fxp& max)
        {
            return (value < min) ? min : ((value > max) ? max : value);
        }

        /**
         * @brief Quadratic ease-in interpolation for accelerating motion.
         *
         * Implements quadratic easing using the formula:
         * t²
         *
         * @details The easing produces this motion:
         * - Starts slow (zero velocity)
         * - Continuously accelerates
         * - Reaches full velocity at end
         *
         * Common uses:
         * - Character movement startup
         * - UI element entrance
         * - Zoom-in effects
         * - Power-up animations
         *
         * @param start Starting value of the interpolation
         * @param end Ending value of the interpolation
         * @param t Interpolation factor in range [0,1]
         * @return Eased value
         *
         * @note For symmetric animation, pair with EaseOut
         */
        static constexpr Fxp EaseIn(const Fxp& start, const Fxp& end, const Fxp& t)
        {
            Fxp factor = t * t;
            return Lerp(start, end, factor);
        }

        /**
         * @brief Quadratic ease-out interpolation for decelerating motion.
         *
         * Implements quadratic easing using the formula:
         * -t * (t - 2)
         *
         * @details The easing produces this motion:
         * - Starts at full velocity
         * - Continuously decelerates
         * - Stops smoothly (zero velocity)
         *
         * Common uses:
         * - Character movement stop
         * - UI element exit
         * - Zoom-out effects
         * - Landing animations
         *
         * @param start Starting value of the interpolation
         * @param end Ending value of the interpolation
         * @param t Interpolation factor in range [0,1]
         * @return Eased value
         *
         * @note For symmetric animation, pair with EaseIn
         */
        static constexpr Fxp EaseOut(const Fxp& start, const Fxp& end, const Fxp& t)
        {
            Fxp factor = -t * (t - Fxp(2));
            return Lerp(start, end, factor);
        }

        /**
         * @brief Cubic ease-in interpolation for stronger acceleration.
         *
         * Implements cubic easing using the formula:
         * t³
         *
         * @details The easing produces this motion:
         * - Very slow start
         * - Rapid acceleration
         * - Maximum velocity at end
         *
         * Common uses:
         * - Dramatic entrances
         * - Power-up effects
         * - Explosion start
         * - Heavy object movement
         *
         * @param start Starting value of the interpolation
         * @param end Ending value of the interpolation
         * @param t Interpolation factor in range [0,1]
         * @return Eased value
         *
         * @note More pronounced than quadratic EaseIn
         */
        static constexpr Fxp CubicEaseIn(const Fxp& start, const Fxp& end, const Fxp& t)
        {
            Fxp factor = t * t * t;
            return Lerp(start, end, factor);
        }

        /**
         * @brief Cubic ease-out interpolation for stronger deceleration.
         *
         * Implements cubic easing using the formula:
         * (t - 1)³ + 1
         *
         * @details The easing produces this motion:
         * - Maximum velocity at start
         * - Rapid deceleration
         * - Very slow end
         *
         * Common uses:
         * - Dramatic exits
         * - Impact effects
         * - Explosion end
         * - Heavy object stopping
         *
         * @param start Starting value of the interpolation
         * @param end Ending value of the interpolation
         * @param t Interpolation factor in range [0,1]
         * @return Eased value
         *
         * @note More pronounced than quadratic EaseOut
         */
        static constexpr Fxp CubicEaseOut(const Fxp& start, const Fxp& end, const Fxp& t)
        {
            Fxp tmp = t - 1;
            Fxp factor = tmp * tmp * tmp + 1;
            return Lerp(start, end, factor);
        }

        /**
         * @brief Elastic ease-in interpolation for spring-like motion.
         *
         * Implements elastic easing with configurable period and amplitude.
         * Uses quadratic approximation of sine for efficiency.
         *
         * @details The easing produces this motion:
         * - Multiple overshoots
         * - Decreasing amplitude
         * - Final snap to position
         *
         * Common uses:
         * - Menu bounces
         * - Character stretching
         * - Projectile charge-up
         * - Spring animations
         *
         * @param start Starting value of the interpolation
         * @param end Ending value of the interpolation
         * @param t Interpolation factor in range [0,1]
         * @return Eased value
         *
         * @note Uses approximated sine to avoid trigonometric tables
         */
        static constexpr Fxp ElasticEaseIn(const Fxp& start, const Fxp& end, const Fxp& t)
        {
            if (t <= 0) return start;
            if (t >= 1) return end;

            constexpr Fxp period = 0.3;
            constexpr Fxp s = period / 4;

            Fxp tmp = t - 1;
            Fxp postFactor = tmp * tmp;
            Fxp preFactor = -Fxp(2).Pow(t * 10);

            Fxp angle = (t - s) * 6.28318530718 / period;
            Fxp sinApprox = (angle - angle * angle * angle / 6) * 4;

            Fxp factor = preFactor * sinApprox;
            return Lerp(start, end, factor);
        }

        /**
         * @brief Bounce ease-out interpolation for bouncing ball effect.
         *
         * Implements bouncing using piecewise quadratic functions.
         * Simulates diminishing bounces of an elastic ball.
         *
         * @details The motion consists of four phases:
         * - Initial fall (36% of time)
         * - First bounce (36% of time)
         * - Second bounce (18% of time)
         * - Final small bounces (10% of time)
         *
         * Common uses:
         * - Dropping objects
         * - UI element landings
         * - Character jump landing
         * - Button clicks
         *
         * @param start Starting value of the interpolation
         * @param end Ending value of the interpolation
         * @param t Interpolation factor in range [0,1]
         * @return Eased value
         *
         * @note Each bounce is approximately 75% the height of previous
         */
        static constexpr Fxp BounceEaseOut(const Fxp& start, const Fxp& end, const Fxp& t)
        {
            Fxp factor;

            if (t < Fxp(0.36363636)) // 4/11
            {
                factor = Fxp(7.5625) * t * t;
            }
            else if (t < Fxp(0.72727272)) // 8/11
            {
                Fxp tmp = t - Fxp(0.54545454); // 6/11
                factor = Fxp(7.5625) * tmp * tmp + Fxp(0.75);
            }
            else if (t < Fxp(0.90909090)) // 10/11
            {
                Fxp tmp = t - Fxp(0.81818181); // 9/11
                factor = Fxp(7.5625) * tmp * tmp + Fxp(0.9375);
            }
            else
            {
                Fxp tmp = t - Fxp(0.95454545); // 21/22
                factor = Fxp(7.5625) * tmp * tmp + Fxp(0.984375);
            }

            return Lerp(start, end, factor);
        }

        /**
         * @brief Bounce ease-in interpolation for reverse bouncing ball effect.
         *
         * Implements bouncing by reversing BounceEaseOut.
         * Creates a series of bounces that converge to the start.
         *
         * @details The motion consists of four phases in reverse:
         * - Small initial bounces (10% of time)
         * - Medium bounce (18% of time)
         * - Large bounce (36% of time)
         * - Final launch (36% of time)
         *
         * Common uses:
         * - Object launches
         * - UI element takeoffs
         * - Character jump startup
         * - Power-up activation
         *
         * @param start Starting value of the interpolation
         * @param end Ending value of the interpolation
         * @param t Interpolation factor in range [0,1]
         * @return Eased value
         *
         * @note Reverses BounceEaseOut for symmetric animations
         */
        static constexpr Fxp BounceEaseIn(const Fxp& start, const Fxp& end, const Fxp& t)
        {
            return Lerp(end, start, BounceEaseOut(Fxp(0), Fxp(1), Fxp(1) - t));
        }
    };
}
