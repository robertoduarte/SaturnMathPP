#pragma once
#include <stdint.h>
#include <concepts>
#include "hardware.hpp"
#include "integer.hpp"
#include "fxp.hpp"

namespace SaturnMath
{
    /**
     * @brief Get absolute value
     *
     * @details Works with any type that supports comparison operators.
     *
     * @tparam ValueType Type of the value
     * @param value Numeric value
     * @return Absolute value
     */
    template<typename ValueType>
    constexpr static ValueType Abs(const ValueType& value)
    {
        return value < 0 ? -value : value;
    }

    /**
     * @brief Get maximum value of two values
     *
     * @details Works with any type that supports comparison operators.
     *
     * @tparam ValueType Type of the value
     *
     * @param first First value
     * @param second Second value
     * @return Maximum value
     */
    template<typename ValueType>
    constexpr static ValueType Max(const ValueType& first, const ValueType& second)
    {
        return first > second ? first : second;
    }

    /**
     * @brief Get minimum value of two values
     *
     * @details Works with any type that supports comparison operators.
     *
     * @tparam ValueType Type of the value
     *
     * @param first First value
     * @param second Second value
     * @return Minimum value
     */
    template<typename ValueType>
    constexpr static ValueType Min(const ValueType& first, const ValueType& second)
    {
        return first < second ? first : second;
    }

    /**
     * @brief Get minimum value of three values
     *
     * @details Works with any type that supports comparison operators.
     *
     * @tparam ValueType Type of the value
     *
     * @param first First value
     * @param second Second value
     * @param third Third value
     * @return Minimum value
     */
    template<typename ValueType>
    constexpr static ValueType Min(const ValueType& first, const ValueType& second, const ValueType& third)
    {
        return Min(Min(first, second), third);
    }

    /**
     * @brief Clamps a value between min and max bounds.
     *
     * @details Works with any type that supports comparison operators.
     *
     * @tparam T Type of the values
     * @param value Value to clamp
     * @param min Minimum allowed value
     * @param max Maximum allowed value
     * @return Clamped value
     */
    template<typename T>
    static constexpr T Clamp(const T& value, const T& min, const T& max)
    {
        return value < min ? min : (value > max ? max : value);
    }

    /**
     * @deprecated Use #include <integer.hpp> and SaturnMath::Integer instead.
     *             To be removed in a future version.
     */
    using Integer [[gnu::deprecated("Use #include <integer.hpp> instead. To be removed in a future version.")]] = SaturnMath::Integer;
}
