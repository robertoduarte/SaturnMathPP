#pragma once

namespace SaturnMath
{
    /**
     * @brief Specifies the precision level for mathematical operations
     * 
     * Different precision levels offer a trade-off between accuracy and performance:
     * - Accurate: Full precision calculations, ideal for critical computations
     * - Fast: Good approximation with better performance
     * - Turbo: Fastest calculation with acceptable accuracy, best for real-time effects
     * - Default: Automatically selects precision based on MATH_PERFORMANCE_MODE macro
     */
    enum class Precision
    {
        Accurate,   ///< Full precision using accurate calculations
        Fast,       ///< Fast approximation with good accuracy
        Turbo,      ///< Fastest approximation with acceptable accuracy
        
        #if defined(MATH_PERFORMANCE_MODE) && MATH_PERFORMANCE_MODE == ACCURATE
            Default = Accurate  ///< Default precision based on build configuration
        #elif defined(MATH_PERFORMANCE_MODE) && MATH_PERFORMANCE_MODE == TURBO
            Default = Turbo     ///< Default precision based on build configuration
        #else
            Default = Fast      ///< Default precision (Fast when not specified)
        #endif
    };
}
