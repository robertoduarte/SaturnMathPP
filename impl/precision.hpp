#pragma once

namespace SaturnMath
{
    /**
     * @brief Specifies the precision level for mathematical operations
     * 
     * Different precision levels offer a trade-off between accuracy and performance:
     * - Standard: Full precision calculations, ideal for critical computations
     * - Fast: Good approximation with better performance
     * - Turbo: Fastest calculation with acceptable accuracy, best for real-time effects
     */
    enum class Precision
    {
        Standard,   ///< Standard precision using full calculations
        Fast,       ///< Fast approximation with good accuracy
        Turbo       ///< Fastest approximation with acceptable accuracy
    };
}
