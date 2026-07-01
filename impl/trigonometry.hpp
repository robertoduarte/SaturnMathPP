#pragma once

#include "fxp.hpp"
#include "angle.hpp"
#include "hardware.hpp"
#include "constmath.hpp"
#include <type_traits>
#include <array>
#include <cstdint>

namespace SaturnMath
{
    using namespace SaturnMath::Types;

    namespace detail
    {
        static constexpr double pi = 3.14159265358979323846;

        // ================================================================
        // LookupCache — interpolation entry with precomputed multiplicand
        // ================================================================

        /**
         * @brief Lookup table entry with hardware-optimized interpolation.
         * @tparam ValueType Result type (int32_t or uint16_t)
         * @tparam InterpolationMask Bit mask for fraction extraction from input
         * @tparam ExtractShift Bit offset for extracting result from 64-bit product
         */
        template<typename ValueType, uint32_t InterpolationMask, uint32_t ExtractShift>
        struct LookupCache
        {
            static constexpr uint32_t mask = InterpolationMask;
            static constexpr uint32_t extractShift = ExtractShift;

            ValueType value;
            ValueType interpolationMultiplicand;

            /**
             * @brief Interpolates between table entries using hardware multiply.
             * @param input Raw angle/fixed-point value containing fractional bits
             * @return Interpolated result in internal fixed-point format
             */
            [[gnu::always_inline]] constexpr ValueType ExtractValue(const auto& input) const
            {
                uint32_t interpolationMultiplier = InterpolationMask & input;

                // Determine if product fits in 32 bits (enables lighter multiply)
                constexpr int maskBits = []() {
                    uint32_t m = InterpolationMask; int b = 0;
                    while (m) { b++; m >>= 1; }
                    return b;
                }();
                constexpr bool fits32 = (maskBits + sizeof(ValueType) * 8) <= 32;

                if consteval
                {
                    if constexpr (fits32)
                    {
                        uint32_t product = interpolationMultiplier *
                            static_cast<uint32_t>(interpolationMultiplicand);
                        return value + static_cast<ValueType>(product >> ExtractShift);
                    }
                    else
                    {
                        int64_t product = static_cast<int64_t>(
                            static_cast<int32_t>(interpolationMultiplier)) *
                            static_cast<int64_t>(interpolationMultiplicand);

                        int32_t mach = static_cast<int32_t>(product >> 32);
                        int32_t macl = static_cast<int32_t>(product & 0xFFFFFFFF);

                        int32_t delta;
                        if constexpr (ExtractShift == 0)
                            delta = macl;
                        else if constexpr (ExtractShift == 16)
                            delta = static_cast<int32_t>(
                                (static_cast<uint32_t>(mach) << 16) |
                                (static_cast<uint32_t>(macl) >> 16));
                        else
                            delta = static_cast<int32_t>(
                                ((static_cast<uint64_t>(static_cast<uint32_t>(mach)) << 32) |
                                 static_cast<uint32_t>(macl)) >> ExtractShift);

                        return value + static_cast<ValueType>(delta);
                    }
                }
                else
                {
                    if constexpr (fits32)
                    {
                        if constexpr (std::is_unsigned_v<ValueType>)
                        {
                            // Unsigned: plain multiply + logical shift
                            // GCC uses mulu.w (16x16->32) + swap.w for >> 16
                            uint32_t product = interpolationMultiplier *
                                static_cast<uint32_t>(interpolationMultiplicand);
                            return value + static_cast<ValueType>(product >> ExtractShift);
                        }
                        else
                        {
                            // Signed: mul.l + arithmetic shift
                            int32_t product = Hardware::Mul32(
                                static_cast<int32_t>(interpolationMultiplier),
                                interpolationMultiplicand);
                            int32_t delta = product;
                            if constexpr (ExtractShift > 0)
                                Hardware::ArithmeticShiftRight<ExtractShift>(delta);
                            return value + static_cast<ValueType>(delta);
                        }
                    }
                    else
                    {
                        // Runtime: SH-2 dmuls.l (signed 32x32->64)
                        int32_t mach, macl;
                        Hardware::Mul64(
                            static_cast<int32_t>(interpolationMultiplier),
                            interpolationMultiplicand, mach, macl);

                        int32_t delta;
                        Hardware::Extract32<ExtractShift>(mach, macl, delta);

                        return value + static_cast<ValueType>(delta);
                    }
                }
            }
        };

        // ================================================================
        // Table generation helpers
        // ================================================================

        /**
         * @brief Computes the interpolation multiplicand for a lookup table entry.
         * @tparam InternalFractionalBits Fixed-point fractional bits of internal representation
         * @tparam ExtractShift Bit offset for extracting result from product
         * @tparam MultiplierBits Number of bits in the interpolation multiplier
         * @param currentValue Function value at the start of the interval
         * @param nextValue Function value at the end of the interval
         * @return Precomputed multiplicand for linear interpolation
         */
        template<int InternalFractionalBits, int ExtractShift, int MultiplierBits>
        constexpr int32_t ComputeMultiplicand(double currentValue, double nextValue)
        {
            int64_t delta = static_cast<int64_t>(nextValue * (1 << InternalFractionalBits))
                          - static_cast<int64_t>(currentValue * (1 << InternalFractionalBits));

            int shiftAmount = ExtractShift - MultiplierBits;
            if (shiftAmount >= 0)
                return static_cast<int32_t>(static_cast<int64_t>(static_cast<uint64_t>(delta) << shiftAmount));
            else
                return static_cast<int32_t>(delta >> (-shiftAmount));
        }

        /**
         * @brief Generates a lookup table array from a compile-time entry builder.
         * @tparam EntryType The LookupCache instantiation for this table
         * @tparam EntryCount Number of entries in the table
         * @tparam EntryBuilder Compile-time function that builds a single entry from an index
         * @return std::array of populated lookup table entries
         */
        template<typename EntryType, int EntryCount, auto EntryBuilder>
        constexpr std::array<EntryType, EntryCount> MakeLookupTable()
        {
            return [&]<int... Indices>(std::integer_sequence<int, Indices...>) {
                return std::array<EntryType, EntryCount>{ EntryBuilder(Indices)... };
            }(std::make_integer_sequence<int, EntryCount>{});
        }

        // ================================================================
        // Sin table — 64 entries, 10-bit interpolation, 8.24 internal format
        // ================================================================

        struct SinSpec
        {
            static constexpr int entryCount = 64;
            static constexpr uint32_t interpolationMask = 0x3FF;
            static constexpr int multiplierBits = 10;
            static constexpr int extractShift = 16;
            static constexpr int internalFractionalBits = 24;
            using ValueType = int32_t;
        };

        using SinEntry = LookupCache<SinSpec::ValueType,
                                     SinSpec::interpolationMask,
                                     SinSpec::extractShift>;

        constexpr SinEntry BuildSinEntry(int index)
        {
            double angle = static_cast<double>(index) / SinSpec::entryCount * 2.0 * pi;
            double nextAngle = static_cast<double>(index + 1) / SinSpec::entryCount * 2.0 * pi;

            return {
                static_cast<int32_t>(ConstexprMath::Sin(angle) * (1 << SinSpec::internalFractionalBits)),
                ComputeMultiplicand<SinSpec::internalFractionalBits,
                                     SinSpec::extractShift,
                                     SinSpec::multiplierBits>(
                    ConstexprMath::Sin(angle), ConstexprMath::Sin(nextAngle))
            };
        }

        inline constexpr auto sinTable = MakeLookupTable<SinEntry, SinSpec::entryCount, BuildSinEntry>();

        // ================================================================
        // Tan table 1 — 15 entries, 10-bit interpolation, 8.24 internal
        // Range: 0 to 0x3C00 (0° to 84.375°), step = 1024 raw units
        // ================================================================

        struct Tan1Spec
        {
            static constexpr int entryCount = 15;
            static constexpr uint32_t interpolationMask = 0x3FF;
            static constexpr int multiplierBits = 10;
            static constexpr int extractShift = 0;
            static constexpr int internalFractionalBits = 24;
            static constexpr uint32_t baseAngle = 0;
            static constexpr int stepSize = 1024;
            using ValueType = int32_t;
        };

        using Tan1Entry = LookupCache<Tan1Spec::ValueType,
                                      Tan1Spec::interpolationMask,
                                      Tan1Spec::extractShift>;

        constexpr Tan1Entry BuildTan1Entry(int index)
        {
            double angle = static_cast<double>(Tan1Spec::baseAngle + index * Tan1Spec::stepSize) / 65536.0 * 2.0 * pi;
            double nextAngle = static_cast<double>(Tan1Spec::baseAngle + (index + 1) * Tan1Spec::stepSize) / 65536.0 * 2.0 * pi;

            return {
                static_cast<int32_t>(ConstexprMath::Tan(angle) * (1 << Tan1Spec::internalFractionalBits)),
                ComputeMultiplicand<Tan1Spec::internalFractionalBits,
                                     Tan1Spec::extractShift,
                                     Tan1Spec::multiplierBits>(
                    ConstexprMath::Tan(angle), ConstexprMath::Tan(nextAngle))
            };
        }

        inline constexpr auto tanTable1 = MakeLookupTable<Tan1Entry, Tan1Spec::entryCount, BuildTan1Entry>();

        // ================================================================
        // Tan table 2 — 3 entries, 8-bit interpolation, 8.24 internal
        // Range: 0x3C00 to 0x3F00 (84.375° to 87.1875°), step = 256 raw units
        // ================================================================

        struct Tan2Spec
        {
            static constexpr int entryCount = 3;
            static constexpr uint32_t interpolationMask = 0x0FF;
            static constexpr int multiplierBits = 8;
            static constexpr int extractShift = 0;
            static constexpr int internalFractionalBits = 24;
            static constexpr uint32_t baseAngle = 0x3C00;
            static constexpr int stepSize = 256;
            using ValueType = int32_t;
        };

        using Tan2Entry = LookupCache<Tan2Spec::ValueType,
                                      Tan2Spec::interpolationMask,
                                      Tan2Spec::extractShift>;

        constexpr Tan2Entry BuildTan2Entry(int index)
        {
            double angle = static_cast<double>(Tan2Spec::baseAngle + index * Tan2Spec::stepSize) / 65536.0 * 2.0 * pi;
            double nextAngle = static_cast<double>(Tan2Spec::baseAngle + (index + 1) * Tan2Spec::stepSize) / 65536.0 * 2.0 * pi;

            return {
                static_cast<int32_t>(ConstexprMath::Tan(angle) * (1 << Tan2Spec::internalFractionalBits)),
                ComputeMultiplicand<Tan2Spec::internalFractionalBits,
                                     Tan2Spec::extractShift,
                                     Tan2Spec::multiplierBits>(
                    ConstexprMath::Tan(angle), ConstexprMath::Tan(nextAngle))
            };
        }

        inline constexpr auto tanTable2 = MakeLookupTable<Tan2Entry, Tan2Spec::entryCount, BuildTan2Entry>();

        // ================================================================
        // Tan table 3 — 3 entries, 6-bit interpolation, 8.24 internal
        // Range: 0x3F00 to 0x3FC0 (87.1875° to 88.59°), step = 64 raw units
        // ================================================================

        struct Tan3Spec
        {
            static constexpr int entryCount = 3;
            static constexpr uint32_t interpolationMask = 0x03F;
            static constexpr int multiplierBits = 6;
            static constexpr int extractShift = 0;
            static constexpr int internalFractionalBits = 24;
            static constexpr uint32_t baseAngle = 0x3F00;
            static constexpr int stepSize = 64;
            using ValueType = int32_t;
        };

        using Tan3Entry = LookupCache<Tan3Spec::ValueType,
                                      Tan3Spec::interpolationMask,
                                      Tan3Spec::extractShift>;

        constexpr Tan3Entry BuildTan3Entry(int index)
        {
            double angle = static_cast<double>(Tan3Spec::baseAngle + index * Tan3Spec::stepSize) / 65536.0 * 2.0 * pi;
            double nextAngle = static_cast<double>(Tan3Spec::baseAngle + (index + 1) * Tan3Spec::stepSize) / 65536.0 * 2.0 * pi;

            return {
                static_cast<int32_t>(ConstexprMath::Tan(angle) * (1 << Tan3Spec::internalFractionalBits)),
                ComputeMultiplicand<Tan3Spec::internalFractionalBits,
                                     Tan3Spec::extractShift,
                                     Tan3Spec::multiplierBits>(
                    ConstexprMath::Tan(angle), ConstexprMath::Tan(nextAngle))
            };
        }

        inline constexpr auto tanTable3 = MakeLookupTable<Tan3Entry, Tan3Spec::entryCount, BuildTan3Entry>();

        // ================================================================
        // Tan table 4 — 3 entries, 4-bit interpolation, 16.16 internal
        // Range: 0x3FC0 to 0x3FF0 (88.59° to 89.45°), step = 16 raw units
        // Values clamped to 32767 to prevent overflow in 16.16 format.
        // ================================================================

        struct Tan4Spec
        {
            static constexpr int entryCount = 3;
            static constexpr uint32_t interpolationMask = 0x00F;
            static constexpr int multiplierBits = 4;
            static constexpr int extractShift = 4;
            static constexpr int internalFractionalBits = 16;
            static constexpr uint32_t baseAngle = 0x3FC0;
            static constexpr int stepSize = 16;
            using ValueType = int32_t;
        };

        using Tan4Entry = LookupCache<Tan4Spec::ValueType,
                                      Tan4Spec::interpolationMask,
                                      Tan4Spec::extractShift>;

        constexpr Tan4Entry BuildTan4Entry(int index)
        {
            double angle = static_cast<double>(Tan4Spec::baseAngle + index * Tan4Spec::stepSize) / 65536.0 * 2.0 * pi;
            double nextAngle = static_cast<double>(Tan4Spec::baseAngle + (index + 1) * Tan4Spec::stepSize) / 65536.0 * 2.0 * pi;

            double tanValue = ConstexprMath::Tan(angle);
            if (tanValue > 32767.0) tanValue = 32767.0;
            double tanNext = ConstexprMath::Tan(nextAngle);
            if (tanNext > 32767.0) tanNext = 32767.0;

            return {
                static_cast<int32_t>(tanValue * (1 << Tan4Spec::internalFractionalBits)),
                ComputeMultiplicand<Tan4Spec::internalFractionalBits,
                                     Tan4Spec::extractShift,
                                     Tan4Spec::multiplierBits>(
                    tanValue, tanNext)
            };
        }

        inline constexpr auto tanTable4 = MakeLookupTable<Tan4Entry, Tan4Spec::entryCount, BuildTan4Entry>();

        // ================================================================
        // Tan table 5 — 5 entries, 2-bit interpolation, 16.16 internal
        // Range: 0x3FF0 to 0x4000 (89.45° to 90°), step = 4 raw units
        // Values clamped to 32767. 5th entry is sentinel for clamping.
        // ================================================================

        struct Tan5Spec
        {
            static constexpr int entryCount = 5;
            static constexpr uint32_t interpolationMask = 0x003;
            static constexpr int multiplierBits = 2;
            static constexpr int extractShift = 2;
            static constexpr int internalFractionalBits = 16;
            static constexpr uint32_t baseAngle = 0x3FF0;
            static constexpr int stepSize = 4;
            using ValueType = int32_t;
        };

        using Tan5Entry = LookupCache<Tan5Spec::ValueType,
                                      Tan5Spec::interpolationMask,
                                      Tan5Spec::extractShift>;

        constexpr Tan5Entry BuildTan5Entry(int index)
        {
            if (index >= 4)
            {
                // Sentinel: tan(90°) clamped to max
                return { 0x7FFFFFFF, 0 };
            }

            double angle = static_cast<double>(Tan5Spec::baseAngle + index * Tan5Spec::stepSize) / 65536.0 * 2.0 * pi;
            double nextAngle = static_cast<double>(Tan5Spec::baseAngle + (index + 1) * Tan5Spec::stepSize) / 65536.0 * 2.0 * pi;

            double tanValue = ConstexprMath::Tan(angle);
            if (tanValue > 32767.0) tanValue = 32767.0;
            double tanNext = ConstexprMath::Tan(nextAngle);
            if (tanNext > 32767.0) tanNext = 32767.0;

            return {
                static_cast<int32_t>(tanValue * (1 << Tan5Spec::internalFractionalBits)),
                ComputeMultiplicand<Tan5Spec::internalFractionalBits,
                                     Tan5Spec::extractShift,
                                     Tan5Spec::multiplierBits>(
                    tanValue, tanNext)
            };
        }

        inline constexpr auto tanTable5 = MakeLookupTable<Tan5Entry, Tan5Spec::entryCount, BuildTan5Entry>();

        // ================================================================
        // Atan2 table — 33 entries, 11-bit interpolation, uint16_t result
        // Maps ratio [0, 1] to angle [0, π/4] in turns
        // ================================================================

        struct Atan2Spec
        {
            static constexpr int entryCount = 33;
            static constexpr uint32_t interpolationMask = 0x7FF;
            static constexpr int multiplierBits = 11;
            static constexpr int extractShift = 16;
            using ValueType = uint16_t;
        };

        using Atan2Entry = LookupCache<Atan2Spec::ValueType,
                                       Atan2Spec::interpolationMask,
                                       Atan2Spec::extractShift>;

        constexpr Atan2Entry BuildAtan2Entry(int index)
        {
            double ratio = static_cast<double>(index) / 32.0;
            double nextRatio = static_cast<double>(index + 1) / 32.0;

            double angle = ConstexprMath::Atan(ratio) / (2.0 * pi);
            double nextAngle = ConstexprMath::Atan(nextRatio) / (2.0 * pi);

            uint16_t value = static_cast<uint16_t>(angle * 65536.0);
            uint16_t nextValue = static_cast<uint16_t>(nextAngle * 65536.0);

            int32_t delta = static_cast<int32_t>(nextValue) - static_cast<int32_t>(value);

            int shiftAmount = Atan2Spec::extractShift - Atan2Spec::multiplierBits;
            int32_t multiplicand;
            if (shiftAmount >= 0)
                multiplicand = static_cast<int32_t>(static_cast<uint32_t>(delta) << shiftAmount);
            else
                multiplicand = delta >> (-shiftAmount);

            return { value, static_cast<uint16_t>(multiplicand) };
        }

        inline constexpr auto atan2Table = MakeLookupTable<Atan2Entry, Atan2Spec::entryCount, BuildAtan2Entry>();
    }

    /**
     * @brief High-performance trigonometric library using 64-bit hardware multiply.
     *
     * @details Uses dmuls.l (signed 32x32->64) + Extract32<shift> for hardware-optimized
     * interpolation. Tables are generated at compile time via constexpr functions.
     *
     * Key features:
     * - 8.24 internal precision for sin/tan1-3/atan2 (16.16 for tan4-5)
     * - 64-bit product eliminates truncation error from 32-bit multiply
     * - Extract32<0|16> replaces arbitrary shift sequences (1 instruction vs 1-4)
     * - Compile-time table generation from mathematical formulas
     * - Template output type allows requesting different fixed-point precisions
     * - Constant-time execution for all operations
     * - Automatic angle wrapping and quadrant handling
     *
     * @see Angle For angle representation and conversion
     * @see Fxp For details on the fixed-point implementation
     */
    class Trigonometry final
    {
    private:
        static constexpr auto& sinTable = detail::sinTable;
        static constexpr auto& tanTable1 = detail::tanTable1;
        static constexpr auto& tanTable2 = detail::tanTable2;
        static constexpr auto& tanTable3 = detail::tanTable3;
        static constexpr auto& tanTable4 = detail::tanTable4;
        static constexpr auto& tanTable5 = detail::tanTable5;
        static constexpr auto& atan2Table = detail::atan2Table;

        template<typename Out, int InternalFractionalBits>
        [[gnu::always_inline]] static constexpr Out ConvertFromInternal(int32_t raw)
        {
            constexpr int shift = InternalFractionalBits - Out::FracBits;
            if constexpr (shift > 0)
            {
                if consteval
                {
                    return Out::BuildRaw(raw >> shift);
                }
                else
                {
                    Hardware::ArithmeticShiftRight<shift>(raw);
                    return Out::BuildRaw(raw);
                }
            }
            else if constexpr (shift < 0)
                return Out::BuildRaw(static_cast<int32_t>(static_cast<uint32_t>(raw) << (-shift)));
            else
                return Out::BuildRaw(raw);
        }

        template<typename Out, int InternalFractionalBits>
        [[gnu::always_inline]] static constexpr Out TanResult(int32_t ret, bool secondQuarter)
        {
            return ConvertFromInternal<Out, InternalFractionalBits>(secondQuarter ? -ret : ret);
        }

        template<typename Out, typename TableType, int InternalFractionalBits>
        [[gnu::always_inline]] static constexpr Out CalcTan(
            const TableType& table, uint16_t tempAngle, uint16_t baseRange, bool secondQuarter)
        {
            using CleanTableType = std::remove_cvref_t<TableType>;
            using EntryType = typename CleanTableType::value_type;
            constexpr uint32_t mask = EntryType::mask;

            // Count mask bits
            constexpr int multBits = [](uint32_t m) {
                int b = 0;
                while (m >> b) b++;
                return b;
            }(mask);

            size_t index = static_cast<uint32_t>(tempAngle - baseRange) >> multBits;
            if (index >= table.size()) index = table.size() - 1;

            int32_t ret = table[index].ExtractValue(tempAngle);
            return TanResult<Out, InternalFractionalBits>(ret, secondQuarter);
        }

    public:
        /**
         * @brief Calculates sine of an angle.
         * @tparam Out Output fixed-point type (default: Fxp 16.16)
         * @param angle Input angle in turns
         * @return Sine value in the specified fixed-point format [-1, 1]
         */
        template<typename Out = Fxp>
        [[gnu::always_inline]] static constexpr Out Sin(const Angle& angle)
        {
            constexpr int indexShift = detail::SinSpec::multiplierBits;
            size_t index = angle.RawValue() >> indexShift;
            int32_t raw = sinTable[index].ExtractValue(angle.RawValue());
            return ConvertFromInternal<Out, detail::SinSpec::internalFractionalBits>(raw);
        }

        /**
         * @brief Calculates cosine of an angle.
         * Implemented as sin(x + π/2) reusing the sine table.
         * @tparam Out Output fixed-point type (default: Fxp 16.16)
         * @param angle Input angle in turns
         * @return Cosine value in the specified fixed-point format [-1, 1]
         */
        template<typename Out = Fxp>
        [[gnu::always_inline]] static constexpr Out Cos(const Angle& angle)
        {
            Angle shifted = angle + Angle::HalfPi();
            constexpr int indexShift = detail::SinSpec::multiplierBits;
            size_t index = shifted.RawValue() >> indexShift;
            int32_t raw = sinTable[index].ExtractValue(shifted.RawValue());
            return ConvertFromInternal<Out, detail::SinSpec::internalFractionalBits>(raw);
        }

        /**
         * @brief Calculates tangent of an angle.
         * Uses dynamic table selection based on proximity to π/2.
         * @tparam Out Output fixed-point type (default: Fxp 16.16)
         * @param angle Input angle in turns
         * @return Tangent value in the specified fixed-point format
         */
        template<typename Out = Fxp>
        [[gnu::always_inline]] static constexpr Out Tan(const Angle& angle)
        {
            uint16_t tempAngle = angle.RawValue();

            if (tempAngle >= Angle::Pi().RawValue())
                tempAngle += Angle::Pi().RawValue();

            bool secondQuarter = tempAngle >= Angle::HalfPi().RawValue();

            if (secondQuarter)
                tempAngle = Angle::Pi().RawValue() - tempAngle;

            if (tempAngle >= detail::Tan5Spec::baseAngle) { return CalcTan<Out, decltype(tanTable5), detail::Tan5Spec::internalFractionalBits>(tanTable5, tempAngle, detail::Tan5Spec::baseAngle, secondQuarter); }
            if (tempAngle >= detail::Tan4Spec::baseAngle) { return CalcTan<Out, decltype(tanTable4), detail::Tan4Spec::internalFractionalBits>(tanTable4, tempAngle, detail::Tan4Spec::baseAngle, secondQuarter); }
            if (tempAngle >= detail::Tan3Spec::baseAngle) { return CalcTan<Out, decltype(tanTable3), detail::Tan3Spec::internalFractionalBits>(tanTable3, tempAngle, detail::Tan3Spec::baseAngle, secondQuarter); }
            if (tempAngle >= detail::Tan2Spec::baseAngle) { return CalcTan<Out, decltype(tanTable2), detail::Tan2Spec::internalFractionalBits>(tanTable2, tempAngle, detail::Tan2Spec::baseAngle, secondQuarter); }
            return CalcTan<Out, decltype(tanTable1), detail::Tan1Spec::internalFractionalBits>(tanTable1, tempAngle, detail::Tan1Spec::baseAngle, secondQuarter);
        }

        /**
         * @brief Calculates arctangent of y/x, handling all quadrants.
         * @tparam T FixedPoint type for both coordinates (default: Fxp 16.16)
         * @param y Y coordinate
         * @param x X coordinate
         * @return Angle in range [0, 1] turns
         */
        template<FixedPointType T = Fxp>
        [[gnu::always_inline]] static constexpr Angle Atan2(const T& y, const T& x)
        {
            if (y == 0) return x >= 0 ? Angle::Zero() : Angle::Pi();
            if (x == 0) return y >= 0 ? Angle::HalfPi() : Angle::ThreeQuarterPi();

            Angle result = x < 0 ? Angle::Pi() : Angle::Zero();

            // Use absolute values for division to avoid the SH-2 hardware DIVU
            // bug: unsigned 64-bit/32-bit division gives wrong results when the
            // numerator is negative (huge unsigned 64-bit value overflows quotient).
            // The sign is restored after the division.
            T divResult;

            if (x.Abs() < y.Abs())
            {
                divResult = x.Abs() / y.Abs();
                if ((x < 0) != (y < 0)) divResult = -divResult;
                result += (divResult < 0) ? Angle::ThreeQuarterPi() : Angle::HalfPi();
            }
            else
            {
                divResult = y.Abs() / x.Abs();
                if ((x < 0) == (y < 0)) divResult = -divResult;
            }

            // Convert ratio to Fxp for table lookup.
            // Ratio is always in [-1, 1], so conversion is safe regardless of source format.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            Fxp ratio = Fxp::Convert(divResult);
#pragma GCC diagnostic pop
            uint32_t absDivResult = ratio.Abs().RawValue();

            constexpr int indexShift = detail::Atan2Spec::multiplierBits;
            size_t index = absDivResult >> indexShift;

            uint16_t atan2Result = atan2Table[index].ExtractValue(absDivResult);

            return result + Angle::BuildRaw(
                (ratio < 0 ? atan2Result : (0x10000 - atan2Result)));
        }

        /**
         * @brief Calculates arcsine using asin(x) = atan2(x, sqrt(1 - x²)).
         * @param x Value in range [-1, 1]
         * @return Angle in range [-π/2, π/2]
         */
        [[gnu::always_inline]] static constexpr Angle Asin(const Fxp& x)
        {
            Fxp clampedX = x;
            if (clampedX < -Fxp(1)) clampedX = -Fxp(1);
            if (clampedX > Fxp(1)) clampedX = Fxp(1);

            Fxp oneMinusXSquared = Fxp(1) - (clampedX * clampedX);
            Fxp sqrtTerm = oneMinusXSquared.Sqrt();

            return Atan2(clampedX, sqrtTerm);
        }
    };
}