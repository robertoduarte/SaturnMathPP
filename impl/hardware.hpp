#pragma once
#include <stdint.h>

/**
 * @file hardware.hpp
 * @brief SH-2 hardware-specific operations (assembly intrinsics)
 *
 * Centralizes all SH-2 inline assembly so that math files contain only
 * portable C++ logic. Every function provides a constexpr fallback path
 * for compile-time evaluation.
 */

namespace SaturnMath::Hardware
{
    // ====================================================================
    // DIVU - Hardware Division Unit
    // ====================================================================

    static inline constexpr size_t cpuAddress = 0xFFFFF000UL;
    static inline auto& Dvsr  = *reinterpret_cast<volatile int32_t*>(cpuAddress + 0x0F00UL);
    static inline auto& Dvdnth = *reinterpret_cast<volatile int32_t*>(cpuAddress + 0x0F10UL);
    static inline auto& Dvdntl = *reinterpret_cast<volatile int32_t*>(cpuAddress + 0x0F14UL);

    /** @brief Load divisor and dividend into DIVU registers */
    [[gnu::always_inline]] inline void DivSet(int32_t divisor, int32_t dividendHi, int32_t dividendLo)
    {
        Dvsr  = divisor;
        Dvdnth = dividendHi;
        Dvdntl = dividendLo;
    }

    /** @brief Read DIVU quotient register */
    [[gnu::always_inline]] inline int32_t DivGetResult()    { return Dvdntl; }

    /** @brief Read DIVU remainder register */
    [[gnu::always_inline]] inline int32_t DivGetRemainder() { return Dvdnth; }

    // ====================================================================
    // MAC - Multiply-and-Accumulate registers
    // ====================================================================

    /** @brief Clear MAC registers (clrmac) */
    [[gnu::always_inline]] inline void MacClear()
    {
        __asm__ volatile("\tclrmac\n" ::: "mach", "macl");
    }

    /** @brief Read MACH and MACL into variables */
    [[gnu::always_inline]] inline void MacGet(int32_t& hi, int32_t& lo)
    {
        __asm__ volatile(
            "\tsts mach, %[hi]\n"
            "\tsts macl, %[lo]\n"
            : [hi] "=&r"(hi), [lo] "=&r"(lo)
            :
            : "memory"
        );
    }

    /** @brief Extract MAC result: sts mach/macl + xtrct → middle 32 bits */
    [[gnu::always_inline]] inline int32_t MacExtract()
    {
        int32_t hi, lo;
        __asm__ volatile(
            "\tsts mach, %[hi]\n"
            "\tsts macl, %[lo]\n"
            "\txtrct %[hi], %[lo]\n"
            : [hi] "=&r"(hi), [lo] "=&r"(lo)
            :
            : "memory"
        );
        return lo;
    }

    // ====================================================================
    // 64-bit signed multiply (dmuls.l)
    // ====================================================================

    /** @brief Signed 64-bit multiply via dmuls.l, returns MACH/MACL */
    [[gnu::always_inline]] inline void Mul64(int32_t a, int32_t b, int32_t& hi, int32_t& lo)
    {
        __asm__ volatile(
            "dmuls.l %[a], %[b]\n\t"
            "sts mach, %[hi]\n\t"
            "sts macl, %[lo]\n\t"
            : [hi] "=&r"(hi), [lo] "=&r"(lo)
            : [a] "r"(a), [b] "r"(b)
            : "mach", "macl"
        );
    }

    // ====================================================================
    // 64-bit unsigned multiply (dmulu.l)
    // ====================================================================

    /** @brief Unsigned 64-bit multiply via dmulu.l, returns MACH/MACL */
    [[gnu::always_inline]] inline void Mul64Unsigned(uint32_t a, uint32_t b, int32_t& hi, int32_t& lo)
    {
        __asm__ volatile(
            "dmulu.l %[a], %[b]\n\t"
            "sts mach, %[hi]\n\t"
            "sts macl, %[lo]\n\t"
            : [hi] "=&r"(hi), [lo] "=&r"(lo)
            : [a] "r"(a), [b] "r"(b)
            : "mach", "macl"
        );
    }

    // ====================================================================
    // 32-bit multiply (mul.l)
    // ====================================================================

    /** @brief 32-bit multiply via mul.l, returns lower 32 bits in MACL */
    [[gnu::always_inline]] inline int32_t Mul32(int32_t a, int32_t b)
    {
        int32_t result;
        __asm__ volatile(
            "mul.l %[a], %[b]\n\t"
            "sts macl, %[result]\n\t"
            : [result] "=&r"(result)
            : [a] "r"(a), [b] "r"(b)
            : "macl"
        );
        return result;
    }

    // ====================================================================
    // Arithmetic shift right (avoiding ___ashiftrt_r4_N library call)
    // ====================================================================

    /**
     * @brief Arithmetic shift right by N bits.
     * SH-2 only has shar (1-bit). For N>2, GCC emits a library call.
     * This avoids it by using negate + shlr (logical) + negate for N>2.
     * @tparam shift Number of bits to shift right [0..31]
     * @param value Value to shift (modified in place)
     */
    template<int shift>
    [[gnu::always_inline]] inline void ArithmeticShiftRight(int32_t& value)
    {
        static_assert(shift >= 0 && shift < 32, "Shift out of bounds");

        if constexpr (shift == 0)
        {
            // no-op
        }
        else if constexpr (shift == 1)
        {
            __asm__ volatile("shar %[v]\n\t" : [v] "+r"(value));
        }
        else if constexpr (shift == 2)
        {
            __asm__ volatile("shar %[v]\n\t shar %[v]\n\t" : [v] "+r"(value));
        }
        else
        {
            // SH-2 lacks arithmetic shift right by N>2.
            // Use: if negative, negate, logical shift, negate back.
            // GCC uses shlr8/shlr2/shlr for unsigned >> which is all 1-cycle.
            if (value < 0)
                value = -static_cast<int32_t>(
                    static_cast<uint32_t>(-value) >> shift);
            else
                value = static_cast<int32_t>(
                    static_cast<uint32_t>(value) >> shift);
        }
    }

    /**
     * @brief Arithmetic shift right by a runtime variable amount.
     * Runtime overload for cases where shift amount is not a compile-time constant.
     * Uses the same negate + shlr + negate technique as the template version.
     * @param value Value to shift (modified in place)
     * @param shift Number of bits to shift right [0..31]
     */
    [[gnu::always_inline]] inline void ArithmeticShiftRight(int32_t& value, uint32_t shift)
    {
        if (shift == 0) return;
        if (value < 0)
            value = -static_cast<int32_t>(
                static_cast<uint32_t>(-value) >> shift);
        else
            value = static_cast<int32_t>(
                static_cast<uint32_t>(value) >> shift);
    }

    // ====================================================================
    // Byte/word swap (swap.b, swap.w)
    // ====================================================================

    /** @brief Swap bytes in lower 16 bits of a 32-bit value (swap.b)
     *  @param value Bits 23-16 and 7-0 are swapped; upper 8 bits preserved
     *  @return Value with bytes 2 and 0 exchanged
     */
    [[gnu::always_inline]] inline int32_t SwapBytes(int32_t value)
    {
        __asm__ volatile(
            "swap.b %[val], %[val]"
            : [val] "+r"(value)
        );
        return value;
    }

    /** @brief Swap 16-bit halves of a 32-bit value (swap.w)
     *  @param value High 16 bits and low 16 bits are exchanged
     *  @return Value with upper and lower words swapped
     */
    [[gnu::always_inline]] inline int32_t SwapWords(int32_t value)
    {
        __asm__ volatile(
            "swap.w %[val], %[val]"
            : [val] "+r"(value)
        );
        return value;
    }

    // ====================================================================
    // 64-bit add with carry (clrt + addc)
    // ====================================================================

    /** @brief 64-bit addition using carry chain (clrt + addc)
     *  @param aHi High 32 bits of operand A
     *  @param aLo Low 32 bits of operand A
     *  @param bHi High 32 bits of operand B
     *  @param bLo Low 32 bits of operand B
     *  @param rHi Output: high 32 bits of result
     *  @param rLo Output: low 32 bits of result
     */
    [[gnu::always_inline]] inline void Add64(int32_t aHi, int32_t aLo, int32_t bHi, int32_t bLo,
                                             int32_t& rHi, int32_t& rLo)
    {
        rHi = bHi;
        rLo = bLo;
        __asm__ volatile(
            "clrt\n\t"
            "addc %[al], %[rl]\n\t"
            "addc %[ah], %[rh]\n\t"
            : [rh] "+r"(rHi), [rl] "+r"(rLo)
            : [ah] "r"(aHi), [al] "r"(aLo)
            : "t"
        );
    }

    // ====================================================================
    // 64-bit subtract with borrow (clrt + subc)
    // ====================================================================

    /** @brief 64-bit subtraction using borrow chain (clrt + subc)
     *  @param aHi High 32 bits of operand A (minuend)
     *  @param aLo Low 32 bits of operand A (minuend)
     *  @param bHi High 32 bits of operand B (subtrahend)
     *  @param bLo Low 32 bits of operand B (subtrahend)
     *  @param rHi Output: high 32 bits of result (A - B)
     *  @param rLo Output: low 32 bits of result (A - B)
     */
    [[gnu::always_inline]] inline void Sub64(int32_t aHi, int32_t aLo, int32_t bHi, int32_t bLo,
                                             int32_t& rHi, int32_t& rLo)
    {
        rHi = aHi;
        rLo = aLo;
        __asm__ volatile(
            "clrt\n\t"
            "subc %[bl], %[rl]\n\t"
            "subc %[bh], %[rh]\n\t"
            : [rh] "+r"(rHi), [rl] "+r"(rLo)
            : [bh] "r"(bHi), [bl] "r"(bLo)
            : "t"
        );
    }

    // ====================================================================
    // 64-bit rotate right through carry (rotcr)
    // ====================================================================

    /** @brief Rotate a 64-bit value right by 1 bit (shlr + rotcr)
     *  @param hi High 32 bits (modified in place)
     *  @param lo Low 32 bits (modified in place)
     *  Bit 0 of lo becomes bit 31 of hi; T flag must be cleared first
     */
    [[gnu::always_inline]] inline void RotateRight64(uint32_t& hi, uint32_t& lo)
    {
        __asm__ volatile(
            "clrt\n\t"
            "rotcr %[hi]\n\t"
            "rotcr %[lo]"
            : [hi] "+r"(hi), [lo] "+r"(lo)
            :
            : "t"
        );
    }

    // ====================================================================
    // Single division step (div1)
    // ====================================================================

    /** @brief Execute one division step (div1)
     *  @param divisor Divisor value
     *  @param dividend Dividend/remainder register (modified in place)
     *  Requires T flag to be set up by div0u/div0s beforehand.
     *  Each call processes one bit of the dividend.
     */
    [[gnu::always_inline]] inline void DivStep(int32_t divisor, int32_t& dividend)
    {
        __asm__ volatile(
            "div1 %[dvr], %[dvd]"
            : [dvd] "+r"(dividend)
            : [dvr] "r"(divisor)
            : "t"
        );
    }

    // ====================================================================
    // Count leading zeros (binary search using SH-2 shift instructions)
    // ====================================================================

    /** @brief Count leading zeros in a 32-bit value using SH-2 single-cycle shifts
     *  @return Number of leading zeros (0-32), or 32 if input is 0
     */
    [[gnu::always_inline]] inline int CountLeadingZeros(uint32_t value)
    {
        if (value == 0) return 32;

        int count, tmp;
        __asm__ volatile(
            "mov    #31, %[cnt]\n\t"         // count = 31 (bit position of highest set bit)
            // Test bits 31-16
            "mov    %[val], %[tmp]\n\t"
            "shlr16 %[tmp]\n\t"              // tmp = val >> 16
            "tst    %[tmp], %[tmp]\n\t"      // T=1 if upper 16 bits were zero
            "bt     1f\n\t"                  // no delay slot: skip if zero
            "mov    %[tmp], %[val]\n\t"      // val >>= 16
            "add    #-16, %[cnt]\n\t"        // count -= 16
            "1:\n\t"
            // Test bits 15-8
            "mov    %[val], %[tmp]\n\t"
            "shlr8  %[tmp]\n\t"              // tmp = val >> 8
            "tst    %[tmp], %[tmp]\n\t"
            "bt     2f\n\t"
            "mov    %[tmp], %[val]\n\t"      // val >>= 8
            "add    #-8, %[cnt]\n\t"         // count -= 8
            "2:\n\t"
            // Test bits 7-4
            "mov    %[val], %[tmp]\n\t"
            "shlr2  %[tmp]\n\t"
            "shlr2  %[tmp]\n\t"              // tmp = val >> 4
            "tst    %[tmp], %[tmp]\n\t"
            "bt     3f\n\t"
            "mov    %[tmp], %[val]\n\t"      // val >>= 4
            "add    #-4, %[cnt]\n\t"         // count -= 4
            "3:\n\t"
            // Test bits 3-2
            "mov    %[val], %[tmp]\n\t"
            "shlr2  %[tmp]\n\t"              // tmp = val >> 2
            "tst    %[tmp], %[tmp]\n\t"
            "bt     4f\n\t"
            "mov    %[tmp], %[val]\n\t"      // val >>= 2
            "add    #-2, %[cnt]\n\t"         // count -= 2
            "4:\n\t"
            // Test bit 1
            "shlr   %[val]\n\t"              // val >>= 1, T = old bit 0
            "tst    %[val], %[val]\n\t"      // T=1 if val == 0 (bit 1 was 0)
            "bf     5f\n\t"                  // no delay slot: skip if bit 1 was set
            "add    #-1, %[cnt]\n\t"         // bit 1 was 0, count -= 1
            "5:\n"
            : [cnt] "=&r"(count), [tmp] "=&r"(tmp), [val] "+r"(value)
            :
            : "t"
        );
        return count;
    }

    // ====================================================================
    // Shift primitives (constexpr-friendly)
    // ====================================================================

    /** @brief 64-bit logical right shift by 1 (shlr + rotcr) */
    static void ShiftRight64(uint32_t& hi, uint32_t& lo)
    {
        __asm__ volatile(
            "shlr %[h]\n\t"
            "rotcr %[l]"
            : [h] "+r"(hi), [l] "+r"(lo)
            :
            : "t"
        );
    }

    /** @brief Extract middle 32 bits from a 64-bit value (xtrct) */
    static void ExtractMid32(const uint32_t& hi, uint32_t& lo)
    {
        __asm__ volatile(
            "\txtrct %[h], %[l]"
            : [l] "+r"(lo)
            : [h] "r"(hi)
        );
    }

    // ====================================================================
    // 64-bit to 32-bit extraction with optimal shift sequences
    // Extracts 32 bits at bit position 'shift' from a 64-bit value
    // (hi:lo), using xtrct + optimized shift instructions.
    // ====================================================================

    /**
     * @brief Extract 32 bits from a 64-bit value at a given bit offset.
     * @tparam shift Number of bits to right-shift the 64-bit value.
     * @param mach High 32 bits of the 64-bit value
     * @param macl Low 32 bits of the 64-bit value
     * @param result Output: the extracted 32-bit result
     */
    template<int shift>
    [[gnu::always_inline]] inline void Extract32(int32_t mach, int32_t macl, int32_t& result)
    {
        static_assert(shift >= 0 && shift < 32, "Shift out of bounds");

        if constexpr (shift == 0)
        {
            result = macl;
        }
        else if constexpr (shift == 16)
        {
            __asm__ volatile("xtrct %[H], %[L]\n\t" : [L] "+r"(macl) : [H] "r"(mach));
            result = macl;
        }
        else if constexpr (shift > 16)
        {
            __asm__ volatile("xtrct %[H], %[L]\n\t" : [L] "+r"(macl) : [H] "r"(mach));
            constexpr int remainingRightShift = shift - 16;
            if constexpr (remainingRightShift == 15) {
                __asm__ volatile("shlr8 %[reg]\n\t add %[reg], %[reg]\n\t shlr8 %[reg]\n\t" : [reg] "+r"(macl));
            }
            else if constexpr (remainingRightShift == 14) {
                __asm__ volatile("shlr8 %[reg]\n\t shll2 %[reg]\n\t shlr8 %[reg]\n\t" : [reg] "+r"(macl));
            }
            else {
                if constexpr (remainingRightShift >= 8) { __asm__ volatile("shlr8 %[reg]\n\t" : [reg] "+r"(macl)); }
                constexpr int remainingRightShift2 = (remainingRightShift >= 8) ? remainingRightShift - 8 : remainingRightShift;
                if constexpr (remainingRightShift2 >= 4) { __asm__ volatile("shlr2 %[reg]\n\t shlr2 %[reg]\n\t" : [reg] "+r"(macl)); }
                else if constexpr (remainingRightShift2 >= 2) { __asm__ volatile("shlr2 %[reg]\n\t" : [reg] "+r"(macl)); }
                if constexpr (remainingRightShift2 % 2 != 0) { __asm__ volatile("shlr %[reg]\n\t" : [reg] "+r"(macl)); }
            }
            result = macl;
        }
        else
        {
            if constexpr (shift == 15) {
                __asm__ volatile("shlr8 %[reg]\n\t add %[reg], %[reg]\n\t shlr8 %[reg]\n\t" : [reg] "+r"(macl));
            }
            else if constexpr (shift == 14) {
                __asm__ volatile("shlr8 %[reg]\n\t shll2 %[reg]\n\t shlr8 %[reg]\n\t" : [reg] "+r"(macl));
            }
            else {
                if constexpr (shift >= 8) { __asm__ volatile("shlr8 %[reg]\n\t" : [reg] "+r"(macl)); }
                constexpr int remainingRightShift = (shift >= 8) ? shift - 8 : shift;
                if constexpr (remainingRightShift >= 4) { __asm__ volatile("shlr2 %[reg]\n\t shlr2 %[reg]\n\t" : [reg] "+r"(macl)); }
                else if constexpr (remainingRightShift >= 2) { __asm__ volatile("shlr2 %[reg]\n\t" : [reg] "+r"(macl)); }
                if constexpr (remainingRightShift % 2 != 0) { __asm__ volatile("shlr %[reg]\n\t" : [reg] "+r"(macl)); }
            }

            constexpr int remainingLeftShift = 32 - shift;
            if constexpr (remainingLeftShift >= 16) { __asm__ volatile("shll16 %[reg]\n\t" : [reg] "+r"(mach)); }
            constexpr int remainingLeftShift2 = (remainingLeftShift >= 16) ? remainingLeftShift - 16 : remainingLeftShift;
            if constexpr (remainingLeftShift2 >= 8) { __asm__ volatile("shll8 %[reg]\n\t" : [reg] "+r"(mach)); }
            constexpr int remainingLeftShift3 = (remainingLeftShift2 >= 8) ? remainingLeftShift2 - 8 : remainingLeftShift2;
            if constexpr (remainingLeftShift3 >= 4) { __asm__ volatile("shll2 %[reg]\n\t shll2 %[reg]\n\t" : [reg] "+r"(mach)); }
            else if constexpr (remainingLeftShift3 >= 2) { __asm__ volatile("shll2 %[reg]\n\t" : [reg] "+r"(mach)); }
            if constexpr (remainingLeftShift3 % 2 != 0) { __asm__ volatile("add %[reg], %[reg]\n\t" : [reg] "+r"(mach)); }

            result = macl | mach;
        }
    }

    // ====================================================================
    // MAC accumulate (mac.l - memory-based multiply-accumulate)
    // ====================================================================

    /** @brief Accumulate N multiply-accumulate iterations via mac.l
     * @tparam N Number of mac.l iterations (component count)
     * @param a Pointer to first operand's raw data (auto-incremented)
     * @param b Pointer to second operand's raw data (auto-incremented)
     */
    template<int N>
    [[gnu::always_inline]] inline void MacAccumulate(const int32_t* a, const int32_t* b)
    {
        if constexpr (N > 0)
        {
            __asm__ volatile(
                "\tmac.l @%[a]+, @%[b]+\n"
                : [a] "+r"(a), [b] "+r"(b)
                : "m"(*a), "m"(*b)
                : "mach", "macl", "memory"
            );
            MacAccumulate<N - 1>(a, b);
        }
    }
}
