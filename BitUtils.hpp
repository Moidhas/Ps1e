#ifndef BIT_UTILS_H
#define BIT_UTILS_H

#include <cassert>
#include <compare>
#include <concepts>
#include <cstdint>

namespace BitUtils {
using s8 = int8_t;
using s16 = int16_t;
using s32 = int32_t;
using s64 = int64_t;
using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

template <std::integral T>
constexpr size_t NumOfBits() noexcept {
    return sizeof(T) * 8;
}

template <std::integral T>
constexpr T NumLimit() noexcept {
    return static_cast<T>(~0);
}

// shifts the extractedBits to least significant end.
u32 extractBits(u32 bits, u32 start, u32 length) {
    constexpr u32 end = sizeof(bits) * 8 - 1;
    assert(end >= start && start + length - 1 <= end);

    u32 shifted = bits >> start;
    u32 mask = static_cast<u64>(1 << length) - 1;

    assert((shifted & mask) <= static_cast<u64>(1 << length) - 1);
    return shifted & mask;
}

// replaces the bits in dst with the bits in src acorrding to the bitRange.
u32 replaceBitRange(const u32 dst, const u32 dstStart, const u32 src,
                    const u32 srcStart, const u32 length) {
    static_assert(sizeof(dst) == sizeof(src));
    assert(NumOfBits<u32>() >= length + dstStart);
    assert(NumOfBits<u32>() >= length + srcStart);
    const u32 lengthOfOnes = (1 << length) - 1;
    const u32 dstMask = ~(lengthOfOnes << dstStart);
    const u32 srcMask = lengthOfOnes << srcStart;
    const s32 distance = dstStart - srcStart;

    return distance <= 0 ? (dst & dstMask) | ((src & srcMask) >> -distance)
                         : (dst & dstMask) | ((src & srcMask) << distance);
}

template <unsigned Index, unsigned Count, std::unsigned_integral Type>
struct BitField {
    Type data;

    static_assert(Count > 0);
    static_assert(Index + Count - 1 < NumOfBits<Type>());

    constexpr Type GetMask() const {
        return (NumLimit<Type>() >> (NumOfBits<Type>() - Count)) << Index;
    }

    constexpr Type GetValue() const { return (GetMask() & data) >> Index; }

    constexpr operator Type() const { return GetValue(); }

    std::strong_ordering operator<=>(const Type rhs) {
        return GetValue() <=> rhs;
    }

    BitField &operator|=(const Type rhs) {
        setValue(GetValue() | rhs);
        return *this;
    }

    BitField &operator&=(const Type rhs) {
        setValue(GetValue() & rhs);
        return *this;
    }

    BitField &operator<<=(const Type rhs) {
        setValue(GetValue() << rhs);
        return *this;
    }

    BitField &operator>>=(const Type rhs) {
        setValue(GetValue() >> rhs);
        return *this;
    }

    BitField &operator=(const Type value) {
        setValue(value);
        return *this;
    }

    void setValue(const Type value) {
        assert(value <= (1 << Count) - 1);
        data = (data & ~GetMask()) | ((value << Index) & GetMask());
    }
};
}  // namespace BitUtils
#endif
