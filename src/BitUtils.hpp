#ifndef BIT_UTILS_H
#define BIT_UTILS_H
#include <cassert>
#include <compare>
#include <concepts>

#include "types.hpp"

namespace BitUtils {
template <std::integral T>
constexpr size_t NumOfBits() noexcept {
    return sizeof(T) * 8;
}

template <std::integral T>
constexpr T NumLimit() noexcept {
    return static_cast<T>(~0);
}

u32 extractBits(u32 bits, u32 start, u32 length);
u32 replaceBitRange(const u32 dst, const u32 dstStart, const u32 src,
                    const u32 srcStart, const u32 length); 

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
