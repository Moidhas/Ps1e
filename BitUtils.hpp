#include <cassert>
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
