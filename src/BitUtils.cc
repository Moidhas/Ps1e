#include "BitUtils.hpp"

namespace BitUtils {
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
};  // namespace BitUtils
