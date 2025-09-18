#include "cop0.hpp"

namespace COP0 {
u32 &COP0::operator[](const u32 idx) {
    switch (idx) {
        case BPC:
            return bpc;
        case BDA:
            return bda;
        case TAR:
            return tar;
        case DCIC:
            return dcic;
        case BadA:
            return bada;
        case BDAM:
            return bdam;
        case BPCM:
            return bpcm;
        case SR:
            return sr.data;
        case CAUSE:
            return cause.data;
        case EPC:
            return epc;
        case PRID:
            return prid;
        default:
            assert(false);
    }
}

u32 COP0::srStackPush(bool isKernalMode, bool isInterruptEnabled) {
    u32 protectionBitsMask = (isKernalMode << 1) | isInterruptEnabled;

    u32 srStack = 0b1111 & sr.data;
    srStack = (srStack << 2) | protectionBitsMask;
    return (sr.data & 0b000000) | srStack;
}

u32 COP0::srStackPop() {
    return BitUtils::replaceBitRange(sr.data, 0, sr.data, 2, 4);
}
}  // namespace COP0
