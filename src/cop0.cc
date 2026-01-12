#include "cop0.hpp"

namespace COP0 {
u32 &COP0::operator[](const u32 idx) {
    switch (idx) {
        case BPC_IDX:
            return bpc;
        case BDA_IDX:
            return bda;
        case TAR_IDX:
            return tar;
        case DCIC_IDX:
            return dcic;
        case BadA_IDX:
            return bada;
        case BDAM_IDX:
            return bdam;
        case BPCM_IDX:
            return bpcm;
        case SR_IDX:
            return sr.data;
        case CAUSE_IDX:
            return cause.data;
        case EPC_IDX:
            return epc;
        case PRID_IDX:
            return prid;
        default:
            assert(false);
    }
}

u32 COP0::srStackPop() {
    return BitUtils::replaceBitRange(sr.data, 0, sr.data, 2, 4);
}
}  // namespace COP0
