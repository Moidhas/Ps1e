#ifndef COP0_H
#define COP0_H

#include "BitUtils.hpp"
#include "types.hpp"

namespace COP0 {

enum class ExcCode : u32 {
    Int,      // Interrupt
    Mod,      // TLB modification, the psx doesn't actually have a TLB.
    TLBL,     // TLB Load
    TLBS,     // TLB Store
    AdEL,     // Address Error on Load or Instruction fetch.
    AdES,     // Address Error on Store.
    IBE,      // Instruction Fetch Bus Error
    DBE,      // Data Load Bus Error
    Syscall,  // Generated unconditionally by a syscall instruction
    Bp,       // Breakpoint
    RI,       // Reserved Instruction
    CpU,      // Co-Processor unusable
    Ov        // Arithmetic Overflow
};

union SR {
    u32 data;
    // Current Interrupt Enable
    BitUtils::BitField<0, 1, u32> IEc;
    // Current Kernel/User Mode
    BitUtils::BitField<1, 1, u32> KUc;
    // Previous Interrupt Enable
    BitUtils::BitField<2, 1, u32> IEp;
    // Previous Kernel/User Mode
    BitUtils::BitField<3, 1, u32> KUp;
    // Old Interrupt Enable
    BitUtils::BitField<4, 1, u32> IEo;
    // Old Kernel/User Mode
    BitUtils::BitField<5, 1, u32> KUo;
    // interrupt mask fields. When set the corresponding interrupts are allowed
    // to cause an exception.
    BitUtils::BitField<8, 8, u32> IM;
    // Isolate Cache (0=No, 1=Isolate) When isolated, all load and store
    // operations are targetted to the Data cache, and never the main memory.
    // (Used by PSX Kernel, in combination with Port FFFE0130h)
    BitUtils::BitField<16, 1, u32> IsC;
    // Swapped cache mode (0=Normal, 1=Swapped) Instruction cache will act as
    // Data cache and vice versa. Use only with Isc to access & invalidate
    // Instr. cache entries. (Not used by PSX Kernel)
    BitUtils::BitField<17, 1, u32> SwC;
    // When set cache parity bits are written as 0.
    BitUtils::BitField<18, 1, u32> PZ;
    // Shows the result of the last load operation with the D-cache isolated. It
    // gets set if the cache really contained data for the addressed memory
    // location.
    BitUtils::BitField<19, 1, u32> CM;
    // Cache parity error (Does not cause exception)
    BitUtils::BitField<20, 1, u32> PE;
    // TLB shutdown. Gets set if a programm address simultaneously matches 2 TLB
    // entries.
    BitUtils::BitField<21, 1, u32> TS;
    // BEV Boot exception vectors in RAM/ROM (0=RAM/KSEG0, 1=ROM/KSEG1)
    BitUtils::BitField<22, 1, u32> BEV;
    //  Reverse endianness   (0=Normal endianness, 1=Reverse endianness)
    //  Reverses the byte order in which data is stored in memory. (lo-hi ->
    //  hi-lo) (Affects only user mode, not kernel mode) (?) (The bit doesn't
    //  exist in PSX ?)
    BitUtils::BitField<25, 1, u32> RE;
    // COP0 Enable (0=Enable only in Kernel Mode, 1=Kernel and User Mode)
    BitUtils::BitField<28, 1, u32> CU0;
    // COP1 Enable (0=Disable, 1=Enable) (none in PSX)
    BitUtils::BitField<29, 1, u32> CU1;
    // COP2 Enable (0=Disable, 1=Enable) (GTE in PSX)
    BitUtils::BitField<30, 1, u32> CU2;
    // COP3 Enable (0=Disable, 1=Enable) (none in PSX)
    BitUtils::BitField<31, 1, u32> CU3;
};

union CAUSE {
    u32 data;
    // Describes what kind of exception occured
    BitUtils::BitField<2, 5, u32> ExcCode;
    // Software Interrupts. Write to these bits to manually cause an exception.
    // Clear them before returning from the exception handler.
    BitUtils::BitField<8, 2, u32> Sw;
    // Interrupt pending field. As long as any of the bits are set they will
    // cause an interrupt if the corresponding bit is set in IM.
    BitUtils::BitField<10, 6, u32> IP;
    // Contains the coprocessor number if the exception occurred because of a
    // coprocessor instuction for a coprocessor which wasn't enabled in SR.
    BitUtils::BitField<28, 2, u32> CE;
    // When BD is set, BT determines whether the branch is taken. The Target
    // Address Register holds the return address.
    BitUtils::BitField<30, 1, u32> BT;
    // Is set when EPC points to the branch instuction instead of the
    // instruction in the branch delay slot, where the exception occurred.
    BitUtils::BitField<31, 1, u32> BD;
};

struct MipsException {
    const u32 cause;
    const u32 EPC;
};

struct COP0 {
    constexpr static u32 BEV = 22;

    u32 bpc;      // Breakpoint Program Counter
    u32 bda;      // Breakpoint Data Address
    u32 tar;      // Target Address
    u32 dcic;     // Debug and Cache Invalidate Control
    u32 bada;     // Bad Address
    u32 bdam;     // Breakpoint Data Address Mask
    u32 bpcm;     // Breakpoint Program Counter Mask
    SR sr;        // Status Register
    CAUSE cause;  // Cause of the last exception
    u32 epc;      // Exception Program Counter
    u32 prid;     // Processor Revision Identifier

    enum IDX {
        BPC = 3,
        BDA = 5,
        TAR = 6,
        DCIC = 7,
        BadA = 8,
        BDAM = 9,
        BPCM = 11,
        SR = 12,
        CAUSE = 13,
        EPC = 14,
        PRID = 15,
    };

    u32 &operator[](const u32 idx);

    u32 srStackPush(bool isKernalMode, bool isInterruptEnabled);

    u32 srStackPop();
};
}  // namespace COP0
#endif
