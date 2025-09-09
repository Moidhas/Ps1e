#include <sys/stat.h>

#include <array>
#include <bitset>
#include <cassert>
#include <climits>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <format>
#include <fstream>
#include <iostream>
#include <print>
#include <span>
#include <stdexcept>
#include <variant>

#include "BitUtils.hpp"
using namespace std;
using namespace BitUtils;

using s8 = int8_t;
using s16 = int16_t;
using s32 = int32_t;
using s64 = int64_t;

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

constexpr u32 KB = (1 << 10);
constexpr u32 MB = (1 << 20);
constexpr u32 GB = (1 << 30);

template <unsigned_integral T, unsigned_integral U>
constexpr T uintNoTruncCast(const U x) noexcept {
    assert(NumLimit<T>() >= x);
    return static_cast<T>(x);
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

// These contain the insturctions that use the most significant six bits of the
// opcode to decode. It contains the necesssary bits to decode.
enum class PrimaryOps : u8 {
    SPECIAL = 0x00,
    BCONDZ = 0x01,
    J = 0x02,
    JAL = 0x03,
    BEQ = 0x04,
    BNE = 0x05,
    BLEZ = 0x06,
    BGTZ = 0x07,
    ADDI = 0x08,
    ADDIU = 0x09,
    SLTI = 0x0a,
    SLTIU = 0x0b,
    ANDI = 0x0c,
    ORI = 0x0d,
    XORI = 0x0e,
    LUI = 0x0f,
    COP0 = 0x10,
    COP1 = 0x11,
    COP2 = 0x12,
    COP3 = 0x13,
    LB = 0X20,
    LH = 0x21,
    LWL = 0x22,
    LW = 0x23,
    LBU = 0x24,
    LHU = 0x25,
    LWR = 0x26,
    SB = 0X28,
    SH = 0x29,
    SWL = 0x2A,
    SW = 0x2B,
    SWR = 0X2E,
    LWC0 = 0X30,
    LWC1 = 0x31,
    LWC2 = 0x32,
    LWC3 = 0x33,
    SWC0 = 0X38,
    SWC1 = 0x39,
    SWC2 = 0x3A,
    SWC3 = 0x3B,
};

// These contain the insturctions that use the least significant six bits of the
// opcode to decode. It contains the necesssary bits to decode.
enum class SecondaryOps : u8 {
    SLL = 0x00,
    SRL = 0x02,
    SRA = 0x03,
    SLLV = 0x04,
    SRLV = 0x06,
    SRAV = 0x07,
    JR = 0x08,
    JALR = 0x09,
    SYSCALL = 0x0C,
    BREAK = 0x0D,
    MFHI = 0x10,
    MTHI = 0x11,
    MFLO = 0x12,
    MTLO = 0x13,
    MULT = 0x18,
    MULTU = 0x19,
    DIV = 0x1A,
    DIVU = 0x1B,
    ADD = 0x20,
    ADDU = 0x21,
    SUB = 0x22,
    SUBU = 0x23,
    AND = 0x24,
    OR = 0x25,
    XOR = 0x26,
    NOR = 0x27,
    SLT = 0x2A,
    SLTU = 0x2B,
};

enum class JumpOp : u8 {
    BLTZ = 0x00,
    BGEZ = 0x01,
    J = 0x02,
    JAL = 0x03,
    BEQ = 0x04,
    BNE = 0x05,
    BLEZ = 0x06,
    BGTZ = 0x07,
    JR = 0x08,
    JALR = 0x09,
    BLTZAL = 0x10,
    BGEZAL = 0x11,
    COUNT
};

struct Registers {
    array<u32, 32> gpr{0};
    u32 pc = 0;
    u32 hi = 0;
    u32 lo = 0;
};

// TODO: Combine the range with buffer.
struct Range {
    u32 start;
    u32 byteLength;
    const u32 mirrorMask;
    constexpr Range(const u32 start, const u32 byteLength,
                    const u32 mirrorMask = 0xFFFF'FFFF)
        : start{start}, byteLength{byteLength}, mirrorMask{mirrorMask} {}

    constexpr bool contains(const u32 addr) const {
        if (byteLength == 0) return false;
        return start <= addr && addr <= start + (byteLength - 1);
    }

    constexpr u32 getOffset(const u32 addr) const {
        assert(contains(addr));
        return (addr & mirrorMask) - start;
    }
};

struct MemCtrl1DelayReg {
    u8 writeDelay;
    u8 readDelay;
    u8 recoveryPeriod;
    u8 holdPeriod;
    u8 floatingPeriod;
    u8 PreStrobePeriod;
    u8 dataBusWidth;
    u8 autoIncrement;
    u8 numberOfAddressBits;
    u8 DMATimingOverride;
    u8 addressErrorFlag;
    u8 DMATimingSelect;
    u8 wideDma;
    u8 wait;
    u32 value;

    MemCtrl1DelayReg(u32 value)
        : writeDelay{static_cast<u8>(extractBits(value, 0, 4))},
          readDelay{static_cast<u8>(extractBits(value, 4, 4))},
          recoveryPeriod{static_cast<u8>(extractBits(value, 8, 1))},
          holdPeriod{static_cast<u8>(extractBits(value, 9, 1))},
          floatingPeriod{static_cast<u8>(extractBits(value, 10, 1))},
          PreStrobePeriod{static_cast<u8>(extractBits(value, 11, 1))},
          dataBusWidth{static_cast<u8>(extractBits(value, 12, 1))},
          autoIncrement{static_cast<u8>(extractBits(value, 13, 1))},
          numberOfAddressBits{static_cast<u8>(extractBits(value, 16, 5))},
          DMATimingOverride{static_cast<u8>(extractBits(value, 24, 4))},
          addressErrorFlag{static_cast<u8>(extractBits(value, 28, 1))},
          DMATimingSelect{static_cast<u8>(extractBits(value, 29, 1))},
          wideDma{static_cast<u8>(extractBits(value, 30, 1))},
          wait{static_cast<u8>(extractBits(value, 31, 1))},
          value{value} {}

    MemCtrl1DelayReg() {}
};

template <typename T>
struct AddressedValue {
    T value;
    Range range;

    AddressedValue(const T &value, const Range &range)
        : value{value}, range{range} {}
    AddressedValue(const Range &range) : range{range} {}
};

struct VAddress {
    u32 m_addr;
};
struct PAddress {
    u32 m_addr;
};

// TODO: Change the code so that the order of an enum never matters.
// TODO: Change the code so that if ranges get changed by bios, the addresses
// are some sort of not_initliazed value.
// TODO: Implement caching.
class MMap {
    // User Memory: KUSEG is intended to contain 2GB virtual memory (on extended
    // MIPS processors), the PSX doesn't support virtual memory, and KUSEG
    // simply contains a mirror of KSEG0/KSEG1 (in the first 512MB) (trying to
    // access memory in the remaining 1.5GB causes an exception).
    // source: https://psx-spx.consoledev.net/memorymap/#write-queue
    static constexpr Range KUSEG_RANGE{0x00000000, 512 * MB};
    static constexpr Range KSEG0_RANGE{0x80000000, 512 * MB};
    static constexpr Range KSEG1_RANGE{0xA0000000, 512 * MB};
    static constexpr Range KSEG2_RANGE{0xC0000000, GB};

    AddressedValue<array<u8, 2 * MB>> ramBuffer{
        Range{0x00000000, 2 * MB, 2 * MB - 1}};

    Range E1_RANGE{0x1F000000, 512 * KB};
    array<u8, 512 * KB> e1Buffer;

    static constexpr Range SCRATCHPAD_RANGE{0x1F800000, KB};
    array<u8, SCRATCHPAD_RANGE.byteLength> scratchpadBuffer;

    static constexpr Range IO_RANGE{0x1F801000, 4 * KB};
    static constexpr Range MEM_CTRL1_RANGE{0x1F801000, 36};
    Range SPU_RANGE{0x1F801D80, 0};
    Range CDROM_RANGE{0x1F801800, 0};

    AddressedValue<array<MemCtrl1DelayReg, 6>> memCtrl1DelayRegs{
        Range{0x1F801008, 6 * 4}};

    AddressedValue<u32> e1BaseAddr{0, Range{0x1F801000, 1}};
    AddressedValue<u32> e2BaseAddr{0, Range{0x1F801004, 1}};

    AddressedValue<u32> ramSize{0, Range{0x1F801060, 4}};

    // TODO: change since, not actually buffer.
    Range E2_RANGE{0x1F802000, 8 * KB};
    array<u8, 8 * KB> e2Buffer;

    // not used by psx.
    Range E3_RANGE{0x1FA00000, 2 * MB};
    // array<u8, 0> e3Buffer;

    AddressedValue<array<u8, 512 * KB>> biosBuffer{Range{0x1FC00000, 512 * KB}};
    AddressedValue<u32> cacheCtrlReg{0, Range{0xFFFE0130, 4}};
    AddressedValue<u32> comDelay{0, Range{0x1F801020, 4}};

    // TODO: make it so that this is somehow with resepct to the addresses.
    enum class DelayIdx {
        E1_DELAY,
        E3_DELAY,
        BIOS_DELAY,
        SPU_DELAY,
        CDROM_DELAY,
        E2_DELAY
    };

    u32 getMemCtrl1DelayRegIdx(const PAddress paddr) {
        assert(memCtrl1DelayRegs.range.contains(paddr.m_addr));
        const u32 idx = memCtrl1DelayRegs.range.getOffset(paddr.m_addr) / 4;
        assert(idx < memCtrl1DelayRegs.value.size());
        return idx;
    }

    u32 getMemCtrl1Reg(const PAddress paddr) {
        const u32 addr = paddr.m_addr;

        if (e1BaseAddr.range.contains(addr)) {
            return e1BaseAddr.value;
        } else if (e2BaseAddr.range.contains(addr)) {
            return e2BaseAddr.value;
        }

        return memCtrl1DelayRegs.value[getMemCtrl1DelayRegIdx(paddr)].value;
    }

    u32 getIoReg(const PAddress pAddr) {
        const u32 addr = pAddr.m_addr;
        if (MEM_CTRL1_RANGE.contains(addr)) {
            return getMemCtrl1Reg(pAddr);
        } else if (ramSize.range.contains(addr)) {
            return ramSize.value;
        } else {
            assert(false);
        }
    }

    void updateDelayRegState(const DelayIdx idx) {
        array<Range *, 6> dict = {&E1_RANGE,  &E3_RANGE,    &biosBuffer.range,
                                  &SPU_RANGE, &CDROM_RANGE, &E2_RANGE};

        assert(static_cast<u32>(idx) < dict.size());

        dict[static_cast<u32>(idx)]->byteLength =
            (1 << memCtrl1DelayRegs.value[static_cast<u32>(idx)]
                      .numberOfAddressBits);
        const u32 byteLength = dict[static_cast<u32>(idx)]->byteLength;

        using enum DelayIdx;
        switch (idx) {
            case E1_DELAY:
                println("E1_DELAY, Length:{}", byteLength);
                break;
            case E3_DELAY:
                println("E3_DELAY, Length:{}", byteLength);
                break;
            case BIOS_DELAY:
                println("BIOS_DELAY, Length:{}", byteLength);
                break;
            case SPU_DELAY:
                println("SPU_DELAY, Length:{}", byteLength);
                break;
            case CDROM_DELAY:
                println("CDROM_DELAY, Length:{}", byteLength);
                break;
            case E2_DELAY:
                println("E2_DELAY, Length:{}", byteLength);
                break;
            default:
                assert(false);
        }
    }

    void writeMemCtrl1Register(const PAddress paddr, const u32 value) {
        const u32 addr = paddr.m_addr;
        if (e1BaseAddr.range.contains(addr)) {
            assert(extractBits(value, 24, 8) == 0x1F);
            assert(value == 0x1F000000);
            println("e1BaseAddr");
            e1BaseAddr.range.start = value;
        } else if (e2BaseAddr.range.contains(addr)) {
            assert(extractBits(value, 24, 8) == 0x1F);
            assert(value == 0x1F802000);
            println("e2BaseAddr");
            e2BaseAddr.range.start = value;
        } else if (memCtrl1DelayRegs.range.contains(paddr.m_addr)) {
            const u32 idx = getMemCtrl1DelayRegIdx(paddr);
            memCtrl1DelayRegs.value[idx] = value;
            updateDelayRegState(static_cast<DelayIdx>(idx));
        } else if (comDelay.range.contains(addr)) {
            println("comDelay");
            comDelay.value = value;
        } else {
            assert(false);
        }
    }

    void writeIoReg(const PAddress pAddr, const u32 value) {
        const u32 addr = pAddr.m_addr;
        if (MEM_CTRL1_RANGE.contains(addr)) {
            writeMemCtrl1Register(pAddr, value);
        } else if (ramSize.range.contains(addr)) {
            println("ramSize");
            const bitset<32> bits = value;
            const u8 ramRangeChoice = (bits[11] << 1) | bits[9];
            const array<u8, 4> ramSizes{1, 4, 2, 8};
            ramBuffer.range.byteLength = ramSizes[ramRangeChoice] * MB;
            ramSize.value = value;
        } else {
            assert(false);
        }
    }

    PAddress getPAddr(const VAddress vAddr) {
        const u32 addr = vAddr.m_addr;
        const bool condition = KSEG1_RANGE.contains(addr) ||
                               KSEG0_RANGE.contains(addr) ||
                               KUSEG_RANGE.contains(addr);
        if (condition) {
            const u32 pAddr{~0xE0000000 & addr};
            assert(!(KSEG1_RANGE.contains(addr) &&
                     SCRATCHPAD_RANGE.contains(pAddr)));
            return PAddress{pAddr};
        } else if (KSEG2_RANGE.contains(addr))
            return PAddress{addr};
        else
            // BUS ERROR
            assert(false);
    }

    u32 handlBufferRead(AddressedValue<span<u8>> addressedBuffer,
                        const PAddress paddr, const u8 numOfBytes) {
        const u32 addr = paddr.m_addr;
        const u32 idx = biosBuffer.range.getOffset(addr);
        assert(addressedBuffer.value.size() >= idx + numOfBytes);
        u32 accBytes{0};
        for (int i = 0; i < numOfBytes; ++i) {
            accBytes |=
                static_cast<u32>(addressedBuffer.value[idx + i] << (8 * i));
        }
        return accBytes;
    }

    u32 handleRead(const PAddress paddr, const u8 numOfBytes) {
        assert(numOfBytes <= 4);
        const u32 addr = paddr.m_addr;

        if (ramBuffer.range.contains(addr)) {
            println("RAM");
            return handlBufferRead({span{ramBuffer.value}, ramBuffer.range},
                                   paddr, numOfBytes);
        } else if (E1_RANGE.contains(addr)) {
            println("E1");
            assert(false);
        } else if (SCRATCHPAD_RANGE.contains(addr)) {
            println("SCRATCHPAD_RANGE");
            assert(false);
        } else if (IO_RANGE.contains(addr)) {
            println("IO_RANGE");
            return getIoReg(PAddress{addr});
        } else if (E2_RANGE.contains(addr)) {
            println("E2_RANGE");
            assert(false);
        } else if (E3_RANGE.contains(addr)) {
            println("E3_RANGE");
            assert(false);
        } else if (biosBuffer.range.contains(addr)) {
            return handlBufferRead({span{biosBuffer.value}, biosBuffer.range},
                                   paddr, numOfBytes);
        } else if (cacheCtrlReg.range.contains(addr)) {
            println("CacheCtrl");
            return cacheCtrlReg.value;
        } else {
            println("{}", addr);
            assert(false);
        }
    }

    void handleWrite(const PAddress paddr, const u32 value,
                     const u8 numOfBytes) {
        assert(numOfBytes <= 4);
        const u32 addr = paddr.m_addr;

        if (ramBuffer.range.contains(addr)) {
            println("RAM");
            const u32 idx = ramBuffer.range.getOffset(addr);
            assert(ramBuffer.value.size() >= idx + numOfBytes);
            for (int i = 0; i < numOfBytes; ++i) {
                ramBuffer.value[idx + i] = value & (0xFF << (8 * i));
            }
        } else if (E1_RANGE.contains(addr)) {
            println("E1");
            assert(false);
        } else if (SCRATCHPAD_RANGE.contains(addr)) {
            println("SCRATCHPAD_RANGE");
            assert(false);
        } else if (IO_RANGE.contains(addr)) {
            println("IO_RANGE");
            writeIoReg(paddr, value);
        } else if (E2_RANGE.contains(addr)) {
            println("E2_RANGE");
            assert(false);
        } else if (E3_RANGE.contains(addr)) {
            println("E3_RANGE");
            assert(false);
        } else if (biosBuffer.range.contains(addr)) {
            assert(false);
        } else if (cacheCtrlReg.range.contains(addr)) {
            println("cacheCtrlReg");
            println("{:0b}", value);
            cacheCtrlReg.value = value;
        } else {
            println("{}", addr);
            assert(false);
        }
    }

   public:
    MMap() {
        struct stat info;
        stat("./BIOS/SCPH1001.BIN", &info);
        if (biosBuffer.value.size() != info.st_size)
            throw runtime_error{"size of BIOS is not 512KB"};

        ifstream bios{"./BIOS/SCPH1001.BIN", ios::binary | ios::in};
        bios.read((char *)biosBuffer.value.data(), biosBuffer.value.size());
    }

    // need to refactor the way shits read/stored;
    u32 load32(VAddress vAddr) {
        const PAddress paddr = getPAddr(vAddr);
        return handleRead(paddr, 4);
    }

    u16 load16(VAddress vAddr) {
        const PAddress paddr = getPAddr(vAddr);
        const u32 value = handleRead(paddr, 2);
        assert(value <= NumOfBits<u16>());
        return value;
    }

    u8 load8(VAddress vAddr) {
        const PAddress paddr = getPAddr(vAddr);
        const u32 value = handleRead(paddr, 1);
        assert(value <= NumOfBits<u8>());
        return value;
    }

    void store32(const VAddress vAddr, const u32 value) {
        const PAddress pAddr = getPAddr(vAddr);
        handleWrite(pAddr, value, 4);
    }

    void store16(const VAddress vAddr, const u32 value) {
        const PAddress pAddr = getPAddr(vAddr);
        handleWrite(pAddr, value, 2);
    }

    void store8(const VAddress vAddr, const u32 value) {
        const PAddress pAddr = getPAddr(vAddr);
        handleWrite(pAddr, value, 1);
    }
};

enum class State {
    NoLoadDelay,
    SpecialLoadDelay,
    LoadDelay,
    OverWritten,
    Linked
};

enum class Type { Load, Write, Branch, Store };

struct LoadDecodedOp {
    u32 dstRegId;
    u32 value;
    u32 addr;
};

struct WriteDecodedOp {
    u32 dstRegId;
    u32 value;
};

struct BranchDecodedOp {
    u32 pc;
};

struct StoreDecodedOp {
    u32 value;
    u32 dstAddr;
};

struct DecodedOp {
    State newState;
    u32 pc;
    variant<LoadDecodedOp, WriteDecodedOp, BranchDecodedOp, StoreDecodedOp>
        instr;
    variant<PrimaryOps, SecondaryOps, JumpOp> opcode;
};

string getPrimaryOpString(const PrimaryOps opcode) {
    using enum PrimaryOps;
    switch (opcode) {
        case SPECIAL:
            return "SPECIAL";
        case BCONDZ:
            return "BCONDZ";
        case J:
            return "J";
        case JAL:
            return "JAL";
        case BEQ:
            return "BEQ";
        case BNE:
            return "BNE";
        case BLEZ:
            return "BLEZ";
        case BGTZ:
            return "BGTZ";
        case ADDI:
            return "ADDI";
        case ADDIU:
            return "ADDIU";
        case SLTI:
            return "SLTI";
        case SLTIU:
            return "SLTIU";
        case ANDI:
            return "ANDI";
        case ORI:
            return "ORI";
        case XORI:
            return "XORI";
        case LUI:
            return "LUI";
        case COP0:
            return "COP0";
        case COP1:
            return "COP1";
        case COP2:
            return "COP2";
        case COP3:
            return "COP3";
        case LB:
            return "LB";
        case LH:
            return "LH";
        case LWL:
            return "LWL";
        case LW:
            return "LW";
        case LBU:
            return "LBU";
        case LHU:
            return "LHU";
        case LWR:
            return "LWR";
        case SB:
            return "SB";
        case SH:
            return "SH";
        case SWL:
            return "SWL";
        case SW:
            return "SW";
        case SWR:
            return "SWR";
        case LWC0:
            return "LWC0";
        case LWC1:
            return "LWC1";
        case LWC2:
            return "LWC2";
        case LWC3:
            return "LWC3";
        case SWC0:
            return "SWC0";
        case SWC1:
            return "SWC1";
        case SWC2:
            return "SWC2";
        case SWC3:
            return "SWC3";
    }
}

string getSecondaryOpString(SecondaryOps opcode) {
    using enum SecondaryOps;
    switch (opcode) {
        case SLL:
            return "SLL";
        case SRL:
            return "SRL";
        case SRA:
            return "SRA";
        case SLLV:
            return "SLLV";
        case SRLV:
            return "SRLV";
        case SRAV:
            return "SRAV";
        case JR:
            return "JR";
        case JALR:
            return "JALR";
        case SYSCALL:
            return "SYSCALL";
        case BREAK:
            return "BREAK";
        case MFHI:
            return "MFHI";
        case MTHI:
            return "MTHI";
        case MFLO:
            return "MFLO";
        case MTLO:
            return "MTLO";
        case MULT:
            return "MULT";
        case MULTU:
            return "MULTU";
        case DIV:
            return "DIV";
        case DIVU:
            return "DIVU";
        case ADD:
            return "ADD";
        case ADDU:
            return "ADDU";
        case SUB:
            return "SUB";
        case SUBU:
            return "SUBU";
        case AND:
            return "AND";
        case OR:
            return "OR";
        case XOR:
            return "XOR";
        case NOR:
            return "NOR";
        case SLT:
            return "SLT";
        case SLTU:
            return "SLTU";
    }
}

string getJumpOpString(JumpOp opcode) {
    using enum JumpOp;
    switch (opcode) {
        case BLTZ:
            return "BLTZ";
        case BGEZ:
            return "BGEZ";
        case J:
            return "J";
        case JAL:
            return "JAL";
        case BEQ:
            return "BEQ";
        case BNE:
            return "BNE";
        case BLEZ:
            return "BLEZ";
        case BGTZ:
            return "BGTZ";
        case JR:
            return "JR";
        case JALR:
            return "JALR";
        case BLTZAL:
            return "BLTZAL";
        case BGEZAL:
            return "BGEZAL";
        case COUNT:
            return "COUNT";
    }
}

string getOpcodeString(
    const variant<PrimaryOps, SecondaryOps, JumpOp> &opcode) {
    if (auto op = get_if<PrimaryOps>(&opcode))
        return getPrimaryOpString(*op);
    else if (auto op = get_if<SecondaryOps>(&opcode))
        return getSecondaryOpString(*op);
    else
        return getJumpOpString(get<JumpOp>(opcode));
}

template <>
struct formatter<DecodedOp> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    auto format(const DecodedOp &decodedOp, auto &ctx) const {
        const string opcodeString = getOpcodeString(decodedOp.opcode);
        if (auto instr = get_if<WriteDecodedOp>(&decodedOp.instr)) {
            return format_to(ctx.out(), "PC:0x{:X}  {}: ${}=0x{:X}",
                             decodedOp.pc, opcodeString, instr->dstRegId,
                             instr->value);
        } else if (auto instr = get_if<LoadDecodedOp>(&decodedOp.instr)) {
            return format_to(ctx.out(), "PC:0x{:X}  {}: ${}=[0x{:X}]=0x{:X}",
                             decodedOp.pc, opcodeString, instr->dstRegId,
                             instr->addr, instr->value);
        } else if (auto instr = get_if<StoreDecodedOp>(&decodedOp.instr)) {
            return format_to(ctx.out(), "PC:0x{:X}  {}: [0x{:X}]=0x{:X}",
                             decodedOp.pc, opcodeString, instr->dstAddr,
                             instr->value);
        } else if (auto instr = get_if<BranchDecodedOp>(&decodedOp.instr)) {
            return format_to(ctx.out(), "PC:0x{:X} {}: $PC=0x{:X}",
                             decodedOp.pc, opcodeString, instr->pc);
        } else {
            assert(false);
        }
    }
};

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
    BitField<0, 1, u32> IEc;
    // Current Kernel/User Mode
    BitField<1, 1, u32> KUc;
    // Previous Interrupt Enable
    BitField<2, 1, u32> IEp;
    // Previous Kernel/User Mode
    BitField<3, 1, u32> KUp;
    // Old Interrupt Enable
    BitField<4, 1, u32> IEo;
    // Old Kernel/User Mode
    BitField<5, 1, u32> KUo;
    // interrupt mask fields. When set the corresponding interrupts are allowed
    // to cause an exception.
    BitField<8, 8, u32> IM;
    // Isolate Cache (0=No, 1=Isolate) When isolated, all load and store
    // operations are targetted to the Data cache, and never the main memory.
    // (Used by PSX Kernel, in combination with Port FFFE0130h)
    BitField<16, 1, u32> IsC;
    // Swapped cache mode (0=Normal, 1=Swapped) Instruction cache will act as
    // Data cache and vice versa. Use only with Isc to access & invalidate
    // Instr. cache entries. (Not used by PSX Kernel)
    BitField<17, 1, u32> SwC;
    // When set cache parity bits are written as 0.
    BitField<18, 1, u32> PZ;
    // Shows the result of the last load operation with the D-cache isolated. It
    // gets set if the cache really contained data for the addressed memory
    // location.
    BitField<19, 1, u32> CM;
    // Cache parity error (Does not cause exception)
    BitField<20, 1, u32> PE;
    // TLB shutdown. Gets set if a programm address simultaneously matches 2 TLB
    // entries.
    BitField<21, 1, u32> TS;
    // BEV Boot exception vectors in RAM/ROM (0=RAM/KSEG0, 1=ROM/KSEG1)
    BitField<22, 1, u32> BEV;
    //  Reverse endianness   (0=Normal endianness, 1=Reverse endianness)
    //  Reverses the byte order in which data is stored in memory. (lo-hi ->
    //  hi-lo) (Affects only user mode, not kernel mode) (?) (The bit doesn't
    //  exist in PSX ?)
    BitField<25, 1, u32> RE;
    // COP0 Enable (0=Enable only in Kernel Mode, 1=Kernel and User Mode)
    BitField<28, 1, u32> CU0;
    // COP1 Enable (0=Disable, 1=Enable) (none in PSX)
    BitField<29, 1, u32> CU1;
    // COP2 Enable (0=Disable, 1=Enable) (GTE in PSX)
    BitField<30, 1, u32> CU2;
    // COP3 Enable (0=Disable, 1=Enable) (none in PSX)
    BitField<31, 1, u32> CU3;
};

union CAUSE {
    u32 data;
    // Describes what kind of exception occured
    BitField<2, 5, u32> ExcCode;
    // Software Interrupts. Write to these bits to manually cause an exception.
    // Clear them before returning from the exception handler.
    BitField<8, 2, u32> Sw;
    // Interrupt pending field. As long as any of the bits are set they will
    // cause an interrupt if the corresponding bit is set in IM.
    BitField<10, 6, u32> IP;
    // Contains the coprocessor number if the exception occurred because of a
    // coprocessor instuction for a coprocessor which wasn't enabled in SR.
    BitField<28, 2, u32> CE;
    // When BD is set, BT determines whether the branch is taken. The Target
    // Address Register holds the return address.
    BitField<30, 1, u32> BT;
    // Is set when EPC points to the branch instuction instead of the
    // instruction in the branch delay slot, where the exception occurred.
    BitField<31, 1, u32> BD;
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

    COP0() { sr.BEV = 1; }

    u32 &operator[](const u32 idx) {
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

    u32 srStackPush(bool isKernalMode, bool isInterruptEnabled) {
        u32 protectionBitsMask = (isKernalMode << 1) | isInterruptEnabled;

        u32 srStack = 0b1111 & sr.data;
        srStack = (srStack << 2) | protectionBitsMask;
        return (sr.data & 0b000000) | srStack;
    }

    u32 srStackPop() { return replaceBitRange(sr.data, 0, sr.data, 2, 4); }
};
}  // namespace COP0

struct Cpu {
    constexpr static u32 RESET_VECTOR = 0xBFC00000;
    const DecodedOp ZERO_INSTR;
    Registers reg;
    COP0::COP0 cop0;
    MMap *mmap;

    Cpu()
        : ZERO_INSTR{State::NoLoadDelay, RESET_VECTOR, WriteDecodedOp{0, 0},
                     SecondaryOps::ADD},
          mmap{new MMap{}} {
        reg.pc = RESET_VECTOR;
    }
    ~Cpu() { delete mmap; }

    // This is where all side effects should be.
    void runCpuLoop() {
        DecodedOp prevDecoded{ZERO_INSTR};

        while (true) {
            try {
                u32 opcode = mmap->load32(VAddress{reg.pc});
                DecodedOp decodedOp = decode(opcode, prevDecoded);
                println("{}", decodedOp);

                // TODO: unless an IRQ occurs between the load and
                // next opcode, in that case the load would complete during IRQ
                // handling, and so, the next opcode would receive the NEW value
                if (auto instr = get_if<LoadDecodedOp>(&decodedOp.instr)) {
                    assert(instr->dstRegId < reg.gpr.size());
                } else if (auto instr =
                               get_if<WriteDecodedOp>(&decodedOp.instr)) {
                    assert(instr->dstRegId < reg.gpr.size());
                    reg.gpr[instr->dstRegId] = instr->value;

                    if (auto prevInstr =
                            get_if<LoadDecodedOp>(&prevDecoded.instr);
                        prevInstr && prevInstr->dstRegId == instr->dstRegId) {
                        prevDecoded.newState = State::OverWritten;
                    }
                }

                reg.pc += 4;
                reg.gpr[31] = prevDecoded.newState == State::Linked
                                  ? reg.pc
                                  : reg.gpr[31];

                if (auto prevInstr =
                        get_if<LoadDecodedOp>(&prevDecoded.instr)) {
                    reg.gpr[prevInstr->dstRegId] = prevInstr->value;
                } else if (auto prevInstr =
                               get_if<BranchDecodedOp>(&prevDecoded.instr)) {
                    reg.pc = prevInstr->pc;
                }

                prevDecoded = decodedOp;
                reg.gpr[0] = 0;
            } catch (const COP0::MipsException &e) {
                cop0.epc = e.EPC;
                cop0.sr.data = cop0.srStackPush(true, false);
                cop0.cause.data = e.cause;
                reg.pc = cop0.sr.BEV ? 0xbfc00180 : 0x80000080;
                prevDecoded = ZERO_INSTR;
            }
        }
    }

    DecodedOp decode(const u32 opcode, const DecodedOp &prevInstr) {
        const u8 primaryOp = uintNoTruncCast<u8>(extractBits(opcode, 26, 6));

        return primaryOp != 0 ? decodePrimary(opcode, prevInstr)
                              : decodeSecondary(opcode);
    }

    // TODO: Standardize argument order.
    DecodedOp decodePrimary(const u32 opcode, const DecodedOp &prevInstr) {
        const u8 primaryOp = uintNoTruncCast<u8>(extractBits(opcode, 26, 6));
        const u8 rd = uintNoTruncCast<u8>(extractBits(opcode, 11, 5));
        const u8 rs = uintNoTruncCast<u8>(extractBits(opcode, 21, 5));
        const u8 rt = uintNoTruncCast<u8>(extractBits(opcode, 16, 5));
        const u8 imm5 = uintNoTruncCast<u8>(extractBits(opcode, 6, 5));
        const u16 imm16 = uintNoTruncCast<u16>(extractBits(opcode, 0, 16));
        const u32 imm26 = extractBits(opcode, 0, 26);

        using enum PrimaryOps;
        switch (static_cast<PrimaryOps>(primaryOp)) {
            case BCONDZ:
                return handleJumpOp(static_cast<JumpOp>(rt), opcode);
            case BEQ:
            case BNE:
            case BGTZ:
            case J:
            case JAL:
            case BLEZ:
                return handleJumpOp(static_cast<JumpOp>(primaryOp), opcode);
            case ADDIU:
                return {State::NoLoadDelay, reg.pc,
                        WriteDecodedOp{rt, addiu(reg.gpr[rs], imm16)}, ADDIU};
            case ADDI:
                return {State::NoLoadDelay, reg.pc,
                        WriteDecodedOp{
                            rt, static_cast<u32>(addi(reg.gpr[rs], imm16))},
                        ADDI};
            case LB: {
                auto [value, addr] = lb(reg.gpr[rs], imm16);
                return {State::LoadDelay, reg.pc,
                        LoadDecodedOp{rt, value, addr}, LB};
            }
            case LH: {
                auto [value, addr] = lh(reg.gpr[rs], imm16);
                return {State::LoadDelay, reg.pc,
                        LoadDecodedOp{rt, value, addr}, LH};
            }
            case LWL:
                if (auto instr = get_if<LoadDecodedOp>(&prevInstr.instr);
                    prevInstr.newState == State::SpecialLoadDelay &&
                    instr->dstRegId == rt) {
                    auto [value, addr] = lwl(instr->value, reg.gpr[rs], imm16);
                    return {State::SpecialLoadDelay, reg.pc,
                            LoadDecodedOp{rt, value, addr}, LWL};
                } else {
                    auto [value, addr] = lwl(reg.gpr[rt], reg.gpr[rs], imm16);
                    return {State::SpecialLoadDelay, reg.pc,
                            LoadDecodedOp{rt, value, addr}, LWL};
                }
            case LW: {
                auto [value, addr] = lw(reg.gpr[rs], imm16);
                return {State::LoadDelay, reg.pc,
                        LoadDecodedOp{rt, value, addr}, LW};
            }
            case LBU: {
                auto [value, addr] = lbu(reg.gpr[rs], imm16);
                return {State::LoadDelay, reg.pc,
                        LoadDecodedOp{rt, value, addr}, LBU};
            }
            case LHU: {
                auto [value, addr] = lhu(reg.gpr[rs], imm16);
                return {State::LoadDelay, reg.pc,
                        LoadDecodedOp{rt, value, addr}, LHU};
            }
            case LWR:
                if (auto instr = get_if<LoadDecodedOp>(&prevInstr.instr);
                    prevInstr.newState == State::SpecialLoadDelay &&
                    instr->dstRegId == rt) {
                    auto [value, addr] = lwr(instr->value, reg.gpr[rs], imm16);
                    return {State::SpecialLoadDelay, reg.pc,
                            LoadDecodedOp{rt, value, addr}, LWR};
                } else {
                    auto [value, addr] = lwr(reg.gpr[rt], reg.gpr[rs], imm16);
                    return {State::SpecialLoadDelay, reg.pc,
                            LoadDecodedOp{rt, value, addr}, LWR};
                }
            case SW:
                return {State::NoLoadDelay, reg.pc,
                        StoreDecodedOp{reg.gpr[rt],
                                       sw(reg.gpr[rt], reg.gpr[rs], imm16)},
                        SW};
            case SH:
                return {State::NoLoadDelay, reg.pc,
                        StoreDecodedOp{reg.gpr[rt],
                                       sh(reg.gpr[rt], reg.gpr[rs], imm16)},
                        SH};
            case SB:
                return {State::NoLoadDelay, reg.pc,
                        StoreDecodedOp{reg.gpr[rt],
                                       sb(reg.gpr[rt], reg.gpr[rs], imm16)},
                        SB};
            case LUI:
                return {State::NoLoadDelay, reg.pc,
                        WriteDecodedOp{rt, lui(imm16)}, LUI};
            case ORI:
                return {State::NoLoadDelay, reg.pc,
                        WriteDecodedOp{rt, ori(reg.gpr[rs], imm16)}, ORI};
            case COP0:
                return handleCOP0(rs, rt, rd, imm16);
            default:
                throw runtime_error{format(
                    "Invalid opcode/Not implemented Yet: {:032b}, "
                    "PrimaryOpcode: {:x}, "
                    "Instruction: {}",
                    opcode, primaryOp,
                    getPrimaryOpString(static_cast<PrimaryOps>(primaryOp)))};
        };
    }

    DecodedOp handleJumpOp(const JumpOp jumpOp, const u32 opcode) {
        const u8 rs = uintNoTruncCast<u8>(extractBits(opcode, 21, 5));
        const u8 rt = uintNoTruncCast<u8>(extractBits(opcode, 16, 5));
        const u16 imm16 = uintNoTruncCast<u16>(extractBits(opcode, 0, 16));
        const u32 imm26 = extractBits(opcode, 0, 26);
        const s32 s = reg.gpr[rs];
        const s32 t = reg.gpr[rt];

        using enum JumpOp;

        array<bool, static_cast<size_t>(COUNT)> cond;
        cond[static_cast<size_t>(JAL)] = true;
        cond[static_cast<size_t>(BEQ)] = (s == t);
        cond[static_cast<size_t>(BNE)] = (s != t);
        cond[static_cast<size_t>(BLTZ)] = (s < 0);
        cond[static_cast<size_t>(BGEZ)] = (s >= 0);
        cond[static_cast<size_t>(BGTZ)] = (s > 0);
        cond[static_cast<size_t>(BLEZ)] = (s <= 0);

        switch (jumpOp) {
            case J:
                return {.newState = State::NoLoadDelay,
                        .pc = reg.pc,
                        .instr = BranchDecodedOp{j(reg.pc, imm26)},
                        .opcode = jumpOp};
            case JAL:
                return {.newState = State::Linked,
                        .pc = reg.pc,
                        .instr = BranchDecodedOp{j(reg.pc, imm26)},
                        .opcode = jumpOp};
            case JR:
                return {.newState = State::NoLoadDelay,
                        .pc = reg.pc,
                        .instr = BranchDecodedOp{jr(reg.gpr[rs])},
                        .opcode = jumpOp};
            case BEQ:
            case BNE:
            case BLTZ:
            case BGEZ:
            case BGTZ:
            case BLEZ:
                return {.newState = State::NoLoadDelay,
                        .pc = reg.pc,
                        .instr = BranchDecodedOp{branchCMPHelper(
                            reg.pc, imm16, cond[static_cast<size_t>(jumpOp)])},
                        .opcode = jumpOp};
            default:
                assert(false);
        }
    }

    DecodedOp handleCOP0(const u32 rs, const u32 rt, const u32 rd,
                         const s16 imm16) {
        enum COP0Opcode { MFC0 = 0b00000, MTC0 = 0b00100, RFE = 0b10000 };

        switch (rs) {
            case MFC0:
                return {.newState = State::LoadDelay,
                        .pc = reg.pc,
                        .instr = LoadDecodedOp{rt, cop0[rd], rd},
                        .opcode = PrimaryOps::COP0};
            case MTC0: {
                cop0[rd] = reg.gpr[rt];
                return {.newState = State::NoLoadDelay,
                        .pc = reg.pc,
                        .instr = StoreDecodedOp{reg.gpr[rt], rd},
                        .opcode = PrimaryOps::COP0};
            }
            case RFE:
                cop0.sr.data = cop0.srStackPop();
                return {.newState = State::NoLoadDelay,
                        .pc = reg.pc,
                        .instr = WriteDecodedOp{0, 0},
                        .opcode = PrimaryOps::COP0};
            default:
                assert(false);
        }
    }

    DecodedOp decodeSecondary(const u32 opcode) {
        const u8 secondaryOp = uintNoTruncCast<u8>(extractBits(opcode, 0, 6));
        const u8 rd = uintNoTruncCast<u8>(extractBits(opcode, 11, 5));
        const u8 rs = uintNoTruncCast<u8>(extractBits(opcode, 21, 5));
        const u8 rt = uintNoTruncCast<u8>(extractBits(opcode, 16, 5));
        const u8 imm5 = uintNoTruncCast<u8>(extractBits(opcode, 6, 5));
        const u16 imm16 = uintNoTruncCast<u16>(extractBits(opcode, 0, 16));
        const u32 imm26 = extractBits(opcode, 0, 26);

        using enum SecondaryOps;
        switch (static_cast<SecondaryOps>(secondaryOp)) {
            case JR:
            case JALR:
                handleJumpOp(static_cast<JumpOp>(secondaryOp), opcode);
            case ADD:
                return {State::NoLoadDelay, reg.pc,
                        WriteDecodedOp{rd, static_cast<u32>(
                                               add(reg.gpr[rs], reg.gpr[rt]))},
                        ADD};
            case ADDU:
                return {State::NoLoadDelay, reg.pc,
                        WriteDecodedOp{rd, addu(reg.gpr[rs], reg.gpr[rt])},
                        ADDU};
            case SUB:
                return {State::NoLoadDelay, reg.pc,
                        WriteDecodedOp{rd, static_cast<u32>(
                                               sub(reg.gpr[rs], reg.gpr[rt]))},
                        SUB};
            case SUBU:
                return {State::NoLoadDelay, reg.pc,
                        WriteDecodedOp{rd, subu(reg.gpr[rs], reg.gpr[rt])},
                        SUBU};
            case SLL:
                return {State::NoLoadDelay, reg.pc,
                        WriteDecodedOp{rd, sll(reg.gpr[rt], imm5)}, SLL};
            case OR:
                return {State::NoLoadDelay, reg.pc,
                        WriteDecodedOp{rd, orInstr(reg.gpr[rs], reg.gpr[rt])},
                        OR};
            default:
                throw runtime_error{
                    format("Invalid opcode/Not implemented Yet: {:032b}, "
                           "SecondaryOpcode: {:x}, "
                           "Instruction: {}",
                           opcode, secondaryOp,
                           getSecondaryOpString(
                               static_cast<SecondaryOps>(secondaryOp)))};
        };
    }

    u32 orInstr(const u32 s, const u32 t) { return s | t; }

    u32 branchCMPHelper(const u32 currPC, const s16 offset,
                        bool isBranchTaken) {
        const u32 delaySlotAddress = (currPC + 4);
        if (isBranchTaken) {
            return delaySlotAddress + (offset << 2);
        } else {
            return delaySlotAddress;
        }
    }

    u32 jr(const u32 s) {
        // if (s % 4 != 0) throw COP0::MipsException{COP0::ExcCode::AdEL,
        // reg.pc};
        return s;
    }

    u32 j(const u32 currPC, const u32 imm26) {
        assert(imm26 <= (1 << 26) - 1);
        const u32 delaySlotAddress = currPC + 4;
        return (delaySlotAddress & 0xF0000000) + (imm26 << 2);
    }

    u32 sw(const u32 value, const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        // if (addr % 4 != 0)
        //     throw COP0::MipsException{COP0::ExcCode::AdEL, reg.pc};
        mmap->store32(VAddress{addr}, value);
        return addr;
    }

    u32 sh(const u32 value, const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        // if (addr % 2 != 0)
        //     throw COP0::MipsException{COP0::ExcCode::AdEL, reg.pc};
        mmap->store16(VAddress{addr}, value);
        return addr;
    }

    u32 sb(const u32 value, const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        mmap->store8(VAddress{addr}, value);
        return addr;
    }

    u32 sll(const u32 t, const u8 imm5) { return t << imm5; }

    u32 ori(const u32 s, const u16 imm16) noexcept {
        return s | static_cast<u32>(imm16);
    }

    u32 lui(const u16 imm16) noexcept { return static_cast<u32>(imm16) << 16; }

    // Reads the most significant bytes of the source into the least
    // significant bytes of the destination.
    tuple<u32, u32> lwr(const u32 dest, const u32 base, const s16 offset) {
        const u32 addr = base + offset;
        const u32 wordAddr = (addr / 4) * 4;
        const u32 src = mmap->load32(VAddress{wordAddr});
        const u32 bitLength = (addr - wordAddr + 1) * 8;
        return {replaceBitRange(dest, 0, src,
                                static_cast<u32>(NumOfBits<u32>() - bitLength),
                                bitLength),
                addr};
    }

    // Reads the least significant bytes of the source into the most significant
    // bytes of the destination.
    tuple<u32, u32> lwl(const u32 dest, const u32 base, const s16 offset) {
        const u32 addr = base + offset;
        const u32 wordAddr = (addr / 4) * 4;
        const u32 src = mmap->load32(VAddress{wordAddr});
        const u32 bitLength = (addr - wordAddr + 1) * 8;
        return {replaceBitRange(dest,
                                static_cast<u32>(NumOfBits<u32>() - bitLength),
                                src, 0, bitLength),
                addr};
    }

    tuple<u32, u32> lb(const u32 s, const s16 imm16) {
        // implicit sign-extension, this does not work in 1's complement.
        return {static_cast<s8>(mmap->load8(VAddress{s + imm16})), s + imm16};
    }

    tuple<u32, u32> lbu(const u32 s, const s16 imm16) {
        return {mmap->load8(VAddress{s + imm16}), s + imm16};
    }

    // implicit sign-extension, this does not work in 1's complement.
    tuple<u32, u32> lh(const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        // if (addr % 2 != 0)
        //     throw COP0::MipsException{COP0::ExcCode::AdEL, reg.pc};
        return {static_cast<s16>(mmap->load16(VAddress{s + imm16})), addr};
    }

    tuple<u32, u32> lhu(const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        // if (addr % 2 != 0)
        //     throw COP0::MipsException{COP0::ExcCode::AdEL, reg.pc};
        return {mmap->load16(VAddress{addr}), addr};
    }

    tuple<u32, u32> lw(const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        // if (addr % 4 != 0)
        //     throw COP0::MipsException{COP0::ExcCode::AdEL, reg.pc};
        return {mmap->load32(VAddress{s + imm16}), addr};
    }

    s32 addi(const s32 s, const s16 imm16) {
        if ((s > 0 && imm16 > INT_MAX - s) || (s < 0 && imm16 < INT_MIN - s)) {
            COP0::CAUSE cause{.data = 0};
            cause.ExcCode = static_cast<u32>(COP0::ExcCode::Ov);
            throw COP0::MipsException{cause.data, reg.pc};
        }

        return s + imm16;
    }

    u32 addiu(const u32 s, const s16 imm16) { return s + imm16; }

    u32 addu(const u32 s, const u32 t) { return s + t; }

    s32 add(const s32 s, const s32 t) {
        if ((s > 0 && t > INT_MAX - s) || (s < 0 && t < INT_MIN - s)) {
            COP0::CAUSE cause{.data = 0};
            cause.ExcCode = static_cast<u32>(COP0::ExcCode::Ov);
            throw COP0::MipsException{cause.data, reg.pc};
        }

        return s + t;
    }

    u32 subu(const u32 s, const u32 t) { return s - t; }

    s32 sub(const s32 s, const s32 t) {
        if ((t > 0 && s < INT_MIN + t) || (t < 0 && s > INT_MAX + t)) {
            COP0::CAUSE cause{.data = 0};
            cause.ExcCode = static_cast<u32>(COP0::ExcCode::Ov);
            throw COP0::MipsException{cause.data, reg.pc};
        }

        return s - t;
    }
};

int main() {
    try {
        Cpu cpu;
        cpu.runCpuLoop();
    } catch (const exception &ex) {
        println("{}", ex.what());
    } catch (...) {
        println("uncaught");
    }
}
