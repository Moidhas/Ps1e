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
#include <span>
#include <stdexcept>
#include <utility>
#include <variant>
using namespace std;

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

template <typename T>
constexpr size_t bitSize() noexcept {
    return sizeof(T) * CHAR_BIT;
}

template <integral T>
constexpr size_t numLimit() noexcept {
    return (1 << bitSize<T>()) - 1;
}

template <unsigned_integral T, unsigned_integral U>
constexpr T uintNoTruncCast(const U x) noexcept {
    assert(numLimit<T>() >= x);
    return static_cast<T>(x);
}

// checkout table 3.2:
// https://student.cs.uwaterloo.ca/~cs350/common/r3000-manual.pdf
enum class ExcCode : u32 {
    Int,
    Mod,
    TLBL,
    TLBS,
    AdEL,
    AdES,
    IBE,
    DBE,
    Syscall,
    Bp,
    RI,
    CpU,
    Ov
};

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

struct MipsException {
    const ExcCode CAUSE_CODE;
    const u32 EPC;

    MipsException(ExcCode CAUSE_CODE, u32 EPC)
        : CAUSE_CODE{CAUSE_CODE}, EPC{EPC} {}
};

struct Registers {
    array<u32, 32> gpr{};
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

// Source: https://psx-spx.consoledev.net/iomap/
struct MemCtrl1 {
    array<u32, 9> reg;

    MemCtrl1() : reg{0} {}

    enum class Register {
        E1_BASE_ADDR,
        E2_BASE_ADDR,
        E1_DELAY,
        E3_DELAY,
        BIOS_DELAY,
        SPU_DELAY,
        CDROM_DELAY,
        E2_DELAY,
        COM_DELAY,
    };

    u32 getRegister(const u32 offset) {
        const u32 idx = offset % 4;
        assert(idx < reg.size());
        return reg[idx];
    }

    void writeRegister(const u32 offset, const u32 value) {
        const u32 idx = offset / 4;
        assert(idx < reg.size());
        using enum Register;
        switch (static_cast<Register>(idx)) {
            case E1_BASE_ADDR:
                writeE1BaseAddr(value);
                break;
            case E2_BASE_ADDR:
                writeE2BaseAddr(value);
            case E1_DELAY:
                writeE1Delay(value);
                break;
            case E3_DELAY:
                writeE3Delay(value);
                break;
            case BIOS_DELAY:
                writeBIOSDelay(value);
                break;
            case SPU_DELAY:
                writeSPUDelay(value);
                break;
            case CDROM_DELAY:
                writeCDROMDelay(value);
                break;
            case E2_DELAY:
                writeE2Delay(value);
                break;
            case COM_DELAY:
                writeCOMDelay(value);
                break;
        }
    }

    void writeE1BaseAddr(const u32 value) {
        // TODO: No support for expanding expansion, and changing the base
        // address.
        assert(false);
    }

    void writeE2BaseAddr(const u32 value) { assert(false); }

    void writeE1Delay(const u32 value) { assert(false); }

    void writeE3Delay(const u32 value) { assert(false); }

    void writeBIOSDelay(const u32 value) {
        const u8 idx{static_cast<u8>(Register::BIOS_DELAY)};
        reg[idx] = value;
    }

    void writeSPUDelay(const u32 value) { assert(false); }

    void writeCDROMDelay(const u32 value) { assert(false); }

    void writeE2Delay(const u32 value) { assert(false); }

    void writeCOMDelay(const u32 value) { assert(false); }
};

struct MemCtrl2 {
    u32 ramSize;
    MemCtrl2() : ramSize{0} {}
};

// TODO: Add operator overloads for V/PAddress to make everything use
// V/PAddress.
// TODO: Find a better way to address stuff, so that you can get an address
// modified specifc to any Range. Need to do this for ram mirroring.
class MMap {
    // User Memory: KUSEG is intended to contain 2GB virtual memory (on extended
    // MIPS processors), the PSX doesn't support virtual memory, and KUSEG
    // simply contains a mirror of KSEG0/KSEG1 (in the first 512MB) (trying to
    // access memory in the remaining 1.5GB causes an exception).
    // source: https://psx-spx.consoledev.net/memorymap/#write-queue
    //
    //
    static constexpr Range KUSEG_RANGE{0x00000000, 512 * MB};
    static constexpr Range KSEG0_RANGE{0x80000000, 512 * MB};
    static constexpr Range KSEG1_RANGE{0xA0000000, 512 * MB};
    static constexpr Range KSEG2_RANGE{0xC0000000, GB};

    // TODO: No support for 8 MB ram, mirrored to 2 MB.
    Range RAM_RANGE{0x00000000, 2 * MB, 2 * MB - 1};
    array<u8, 2 * MB> ramBuffer;

    // TODO: No support for expanding expansion, and changing the base address.
    static constexpr Range E1_RANGE{0x1F000000, 512 * KB};
    array<u8, 512 * KB> e1Buffer;

    static constexpr Range SCRATCHPAD_RANGE{0x1F800000, KB};
    array<u8, SCRATCHPAD_RANGE.byteLength> scratchpadBuffer;

    static constexpr Range IO_RANGE{0x1F801000, 4 * KB};
    static constexpr Range MEM_CTRL1_RANGE{0x1F801000, 36};
    MemCtrl1 memCtrl1;
    static constexpr Range MEM_CTRL2_RANGE{0x1F801060, 4};
    MemCtrl2 memCtrl2;

    // TODO: change since, not actually buffer.
    Range E2_RANGE{0x1F802000, 8 * KB};
    array<u8, 8 * KB> e2Buffer;

    static constexpr Range E3_RANGE{0x1FA00000, 2 * MB};
    array<u8, 2 * MB> e3Buffer;

    Range BIOS_RANGE{0x1FC00000, 512 * KB};
    array<u8, 512 * KB> biosBuffer;

    struct VAddress {
        u32 m_addr;
    };
    struct PAddress {
        u32 m_addr;
    };

    enum class BufferType {
        Ram,
        E1,
        Scratchpad,
        Io,
        E2,
        E3,
        Bios,
    };

    span<const u8> getReadBuffer(BufferType type) {
        using enum BufferType;
        switch (type) {
            case Ram:
                return ramBuffer;
            case E1:
                return e1Buffer;
            case Scratchpad:
                return scratchpadBuffer;
            case E2:
                return e2Buffer;
            case E3:
                return e3Buffer;
            case Bios:
                return biosBuffer;
            case Io:
                assert(false);
        }
    }

    u32 getIoReg(const PAddress pAddr) {
        const u32 addr = pAddr.m_addr;
        if (MEM_CTRL1_RANGE.contains(addr)) {
            return memCtrl1.getRegister(MEM_CTRL1_RANGE.getOffset(addr));
        } else if (MEM_CTRL2_RANGE.contains(addr)) {
            return memCtrl2.ramSize;
        } else {
            assert(false);
        }
    }

    void writeIoReg(const PAddress pAddr, const u32 value) {
        const u32 addr = pAddr.m_addr;
        if (MEM_CTRL1_RANGE.contains(addr)) {
            memCtrl1.writeRegister(MEM_CTRL1_RANGE.getOffset(addr), value);
        } else if (MEM_CTRL2_RANGE.contains(addr)) {
            const bitset<32> bits = value;
            const u8 ramRangeChoice = (bits[11] << 1) | bits[9];
            const array<u8, 4> ramSizes{1, 4, 2, 8};
            RAM_RANGE.byteLength = ramSizes[ramRangeChoice];
            memCtrl2.ramSize = value;
        } else {
            assert(false);
        }
    }

    span<u8> getWriteBuffer(BufferType type) {
        using enum BufferType;
        switch (type) {
            case Ram:
                return ramBuffer;
            case E1:
                return e1Buffer;
            case Scratchpad:
                return scratchpadBuffer;
            case E2:
                return e2Buffer;
            case E3:
                return e3Buffer;
            case Io:
                assert(false);
            case Bios:
                assert(false);
        }
    }

    BufferType getType(const PAddress pAddr) {
        const u32 addr = pAddr.m_addr;
        using enum BufferType;
        if (RAM_RANGE.contains(addr)) {
            println("RAM");
            assert(false);
            return Ram;
        } else if (E1_RANGE.contains(addr)) {
            println("E1");
            assert(false);
            return E1;
        } else if (SCRATCHPAD_RANGE.contains(addr)) {
            println("SCRATCHPAD_RANGE");
            assert(false);
            return Scratchpad;
        } else if (IO_RANGE.contains(addr)) {
            println("IO_RANGE");
            return Io;
        } else if (E2_RANGE.contains(addr)) {
            println("E2_RANGE");
            assert(false);
            return E2;
        } else if (E3_RANGE.contains(addr)) {
            println("E3_RANGE");
            assert(false);
            return E3;
        } else if (BIOS_RANGE.contains(addr)) {
            return Bios;
        } else {
            println("{:X}", addr);
            assert(false);
        }
    }

    u32 getOffset(const BufferType type, const PAddress pAddr) {
        const u32 addr = pAddr.m_addr;
        using enum BufferType;
        switch (type) {
            case Ram:
                return RAM_RANGE.getOffset(addr);
            case E1:
                return E1_RANGE.getOffset(addr);
            case Scratchpad:
                return SCRATCHPAD_RANGE.getOffset(addr);
            case Io:
                return IO_RANGE.getOffset(addr);
            case E2:
                return E2_RANGE.getOffset(addr);
            case E3:
                return E3_RANGE.getOffset(addr);
            case Bios:
                return BIOS_RANGE.getOffset(addr);
        }
    }

    // TODO: need to implement this behaviour, KSEG1 addresses can't access
    // scratchpad.
    PAddress paddr(const VAddress vAddr) {
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

   public:
    MMap() {
        struct stat info;
        stat("SCPH1001.BIN", &info);
        if (biosBuffer.size() != info.st_size)
            throw runtime_error{"size of BIOS is not 512KB"};

        ifstream bios{"SCPH1001.BIN", ios::binary | ios::in};
        bios.read((char *)biosBuffer.data(), biosBuffer.size());
    }

    u32 load32(u32 addr) {
        VAddress vAddr{addr};
        const PAddress pAddr = paddr(vAddr);
        BufferType type = getType(pAddr);

        span<const u8> buffer = getReadBuffer(type);
        const u32 offset = getOffset(type, pAddr);
        assert(buffer.size() > offset + 3);
        u32 byte0 = (u32)buffer[offset + 0];
        u32 byte1 = (u32)buffer[offset + 1] << 8;
        u32 byte2 = (u32)buffer[offset + 2] << 16;
        u32 byte3 = (u32)buffer[offset + 3] << 24;

        return byte0 | byte1 | byte2 | byte3;
    }

    u16 load16(u32 addr) {
        VAddress vAddr{addr};
        const PAddress pAddr = paddr(vAddr);
        BufferType type = getType(pAddr);
        span<const u8> buffer = getReadBuffer(type);
        const u32 offset = getOffset(type, pAddr);

        assert(buffer.size() > offset + 1);
        u32 byte0 = (u32)buffer[offset];
        u32 byte1 = (u32)buffer[offset + 1] << 8;
        return byte0 | byte1;
    }

    u8 load8(u32 addr) {
        VAddress vAddr{addr};
        const PAddress pAddr = paddr(vAddr);
        BufferType type = getType(pAddr);
        span<const u8> buffer = getReadBuffer(type);
        const u32 offset = getOffset(type, pAddr);

        assert(buffer.size() > offset);
        u32 byte0 = (u32)buffer[offset];
        return byte0;
    }

    // TODO: During an 8-bit or 16-bit store, all 32 bits of the GPR are placed
    // on the bus. As such, when writing to certain 32-bit IO registers with an
    // 8 or 16-bit store, it will behave like a 32-bit store, using the
    // register's full value. The soundscope on some shells is known to rely on
    // this, as it uses sh to write to certain DMA registers. If this is not
    // properly emulated, the soundscope will hang, waiting for an interrupt
    // that will never be fired.
    void store32(const u32 addr, const u32 value) {
        const VAddress vAddr{addr};
        const PAddress pAddr = paddr(vAddr);
        const BufferType type = getType(pAddr);

        if (type == BufferType::Io) {
            writeIoReg(pAddr, value);
            return;
        }

        span<u8> buffer = getWriteBuffer(type);
        const u32 offset = getOffset(type, pAddr);

        assert(buffer.size() > offset + 3);
        buffer[0] = value & 0x00'00'00'FF;
        buffer[1] = value & 0x00'00'FF'00;
        buffer[2] = value & 0x00'FF'00'00;
        buffer[3] = value & 0xFF'00'00'00;
    }

    void store16(const u32 addr, const u32 value) {
        const VAddress vAddr{addr};
        const PAddress pAddr = paddr(vAddr);
        const BufferType type = getType(pAddr);

        if (type == BufferType::Io) {
            writeIoReg(pAddr, value);
            return;
        }

        span<u8> buffer = getWriteBuffer(type);
        const u32 offset = getOffset(type, pAddr);

        assert(buffer.size() > offset + 1);

        buffer[0] = value & 0x00'00'00'FF;
        buffer[1] = value & 0x00'00'FF'00;
    }

    void store8(const u32 addr, const u32 value) {
        const VAddress vAddr{addr};
        const PAddress pAddr = paddr(vAddr);
        const BufferType type = getType(pAddr);

        if (type == BufferType::Io) {
            writeIoReg(pAddr, value);
            return;
        }

        span<u8> buffer = getWriteBuffer(type);
        const u32 offset = getOffset(type, pAddr);

        assert(buffer.size() > offset);
        buffer[0] = value & 0x00'00'00'FF;
    }
};

struct COP0 {
    array<u32, 64> gpr{};
    u32 &cause;
    u32 &epc;
    u32 &sr;
    const u32 BEV = 22;
    COP0() : cause{gpr[13]}, epc{gpr[14]}, sr{gpr[12]} { sr |= (1 << BEV); }

    u32 srStackPush(bool isKernalMode, bool isInterruptEnabled) {
        u32 protectionBitsMask = (isKernalMode << 1) | isInterruptEnabled;

        u32 srStack = 0b1111 & sr;
        srStack = (srStack << 2) | protectionBitsMask;
        return (sr & 0b000000) | srStack;
    }
};

enum class State { NoLoadDelay, SpecialLoadDelay, LoadDelay, OverWritten };

enum class Type { Load, Write, Branch, Store };

struct LoadDecodedOp {
    u32 dstRegId;
    u32 value;
    u32 addr;
    LoadDecodedOp(u32 dstRegId, u32 value, u32 addr)
        : dstRegId{dstRegId}, value{value}, addr{addr} {}
};

struct WriteDecodedOp {
    u32 dstRegId;
    u32 value;
    WriteDecodedOp(u32 dstRegId, u32 value)
        : dstRegId{dstRegId}, value{value} {}
};

struct BranchDecodedOp {
    u32 pc;
    BranchDecodedOp(const u32 pc) : pc{pc} {}
};

struct StoreDecodedOp {
    u32 value;
    u32 dstAddr;
    StoreDecodedOp(u32 value, u32 dstAddr) : value{value}, dstAddr{dstAddr} {}
};

struct DecodedOp {
    State newState;
    u32 pc;
    variant<LoadDecodedOp, WriteDecodedOp, BranchDecodedOp, StoreDecodedOp>
        instr;
    variant<PrimaryOps, SecondaryOps> opcode;

    DecodedOp(
        State newState, u32 pc,
        variant<LoadDecodedOp, WriteDecodedOp, BranchDecodedOp, StoreDecodedOp>
            instr,
        variant<PrimaryOps, SecondaryOps> opcode)
        : newState{newState}, pc{pc}, instr{instr}, opcode{opcode} {}
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

string getSecondaryOps(SecondaryOps opcode) {
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

string getOpcodeString(const variant<PrimaryOps, SecondaryOps> &opcode) {
    if (auto op = get_if<PrimaryOps>(&opcode))
        return getPrimaryOpString(*op);
    else
        return getSecondaryOps(get<SecondaryOps>(opcode));
}

template <>
struct formatter<DecodedOp> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    auto format(const DecodedOp &decodedOp, auto &ctx) const {
        const string opcodeString = getOpcodeString(decodedOp.opcode);
        if (auto instr = get_if<WriteDecodedOp>(&decodedOp.instr)) {
            return format_to(ctx.out(), "PC:{:X}  {}: ${}={:X}", decodedOp.pc,
                             opcodeString, instr->dstRegId, instr->value);
        } else if (auto instr = get_if<LoadDecodedOp>(&decodedOp.instr)) {
            return format_to(ctx.out(), "PC:{:X}  {}: ${}=[{:X}]={:X}",
                             decodedOp.pc, opcodeString, instr->dstRegId,
                             instr->addr, instr->value);
        } else if (auto instr = get_if<StoreDecodedOp>(&decodedOp.instr)) {
            return format_to(ctx.out(), "PC:{:X}  {}: [{:X}]={:X}", decodedOp.pc,
                             opcodeString, instr->dstAddr, instr->value);
        } else if (auto instr = get_if<BranchDecodedOp>(&decodedOp.instr)) {
            return format_to(ctx.out(), "PC:{:X} {}: $PC={:X}", decodedOp.pc,
                             opcodeString, instr->pc);
        } else {
            assert(false);
        }
    }
};

struct Cpu {
    constexpr static u32 RESET_VECTOR = 0xBFC00000;
    const DecodedOp ZERO_INSTR;
    Registers reg;
    COP0 cop0;
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
                u32 opcode = mmap->load32(reg.pc);
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

                if (auto prevInstr =
                        get_if<LoadDecodedOp>(&prevDecoded.instr)) {
                    reg.gpr[prevInstr->dstRegId] = prevInstr->value;
                }

                reg.pc += 4;
                prevDecoded = decodedOp;
                reg.gpr[0] = 0;
            } catch (const MipsException &e) {
                cop0.epc = e.EPC;
                cop0.sr = cop0.srStackPush(true, false);
                cop0.cause = extractBits(to_underlying(e.CAUSE_CODE), 2, 5);
                reg.pc =
                    extractBits(cop0.sr, cop0.BEV, 1) ? 0xbfc00180 : 0x80000080;
            }
        }
    }

    // shifts the extractedBits to least significant end.
    u32 extractBits(u32 opcode, u32 start, u32 length) {
        const u32 end = sizeof(opcode) * 8 - 1;
        assert(end >= start && start + length - 1 <= end);

        u32 shifted = opcode >> start;
        u32 mask = static_cast<u64>(1 << length) - 1;

        assert((shifted & mask) <= static_cast<u64>(1 << length) - 1);
        return shifted & mask;
    }

    // replaces the bits in dst with the bits in src acorrding to the bitRange.
    u32 replaceBitRange(const u32 dst, const u32 dstStart, const u32 src,
                        const u32 srcStart, const u32 length) {
        static_assert(sizeof(dst) == sizeof(src));
        assert(bitSize<u32>() >= length + dstStart);
        assert(bitSize<u32>() >= length + srcStart);
        const u32 lengthOfOnes = (1 << length) - 1;
        const u32 dstMask = ~(lengthOfOnes << dstStart);
        const u32 srcMask = lengthOfOnes << srcStart;
        const s32 distance = dstStart - srcStart;

        return distance <= 0 ? (dst & dstMask) | ((src & srcMask) >> -distance)
                             : (dst & dstMask) | ((src & srcMask) << distance);
    }

    DecodedOp decode(const u32 opcode, const DecodedOp &prevInstr) {
        const u8 primaryOp = uintNoTruncCast<u8>(extractBits(opcode, 26, 6));

        return primaryOp != 0 ? decodePrimary(opcode, prevInstr)
                              : decodeSecondary(opcode);
    }

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
            default:
                throw runtime_error{format(
                    "Invalid opcode/Not implemented Yet: {:032b}, "
                    "PrimaryOpcode: {:x}, "
                    "Instruction: {}",
                    opcode, primaryOp,
                    getPrimaryOpString(static_cast<PrimaryOps>(primaryOp)))};
        };
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
            default:
                throw runtime_error{format(
                    "Invalid opcode/Not implemented Yet: {:032b}, "
                    "SecondaryOpcode: {:x}, "
                    "Instruction: {}",
                    opcode, secondaryOp,
                    getSecondaryOps(static_cast<SecondaryOps>(secondaryOp)))};
        };
    }

    u32 sw(const u32 value, const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        if (addr % 4 != 0) throw MipsException{ExcCode::AdEL, reg.pc};
        mmap->store32(addr, value);
        return addr;
    }

    u32 sh(const u32 value, const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        if (addr % 2 != 0) throw MipsException{ExcCode::AdEL, reg.pc};
        mmap->store16(addr, value);
        return addr;
    }

    u32 sb(const u32 value, const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        mmap->store8(addr, value);
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
        const u32 src = mmap->load32(wordAddr);
        const u32 bitLength = (addr - wordAddr + 1) * 8;
        return {replaceBitRange(dest, 0, src,
                                static_cast<u32>(bitSize<u32>() - bitLength),
                                bitLength),
                addr};
    }

    // Reads the least significant bytes of the source into the most significant
    // bytes of the destination.
    tuple<u32, u32> lwl(const u32 dest, const u32 base, const s16 offset) {
        const u32 addr = base + offset;
        const u32 wordAddr = (addr / 4) * 4;
        const u32 src = mmap->load32(wordAddr);
        const u32 bitLength = (addr - wordAddr + 1) * 8;
        return {
            replaceBitRange(dest, static_cast<u32>(bitSize<u32>() - bitLength),
                            src, 0, bitLength),
            addr};
    }

    tuple<u32, u32> lb(const u32 s, const s16 imm16) {
        // implicit sign-extension, this does not work in 1's complement.
        return {static_cast<s8>(mmap->load8(s + imm16)), s + imm16};
    }

    tuple<u32, u32> lbu(const u32 s, const s16 imm16) {
        return {mmap->load8(s + imm16), s + imm16};
    }

    // implicit sign-extension, this does not work in 1's complement.
    tuple<u32, u32> lh(const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        if (addr % 2 != 0) throw MipsException{ExcCode::AdEL, reg.pc};
        return {static_cast<s16>(mmap->load16(s + imm16)), addr};
    }

    tuple<u32, u32> lhu(const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        if (addr % 2 != 0) throw MipsException{ExcCode::AdEL, reg.pc};
        return {mmap->load16(addr), addr};
    }

    tuple<u32, u32> lw(const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        if (addr % 4 != 0) throw MipsException{ExcCode::AdEL, reg.pc};
        return {mmap->load32(s + imm16), addr};
    }

    s32 addi(const s32 s, const s16 imm16) {
        if ((s > 0 && imm16 > INT_MAX - s) || (s < 0 && imm16 < INT_MIN - s)) {
            throw MipsException{ExcCode::Ov, reg.pc};
        }

        return s + imm16;
    }

    u32 addiu(const u32 s, const s16 imm16) { return s + imm16; }

    u32 addu(const u32 s, const u32 t) { return s + t; }

    s32 add(const s32 s, const s32 t) {
        if ((s > 0 && t > INT_MAX - s) || (s < 0 && t < INT_MIN - s)) {
            throw MipsException{ExcCode::Ov, reg.pc};
        }

        return s + t;
    }

    u32 subu(const u32 s, const u32 t) { return s - t; }

    s32 sub(const s32 s, const s32 t) {
        if ((t > 0 && s < INT_MIN + t) || (t < 0 && s > INT_MAX + t)) {
            throw MipsException{ExcCode::Ov, reg.pc};
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

    // assert(cpu.replaceBitRange(0b10001, 0, 0b00000, 0, 5) == 0);
    // assert(cpu.replaceBitRange(0b10001, 0, 0b00000, 0, 1) == 0b10000);
    // assert(cpu.replaceBitRange(0b10001, 1, 0b10000, 4, 1) == 0b10011);
    // assert(cpu.replaceBitRange(0x0A'0B'0C'0D, 8, 0x00'01'02'03, 0, 24) ==
    //        0x01'02'03'0D);
}
