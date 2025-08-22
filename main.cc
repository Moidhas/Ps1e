#include <sys/stat.h>

#include <array>
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

struct Range {
    const u32 start;
    const u32 length;
    constexpr Range(const u32 start, const u32 length)
        : start{start}, length{length} {}

    constexpr bool contains(const u32 addr) const {
        if (length == 0) return false;
        return start <= addr && addr <= start + (length - 1);
    }

    constexpr u32 getOffset(const u32 addr) const {
        assert(contains(addr));
        return addr - start;
    }
};

class MMap {
    // User Memory: KUSEG is intended to contain 2GB virtual memory (on extended
    // MIPS processors), the PSX doesn't support virtual memory, and KUSEG
    // simply contains a mirror of KSEG0/KSEG1 (in the first 512MB) (trying to
    // access memory in the remaining 1.5GB causes an exception).
    // source: https://psx-spx.consoledev.net/memorymap/#write-queue
    constexpr static Range KUSEG_RANGE{0x00000000, 512 * MB};
    constexpr static Range KSEG0_RANGE{0x80000000, 512 * MB};
    constexpr static Range KSEG1_RANGE{0xA0000000, 512 * MB};
    constexpr static Range KSEG2_RANGE{0xC0000000, GB};
    constexpr static Range PRAM_RANGE{0x00000000, 2 * MB};
    constexpr static Range PE1_RANGE{0x1F000000, 8 * MB};
    constexpr static Range PSCRATCHPAD_RANGE{0x1F800000, KB};
    constexpr static Range PIO_RANGE{0x1F801000, 4 * KB};
    constexpr static Range PE2_RANGE{0x1F802000, 8 * KB};
    constexpr static Range PE3_RANGE{0x1FA00000, 2 * MB};
    constexpr static Range PBIOS_RANGE{0x1FC00000, 512 * KB};

    array<u8, PRAM_RANGE.length> ramBuffer;
    array<u8, PE1_RANGE.length> e1Buffer;
    array<u8, PSCRATCHPAD_RANGE.length> scratchpadBuffer;
    array<u8, PIO_RANGE.length> ioBuffer;
    array<u8, PE2_RANGE.length> e2Buffer;
    array<u8, PE3_RANGE.length> e3Buffer;
    array<u8, PBIOS_RANGE.length> biosBuffer;

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

    span<u8> getBuffer(BufferType type) {
        using enum BufferType;
        switch (type) {
            case Ram:
                return ramBuffer;
            case E1:
                return e1Buffer;
            case Scratchpad:
                return scratchpadBuffer;
            case Io:
                return ioBuffer;
            case E2:
                return e2Buffer;
            case E3:
                return e3Buffer;
            case Bios:
                return biosBuffer;
        }
    }

    BufferType getType(const PAddress pAddr) {
        const u32 addr = pAddr.m_addr;
        using enum BufferType;
        if (PRAM_RANGE.contains(addr))
            return Ram;
        else if (PE1_RANGE.contains(addr))
            return E1;
        else if (PSCRATCHPAD_RANGE.contains(addr))
            return Scratchpad;
        else if (PIO_RANGE.contains(addr))
            return Io;
        else if (PE2_RANGE.contains(addr))
            return E2;
        else if (PE3_RANGE.contains(addr))
            return E3;
        else if (PBIOS_RANGE.contains(addr))
            return Bios;
        else {
            println("{:X}", addr);
            assert(false);
        }
    }

    u32 getOffset(const BufferType type, const PAddress pAddr) {
        const u32 addr = pAddr.m_addr;
        using enum BufferType;
        switch (type) {
            case Ram:
                return PRAM_RANGE.getOffset(addr);
            case E1:
                return PE1_RANGE.getOffset(addr);
            case Scratchpad:
                return PSCRATCHPAD_RANGE.getOffset(addr);
            case Io:
                return PIO_RANGE.getOffset(addr);
            case E2:
                return PE2_RANGE.getOffset(addr);
            case E3:
                return PE3_RANGE.getOffset(addr);
            case Bios:
                return PBIOS_RANGE.getOffset(addr);
        }
    }

    // TODO: need to implement this behaviour, KSEG1 addresses can't access
    // scratchpad.
    PAddress paddr(const VAddress vAddr) {
        const u32 addr = vAddr.m_addr;
        const bool condition = KSEG1_RANGE.contains(addr) ||
                               KSEG0_RANGE.contains(addr) ||
                               KUSEG_RANGE.contains(addr);
        if (condition)
            return PAddress{~0xE0000000 & addr};
        else if (KSEG2_RANGE.contains(addr))
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
        span<u8> buffer = getBuffer(type);
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
        span<u8> buffer = getBuffer(type);
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
        span<u8> buffer = getBuffer(type);
        const u32 offset = getOffset(type, pAddr);

        assert(buffer.size() > offset);
        u32 byte0 = (u32)buffer[offset];
        return byte0;
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

enum class Type { Write, Branch };

struct DecodedOp {
    State newState;
    Type type;
    u32 dstRegId;
    u32 value;
    variant<PrimaryOps, SecondaryOps> opcode;

    DecodedOp(State newState, Type type, u32 dstRegId, u32 value,
              variant<PrimaryOps, SecondaryOps> opcode)
        : newState{newState},
          type{type},
          dstRegId{dstRegId},
          value{value},
          opcode{opcode} {}
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
    constexpr auto parse(std::format_parse_context& ctx) {
        return ctx.begin();
    }

    auto format(const DecodedOp &instr, auto &ctx) const {
        return format_to(ctx.out(), "opcode {}: ${}={}",  getOpcodeString(instr.opcode), instr.dstRegId, instr.value);
    }
};

struct Cpu {
    constexpr static u32 RESET_VECTOR = 0xBFC00000;
    const DecodedOp ZERO_INSTR;
    Registers reg;
    COP0 cop0;
    MMap *mmap;

    Cpu()
        : ZERO_INSTR{State::NoLoadDelay, Type::Write, 0, 0, SecondaryOps::ADD},
          mmap{new MMap{}} {
        reg.pc = RESET_VECTOR;
    }
    ~Cpu() { delete mmap; }

    // This is where all side effects should be.
    void runCpuLoop() {
        DecodedOp prevInstr{ZERO_INSTR};

        while (true) {
            u32 opcode = mmap->load32(reg.pc);
            try {
                DecodedOp decodedOp = decode(opcode, prevInstr);
                assert(decodedOp.dstRegId < reg.gpr.size());
                println("{}", decodedOp);

                // TODO: unless an IRQ occurs between the load and
                // next opcode, in that case the load would complete during IRQ
                // handling, and so, the next opcode would receive the NEW value
                if (decodedOp.newState == State::NoLoadDelay &&
                    decodedOp.type == Type::Write) {
                    reg.gpr[decodedOp.dstRegId] = decodedOp.value;

                    if (prevInstr.dstRegId == decodedOp.dstRegId) {
                        prevInstr.newState = State::OverWritten;
                    }
                }

                if (prevInstr.newState == State::LoadDelay ||
                    prevInstr.newState == State::SpecialLoadDelay) {
                    reg.gpr[prevInstr.dstRegId] = prevInstr.value;
                }

                reg.pc += 4;
                prevInstr = decodedOp;
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
            case LB:
                return {State::LoadDelay, Type::Write, rt,
                        lb(reg.gpr[rs], imm16), LB};
            case LH:
                return {State::LoadDelay, Type::Write, rt,
                        lh(reg.gpr[rs], imm16), LH};
            case LWL:
                if (prevInstr.newState == State::SpecialLoadDelay &&
                    prevInstr.dstRegId == rt) {
                    return {State::SpecialLoadDelay, Type::Write, rt,
                            lwl(prevInstr.value, reg.gpr[rs], imm16), LWL};
                } else {
                    return {State::SpecialLoadDelay, Type::Write, rt,
                            lwl(reg.gpr[rt], reg.gpr[rs], imm16), LWL};
                }
            case LW:
                return {State::LoadDelay, Type::Write, rt,
                        lw(reg.gpr[rs], imm16), LW};
            case LBU:
                return {State::LoadDelay, Type::Write, rt,
                        lbu(reg.gpr[rs], imm16), LBU};
            case LHU:
                return {State::LoadDelay, Type::Write, rt,
                        lhu(reg.gpr[rs], imm16), LHU};
            case LWR:
                if (prevInstr.newState == State::SpecialLoadDelay &&
                    prevInstr.dstRegId == rt) {
                    return {State::SpecialLoadDelay, Type::Write, rt,
                            lwr(prevInstr.value, reg.gpr[rs], imm16), LWR};
                } else {
                    return {State::SpecialLoadDelay, Type::Write, rt,
                            lwr(reg.gpr[rt], reg.gpr[rs], imm16), LWR};
                }
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
                return {State::NoLoadDelay, Type::Write, rd,
                        static_cast<u32>(add(rs, rt)), ADD};
            case ADDU:
                return {State::NoLoadDelay, Type::Write, rd, addu(rs, rt),
                        ADDU};
            case SUB:
                return {State::NoLoadDelay, Type::Write, rd,
                        static_cast<u32>(sub(rs, rt)), SUB};
            case SUBU:
                return {State::NoLoadDelay, Type::Write, rd, subu(rs, rt),
                        SUBU};
            default:
                throw runtime_error{format(
                    "Invalid opcode/Not implemented Yet: {:032b}, "
                    "SecondaryOpcode: {:x}, "
                    "Instruction: {}",
                    opcode, secondaryOp,
                    getSecondaryOps(static_cast<SecondaryOps>(secondaryOp)))};
        };
    }

    // Reads the most significant bytes of the source into the least
    // significant bytes of the destination.
    u32 lwr(const u32 dest, const u32 base, const s16 offset) {
        const u32 addr = base + offset;
        const u32 wordAddr = (addr / 4) * 4;
        const u32 src = mmap->load32(wordAddr);
        const u32 bitLength = (addr - wordAddr + 1) * 8;
        return replaceBitRange(dest, 0, src,
                               static_cast<u32>(bitSize<u32>() - bitLength),
                               bitLength);
    }

    // Reads the least significant bytes of the source into the most significant
    // bytes of the destination.
    u32 lwl(const u32 dest, const u32 base, const s16 offset) {
        const u32 addr = base + offset;
        const u32 wordAddr = (addr / 4) * 4;
        const u32 src = mmap->load32(wordAddr);
        const u32 bitLength = (addr - wordAddr + 1) * 8;
        return replaceBitRange(dest,
                               static_cast<u32>(bitSize<u32>() - bitLength),
                               src, 0, bitLength);
    }

    u32 lb(const u32 s, const s16 imm16) {
        // implicit sign-extension, this does not work in 1's complement.
        return static_cast<s8>(mmap->load8(s + imm16));
    }

    u32 lbu(const u32 s, const s16 imm16) { return mmap->load8(s + imm16); }

    // implicit sign-extension, this does not work in 1's complement.
    u32 lh(const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        if (addr % 2 != 0) throw MipsException{ExcCode::AdEL, reg.pc};
        return static_cast<s16>(mmap->load16(s + imm16));
    }

    u32 lhu(const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        if (addr % 2 != 0) throw MipsException{ExcCode::AdEL, reg.pc};
        return mmap->load16(addr);
    }

    u32 lw(const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        if (addr % 4 != 0) throw MipsException{ExcCode::AdEL, reg.pc};
        return mmap->load32(s + imm16);
    }

    u32 addu(u32 rs, u32 rt) {
        const u32 s = reg.gpr[rs];
        const u32 t = reg.gpr[rt];
        return s + t;
    }

    s32 add(u32 rs, u32 rt) {
        const s32 s = reg.gpr[rs];
        const s32 t = reg.gpr[rt];

        if ((s > 0 && t > INT_MAX - s) || (s < 0 && t < INT_MIN - s)) {
            throw MipsException{ExcCode::Ov, reg.pc};
        }

        return s + t;
    }

    u32 subu(u32 rs, u32 rt) {
        const u32 s = reg.gpr[rs];
        const u32 t = reg.gpr[rt];
        return s - t;
    }

    s32 sub(u32 rs, u32 rt) {
        const s32 s = reg.gpr[rs];
        const s32 t = reg.gpr[rt];

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
