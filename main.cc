#include <sys/stat.h>

#include <array>
#include <cassert>
#include <climits>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <utility>
using namespace std;

using s8 = int8_t;
using s16 = int16_t;
using s32 = int32_t;
using s64 = int64_t;

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

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

// These contain the insturctions that use the most significant six bits of the opcode to decode.
// It contains the necesssary bits to decode.
enum class PrimaryOps: u8 {
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

// These contain the insturctions that use the least significant six bits of the opcode to decode.
// It contains the necesssary bits to decode.
enum class SecondaryOps : u8 {
    SLL     = 0x00,
    SRL     = 0x02,
    SRA     = 0x03,
    SLLV    = 0x04,
    SRLV    = 0x06,
    SRAV    = 0x07,
    JR      = 0x08,
    JALR    = 0x09,
    SYSCALL = 0x0C,
    BREAK   = 0x0D,
    MFHI    = 0x10,
    MTHI    = 0x11,
    MFLO    = 0x12,
    MTLO    = 0x13,
    MULT    = 0x18,
    MULTU   = 0x19,
    DIV     = 0x1A,
    DIVU    = 0x1B,
    ADD     = 0x20,
    ADDU    = 0x21,
    SUB     = 0x22,
    SUBU    = 0x23,
    AND     = 0x24,
    OR      = 0x25,
    XOR     = 0x26,
    NOR     = 0x27,
    SLT     = 0x2A,
    SLTU    = 0x2B,
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

struct Bios {
    array<u8, 512 * 1024> buffer;

    Bios() {
        struct stat info;
        stat("SCPH1001.BIN", &info);
        if (buffer.size() != info.st_size)
            throw runtime_error{"size of BIOS is not 512KB"};

        ifstream bios{"SCPH1001.BIN", ios::binary | ios::in};
        bios.read((char *)buffer.data(), buffer.size());
    }

    u32 load32(u32 offset) {
        assert(buffer.size() > offset + 3 && offset >= 0);
        u32 byte0 = (u32)buffer[offset + 0];
        u32 byte1 = (u32)buffer[offset + 1] << 8;
        u32 byte2 = (u32)buffer[offset + 2] << 16;
        u32 byte3 = (u32)buffer[offset + 3] << 24;

        return byte0 | byte1 | byte2 | byte3;
    }

    u16 load16(u32 offset) {
        assert(buffer.size() > offset + 1 && offset >= 0);
        u32 byte0 = (u32)buffer[offset];
        u32 byte1 = (u32)buffer[offset + 1] << 8;
        return byte0 | byte1;
    }

    u8 load8(u32 offset) {
        assert(buffer.size() > offset && offset >= 0);
        u32 byte0 = (u32)buffer[offset];

        return byte0;
    }
};

struct Range {
    const u32 start;
    const u32 length;
    Range(u32 start, u32 length) : start{start}, length{length} {}

    bool contains(u32 addr) const {
        return start <= addr && addr < start + length;
    }

    u32 getOffset(u32 addr) const {
        assert(contains(addr));
        return addr - start;
    }
};

// Memory is little-endian.
struct Memory {
    const Range BIOS_RANGE{0xBFC00000, 512 * (1 << 10)};
    Bios bios;

    u32 load32(u32 addr) {
        assert(BIOS_RANGE.contains(addr));
        assert(addr % 4 == 0);
        return bios.load32(BIOS_RANGE.getOffset(addr));
    }

    u16 load16(u32 addr) {
        assert(BIOS_RANGE.contains(addr));
        assert(addr % 2 != 0);
        return bios.load8(BIOS_RANGE.getOffset(addr));
    }

    u8 load8(u32 addr) {
        assert(BIOS_RANGE.contains(addr));
        return bios.load8(BIOS_RANGE.getOffset(addr));
    }

    u32 store32(u32 addr) {}
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

enum class State { NoLoadDelay, LoadDelay };

template <typename T, size_t Size>
class FQueue {
    array<T, Size> buffer;
    size_t i = 0;

   public:
    T top() { return buffer[i]; }

    void pop() { i = (i + 1) % buffer.size(); }

    void push(const T &val) { buffer[(i + 1) % buffer.size()] = val; }

    size_t size() { return buffer.size(); }
};

struct DelaySlot {
    u32 regId;
    u32 value;
};

struct Cpu {
    Registers reg;
    State state;
    FQueue<DelaySlot, 2> loadDelaySlots;
    COP0 cop0;
    Memory mem;

    Cpu() { reg.pc = mem.BIOS_RANGE.start; }

    void runCpuLoop() {
        while (true) {
            u32 opcode = mem.load32(reg.pc);
            try {
                const State newState = exeInstr(opcode);
                reg.pc += 4;
                if (state == State::LoadDelay) {
                    const DelaySlot delaySlot = loadDelaySlots.top();
                    loadDelaySlots.pop();
                    reg.gpr[delaySlot.regId] = delaySlot.value;
                }
                state = newState;
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
        u32 mask = (1 << length) - 1;

        assert((shifted & mask) <= (1 << length) - 1);
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

    State exeInstr(u32 opcode) {
        const u8 op = uintNoTruncCast<u8>(extractBits(opcode, 26, 6));
        const u8 op2 = uintNoTruncCast<u8>(extractBits(opcode, 0, 6));
        const u8 rd = uintNoTruncCast<u8>(extractBits(opcode, 11, 5));
        const u8 rs = uintNoTruncCast<u8>(extractBits(opcode, 21, 5));
        const u8 rt = uintNoTruncCast<u8>(extractBits(opcode, 16, 5));
        const u8 imm5 = uintNoTruncCast<u8>(extractBits(opcode, 6, 5));
        const u16 imm16 = uintNoTruncCast<u16>(extractBits(opcode, 0, 16));
        const u32 imm26 = extractBits(opcode, 0, 26);

        if (op != 0) {
            switch (static_cast<PrimaryOps>(op)) {
                case PrimaryOps::LB:
                    loadDelaySlots.push({rt, lb(reg.gpr[rs], imm16)});
                    return State::LoadDelay;
                case PrimaryOps::LH:
                    loadDelaySlots.push({rt, lh(reg.gpr[rs], imm16)});
                    return State::LoadDelay;
                case PrimaryOps::LWL:  // LWL
                case PrimaryOps::LW:
                    loadDelaySlots.push({rt, lw(reg.gpr[rs], imm16)});
                    return State::LoadDelay;
                case PrimaryOps::LBU:
                    loadDelaySlots.push({rt, lbu(reg.gpr[rs], imm16)});
                    return State::LoadDelay;
                case PrimaryOps::LHU:
                    loadDelaySlots.push({rt, lhu(reg.gpr[rs], imm16)});
                    return State::LoadDelay;
                case PrimaryOps::LWR: 
                default:
                    throw runtime_error{
                        format("invalid opcode: {:032b}, op: {:x}, op2: {:x}",
                               opcode, op, op2)};
            };
        } else {
            // R-Type
            switch (static_cast<SecondaryOps>(op2)) {
                case SecondaryOps::ADD:
                    reg.gpr[rd] = add(rs, rt);
                    break;
                case SecondaryOps::ADDU:
                    reg.gpr[rd] = addu(rs, rt);
                    break;
                case SecondaryOps::SUB:
                    reg.gpr[rd] = sub(rs, rt);
                    break;
                case SecondaryOps::SUBU:
                    reg.gpr[rd] = subu(rs, rt);
                    break;
                default:
                    throw runtime_error{
                        format("invalid opcode: {:032b}, op: {:x}, op2: {:x}",
                               opcode, op, op2)};
            };
        }

        return State::NoLoadDelay;
    }

    // Right means read the most significant bytes of the source into the low
    // bytes of the destination.
    u32 lwr(const u32 r, const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        const u32 wordAddr = (addr / 4) * 4;
        const u32 word = mem.load32(wordAddr);
        const u32 bitLength = (addr - wordAddr + 1) * 8;
        return replaceBitRange(r, 0, word,
                               static_cast<u32>(bitSize<u32>() - bitLength),
                               bitLength);
    }

    // Means read the least significant bytes of the source into the high bytes
    // of the destination.
    u32 lwl(const u32 r, const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        const u32 wordAddr = (addr / 4) * 4;
        const u32 word = mem.load32(wordAddr);
        const u32 bitLength = (addr - wordAddr + 1) * 8;
        return replaceBitRange(r, static_cast<u32>(bitSize<u32>() - bitLength),
                               word, 0, bitLength);
    }

    u32 lb(const u32 s, const s16 imm16) {
        // implicit sign-extension, this does not work in 1's complement.
        return static_cast<s8>(mem.load8(s + imm16));
    }

    u32 lbu(const u32 s, const s16 imm16) { return mem.load8(s + imm16); }

    // implicit sign-extension, this does not work in 1's complement.
    u32 lh(const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        if (addr % 2 != 0) throw MipsException{ExcCode::AdEL, reg.pc};
        return static_cast<s16>(mem.load16(s + imm16));
    }

    u32 lhu(const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        if (addr % 2 != 0) throw MipsException{ExcCode::AdEL, reg.pc};
        return mem.load16(addr);
    }

    u32 lw(const u32 s, const s16 imm16) {
        const u32 addr = s + imm16;
        if (addr % 4 != 0) throw MipsException{ExcCode::AdEL, reg.pc};
        return mem.load32(s + imm16);
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
    Cpu cpu;
    assert(cpu.replaceBitRange(0b10001, 0, 0b00000, 0, 5) == 0);
    assert(cpu.replaceBitRange(0b10001, 0, 0b00000, 0, 1) == 0b10000);
    assert(cpu.replaceBitRange(0b10001, 1, 0b10000, 4, 1) == 0b10011);
    assert(cpu.replaceBitRange(0x0A'0B'0C'0D, 8, 0x00'01'02'03, 0, 24) ==
           0x01'02'03'0D);

    // try {
    //     Cpu cpu;
    //     cpu.runCpuLoop();
    // } catch (runtime_error &e) {
    //     cerr << e.what() << endl;
    // }
}
