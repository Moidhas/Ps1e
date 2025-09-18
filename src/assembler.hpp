#ifndef ASSEMBLER_H
#define ASSEMBLER_H

#include <cassert>
#include <format>
#include <string>
#include <variant>

#include "types.hpp"

namespace Opcode {

enum class COP0Opcode : u8 { MFC0 = 0b00000, MTC0 = 0b00100, RFE = 0b10000 };

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

std::string getPrimaryOpString(const PrimaryOps opcode);

std::string getSecondaryOpString(SecondaryOps opcode);

std::string getJumpOpString(JumpOp opcode);

std::string getCOPString(COP0Opcode opcode);

std::string getOpcodeString(
    const std::variant<PrimaryOps, SecondaryOps, JumpOp, COP0Opcode> &opcode);

enum Id {
    zero = 0,
    at = 1,
    v0 = 2,
    v1 = 3,
    a0 = 4,
    a1 = 5,
    a2 = 6,
    a3 = 7,
    t0 = 8,
    t1 = 9,
    t2 = 10,
    t3 = 11,
    t4 = 12,
    t5 = 13,
    t6 = 14,
    t7 = 15,
    s0 = 16,
    s1 = 17,
    s2 = 18,
    s3 = 19,
    s4 = 20,
    s5 = 21,
    s6 = 22,
    s7 = 23,
    t8 = 24,
    t9 = 25,
    k0 = 26,
    k1 = 27,
    gp = 28,
    sp = 29,
    fp = 30,
    ra = 31,
    pc = 32,
    hi = 33,
    lo = 34,
};

struct OpcodeNode {
    u32 value{0};
    Id dstId{zero};
};

enum class DebugType { Immediate, Jump, Register };

struct DebugInfo {
    std::variant<PrimaryOps, SecondaryOps, JumpOp, COP0Opcode> opcode;
    DebugType type;
    u32 pc;
    u8 op;
    u8 rs;
    u8 rt;
    u8 rd;
    u32 s;
    u32 t;
    u32 d;
    u8 shamt;
    u8 funct;
    u16 imm16;
    u32 imm26;  // target

    template <typename It>
    It format_to(It out) const {
        out =
            std::format_to(out, "PC: {:#X} {}: ", pc, getOpcodeString(opcode));
        if (auto op = get_if<PrimaryOps>(&opcode))
            return format_to(out, *op);
        else if (auto op = get_if<SecondaryOps>(&opcode))
            return format_to(out, *op);
        else if (auto op = get_if<COP0Opcode>(&opcode))
            return format_to(out, *op);
        else
            return format_to(out, get<JumpOp>(opcode));
    }

    template <typename It>
    It format_to(It out, COP0Opcode copOp) const {
        using enum COP0Opcode;
        switch (copOp) {
            case MFC0:
                out = std::format_to(out, "rt=${}, rd=${}, rt <- cop0[rd]", rt,
                                     rd);
                break;
            case MTC0:
                out = std::format_to(
                    out, "rt=${}={:#X}, rd=${}, cop0[rd] <- rt", rt, t, rd);
                break;
            case RFE:
                break;
            default:
                assert(false);
        }
        return out;
    }

    template <typename It>
    It format_to(It out, SecondaryOps secondary) const {
        using enum SecondaryOps;
        switch (secondary) {
            case SLL:
            case ADDU:
            case SUB:
            case SUBU:
            case SLT:
            case SLTU:
            case AND:
            case OR:
            case XOR:
            case NOR:
            case ADD:
                out = std::format_to(out,
                                     "rd=${}={:#X}, rs=${}={:#X}, rt=${}={:#X}",
                                     rd, d, rs, s, rt, t);
                break;
            default:
                out = std::format_to(out, "No debug Info for {}",
                                     getSecondaryOpString(secondary));
        }
        return out;
    }

    template <typename It>
    It format_to(It out, JumpOp jumpOp) const {
        using enum JumpOp;
        switch (jumpOp) {
            case BLTZ:
            case BGEZ:
            case BEQ:
            case BNE:
            case BLEZ:
            case BGTZ:
            case BLTZAL:
            case BGEZAL:
                out = std::format_to(out,
                                     "rt=${}={:#X}, rs=${}={:#X}, imm16={:#X}",
                                     rt, t, rs, s, imm16);
                break;
            case J:
            case JAL:
                out = std::format_to(out, "imm26={:#X}", imm26);
                break;
            case JALR:
            case JR:
                out = std::format_to(out, "rs=${}={:#X}", rs, s);
                break;
            default:
                assert(false);
        }
        return out;
    }

    template <typename It>
    It format_to(It out, PrimaryOps primary) const {
        using enum PrimaryOps;

        switch (primary) {
            case SB:
            case LUI:
            case SH:
            case SW:
            case LB:
            case LBU:
            case LH:
            case LHU:
            case LW:
            case ADDIU:
            case ADDI:
            case ANDI:
            case ORI:
            case XORI:
            case LWL:
            case LWR:
            case SWR:
            case SWL:
                out = std::format_to(out,
                                     "rt=${}={:#X}, rs=${}={:#X}, imm16={:#X}",
                                     rt, t, rs, s, imm16);
                break;
            default:
                out = std::format_to(out, "No debug Info for {}",
                                     getPrimaryOpString(primary));
        }

        return out;
    }
};

struct DecodedOp {
    OpcodeNode instr;
    OpcodeNode delay;
    std::optional<DebugInfo> debug;
    DecodedOp &withDebug(const DebugInfo &otherDebug);
};

};  // namespace Opcode

template <>
struct std::formatter<Opcode::DecodedOp> {
    constexpr auto parse(const std::format_parse_context &ctx) {
        auto it = ctx.begin();
        assert(it == ctx.end() || *it == '}');
        return it;
    }

    auto format(const Opcode::DecodedOp &decodedOp, auto &ctx) const {
        if (!decodedOp.debug.has_value())
            return std::format_to(ctx.out(), "No debug info");

        Opcode::DebugInfo debugInfo = decodedOp.debug.value();
        return debugInfo.format_to(ctx.out());
    }
};
#endif
