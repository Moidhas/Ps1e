#include "assembler.hpp"
#include <print>
#include "BitUtils.hpp"

using namespace BitUtils;

namespace Opcode {

std::string getPrimaryOpString(const PrimaryOps opcode) {
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
        default:
            assert(false);
    }
}

std::string getSecondaryOpString(SecondaryOps opcode) {
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
        default:
            assert(false);
    }
}

std::string getJumpOpString(JumpOp opcode) {
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
            assert(false);
    }
}

std::string getCOPString(COP0Opcode opcode) {
    switch (opcode) {
        case COP0Opcode::MFC0:
            return "MFC0";
        case COP0Opcode::MTC0:
            return "MTC0";
        case COP0Opcode::RFE:
            return "RFE";
        default:
            assert(false);
    }
}

std::string getOpcodeString(
    const std::variant<PrimaryOps, SecondaryOps, JumpOp, COP0Opcode> &opcode) {
    if (auto op = get_if<PrimaryOps>(&opcode))
        return getPrimaryOpString(*op);
    else if (auto op = get_if<SecondaryOps>(&opcode))
        return getSecondaryOpString(*op);
    else if (auto op = get_if<COP0Opcode>(&opcode))
        return getCOPString(*op);
    else
        return getJumpOpString(get<JumpOp>(opcode));
}



DecodedOp &DecodedOp::withDebug(const DebugInfo &otherDebug) {
    debug = otherDebug;
    return *this;
}


bool DebugInfo::operator==(std::variant<PrimaryOps, SecondaryOps, JumpOp, COP0Opcode> rhs) {
    if (rhs.index() != opcode.index()) return false;

    if (std::holds_alternative<PrimaryOps>(rhs)) {
        return get<PrimaryOps>(rhs) == get<PrimaryOps>(opcode);
    } else if (std::holds_alternative<SecondaryOps>(rhs)) {
        return get<SecondaryOps>(rhs) == get<SecondaryOps>(opcode);
    }  else if (std::holds_alternative<JumpOp>(rhs)) {
        return get<JumpOp>(rhs) == get<JumpOp>(opcode);
    } else {
        return get<COP0Opcode>(rhs) == get<COP0Opcode>(opcode);
    }
}

bool DebugInfo::operator==(const DebugInfo &rhs) {
    return rhs.opcode == opcode;
}


DebugInfo handleJumpOp(const JumpOp jumpOp, const u32 opcode) {
    const u8 rs = extractBits(opcode, 21, 5);
    const u8 rt = extractBits(opcode, 16, 5);
    const u16 imm16 = extractBits(opcode, 0, 16);
    const u32 imm26 = extractBits(opcode, 0, 26);

    using enum JumpOp;

    const DebugInfo debug{.opcode = jumpOp,
                          .rs = rs,
                          .rt = rt,
                          .imm16 = imm16,
                          .imm26 = imm26};

    switch (jumpOp) {
        case JALR:
            assert(false && "DO NOT IMPLEMENT THIS UNTIL REWRITE");
            break;
        case J:
        case JAL:
        case JR:
        case BEQ:
        case BNE:
        case BLTZ:
        case BGEZ:
        case BGTZ:
        case BLEZ: 
            break;
        default:
            std::println("{}", getJumpOpString(jumpOp));
            assert(false);
    }

    return debug;
}

DebugInfo handleCOP0(const u8 rs, const u8 rt, const u8 rd,
                          const s16 imm16) {

    const DebugInfo debug{
        .opcode = static_cast<COP0Opcode>(rs),
        .op = rs,
        .rs = rs,
        .rt = rt,
        .rd = rd,
    };

    DecodedOp decodedOp{};

    using enum COP0Opcode;
    switch (static_cast<COP0Opcode>(rs)) {
        case MFC0:
        case MTC0:
            break;
        case RFE:
            assert(false);
            break;
        default:
            assert(false);
    }

    return debug;
}

DebugInfo decodeSecondary(const u32 opcode) {
    const u8 secondaryOp = extractBits(opcode, 0, 6);
    const u8 rd = extractBits(opcode, 11, 5);
    const u8 rs = extractBits(opcode, 21, 5);
    const u8 rt = extractBits(opcode, 16, 5);
    const u8 imm5 = extractBits(opcode, 6, 5);

    const DebugInfo debug{.opcode = static_cast<SecondaryOps>(secondaryOp),
                          .op = secondaryOp,
                          .rs = rs,
                          .rt = rt,
                          .rd = rd,};

    DecodedOp decodedOp{};

    using enum SecondaryOps;
    switch (static_cast<SecondaryOps>(secondaryOp)) {
        case JALR:
            assert(false);
            break;
        case JR:
        case AND:
        case ADD:
            break;
        case ADDU:
        case SUB:
        case SUBU:
        case SLL:
        case OR:
        case SLT:
        case SLTU:
            break;
        case SYSCALL:
            assert(false);
            break;
        default:
            throw std::runtime_error{format(
                "Invalid opcode/Not implemented Yet: {:032b}, "
                "SecondaryOpcode: {:x}, "
                "Instruction: {}",
                opcode, secondaryOp,
                getSecondaryOpString(static_cast<SecondaryOps>(secondaryOp)))};
    };

    return debug;
}


DebugInfo decodePrimary(const u32 opcode) {
    const u8 primaryOp = extractBits(opcode, 26, 6);
    const u8 rd = extractBits(opcode, 11, 5);
    const u8 rs = extractBits(opcode, 21, 5);
    const u8 rt = extractBits(opcode, 16, 5);
    const u16 imm16 = extractBits(opcode, 0, 16);
    const u32 imm26 = extractBits(opcode, 0, 26);

    const DebugInfo debug = {
        .opcode = static_cast<PrimaryOps>(primaryOp),
        .op = primaryOp,
        .rs = rs,
        .rt = rt,
        .rd = rd,
        .imm16 = imm16,
        .imm26 = imm26,
    };

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
            break;
        case COP0:
            return handleCOP0(rs, rt, rd, imm16);
            break;
        // default:
        //     throw std::runtime_error{
        //         format("Invalid opcode/Not implemented Yet: {:032b}, "
        //                "PrimaryOpcode: {:x}, "
        //                "Instruction: {}",
        //                opcode, primaryOp,
        //                getPrimaryOpString(static_cast<PrimaryOps>(primaryOp)))};
    };

    return debug;
}

DebugInfo decode(const u32 opcode) {
    const u8 primaryOp = BitUtils::extractBits(opcode, 26, 6);

    return primaryOp != 0 ? decodePrimary(opcode)
                          : decodeSecondary(opcode);
}

};  // namespace Opcode
