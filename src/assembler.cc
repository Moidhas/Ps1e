#include "assembler.hpp"

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
            return "MFCO";
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

};  // namespace Opcode
