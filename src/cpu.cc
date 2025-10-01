#include "cpu.hpp"

#include "BitUtils.hpp"
#include "assembler.hpp"
#include "cop0.hpp"
#include "memory.hpp"

using namespace std;
using namespace Opcode;
using namespace BitUtils;

namespace Mips {

u32 &Registers::pc() { return gpr[Id::pc]; }

u32 &Registers::operator[](size_t index) {
    assert(index <= gpr.size());
    return gpr[index];
}

Cpu::Cpu() : mmap{new MMap{}} { reg.pc() = RESET_VECTOR; }
Cpu::~Cpu() { delete mmap; }

void Cpu::runCpuLoop() {
    println(
        "HIGH PRIORITY IMPLEMENT THE CACHE, THE ISOLATE BIT IN SR DEPENDS ON "
        "IT");
    while (true) {
        runNextInstr();
    }
    println(
        "HIGH PRIORITY IMPLEMENT THE CACHE, THE ISOLATE BIT IN SR DEPENDS ON "
        "IT");
}

namespace Kernel {
    void putc(u32 pc, u32 t1, u8 a0) {
        pc &= 0x1FFFFFFF;
        if ((pc == 0xA0 && t1 == 0x3C) || (pc == 0xB0 && t1 == 0x3D)) {
            print("{}", (char)a0);
        }
    }
};

void Cpu::runNextInstr() {
    // BD is set when the Current Instruction being ran is in a Branch Delay.
    bool BD = prevDecoded.nextInstrIsBranchDelay;
    try {
        if (reg.pc() == 0x80030000) {
            HeaderReg hreg = mmap->sideload();
            reg.gpr[Id::pc] = hreg.pc;
            reg.gpr[Id::gp] = hreg.gp;
            reg.gpr[Id::sp] = hreg.sp == 0 ? reg.gpr[Id::sp] : hreg.sp;
        }

        Kernel::putc(reg.gpr[Id::pc], reg.gpr[Id::t1], reg.gpr[Id::a0]);
        u32 opcode = mmap->load32(VAddress{reg.gpr[Id::pc]});
        DecodedOp decodedOp = decode(opcode, prevDecoded);
        reg.gpr[Id::pc] += 4;

        assert(0 <= prevDecoded.delay.dstId &&
               prevDecoded.delay.dstId <= reg.gpr.size());
        reg.gpr[prevDecoded.delay.dstId] = prevDecoded.delay.value;

        assert(0 <= decodedOp.instr.dstId &&
               decodedOp.instr.dstId <= reg.gpr.size());
        reg.gpr[decodedOp.instr.dstId] = decodedOp.instr.value;

        prevDecoded = decodedOp;
        assert(reg.gpr[Id::zero] == 0);
    } catch (const COP0::MipsException &e) {
        handleMipsException(e, BD);
    }
}

void Cpu::handleMipsException(const COP0::MipsException &e, bool BD) {
    assert(reg.gpr[Id::pc] >= 4);

    cop0.epc = BD ? reg.gpr[Id::pc] - 4 : reg.gpr[Id::pc];
    cop0.cause.ExcCode = static_cast<u32>(e.code);
    cop0.cause.BD = BD;

    cop0.sr.data = cop0.srStackPush(true, false);
    reg.gpr[Id::pc] = cop0.sr.BEV ? 0xbfc00180 : 0x80000080;
    // println("EPC: {:#X}, CAUSE: {:#X}", cop0.epc, cop0.cause.ExcCode.GetValue());
}

DecodedOp Cpu::decode(const u32 opcode, const DecodedOp &prevInstr) {
    const u8 primaryOp = extractBits(opcode, 26, 6);

    return primaryOp != 0 ? decodePrimary(opcode, prevInstr)
                          : decodeSecondary(opcode);
}

DecodedOp Cpu::decodePrimary(const u32 opcode, const DecodedOp &prevInstr) {
    const u8 primaryOp = extractBits(opcode, 26, 6);
    const u8 rd = extractBits(opcode, 11, 5);
    const u8 rs = extractBits(opcode, 21, 5);
    const u8 rt = extractBits(opcode, 16, 5);
    const u16 imm16 = extractBits(opcode, 0, 16);
    const u32 imm26 = extractBits(opcode, 0, 26);

    const DebugInfo debug = {
        .opcode = static_cast<PrimaryOps>(primaryOp),
        .pc = reg.gpr[Id::pc],
        .op = primaryOp,
        .rs = rs,
        .rt = rt,
        .rd = rd,
        .s = reg.gpr[rs],
        .t = reg.gpr[rt],
        .d = reg.gpr[rd],
        .imm16 = imm16,
        .imm26 = imm26,
    };

    DecodedOp decodedOp;

    using enum PrimaryOps;
    switch (static_cast<PrimaryOps>(primaryOp)) {
        case BCONDZ:
            return handleJumpOp(static_cast<JumpOp>(rt), opcode);
            break;
        case BLEZ:
        case BGTZ:
        case BEQ:
        case BNE:
        case J:
        case JAL:
            return handleJumpOp(static_cast<JumpOp>(primaryOp), opcode);
            break;
        case ADDIU:
            decodedOp = {.instr = {.value = addiu(reg.gpr[rs], imm16),
                                   .dstId = static_cast<Id>(rt)}};
            break;
        case ADDI:
            decodedOp = {
                .instr = {.value = static_cast<u32>(addi(reg.gpr[rs], imm16)),
                          .dstId = static_cast<Id>(rt)}};
            break;
        case LB: {
            auto [value, addr] = lb(reg.gpr[rs], imm16);
            assert(value <= NumLimit<s8>());
            decodedOp = {
                .delay = {.value = value, .dstId = static_cast<Id>(rt)}};
            break;
        }
        case LH: {
            auto [value, addr] = lh(reg.gpr[rs], imm16);
            assert(value <= NumLimit<s16>());
            decodedOp = {
                .delay = {.value = value, .dstId = static_cast<Id>(rt)}};
            break;
        }
        case LWL: {
            assert(false);
            auto [value, addr] =
                lwl(prevInstr.delay.dstId == rt ? prevInstr.delay.value
                                                : reg.gpr[rt],
                    reg.gpr[rs], imm16);
            decodedOp = {
                .delay = {.value = value, .dstId = static_cast<Id>(rt)}};
            break;
        }
        case LW: {
            auto [value, addr] = lw(reg.gpr[rs], imm16);
            decodedOp = {
                .delay = {.value = value, .dstId = static_cast<Id>(rt)}};
            break;
        }
        case LBU: {
            auto [value, addr] = lbu(reg.gpr[rs], imm16);
            decodedOp = {
                .delay = {.value = value, .dstId = static_cast<Id>(rt)}};
            break;
        }
        case LHU: {
            auto [value, addr] = lhu(reg.gpr[rs], imm16);
            decodedOp = {
                .delay = {.value = value, .dstId = static_cast<Id>(rt)}};
            break;
        }
        case LWR: {
            assert(false);
            auto [value, addr] =
                lwr(prevInstr.delay.dstId == rt ? prevInstr.delay.value
                                                : reg.gpr[rt],
                    reg.gpr[rs], imm16);
            decodedOp = {
                .delay = {.value = value, .dstId = static_cast<Id>(rt)}};
            break;
        }
        case SW: {
            sw(reg.gpr[rt], reg.gpr[rs], imm16);
            decodedOp = {};
            break;
        }
        case SH: {
            sh(reg.gpr[rt], reg.gpr[rs], imm16);
            decodedOp = {};
            break;
        }
        case SB: {
            sb(reg.gpr[rt], reg.gpr[rs], imm16);
            decodedOp = {};
            break;
        }
        case LUI:
            decodedOp = {
                .instr = {.value = lui(imm16), .dstId = static_cast<Id>(rt)}};
            break;
        case ORI:
            decodedOp = {.instr = {.value = ori(reg.gpr[rs], imm16),
                                   .dstId = static_cast<Id>(rt)}};
            break;
        case ANDI:
            decodedOp = {.instr = {.value = andi(reg.gpr[rs], imm16),
                                   .dstId = static_cast<Id>(rt)}};
            break;
        case SLTI:
            decodedOp = {.instr = {.value = slt(reg.gpr[rs], imm16),
                                   .dstId = static_cast<Id>(rt)}};
            break;
        case SLTIU:
            decodedOp = {.instr = {.value = sltiu(reg.gpr[rs], imm16),
                                   .dstId = static_cast<Id>(rt)}};
            break;
        case COP0:
            return handleCOP0(rs, rt, rd, imm16);
            break;
        default:
            throw runtime_error{
                format("Invalid opcode/Not implemented Yet: {:032b}, "
                       "PrimaryOpcode: {:x}, "
                       "Instruction: {}",
                       opcode, primaryOp,
                       getPrimaryOpString(static_cast<PrimaryOps>(primaryOp)))};
    };

    return decodedOp.withDebug(debug);
}

DecodedOp Cpu::handleJumpOp(const JumpOp jumpOp, const u32 opcode) {
    const u8 rs = extractBits(opcode, 21, 5);
    const u8 rt = extractBits(opcode, 16, 5);
    const u8 rd = extractBits(opcode, 11, 5);
    const u16 imm16 = extractBits(opcode, 0, 16);
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

    const u32 pcAfterDelaySlot = reg.gpr[Id::pc] + 8;
    DecodedOp decodedOp{};
    const DebugInfo debug{.opcode = jumpOp,
                          .pc = reg.gpr[Id::pc],
                          .rs = rs,
                          .rt = rt,
                          .rd = rd,
                          .s = reg.gpr[rs],
                          .t = reg.gpr[rt],
                          .d = reg.gpr[rd],
                          .imm16 = imm16,
                          .imm26 = imm26};

    switch (jumpOp) {
        case JALR:
            assert(rs != rd);
            decodedOp = {.instr = {.value = pcAfterDelaySlot,
                                   .dstId = static_cast<Id>(rd)},
                         .delay = {.value = jr(reg.gpr[rs]), .dstId = Id::pc}};
            break;
        case J:
            decodedOp = {
                .delay = {.value = j(reg.gpr[Id::pc], imm26), .dstId = Id::pc}};
            break;
        case JAL:
            decodedOp = {
                .instr = {.value = pcAfterDelaySlot, .dstId = Id::ra},
                .delay = {.value = j(reg.gpr[Id::pc], imm26), .dstId = Id::pc}};
            break;
        case JR:
            decodedOp = {.delay = {.value = jr(reg.gpr[rs]), .dstId = Id::pc}};
            break;
        case BGTZ:
        case BEQ:
        case BNE:
        case BLTZ:
        case BGEZ:
        case BLEZ: {
            optional<u32> newPc = branchCMPHelper(
                reg.gpr[Id::pc], imm16, cond[static_cast<size_t>(jumpOp)]);
            decodedOp = newPc.has_value()
                            ? DecodedOp{.delay = {.value = newPc.value(),
                                                  .dstId = Id::pc}}
                            : DecodedOp{};
            break;
        }
        default:
            println("{}", getJumpOpString(jumpOp));
            assert(false);
    }

    decodedOp.nextInstrIsBranchDelay = true;
    return decodedOp.withDebug(debug);
}

DecodedOp Cpu::handleCOP0(const u8 rs, const u8 rt, const u8 rd,
                          const s16 imm16) {
    DebugInfo debug{
        .opcode = static_cast<COP0Opcode>(rs),
        .pc = reg.gpr[Id::pc],
        .op = rs,
        .rs = rs,
        .rt = rt,
        .rd = rd,
        .s = reg.gpr[rs],
        .t = reg.gpr[rt],
        .d = reg.gpr[rd],
    };

    DecodedOp decodedOp{};

    using enum COP0Opcode;
    switch (static_cast<COP0Opcode>(rs)) {
        case MFC0:
            decodedOp = {
                .delay = {.value = cop0[rd], .dstId = static_cast<Id>(rt)}};
            debug.coprd = decodedOp.delay.value;
            break;
        case MTC0:
            cop0[rd] = reg.gpr[rt];
            break;
        case RFE:
            cop0.sr.data = cop0.srStackPop();
            break;
        default:
            assert(false);
    }

    return decodedOp.withDebug(debug);
}

DecodedOp Cpu::decodeSecondary(const u32 opcode) {
    const u8 secondaryOp = extractBits(opcode, 0, 6);
    const u8 rd = extractBits(opcode, 11, 5);
    const u8 rs = extractBits(opcode, 21, 5);
    const u8 rt = extractBits(opcode, 16, 5);
    const u8 imm5 = extractBits(opcode, 6, 5);

    const DebugInfo debug{.opcode = static_cast<SecondaryOps>(secondaryOp),
                          .pc = reg.gpr[Id::pc],
                          .op = secondaryOp,
                          .rs = rs,
                          .rt = rt,
                          .rd = rd,
                          .s = reg.gpr[rs],
                          .t = reg.gpr[rt],
                          .d = reg.gpr[rd]};

    DecodedOp decodedOp{};

    using enum SecondaryOps;
    switch (static_cast<SecondaryOps>(secondaryOp)) {
        case JALR:
        case JR:
            return handleJumpOp(static_cast<JumpOp>(secondaryOp), opcode);
        case AND:
            decodedOp = {.instr = {.value = andi(reg.gpr[rs], reg.gpr[rt]),
                                   .dstId = static_cast<Id>(rd)}};
            break;
        case ADD:
            decodedOp = {.instr = {.value = static_cast<u32>(
                                       add(reg.gpr[rs], reg.gpr[rt])),
                                   .dstId = static_cast<Id>(rd)}};
            break;
        case ADDU:
            decodedOp = {.instr = {.value = addu(reg.gpr[rs], reg.gpr[rt]),
                                   .dstId = static_cast<Id>(rd)}};
            break;
        case SUB:
            decodedOp = {.instr = {.value = static_cast<u32>(
                                       sub(reg.gpr[rs], reg.gpr[rt])),
                                   .dstId = static_cast<Id>(rd)}};
            break;
        case SUBU:
            decodedOp = {.instr = {.value = subu(reg.gpr[rs], reg.gpr[rt]),
                                   .dstId = static_cast<Id>(rd)}};
            break;
        case SLL:
            decodedOp = {.instr = {.value = sll(reg.gpr[rt], imm5),
                                   .dstId = static_cast<Id>(rd)}};
            break;
        case SRL:
            decodedOp = {.instr = {.value = srl(reg.gpr[rt], imm5),
                                   .dstId = static_cast<Id>(rd)}};
            break;
        case SRA:
            decodedOp = {.instr = {.value = sra(reg.gpr[rt], imm5),
                                   .dstId = static_cast<Id>(rd)}};
            break;
        case SRAV:
            decodedOp = {.instr = {.value = srav(reg.gpr[rs], reg.gpr[rt]),
                                   .dstId = static_cast<Id>(rd)}};
            break;
        case SRLV:
            decodedOp = {.instr = {.value = srlv(reg.gpr[rs], reg.gpr[rt]),
                                   .dstId = static_cast<Id>(rd)}};
            break;
        case SLLV:
            decodedOp = {.instr = {.value = sllv(reg.gpr[rs], reg.gpr[rt]),
                                   .dstId = static_cast<Id>(rd)}};
            break;
        case OR:
            decodedOp = {.instr = {.value = orInstr(reg.gpr[rs], reg.gpr[rt]),
                                   .dstId = static_cast<Id>(rd)}};
            break;
        case SLT:
            decodedOp = {.instr = {.value = slt(reg.gpr[rs], reg.gpr[rt]),
                                   .dstId = static_cast<Id>(rd)}};
            break;
        case SLTU:
            decodedOp = {.instr = {.value = sltu(reg.gpr[rs], reg.gpr[rt]),
                                   .dstId = static_cast<Id>(rd)}};
            break;
        case DIV: {
            auto [lo, hi] = div(reg.gpr[rs], reg.gpr[rt]);
            reg.lo = lo;
            reg.hi = hi;
            break;
        }
        case DIVU: {
            auto [lo, hi] = divu(reg.gpr[rs], reg.gpr[rt]);
            reg.lo = lo;
            reg.hi = hi;
            break;
        }
        case MULTU: {
            auto [lo, hi] = multu(reg.gpr[rs], reg.gpr[rt]); 
            reg.lo = lo;
            reg.hi = hi;
            break;
        }
        // Technically I should enforce the fact that the next two
        // instructions can't write to lo/hi after reads from lo/hi
        // respectively.
        case MFLO:
            decodedOp = {
                .instr = {.value = reg.lo, .dstId = static_cast<Id>(rd)}};
            break;
        case MFHI:
            decodedOp = {
                .instr = {.value = reg.hi, .dstId = static_cast<Id>(rd)}};
            break;
        case MTLO:
            reg.lo = reg.gpr[rs];
            break;
        case MTHI:
            reg.hi = reg.gpr[rs];
            break;
        case NOR:
            decodedOp = {.instr = {.value = nor(reg.gpr[rs], reg.gpr[rt]),
                                   .dstId = static_cast<Id>(rd)}};
            break;
        case SYSCALL:
            syscall();
            break;
        default:
            throw runtime_error{format(
                "Invalid opcode/Not implemented Yet: {:032b}, "
                "SecondaryOpcode: {:x}, "
                "Instruction: {}",
                opcode, secondaryOp,
                getSecondaryOpString(static_cast<SecondaryOps>(secondaryOp)))};
    };

    return decodedOp.withDebug(debug);
}

void Cpu::syscall() { throw COP0::MipsException{COP0::ExcCode::Syscall}; }

tuple<u32, u32> Cpu::multu(u64 s, u64 t) {
    const u64 res = s * t;
    const u32 hi = res >> 32;
    const u32 lo = res;
    return {lo, hi};
}

tuple<u32, u32> Cpu::divu(u32 s, u32 t) {
    if (t == 0) return {INT_MAX, s};

    return {s / t, s % t};
}

tuple<s32, s32> Cpu::div(s32 s, s32 t) {
    if (s >= 0 && t == 0) return {-1, s};
    if (s < 0 && t == 0) return {1, s};
    if (s == INT_MIN && t == -1) return {INT_MIN, 0};

    return {s / t, s % t};
}

u32 Cpu::andi(const u32 op1, const u32 op2) { return op1 & op2; }

u32 Cpu::slt(const s32 s, const s32 t) { return s < t; }

u32 Cpu::sltiu(const u32 s, const u16 imm16) {
    // this demonic looking thing is for sign extension,
    // but the comparison must be unsigned.
    return s < static_cast<u32>(static_cast<s32>(static_cast<s16>(imm16)));
}

u32 Cpu::sltu(const u32 s, const u32 t) { return s < t; }

u32 Cpu::nor(const u32 s, const u32 t) { return ~(s | t); }

u32 Cpu::orInstr(const u32 s, const u32 t) { return s | t; }

optional<u32> Cpu::branchCMPHelper(const u32 currPC, const s16 offset,
                                   const bool isBranchTaken) {
    const u32 delaySlotAddress = (currPC + 4);
    if (isBranchTaken) return delaySlotAddress + (static_cast<s32>(offset) * 4);

    return {};
}

u32 Cpu::jr(const u32 s) {
    assert(s % 4 == 0);
    return s;
}

u32 Cpu::j(const u32 currPC, const u32 imm26) {
    assert(imm26 <= (1 << 26) - 1);
    const u32 delaySlotAddress = currPC + 4;
    return (delaySlotAddress & 0xF0000000) | (imm26 << 2);
}

u32 Cpu::sw(const u32 value, const u32 s, const s16 imm16) {
    const u32 addr = s + imm16;
    assert(addr % 4 == 0);
    if (cop0.sr.IsC == 1) {
        // println("Ignore store for now need to implement cache for isolation");
        return addr;
    }
    mmap->store32(VAddress{addr}, value);
    return addr;
}

u32 Cpu::sh(const u32 value, const u32 s, const s16 imm16) {
    const u32 addr = s + imm16;
    assert(addr % 2 == 0);
    if (cop0.sr.IsC == 1) {
        // println("Ignore store for now need to implement cache for isolation");
        return addr;
    }
    mmap->store16(VAddress{addr}, value);
    return addr;
}

u32 Cpu::sb(const u32 value, const u32 s, const s16 imm16) {
    const u32 addr = s + imm16;
    if (cop0.sr.IsC == 1) {
        // println("Ignore store for now need to implement cache for isolation");
        return addr;
    }
    mmap->store8(VAddress{addr}, value);
    return addr;
}

u32 Cpu::sllv(const u32 s, const u32 t) { return sll(t, s & 0x1F); }

u32 Cpu::srav(const u32 s, const u32 t) { return sra(t, s & 0x1F); }

u32 Cpu::srlv(const u32 s, const u32 t) { return srl(t, s & 0x1F); }

u32 Cpu::sra(const s32 t, const u8 imm5) {
    assert(imm5 <= 31);
    return t >> imm5;
}

u32 Cpu::srl(const u32 t, const u8 imm5) {
    assert(imm5 <= 31);
    return t >> imm5;
}

u32 Cpu::sll(const u32 t, const u8 imm5) {
    assert(imm5 <= 31);
    return t << imm5;
}

u32 Cpu::ori(const u32 s, const u16 imm16) noexcept {
    return s | static_cast<u32>(imm16);
}

u32 Cpu::lui(const u16 imm16) noexcept { return static_cast<u32>(imm16) << 16; }

// Reads the most significant bytes of the source into the least
// significant bytes of the destination.
tuple<u32, u32> Cpu::lwr(const u32 dest, const u32 base, const s16 offset) {
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
tuple<u32, u32> Cpu::lwl(const u32 dest, const u32 base, const s16 offset) {
    const u32 addr = base + offset;
    const u32 wordAddr = (addr / 4) * 4;
    const u32 src = mmap->load32(VAddress{wordAddr});
    const u32 bitLength = (addr - wordAddr + 1) * 8;
    return {
        replaceBitRange(dest, static_cast<u32>(NumOfBits<u32>() - bitLength),
                        src, 0, bitLength),
        addr};
}

// implicit sign-extension, this does not work in 1's complement.
tuple<u32, u32> Cpu::lb(const u32 s, const s16 imm16) {
    const u32 addr = s + imm16;
    const s32 value = static_cast<s8>(mmap->load8(VAddress{addr}));
    return {value, addr};
}

tuple<u32, u32> Cpu::lbu(const u32 s, const s16 imm16) {
    const u32 addr = s + imm16;
    return {mmap->load8(VAddress{addr}), addr};
}

// implicit sign-extension, this does not work in 1's complement.
tuple<u32, u32> Cpu::lh(const u32 s, const s16 imm16) {
    const u32 addr = s + imm16;
    assert(addr % 2 == 0);
    const s32 value = static_cast<s16>(mmap->load16(VAddress{addr}));
    return {value, addr};
}

tuple<u32, u32> Cpu::lhu(const u32 s, const s16 imm16) {
    const u32 addr = s + imm16;
    assert(addr % 2 == 0);
    return {mmap->load16(VAddress{addr}), addr};
}

tuple<u32, u32> Cpu::lw(const u32 s, const s16 imm16) {
    const u32 addr = s + imm16;
    assert(addr % 4 == 0);
    return {mmap->load32(VAddress{s + imm16}), addr};
}

s32 Cpu::addi(const s32 s, const s16 imm16) {
    if ((s > 0 && imm16 > INT_MAX - s) || (s < 0 && imm16 < INT_MIN - s)) {
        assert(false);
        throw COP0::MipsException{COP0::ExcCode::Ov};
    }

    return s + imm16;
}

u32 Cpu::addiu(const u32 s, const s16 imm16) { return s + imm16; }

u32 Cpu::addu(const u32 s, const u32 t) { return s + t; }

s32 Cpu::add(const s32 s, const s32 t) {
    if ((s > 0 && t > INT_MAX - s) || (s < 0 && t < INT_MIN - s)) {
        assert(false && "OVERFLOW IN ADD");
        throw COP0::MipsException{COP0::ExcCode::Ov};
    }

    return s + t;
}

u32 Cpu::subu(const u32 s, const u32 t) { return s - t; }

s32 Cpu::sub(const s32 s, const s32 t) {
    if ((t > 0 && s < INT_MIN + t) || (t < 0 && s > INT_MAX + t)) {
        assert(false);
        throw COP0::MipsException{COP0::ExcCode::Ov};
    }

    return s - t;
}

};  // namespace Mips
