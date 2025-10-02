#ifndef CPU_H
#define CPU_H
#include <array>

#include "assembler.hpp"
#include "cop0.hpp"
#include "memory.hpp"
#include "types.hpp"

namespace Mips {

using namespace Opcode;

struct Registers {
    std::array<u32, 33> gpr;
    u32 &pc();
    u32 hi;
    u32 lo;

    u32 &operator[](size_t index);
};

// Have to be careful about pc when setting EPC,
// EPC should usually point to pc + 4,
// since pc is incremented after instr commpletes.
struct Cpu {
    constexpr static u32 RESET_VECTOR = 0xBFC00000;
    constexpr static DecodedOp ZERO_INSTR;
    bool flag{false};

    Registers reg;
    COP0::COP0 cop0;
    MMap *mmap;
    DecodedOp prevDecoded{ZERO_INSTR};

    Cpu();
    ~Cpu();

    void runCpuLoop();

    void runNextInstr();

    void handleMipsException(const COP0::MipsException &e, bool BD);

    DecodedOp decode(const u32 opcode, const DecodedOp &prevInstr);

    DecodedOp decodePrimary(const u32 opcode, const DecodedOp &prevInstr);

    DecodedOp handleJumpOp(const JumpOp jumpOp, const u32 opcode);

    DecodedOp handleCOP0(const u8 rs, const u8 rt, const u8 rd,
                         const s16 imm16);

    DecodedOp decodeSecondary(const u32 opcode);

    void syscall();

    std::tuple<u32, u32> multu(u64 s, u64 t);
    std::tuple<u32, u32> divu(u32 s, u32 t);
    std::tuple<s32, s32> div(s32 s, s32 t);

    u32 andi(const u32 op1, const u32 op2);

    u32 slt(const s32 s, const s32 t);

    u32 sltiu(const u32 s, const u16 imm16);

    u32 sltu(const u32 s, const u32 t);

    u32 orInstr(const u32 s, const u32 t);

    std::optional<u32> branchCMPHelper(const u32 currPC, const s16 offset,
                                       const bool isBranchTaken);

    u32 jr(const u32 s);

    u32 j(const u32 currPC, const u32 imm26);

    u32 sw(const u32 value, const u32 s, const s16 imm16);

    u32 sh(const u32 value, const u32 s, const s16 imm16);

    u32 sb(const u32 value, const u32 s, const s16 imm16);

    u32 sllv(const u32 s, const u32 t);
    u32 srlv(const u32 s, const u32 t);
    u32 srav(const u32 s, const u32 t);
    u32 sll(const u32 t, const u8 imm5);
    u32 srl(const u32 t, const u8 imm5);
    u32 sra(const s32 t, const u8 imm5); 
    u32 lui(const u16 imm16) noexcept;

    u32 ori(const u32 s, const u16 imm16) noexcept;

    u32 nor(const u32 s, const u32 t);


    // Reads the most significant bytes of the source into the least
    // significant bytes of the destination.
    std::tuple<u32, u32> lwr(const u32 dest, const u32 base, const s16 offset);

    // Reads the least significant bytes of the source into the most significant
    // bytes of the destination.
    std::tuple<u32, u32> lwl(const u32 dest, const u32 base, const s16 offset);

    // implicit sign-extension, this does not work in 1's complement.
    std::tuple<u32, u32> lb(const u32 s, const s16 imm16);

    std::tuple<u32, u32> lbu(const u32 s, const s16 imm16);

    // implicit sign-extension, this does not work in 1's complement.
    std::tuple<u32, u32> lh(const u32 s, const s16 imm16);

    std::tuple<u32, u32> lhu(const u32 s, const s16 imm16);

    std::tuple<u32, u32> lw(const u32 s, const s16 imm16);

    s32 addi(const s32 s, const s16 imm16);
    u32 addiu(const u32 s, const s16 imm16);
    u32 addu(const u32 s, const u32 t);
    s32 add(const s32 s, const s32 t);
    u32 subu(const u32 s, const u32 t);
    s32 sub(const s32 s, const s32 t);
};

};  // namespace Mips
#endif
