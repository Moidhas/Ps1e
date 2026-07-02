#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <mach/mach.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/fileport.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
typedef int64_t i64;
typedef int32_t i32;
typedef int16_t i16;
typedef int8_t i8;

/* all of these are in bytes. */
#define KB (1 << 10)
#define MB (1 << 20)
#define GB (1 << 30)

/* all the sizes are in bytes. */
#define BIOS_SIZE (512 * KB)
#define IO_PORTS_SIZE (4 * KB)
#define RAM_SIZE (2 * MB)
#define MIRROR_RAM_SIZE (8 * MB)
#define EXPANSION_REGION1_SIZE (8 * MB)
/* since the psx uses 32 bit addresses, total address space is 2^32 */
#define ADDR_SPACE_SIZE (0x100000000)
#define KUSEG_SIZE (0x80000000) /* only the first 512 MB are valid */
#define KSEG0_SIZE (512 * MB)
#define KSEG1_SIZE (KSEG0_SIZE)
#define KSEG2_SIZE (GB)

#define KUSEG_START (0)
#define KSEG0_START (KUSEG_START + KUSEG_SIZE)
#define KSEG1_START (KSEG0_START + KSEG0_SIZE)
#define KSEG2_START (KSEG1_START + KSEG1_SIZE)

#define BASE_EXPANSION_REGION1_ADDR 0x1F000000
#define BASE_BIOS_ADDR 0x1FC00000
#define RESET_EXCEPTION_VECTOR (KSEG1_START + BASE_BIOS_ADDR)
#define BASE_IO_PORTS_ADDR 0x1F801000

#define BIOS_PATH "./BIOS/SCPH1001.BIN"

#define GPR_SIZE 32
/* for all macros assume opcode is sizeof(u32) bytes */
#define GET_PRIMARY(opcode) ((opcode) >> 26)
#define GET_SECONDARY(opcode) ((opcode) & 0x3F)
#define GET_IMM26(opcode) ((opcode) & 0x3FFFFFF)
#define GET_UIMM16(opcode) ((opcode) & 0xFFFF)      /* Unsigned */
#define GET_SIMM16(opcode) (i16)((opcode) & 0xFFFF) /* Signed */
#define GET_IMM5(opcode) (((opcode) >> 6) & 0x1F)
#define GET_RD(opcode) (((opcode) >> 11) & 0x1F) /* Bits 11 to 15 */
#define GET_RT(opcode) (((opcode) >> 16) & 0x1F)
#define GET_RS(opcode) (((opcode) >> 21) & 0x1F) /* Bits 21 to 25 */

/* SR MACROS */
#define SR_GET_ISC(sr) (((sr) >> 16) & 0x1)

#define SWAP(type, a, b)                                                       \
    do {                                                                       \
        type temp = b;                                                         \
        b = a;                                                                 \
        a = temp;                                                              \
    } while (0)

#define CHECK_ERR(expr, fstr, fmt)                                             \
    do {                                                                       \
        if (expr) {                                                            \
            fprintf(stderr, fstr, fmt);                                        \
            exit(EXIT_FAILURE);                                                \
        }                                                                      \
    } while (0)

size_t get_page_size()
{
    static size_t page_size = 0;
    if (page_size == 0) {
        page_size = sysconf(_SC_PAGESIZE);
    }
    return page_size;
}

struct psx {
    union {
        u32 regs[35];
        struct {
            u32 zero; /* Constant (always 0) */
            u32 at;   /* Assembler temporary (destroyed by some assembler
                         pseudoinstructions!) */
            u32 v0;
            u32 v1; /* Subroutine return values, may be changed by subroutines
                     */
            u32 a0;
            u32 a1;
            u32 a2;
            u32 a3; /* Subroutine arguments, may be changed by subroutines */
            u32 t0;
            u32 t1;
            u32 t2;
            u32 t3;
            u32 t4;
            u32 t5;
            u32 t6;
            u32 t7; /* Temporaries, may be changed by subroutines */
            u32 s0;
            u32 s1;
            u32 s2;
            u32 s3;
            u32 s4;
            u32 s5;
            u32 s6;
            u32 s7; /* Static variables, must be saved by subs */
            u32 t8;
            u32 t9; /* Temporaries, may be changed by subroutines */
            u32 k0;
            u32 k1; /* Reserved for kernel (destroyed by some IRQ handlers!)
                     */
            u32 gp; /* Global pointer (rarely used) */
            u32 sp; /* Stack pointer */
            u32 fp; /* Frame Pointer, or 9th Static variable, must be saved */
            u32 ra; /* Return address (used so by JAL,BLTZAL,BGEZAL opcodes)
                     */
            u32 pc; /* Program Counter, used to get the current instruction to
                       run from memory. Important the convention is that when
                       executing instruction the pc points to next instruction
                       to execute. */
            u32 hi;
            u32 lo; /* These registers are not general purpose (can't be
                       accessed by opcodes) */
        };
    };

    /* Loading operations store values into delay slots which get flushed out
     * after the next opcode completes.
     * The current behaviour does not take into account the following scenarios:
     * scene1: lw rn, value, add rn, value1
     * scene2: lw rn, value, lw rn value1 */
    struct load_delay_slot {
        /* index into regs */
        u32 reg_index;
        /* value to be stored at regs[reg_index] */
        u32 value;
    } load_delay_slots[2];

    union {
        u32 cop0[16];
        struct {
            u32 useless[12];
            u32 sr; /* System status register (R/W) */
            u32 useless2[3];
        };
    };

    /* Used to handle branch delays, denotes the value of pc after the current
     * instruction completes. Jump instructions write to this. */
    u32 next_pc;

    /* These are little endian, and m1 chip is also little endian.
     * load and store operations use this to access memory. Gets changed to
     * isolated_cache on ISC = 1, on 0 it's memmap. */
    u8 *addr;
    /* main memory, program counter uses this to access memory. */
    u8 *memmap;
    /* For now just a dummy address space that gets written to (instead of main
     * memory) when ISC = 1. */
    u8 *isolated_cache;
};

/* I treat zero as empty technically this could fail if you load zero into this
 * but for now idc. */
struct load_delay_slot lds_pop(struct load_delay_slot lds[2])
{
    struct load_delay_slot top = lds[0];

    lds[0].reg_index = lds[1].reg_index;
    lds[0].value = lds[1].value;

    lds[1].reg_index = 0;
    lds[1].value = 0;

    return top;
}

/* just push into back load delay slot. */
void lds_push(struct load_delay_slot lds[2], u32 reg_index, u32 value)
{
    assert(!(reg_index == 0 && value != 0));
    lds[1].reg_index = reg_index;
    lds[1].value = value;
}

void sll(struct psx *psx, u32 opcode)
{
    psx->regs[GET_RD(opcode)] = GET_RT(opcode) << GET_IMM5(opcode);
}

void sltu(struct psx *psx, u32 opcode)
{
    psx->regs[GET_RD(opcode)] =
        (psx->regs[GET_RS(opcode)] < psx->regs[GET_RT(opcode)]);
}

void lui(struct psx *psx, u32 opcode)
{
    psx->regs[GET_RT(opcode)] = opcode << 16;
}

/* In mips the instruction being processed has the pc pointing to the next
 * instruction to be run. */

void jr(struct psx *psx, u32 opcode)
{
    assert(psx->regs[GET_RS(opcode)] % 4 == 0 && "unaligned address");
    psx->next_pc = psx->regs[GET_RS(opcode)];
}

void j(struct psx *psx, u32 opcode)
{
    psx->next_pc = (psx->pc & 0xF0000000) + (GET_IMM26(opcode) << 2);
}

void jal(struct psx *psx, u32 addr_to_instr, u32 opcode)
{
    psx->next_pc = (psx->pc & 0xF0000000) + (GET_IMM26(opcode) << 2);
    psx->ra = addr_to_instr + 8;
}

void beq(struct psx *psx, u32 addr_to_instr, u32 opcode)
{
    if (psx->regs[GET_RS(opcode)] == psx->regs[(GET_RT(opcode))]) {
        psx->next_pc = addr_to_instr + 4 + (GET_SIMM16(opcode) << 2);
    }
}

void bne(struct psx *psx, u32 addr_to_instr, u32 opcode)
{
    if (psx->regs[GET_RS(opcode)] != psx->regs[(GET_RT(opcode))]) {
        psx->next_pc = addr_to_instr + 4 + (GET_SIMM16(opcode) << 2);
    }
}

void addiu(struct psx *psx, u32 opcode)
{
    psx->regs[GET_RT(opcode)] = psx->regs[GET_RS(opcode)] + GET_SIMM16(opcode);
}

void addi(struct psx *psx, u32 opcode)
{
    i32 val = psx->regs[GET_RS(opcode)];
    i16 imm = GET_SIMM16(opcode);
    i64 sum = (i64)val + imm;

    if (INT_MAX < sum || sum < INT_MIN) {
        assert(false);
    }

    psx->regs[GET_RT(opcode)] = sum;
}

void add(struct psx *psx, u32 opcode)
{
    i32 a = psx->regs[GET_RS(opcode)];
    i32 b = psx->regs[GET_RT(opcode)];
    i64 sum = (i64)a + b;

    if (INT_MAX < sum || sum < INT_MIN) {
        assert(false);
    }

    psx->regs[GET_RD(opcode)] = sum;
}

void addu(struct psx *psx, u32 opcode)
{
    psx->regs[GET_RD(opcode)] =
        psx->regs[GET_RS(opcode)] + psx->regs[GET_RT(opcode)];
}

void ori(struct psx *psx, u32 opcode)
{
    psx->regs[GET_RT(opcode)] = psx->regs[GET_RS(opcode)] | GET_UIMM16(opcode);
}

void andi(struct psx *psx, u32 opcode)
{
    psx->regs[GET_RT(opcode)] = psx->regs[GET_RS(opcode)] & GET_UIMM16(opcode);
}

void sb(struct psx *psx, u32 opcode)
{
    u32 addr = psx->regs[GET_RS(opcode)] + GET_SIMM16(opcode);
    u16 value = psx->regs[GET_RT(opcode)] & 0xFF;
    printf("\tstoring to memory at: 0x%08x, value: 0x%08x\n", addr, value);

    /* assuming machine is little endian */
    u8 *byte = &psx->addr[addr];
    byte[0] = value;
}

void sh(struct psx *psx, u32 opcode)
{
    u32 addr = psx->regs[GET_RS(opcode)] + GET_SIMM16(opcode);
    /* has to be 2 bytes aligned */
    assert(addr % sizeof(u16) == 0);

    u16 value = psx->regs[GET_RT(opcode)] & 0xFFFF;
    printf("\tstoring to memory at: 0x%08x, value: 0x%08x\n", addr, value);

    /* assuming machine is little endian */
    u16 *half_word = (u16 *)&psx->addr[addr];
    half_word[0] = value;
}

void sw(struct psx *psx, u32 opcode)
{
    u32 addr = psx->regs[GET_RS(opcode)] + GET_SIMM16(opcode);
    printf("\tstoring to memory at: 0x%08x, value: 0x%08x\n", addr,
           psx->regs[GET_RT(opcode)]);

    /* has to be 4 bytes aligned */
    assert(addr % sizeof(u32) == 0);


    /* assuming machine is little endian */
    u32 *word = (u32 *)&psx->addr[addr];
    word[0] = psx->regs[GET_RT(opcode)];
}

void lhu(struct psx *psx, u32 opcode)
{
    assert(false);
    /* has to be 2 bytes aligned */
    u32 word = (i32)GET_SIMM16(opcode) + psx->regs[GET_RS(opcode)];
    assert(word % 2 == 0);

    printf("\tloading from memory at: 0x%08x, value: 0x%08x\n", word,
           *(u16 *)&psx->addr[word]);

    /* assuming machine is little endian */
    psx->regs[GET_RT(opcode)] = *(u16 *)&psx->addr[word];
}

void and(struct psx *psx, u32 opcode)
{
    psx->regs[GET_RD(opcode)] =
        psx->regs[GET_RS(opcode)] & psx->regs[GET_RT(opcode)];
}

void or(struct psx *psx, u32 opcode)
{
    psx->regs[GET_RD(opcode)] =
        psx->regs[GET_RS(opcode)] | psx->regs[GET_RT(opcode)];
}

void lb(struct psx *psx, u32 opcode)
{
    u32 addr = (i32)GET_SIMM16(opcode) + psx->regs[GET_RS(opcode)];

    u8 byte = psx->addr[addr];

    printf("\tloading from memory at: 0x%08x, value: 0x%08x\n", addr, byte);

    lds_push(psx->load_delay_slots, GET_RT(opcode), byte);
}

void lw(struct psx *psx, u32 opcode)
{
    u32 addr = (i32)GET_SIMM16(opcode) + psx->regs[GET_RS(opcode)];
    /* needs to be an exception if not 4 bytes aligned. */
    assert(addr % 4 == 0);

    /* assuming machine is little endian. change to memcpy. */
    u32 word = *(u32 *)&psx->addr[addr];

    printf("\tloading from memory at: 0x%08x, value: 0x%08x\n", addr, word);

    lds_push(psx->load_delay_slots, GET_RT(opcode), word);
}

void cop0_exec(struct psx *psx, u32 opcode)
{
    if (GET_RS(opcode) == 0) { // mfc0
        lds_push(psx->load_delay_slots, GET_RT(opcode),
                 psx->cop0[GET_RD(opcode)]);
        printf("\tloading from cop0 at: %d, value: 0x%08x\n", GET_RD(opcode),
               psx->cop0[GET_RD(opcode)]);
    } else if (GET_RS(opcode) == 0b00100) { // mtc0 ignoring store delays
        assert(GET_RD(opcode) < (sizeof(psx->cop0) / sizeof(psx->cop0[0])));
        psx->cop0[GET_RD(opcode)] = psx->regs[GET_RT(opcode)];

        printf("\tstoring to cop0 at: %d, value: 0x%08x\n", GET_RD(opcode),
               psx->regs[GET_RT(opcode)]);
    } else {
        assert(false);
    }

    if (SR_GET_ISC(psx->sr) == 1) {
        psx->addr = psx->isolated_cache;
    } else if (SR_GET_ISC(psx->sr) == 0) {
        psx->addr = psx->memmap;
    } else {
        assert(false);
    }
}

void start(struct psx *psx)
{
    static void *dispatch_tbl[] = {
        &&h_na,   &&h_bcondz, &&h_j,    &&h_jal,   &&h_beq,  &&h_bne,
        &&h_blez, &&h_bgtz,   &&h_addi, &&h_addiu, &&h_slti, &&h_sltiu,
        &&h_andi, &&h_ori,    &&h_xori, &&h_lui,   &&h_cop0, &&h_cop1,
        &&h_cop2, &&h_cop3,   &&h_na,   &&h_na,    &&h_na,   &&h_na,
        &&h_na,   &&h_na,     &&h_na,   &&h_na,    &&h_na,   &&h_na,
        &&h_na,   &&h_na,     &&h_lb,   &&h_lh,    &&h_lwl,  &&h_lw,
        &&h_lbu,  &&h_lhu,    &&h_lwr,  &&h_na,    &&h_sb,   &&h_sh,
        &&h_swl,  &&h_sw,     &&h_na,   &&h_na,    &&h_swr,  &&h_na,
        &&h_lwc0, &&h_lwc1,   &&h_lwc2, &&h_lwc3,  &&h_na,   &&h_na,
        &&h_na,   &&h_na,     &&h_swc0, &&h_swc1,  &&h_swc2, &&h_swc3,
        &&h_na,   &&h_na,     &&h_na,   &&h_na,
    };

    static void *sp_dispatch_tbl[] = {
        &&h_sll,     &&h_na,    &&h_srl, &&h_sra,  &&h_sllv, &&h_na,
        &&h_srlv,    &&h_srav,  &&h_jr,  &&h_jalr, &&h_na,   &&h_na,
        &&h_syscall, &&h_break, &&h_na,  &&h_na,   &&h_mfhi, &&h_mthi,
        &&h_mflo,    &&h_mtlo,  &&h_na,  &&h_na,   &&h_na,   &&h_na,
        &&h_mult,    &&h_multu, &&h_div, &&h_divu, &&h_na,   &&h_na,
        &&h_na,      &&h_na,    &&h_add, &&h_addu, &&h_sub,  &&h_subu,
        &&h_and,     &&h_or,    &&h_xor, &&h_nor,  &&h_na,   &&h_na,
        &&h_slt,     &&h_sltu,  &&h_na,  &&h_na,   &&h_na,   &&h_na,
        &&h_na,      &&h_na,    &&h_na,  &&h_na,   &&h_na,   &&h_na,
        &&h_na,      &&h_na,    &&h_na,  &&h_na,   &&h_na,   &&h_na,
        &&h_na,      &&h_na,    &&h_na,  &&h_na,
    };

    u32 opcode = 0;
    psx->pc = RESET_EXCEPTION_VECTOR;
    psx->next_pc = psx->pc + 4;
    u32 addr_to_instr = psx->pc;

#define DEBUG_INFO(symbol_name)                                                \
    do {                                                                       \
        printf("pc: 0x%08x, %s, opcode: 0x%08x, primary: 0x%02x, secondary: "  \
               "0x%02x\n",                                                     \
               psx->pc, symbol_name, opcode, GET_PRIMARY(opcode),              \
               GET_SECONDARY(opcode));                                         \
    } while (0)

#define PRE_OP()                                                               \
    do {                                                                       \
        psx->pc = psx->next_pc;                                                \
        psx->next_pc = psx->pc + 4;                                            \
    } while (0)

#define POST_OP()                                                              \
    do {                                                                       \
        struct load_delay_slot top = lds_pop(psx->load_delay_slots);           \
        psx->regs[top.reg_index] = top.value;                                  \
    } while (0)

#define DISPATCH()                                                             \
    do {                                                                       \
        assert(psx->zero == 0);                                                \
        opcode = *((u32 *)&psx->memmap[psx->pc]); /* m1 is little endian */    \
        assert(GET_PRIMARY(opcode) <                                           \
               sizeof(dispatch_tbl) / sizeof(dispatch_tbl[0]));                \
        if (GET_PRIMARY(opcode) != 0) {                                        \
            goto *dispatch_tbl[GET_PRIMARY(opcode)];                           \
        } else {                                                               \
            assert(GET_SECONDARY(opcode) <                                     \
                   sizeof(sp_dispatch_tbl) / sizeof(sp_dispatch_tbl[0]));      \
            goto *sp_dispatch_tbl[GET_SECONDARY(opcode)];                      \
        }                                                                      \
    } while (0)

    DISPATCH();

h_bcondz:
    DEBUG_INFO("h_bcondz");
    assert(false);
h_j:
    DEBUG_INFO("h_j");
    psx->pc = psx->next_pc;
    j(psx, opcode);
    POST_OP();
    DISPATCH();
h_jal:
    DEBUG_INFO("h_jal");
    addr_to_instr = psx->pc;
    psx->pc = psx->next_pc;
    jal(psx, addr_to_instr, opcode);
    POST_OP();
    DISPATCH();
h_beq:
    DEBUG_INFO("h_beq");
    addr_to_instr = psx->pc;
    PRE_OP();
    beq(psx, addr_to_instr, opcode);
    POST_OP();
    DISPATCH();
h_bne:
    DEBUG_INFO("h_bne");
    addr_to_instr = psx->pc;
    PRE_OP();
    bne(psx, addr_to_instr, opcode);
    POST_OP();
    DISPATCH();
h_blez:
    DEBUG_INFO("h_blez");
    assert(false);
h_bgtz:
    DEBUG_INFO("h_bgtz");
    assert(false);
h_addi:
    DEBUG_INFO("h_addi");
    PRE_OP();
    addi(psx, opcode);
    POST_OP();
    DISPATCH();
h_addiu:
    DEBUG_INFO("h_addiu");
    PRE_OP();
    addiu(psx, opcode);
    POST_OP();
    DISPATCH();
h_slti:
    DEBUG_INFO("h_slti");
    assert(false);
h_sltiu:
    DEBUG_INFO("h_sltiu");
    assert(false);
h_andi:
    DEBUG_INFO("h_andi");
    PRE_OP();
    andi(psx, opcode);
    POST_OP();
    DISPATCH();
h_ori:
    DEBUG_INFO("h_ori");
    PRE_OP();
    ori(psx, opcode);
    POST_OP();
    DISPATCH();
h_xori:
    DEBUG_INFO("h_xori");
    assert(false);
h_lui:
    DEBUG_INFO("h_lui");
    PRE_OP();
    lui(psx, opcode);
    POST_OP();
    DISPATCH();
h_cop0:
    DEBUG_INFO("h_cop0");
    PRE_OP();
    cop0_exec(psx, opcode);
    POST_OP();
    DISPATCH();
h_cop1:
    DEBUG_INFO("h_cop1");
    assert(false);
h_cop2:
    DEBUG_INFO("h_cop2");
    assert(false);
h_cop3:
    DEBUG_INFO("h_cop3");
    assert(false);
h_lb:
    DEBUG_INFO("h_lb");
    PRE_OP();
    lb(psx, opcode);
    POST_OP();
    DISPATCH();
    assert(false);
h_lh:
    DEBUG_INFO("h_lh");
    assert(false);
h_lwl:
    DEBUG_INFO("h_lwl");
    assert(false);
h_lw:
    DEBUG_INFO("h_lw");
    PRE_OP();
    lw(psx, opcode);
    POST_OP();
    DISPATCH();
h_lbu:
    DEBUG_INFO("h_lbu");
    assert(false);
h_lhu:
    DEBUG_INFO("h_lhu");
    assert(false);
    PRE_OP();
    lhu(psx, opcode);
    POST_OP();
    DISPATCH();
h_lwr:
    DEBUG_INFO("h_lwr");
    assert(false);
h_sb:
    DEBUG_INFO("h_sb");
    PRE_OP();
    sb(psx, opcode);
    POST_OP();
    DISPATCH();
h_sh:
    DEBUG_INFO("h_sh");
    PRE_OP();
    sh(psx, opcode);
    POST_OP();
    DISPATCH();
h_swl:
    DEBUG_INFO("h_swl");
    assert(false);
h_sw:
    DEBUG_INFO("h_sw");
    PRE_OP();
    sw(psx, opcode);
    POST_OP();
    DISPATCH();
h_swr:
    DEBUG_INFO("h_swr");
    assert(false);
h_lwc0:
    DEBUG_INFO("h_lwc0");
    assert(false);
h_lwc1:
    DEBUG_INFO("h_lwc1");
    assert(false);
h_lwc2:
    DEBUG_INFO("h_lwc2");
    assert(false);
h_lwc3:
    DEBUG_INFO("h_lwc3");
    assert(false);
h_swc0:
    DEBUG_INFO("h_swc0");
    assert(false);
h_swc1:
    DEBUG_INFO("h_swc1");
    assert(false);
h_swc2:
    DEBUG_INFO("h_swc2");
    assert(false);
h_swc3:
    DEBUG_INFO("h_swc3");
    assert(false);
/* special */
h_sll:
    DEBUG_INFO("h_sll");
    PRE_OP();
    sll(psx, opcode);
    POST_OP();
    DISPATCH();
h_srl:
    DEBUG_INFO("h_srl");
    assert(false);
h_sra:
    DEBUG_INFO("h_sra");
    assert(false);
h_sllv:
    DEBUG_INFO("h_sllv");
    assert(false);
h_srlv:
    DEBUG_INFO("h_srlv");
    assert(false);
h_srav:
    DEBUG_INFO("h_srav");
    assert(false);
h_jr:
    DEBUG_INFO("h_jr");
    psx->pc = psx->next_pc;
    jr(psx, opcode);
    POST_OP();
    DISPATCH();
    assert(false);
h_jalr:
    DEBUG_INFO("h_jalr");
    assert(false);
h_syscall:
    DEBUG_INFO("h_syscall");
    assert(false);
h_break:
    DEBUG_INFO("h_break");
    assert(false);
h_mfhi:
    DEBUG_INFO("h_mfhi");
    assert(false);
h_mthi:
    DEBUG_INFO("h_mthi");
    assert(false);
h_mflo:
    DEBUG_INFO("h_mflo");
    assert(false);
h_mtlo:
    DEBUG_INFO("h_mtlo");
    assert(false);
h_mult:
    DEBUG_INFO("h_mult");
    assert(false);
h_multu:
    DEBUG_INFO("h_multu");
    assert(false);
h_div:
    DEBUG_INFO("h_div");
    assert(false);
h_divu:
    DEBUG_INFO("h_divu");
    assert(false);
h_add:
    DEBUG_INFO("h_add");
    PRE_OP();
    add(psx, opcode);
    POST_OP();
    DISPATCH();
h_addu:
    DEBUG_INFO("h_addu");
    PRE_OP();
    addu(psx, opcode);
    POST_OP();
    DISPATCH();
h_sub:
    DEBUG_INFO("h_sub");
    assert(false);
h_subu:
    DEBUG_INFO("h_subu");
    assert(false);
h_and:
    DEBUG_INFO("h_and");
    PRE_OP();
    and(psx, opcode);
    POST_OP();
    DISPATCH();
h_or:
    DEBUG_INFO("h_or");
    PRE_OP();
    or(psx, opcode);
    POST_OP();
    DISPATCH();
h_xor:
    DEBUG_INFO("h_xor");
    assert(false);
h_nor:
    DEBUG_INFO("h_nor");
    assert(false);
h_slt:
    DEBUG_INFO("h_slt");
    assert(false);
h_sltu:
    DEBUG_INFO("h_sltu");
    PRE_OP();
    sltu(psx, opcode);
    POST_OP();
    DISPATCH();
h_na:
    DEBUG_INFO("h_na");
    assert(false);

#undef DISPATCH
}

void make_kseg(vm_address_t base_kseg, mem_entry_name_port_t shared_ram,
               int bios_fd, mem_entry_name_port_t shared_er1,
               mem_entry_name_port_t shared_io)
{
    kern_return_t ret = KERN_SUCCESS;

    for (vm_address_t ram_mirror = base_kseg;
         ram_mirror < base_kseg + MIRROR_RAM_SIZE; ram_mirror += RAM_SIZE) {

        ret = vm_map(mach_task_self(), &ram_mirror, RAM_SIZE, 0,
                     VM_FLAGS_FIXED | VM_FLAGS_OVERWRITE, shared_ram, 0, false,
                     VM_PROT_READ | VM_PROT_WRITE, VM_PROT_READ | VM_PROT_WRITE,
                     VM_INHERIT_DEFAULT);
        CHECK_ERR(ret != KERN_SUCCESS, "error allocating ram kseg space: %d\n",
                  ret);
    }

    vm_address_t base_addr = base_kseg + BASE_EXPANSION_REGION1_ADDR;
    ret = vm_map(mach_task_self(), &base_addr, EXPANSION_REGION1_SIZE, 0,
                 VM_FLAGS_FIXED | VM_FLAGS_OVERWRITE, shared_er1, 0, false,
                 VM_PROT_READ | VM_PROT_WRITE, VM_PROT_READ | VM_PROT_WRITE,
                 VM_INHERIT_DEFAULT);
    CHECK_ERR(ret != KERN_SUCCESS,
              "error allocating expansion region 1 space: %d\n", ret);

    base_addr = base_kseg + BASE_IO_PORTS_ADDR;
    ret = vm_map(mach_task_self(), &base_addr, IO_PORTS_SIZE, 0,
                 VM_FLAGS_FIXED | VM_FLAGS_OVERWRITE, shared_io, 0, false,
                 VM_PROT_READ | VM_PROT_WRITE, VM_PROT_READ | VM_PROT_WRITE,
                 VM_INHERIT_DEFAULT);
    CHECK_ERR(ret != KERN_SUCCESS, "error allocating shared io space: %d\n",
              ret);

    u8 *bios = mmap((u8 *)base_kseg + BASE_BIOS_ADDR, BIOS_SIZE, PROT_READ,
                    MAP_PRIVATE | MAP_FIXED, bios_fd, 0);
    CHECK_ERR(bios == MAP_FAILED, "error allocating bios space: %d\n", errno);
}

void test_addr_space(u8 *base)
{
    base[0] = 'A';
    base[1] = 'S';
    base[2] = 'S';
    assert(memcmp(base, base + KSEG0_START, 3) == 0);
    assert(memcmp(base, base + KSEG1_START, 3) == 0);
    assert(memcmp(base, base + RAM_SIZE, 3) == 0);
    assert(memcmp(base, base + 2 * RAM_SIZE, 3) == 0);

    assert(memcmp(base + BASE_BIOS_ADDR, base + KSEG0_START + BASE_BIOS_ADDR,
                  8) == 0);
    assert(memcmp(base + BASE_BIOS_ADDR, base + RESET_EXCEPTION_VECTOR, 8) ==
           0);

    base[BASE_EXPANSION_REGION1_ADDR] = 'A';
    assert(memcmp(base + BASE_EXPANSION_REGION1_ADDR,
                  base + KSEG0_START + BASE_EXPANSION_REGION1_ADDR, 1) == 0);
    assert(memcmp(base + BASE_EXPANSION_REGION1_ADDR,
                  base + KSEG1_START + BASE_EXPANSION_REGION1_ADDR, 1) == 0);

    base[BASE_IO_PORTS_ADDR] = 'A';
    assert(memcmp(base + BASE_IO_PORTS_ADDR,
                  base + KSEG0_START + BASE_IO_PORTS_ADDR, 1) == 0);
    assert(memcmp(base + BASE_IO_PORTS_ADDR,
                  base + KSEG1_START + BASE_IO_PORTS_ADDR, 1) == 0);
}

void *make_psx_addr_space()
{
    /* mirrored memory objects */
    memory_object_size_t size = RAM_SIZE;
    mem_entry_name_port_t shared_ram = MACH_PORT_NULL;

    kern_return_t ret = mach_make_memory_entry_64(
        mach_task_self(), &size, 0,
        MAP_MEM_NAMED_CREATE | VM_PROT_READ | VM_PROT_WRITE, &shared_ram,
        MACH_PORT_NULL);
    CHECK_ERR(ret != KERN_SUCCESS, "error making shared_ram: %d\n", ret);

    size = EXPANSION_REGION1_SIZE;
    mem_entry_name_port_t shared_er1 = MACH_PORT_NULL;
    ret = mach_make_memory_entry_64(mach_task_self(), &size, 0,
                                    MAP_MEM_NAMED_CREATE | VM_PROT_READ |
                                        VM_PROT_WRITE,
                                    &shared_er1, MACH_PORT_NULL);
    CHECK_ERR(ret != KERN_SUCCESS, "error making shared_er1: %d\n", ret);

    size = IO_PORTS_SIZE;
    mem_entry_name_port_t shared_io = MACH_PORT_NULL;
    ret = mach_make_memory_entry_64(mach_task_self(), &size, 0,
                                    MAP_MEM_NAMED_CREATE | VM_PROT_READ |
                                        VM_PROT_WRITE,
                                    &shared_io, MACH_PORT_NULL);
    CHECK_ERR(ret != KERN_SUCCESS, "error making shared_io: %d\n", ret);

    vm_address_t addr = 0;
    ret = vm_map(mach_task_self(), &addr, ADDR_SPACE_SIZE, 0, VM_FLAGS_ANYWHERE,
                 MEMORY_OBJECT_NULL, 0, false, VM_PROT_READ | VM_PROT_WRITE,
                 VM_PROT_READ | VM_PROT_WRITE, VM_INHERIT_DEFAULT);
    CHECK_ERR(ret != KERN_SUCCESS, "error allocating addr space: %d\n", ret);

    int bios_fd = open(BIOS_PATH, O_RDONLY);
    CHECK_ERR(bios_fd < 0, "error opening bios: %d\n", errno);
    struct stat st;
    CHECK_ERR(fstat(bios_fd, &st) < 0, "error stats bios: %d\n", errno);
    assert(st.st_size == BIOS_SIZE);

    make_kseg(addr + KUSEG_START, shared_ram, bios_fd, shared_er1, shared_io);
    make_kseg(addr + KSEG0_START, shared_ram, bios_fd, shared_er1, shared_io);
    make_kseg(addr + KSEG1_START, shared_ram, bios_fd, shared_er1, shared_io);

    /* cleanup */
    ret = mach_port_deallocate(mach_task_self(), shared_ram);
    CHECK_ERR(ret != KERN_SUCCESS, "error deallocating shared_ram: %d\n", ret);
    ret = mach_port_deallocate(mach_task_self(), shared_er1);
    CHECK_ERR(ret != KERN_SUCCESS, "error deallocating shared_ram: %d\n", ret);
    ret = mach_port_deallocate(mach_task_self(), shared_io);
    CHECK_ERR(ret != KERN_SUCCESS, "error deallocating shared_io: %d\n", ret);
    close(bios_fd);

    test_addr_space((u8 *)addr);

    return (void *)addr;
}

int main()
{
    struct psx psx;
    memset(&psx, 0, sizeof(psx));
    psx.addr = make_psx_addr_space();
    psx.memmap = psx.addr;

    vm_address_t isolated_cache = 0;
    kern_return_t ret =
        vm_map(mach_task_self(), &isolated_cache, ADDR_SPACE_SIZE, 0,
               VM_FLAGS_ANYWHERE, MEMORY_OBJECT_NULL, 0, false,
               VM_PROT_READ | VM_PROT_WRITE, VM_PROT_READ | VM_PROT_WRITE,
               VM_INHERIT_DEFAULT);
    CHECK_ERR(ret != KERN_SUCCESS, "error allocating addr space: %d\n", ret);
    psx.isolated_cache = (u8 *)isolated_cache;

    start(&psx);
    vm_deallocate(mach_task_self(), (vm_address_t)psx.addr, ADDR_SPACE_SIZE);
    vm_deallocate(mach_task_self(), (vm_address_t)psx.isolated_cache,
                  ADDR_SPACE_SIZE);
}
