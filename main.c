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

/* DEBUG RELATED MACROS */
#define DEBUG 1 // not 0 debug, 0 no debug.
// always 7 spaces between opcode and register fields in debug format.
#define OPCODE_ALIGNMENT 7

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
#define PADDR(vaddr) ((vaddr) & 0x1FFFFFFF)

#define BIOS_PATH "./BIOS/SCPH1001.BIN"

/* for all macros assume instr is sizeof(u32) bytes */
#define GET_OPCODE(instr) ((instr) >> 26)
#define GET_FUNCT(instr) ((instr) & 0x3F)
#define GET_RS(instr) (((instr) >> 21) & 0x1F) /* Bits 21 to 25 */
#define GET_RT(instr) (((instr) >> 16) & 0x1F)
#define GET_RD(instr) (((instr) >> 11) & 0x1F) /* Bits 11 to 15 */
#define GET_SHAMT(instr) (((instr) >> 6) & 0x1F)

#define GET_IMM26(instr) ((instr) & 0x3FFFFFF)
#define GET_UIMM16(instr) ((instr) & 0xFFFF)       /* Unsigned */
#define GET_IMM16(instr) ((i16)((instr) & 0xFFFF)) /* Signed */

#define GET_TARGET(pc, instr) (((pc) & 0xF0000000) | (GET_IMM26(instr) << 2))
#define GET_BRANCH(addr_to_instr, instr)                                       \
    ((addr_to_instr) + 4 + (GET_IMM16(instr) << 2))

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

const char *gpr_name[] = {
    "r0", "at", "v0", "v1", "a0", "a1", "a2", "a3", "t0", "t1", "t2", "t3",
    "t4", "t5", "t6", "t7", "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
    "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra", "pc", "hi", "lo",
};

const char *cop0_name[] = {
    "Index",    "Random", "EntryLo0", "BPC",      // 00
    "Context",  "BDA",    "PIDMask",  "DCIC",     // 04
    "BadVAddr", "BDAM",   "EntryHi",  "BPCM",     // 08
    "Status",   "Cause",  "EPC",      "PRid",     // 0c
    "Config",   "LLAddr", "WatchLo",  "WatchHi",  // 10
    "XContext", "*RES*",  "*RES*",    "*RES*",    // 14
    "*RES*",    "*RES*",  "ECC",      "CacheErr", // 18
    "TagLo",    "TagHi",  "ErrorEPC", "*RES*",
};

struct psx {
    union {
        u32 regs[35];
        struct {
            u32 r0; /* Constant (always 0) */
            u32 at; /* Assembler temporary (destroyed by some assembler
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
     * after the next instruction completes.
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

    /* Address to current instruction executing.
     * Needed since pc points to the address of the next instruction to be
     * executed during execution of the current instruction. */
    u32 addr_to_instr;

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

static inline void print_imm16(i16 imm)
{
    if (imm < 0) {
        printf("-0x%04x", -imm);
    } else {
        printf("0x%04x", imm);
    }
}

static inline void print_word(struct psx *psx, u32 instr)
{
    u32 addr = psx->regs[GET_RS(instr)] + GET_IMM16(instr);
    u32 word = *(u32 *)(&(psx->addr[addr]));
    printf("([%08x] = %08x)", addr, word);
}

static inline void print_halfword(struct psx *psx, u32 instr)
{
    u32 addr = psx->regs[GET_RS(instr)] + GET_IMM16(instr);
    u16 halfword = *(u16 *)&psx->addr[addr];
    printf("([%08x] = %04x)", addr, halfword);
}

static inline void print_byte(struct psx *psx, u32 instr)
{
    u32 addr = psx->regs[GET_RS(instr)] + GET_IMM16(instr);
    u8 byte = psx->addr[addr];
    printf("([%08x] = %02x)", addr, byte);
}

enum dasm_op {
    d_RS,
    d_RT,
    d_RD,
    d_SHAMT,
    d_UIMM16,
    d_IMM16,
    d_WORD,
    d_HALFWORD,
    d_BYTE,
    d_TARGET,
    d_COP0_RD,
    d_BRANCH,
    STOP,
};

void dasm_print(struct psx *psx, u32 instr, const char *sym, ...)
{
    printf("%08x %08x: %-*s", psx->addr_to_instr, instr, OPCODE_ALIGNMENT, sym);

    bool comma = false;

    u8 rids[] = {GET_RS(instr), GET_RT(instr), GET_RD(instr)};

    va_list args;
    va_start(args, sym);
    for (enum dasm_op op = va_arg(args, enum dasm_op); op != STOP;
         op = va_arg(args, enum dasm_op)) {

        if (comma)
            printf(", ");

        switch (op) {
        case d_RS:
        case d_RT:
        case d_RD:
            printf("$%s(%08x)", gpr_name[rids[op]], psx->regs[rids[op]]);
            break;
        case d_SHAMT:
            printf("0x%02x", GET_SHAMT(instr));
            break;
        case d_UIMM16:
            printf("0x%04x", GET_UIMM16(instr));
            break;
        case d_IMM16:
            print_imm16(GET_IMM16(instr));
            break;
        case d_WORD:
            print_imm16(GET_IMM16(instr));
            printf("(%s)", gpr_name[GET_RS(instr)]);
            print_word(psx, instr);
            break;
        case d_HALFWORD:
            print_imm16(GET_IMM16(instr));
            printf("(%s)", gpr_name[GET_RS(instr)]);
            print_halfword(psx, instr);
            break;
        case d_BYTE:
            print_imm16(GET_IMM16(instr));
            printf("(%s)", gpr_name[GET_RS(instr)]);
            print_byte(psx, instr);
            break;
        case d_TARGET:
            printf("0x%08x", GET_TARGET(psx->pc, instr));
            break;
        case d_COP0_RD:
            printf("$%s(%08x)", cop0_name[GET_RD(instr)],
                   psx->cop0[GET_RD(instr)]);
            break;
        case d_BRANCH:
            printf("0x%08x", GET_BRANCH(psx->addr_to_instr, instr));
            break;
        default:
            assert(false);
        }

        comma = true;
    }

    va_end(args);
    printf("\n");
}

void dasm_lui(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "lui", d_RT, d_UIMM16, STOP);
}

void dasm_ori(struct psx *psx, u32 instr)
{
    if (GET_RS(instr) == 0) {
        dasm_print(psx, instr, "li", d_RT, d_UIMM16, STOP);
    } else if (GET_RS(instr) == GET_RT(instr)) {
        dasm_print(psx, instr, "ori", d_RT, d_UIMM16, STOP);
    } else {
        dasm_print(psx, instr, "ori", d_RT, d_RS, d_UIMM16, STOP);
    }
}

void dasm_andi(struct psx *psx, u32 instr)
{
    if (GET_RS(instr) == GET_RT(instr)) {
        dasm_print(psx, instr, "andi", d_RT, d_UIMM16, STOP);
    } else {
        dasm_print(psx, instr, "andi", d_RT, d_RS, d_UIMM16, STOP);
    }
}

void dasm_sw(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "sw", d_RT, d_WORD, STOP);
}

void dasm_sh(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "sh", d_RT, d_HALFWORD, STOP);
}

void dasm_sb(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "sb", d_RT, d_BYTE, STOP);
}

void dasm_lw(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "lw", d_RT, d_WORD, STOP);
}

void dasm_lb(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "lb", d_RT, d_BYTE, STOP);
}

void dasm_sll(struct psx *psx, u32 instr)
{
    if (instr == 0) {
        dasm_print(psx, instr, "nop", STOP);
    } else if (GET_RD(instr) == GET_RT(instr)) {
        dasm_print(psx, instr, "sll", d_RD, d_SHAMT, STOP);
    } else {
        dasm_print(psx, instr, "sll", d_RD, d_RT, d_SHAMT, STOP);
    }
}

void dasm_addiu(struct psx *psx, u32 instr)
{
    if (GET_RS(instr) == 0) {
        dasm_print(psx, instr, "li", d_RT, d_IMM16, STOP);
    } else if (GET_RT(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "addiu", d_RT, d_IMM16, STOP);
    } else {
        dasm_print(psx, instr, "addiu", d_RT, d_RS, d_IMM16, STOP);
    }
}

void dasm_addi(struct psx *psx, u32 instr)
{
    if (GET_RT(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "addi", d_RT, d_IMM16, STOP);
    } else {
        dasm_print(psx, instr, "addi", d_RT, d_RS, d_IMM16, STOP);
    }
}

void dasm_j(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "j", d_TARGET, STOP);
}

void dasm_jr(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "jr", d_RS, STOP);
}

void dasm_jal(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "jal", d_TARGET, STOP);
}

void dasm_or(struct psx *psx, u32 instr)
{
    if ((GET_RS(instr) == GET_RT(instr)) || (GET_RT(instr) == 0)) {
        dasm_print(psx, instr, "move", d_RD, d_RS, STOP);
    } else if (GET_RS(instr) == 0) {
        dasm_print(psx, instr, "move", d_RD, d_RT, STOP);
    } else if (GET_RD(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "or", d_RD, d_RT, STOP);
    } else {
        dasm_print(psx, instr, "or", d_RD, d_RS, d_RT, STOP);
    }
}

void dasm_and(struct psx *psx, u32 instr)
{
    if (GET_RD(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "and", d_RD, d_RT, STOP);
    } else {
        dasm_print(psx, instr, "and", d_RD, d_RS, d_RT, STOP);
    }
}

void dasm_mtc0(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "mtc0", d_COP0_RD, d_RT, STOP);
}

void dasm_mfc0(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "mfc0", d_RT, d_COP0_RD, STOP);
}

void dasm_bne(struct psx *psx, u32 instr)
{
    if (GET_RS(instr) == 0) {
        dasm_print(psx, instr, "bnez", d_RT, d_BRANCH, STOP);
    } else if (GET_RT(instr) == 0) {
        dasm_print(psx, instr, "bnez", d_RS, d_BRANCH, STOP);
    } else {
        dasm_print(psx, instr, "bne", d_RS, d_RT, d_BRANCH, STOP);
    }
}

void dasm_beq(struct psx *psx, u32 instr)
{
    if (GET_RS(instr) == GET_RT(instr)) {
        dasm_print(psx, instr, "b", d_BRANCH, STOP);
    } else if (GET_RS(instr) == 0) {
        dasm_print(psx, instr, "beqz", d_RT, d_BRANCH, STOP);
    } else if (GET_RT(instr) == 0) {
        dasm_print(psx, instr, "beqz", d_RS, d_BRANCH, STOP);
    } else {
        dasm_print(psx, instr, "beq", d_RS, d_RT, d_BRANCH, STOP);
    }
}

void dasm_sltu(struct psx *psx, u32 instr)
{
    if (GET_RD(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "sltu", d_RD, d_RT, STOP);
    } else {
        dasm_print(psx, instr, "sltu", d_RD, d_RS, d_RT, STOP);
    }
}

void dasm_addu(struct psx *psx, u32 instr)
{
    if (GET_RT(instr) == 0) {
        dasm_print(psx, instr, "move", d_RD, d_RS, STOP);
    } else if (GET_RT(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "addu", d_RD, d_RT, STOP);
    } else {
        dasm_print(psx, instr, "addu", d_RD, d_RS, d_RT, STOP);
    }
}

void dasm_add(struct psx *psx, u32 instr)
{
    if (GET_RT(instr) == 0) {
        dasm_print(psx, instr, "move", d_RD, d_RS, STOP);
    } else if (GET_RS(instr) == 0) {
        dasm_print(psx, instr, "move", d_RD, d_RT, STOP);
    } else if (GET_RD(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "add", d_RD, d_RT, STOP);
    } else {
        dasm_print(psx, instr, "add", d_RD, d_RS, d_RT, STOP);
    }
}

void sll(struct psx *psx, u32 instr)
{
    assert(GET_RD(instr) <= 31 && GET_RT(instr) <= 31);
    psx->regs[GET_RD(instr)] = psx->regs[GET_RT(instr)] << GET_SHAMT(instr);
}

void sltu(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] =
        (psx->regs[GET_RS(instr)] < psx->regs[GET_RT(instr)]);
}

void lui(struct psx *psx, u32 instr)
{
    u32 rt = GET_RT(instr);
    psx->regs[rt] = instr << 16;
}

void jr(struct psx *psx, u32 instr)
{
    assert(psx->regs[GET_RS(instr)] % 4 == 0 && "unaligned address");
    psx->next_pc = psx->regs[GET_RS(instr)];
}

void j(struct psx *psx, u32 instr)
{
    psx->next_pc = GET_TARGET(psx->pc, instr);
}

void jal(struct psx *psx, u32 instr)
{
    psx->next_pc = GET_TARGET(psx->pc, instr);
    psx->ra = psx->addr_to_instr + 8;
}

void beq(struct psx *psx, u32 instr)
{
    if (psx->regs[GET_RS(instr)] == psx->regs[(GET_RT(instr))]) {
        psx->next_pc = GET_BRANCH(psx->addr_to_instr, instr);
    }
}

void bne(struct psx *psx, u32 instr)
{
    if (psx->regs[GET_RS(instr)] != psx->regs[(GET_RT(instr))]) {
        psx->next_pc = GET_BRANCH(psx->addr_to_instr, instr);
    }
}

void addiu(struct psx *psx, u32 instr)
{
    psx->regs[GET_RT(instr)] = psx->regs[GET_RS(instr)] + GET_IMM16(instr);
}

void addi(struct psx *psx, u32 instr)
{
    i32 val = psx->regs[GET_RS(instr)];
    i16 imm = GET_IMM16(instr);
    i64 sum = (i64)val + imm;

    if (INT_MAX < sum || sum < INT_MIN) {
        assert(false);
    }

    psx->regs[GET_RT(instr)] = sum;
}

void add(struct psx *psx, u32 instr)
{
    i32 a = psx->regs[GET_RS(instr)];
    i32 b = psx->regs[GET_RT(instr)];
    i64 sum = (i64)a + b;

    if (INT_MAX < sum || sum < INT_MIN) {
        assert(false);
    }

    psx->regs[GET_RD(instr)] = sum;
}

void addu(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] =
        psx->regs[GET_RS(instr)] + psx->regs[GET_RT(instr)];
}

void ori(struct psx *psx, u32 instr)
{
    psx->regs[GET_RT(instr)] = psx->regs[GET_RS(instr)] | GET_UIMM16(instr);
}

void andi(struct psx *psx, u32 instr)
{
    psx->regs[GET_RT(instr)] = psx->regs[GET_RS(instr)] & GET_UIMM16(instr);
}

void sb(struct psx *psx, u32 instr)
{
    u32 addr = psx->regs[GET_RS(instr)] + GET_IMM16(instr);
    u8 value = psx->regs[GET_RT(instr)] & 0xFF;

    /* 0x1f802041 is the Post Status byte in the BIOS.
     * An ID byte to say how far along the boot has progressed. */
    if (addr == 0x1f802041) {
        printf("BIOS Trace1: 0x%02x\n", value);
    }

    /* assuming machine is little endian */
    u8 *byte = &(psx->addr[addr]);
    byte[0] = value;
}

void sh(struct psx *psx, u32 instr)
{
    u32 addr = psx->regs[GET_RS(instr)] + GET_IMM16(instr);
    /* has to be 2 bytes aligned */
    assert(addr % sizeof(u16) == 0);

    u16 value = psx->regs[GET_RT(instr)] & 0xFFFF;

    /* assuming machine is little endian */
    u16 *half_word = (u16 *)(&(psx->addr[addr]));
    half_word[0] = value;
}

void sw(struct psx *psx, u32 instr)
{
    u32 addr = psx->regs[GET_RS(instr)] + GET_IMM16(instr);

    /* has to be 4 bytes aligned */
    assert(addr % sizeof(u32) == 0);

    /* assuming machine is little endian */
    u32 *word = (u32 *)(&(psx->addr[addr]));
    word[0] = psx->regs[GET_RT(instr)];
}

void lhu(struct psx *psx, u32 instr)
{
    assert(false);
    /* has to be 2 bytes aligned */
    u32 word = (i32)GET_IMM16(instr) + psx->regs[GET_RS(instr)];
    assert(word % 2 == 0);

    printf("\tloading from memory at: 0x%08x, value: 0x%08x\n", word,
           *(u16 *)&psx->addr[word]);

    /* assuming machine is little endian */
    psx->regs[GET_RT(instr)] = *(u16 *)&psx->addr[word];
}

void and(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] =
        psx->regs[GET_RS(instr)] & psx->regs[GET_RT(instr)];
}

void or(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] =
        psx->regs[GET_RS(instr)] | psx->regs[GET_RT(instr)];
}

void lb(struct psx *psx, u32 instr)
{
    u32 addr = ((i32)GET_IMM16(instr)) + psx->regs[GET_RS(instr)];

    if (PADDR(addr) == 0x1f000084 && addr != 0xfffe0130) {
        lds_push(psx->load_delay_slots, GET_RT(instr), (i32)(i8)(0xff));
        return;
    }

    u8 byte = psx->addr[addr];

    lds_push(psx->load_delay_slots, GET_RT(instr), (i32)(i8)byte);
}

void lw(struct psx *psx, u32 instr)
{
    // assert(false &&
    //        "the bug is probably because I don't ignore loads when ISC = 1");
    u32 addr = GET_IMM16(instr) + psx->regs[GET_RS(instr)];
    /* needs to be an exception if not 4 bytes aligned. */
    assert(addr % 4 == 0);

    /* assuming machine is little endian. change to memcpy. */
    u32 word = *(u32 *)&psx->addr[addr];

    lds_push(psx->load_delay_slots, GET_RT(instr), word);
}

void mfc0(struct psx *psx, u32 instr)
{
    lds_push(psx->load_delay_slots, GET_RT(instr), psx->cop0[GET_RD(instr)]);
}

void mtc0(struct psx *psx, u32 instr)
{
    assert(GET_RD(instr) < (sizeof(psx->cop0) / sizeof(psx->cop0[0])));
    psx->cop0[GET_RD(instr)] = psx->regs[GET_RT(instr)];

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
        &&h_special, &&h_bcondz, &&h_j,    &&h_jal,   &&h_beq,  &&h_bne,
        &&h_blez,    &&h_bgtz,   &&h_addi, &&h_addiu, &&h_slti, &&h_sltiu,
        &&h_andi,    &&h_ori,    &&h_xori, &&h_lui,   &&h_cop0, &&h_cop1,
        &&h_cop2,    &&h_cop3,   &&h_na,   &&h_na,    &&h_na,   &&h_na,
        &&h_na,      &&h_na,     &&h_na,   &&h_na,    &&h_na,   &&h_na,
        &&h_na,      &&h_na,     &&h_lb,   &&h_lh,    &&h_lwl,  &&h_lw,
        &&h_lbu,     &&h_lhu,    &&h_lwr,  &&h_na,    &&h_sb,   &&h_sh,
        &&h_swl,     &&h_sw,     &&h_na,   &&h_na,    &&h_swr,  &&h_na,
        &&h_lwc0,    &&h_lwc1,   &&h_lwc2, &&h_lwc3,  &&h_na,   &&h_na,
        &&h_na,      &&h_na,     &&h_swc0, &&h_swc1,  &&h_swc2, &&h_swc3,
        &&h_na,      &&h_na,     &&h_na,   &&h_na,
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

    static void *cop0_dispatch_tbl[] = {
        &&h_mfc0, &&h_na, &&h_cfc0, &&h_na, // 00
        &&h_mtc0, &&h_na, &&h_ctc0, &&h_na, // 04
        &&h_na,   &&h_na, &&h_na,   &&h_na, // 08
        &&h_na,   &&h_na, &&h_na,   &&h_na, // 0c
        &&h_rfe,  &&h_na, &&h_na,   &&h_na, // 10
        &&h_na,   &&h_na, &&h_na,   &&h_na, // 14
        &&h_na,   &&h_na, &&h_na,   &&h_na, // 18
        &&h_na,   &&h_na, &&h_na,   &&h_na, // 1c
    };

    u32 instr = 0;
    psx->pc = RESET_EXCEPTION_VECTOR;
    psx->next_pc = psx->pc + 4;

#define PRE_OP()                                                               \
    do {                                                                       \
        assert(psx->r0 == 0);                                                  \
        psx->addr_to_instr = psx->pc;                                          \
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
        assert(psx->r0 == 0);                                                  \
        instr = *((u32 *)&psx->memmap[psx->pc]); /* m1 is little endian */     \
        assert(GET_OPCODE(instr) <                                             \
               sizeof(dispatch_tbl) / sizeof(dispatch_tbl[0]));                \
        goto *dispatch_tbl[GET_OPCODE(instr)];                                 \
    } while (0)

    DISPATCH();

h_special:
    assert(GET_FUNCT(instr) <
           sizeof(sp_dispatch_tbl) / sizeof(sp_dispatch_tbl[0]));
    goto *sp_dispatch_tbl[GET_FUNCT(instr)];
h_bcondz:
    assert(false);
h_j:
    PRE_OP();
    dasm_j(psx, instr);
    j(psx, instr);
    POST_OP();
    DISPATCH();
h_jal:
    PRE_OP();
    dasm_jal(psx, instr);
    jal(psx, instr);
    POST_OP();
    DISPATCH();
h_beq:
    PRE_OP();
    dasm_beq(psx, instr);
    beq(psx, instr);
    POST_OP();
    DISPATCH();
h_bne:
    PRE_OP();
    dasm_bne(psx, instr);
    bne(psx, instr);
    POST_OP();
    DISPATCH();
h_blez:
    assert(false);
h_bgtz:
    assert(false);
h_addi:
    PRE_OP();
    dasm_addi(psx, instr);
    addi(psx, instr);
    POST_OP();
    DISPATCH();
h_addiu:
    PRE_OP();
    dasm_addiu(psx, instr);
    addiu(psx, instr);
    POST_OP();
    DISPATCH();
h_slti:
    assert(false);
h_sltiu:
    assert(false);
h_andi:
    PRE_OP();
    dasm_andi(psx, instr);
    andi(psx, instr);
    POST_OP();
    DISPATCH();
h_ori:
    PRE_OP();
    dasm_ori(psx, instr);
    ori(psx, instr);
    POST_OP();
    DISPATCH();
h_xori:
    assert(false);
h_lui:
    PRE_OP();
    dasm_lui(psx, instr);
    lui(psx, instr);
    POST_OP();
    DISPATCH();
h_cop0:
    assert(GET_RS(instr) <
           sizeof(cop0_dispatch_tbl) / sizeof(cop0_dispatch_tbl[0]));
    goto *cop0_dispatch_tbl[GET_RS(instr)];
h_mfc0:
    PRE_OP();
    dasm_mfc0(psx, instr);
    mfc0(psx, instr);
    POST_OP();
    DISPATCH();
h_cfc0:
    assert(false);
h_mtc0:
    PRE_OP();
    dasm_mtc0(psx, instr);
    mtc0(psx, instr);
    POST_OP();
    DISPATCH();
h_ctc0:
    assert(false);
h_rfe:
    assert(false);
h_cop1:
    assert(false);
h_cop2:
    assert(false);
h_cop3:
    assert(false);
h_lb:
    PRE_OP();
    dasm_lb(psx, instr);
    lb(psx, instr);
    POST_OP();
    DISPATCH();
h_lh:
    assert(false);
h_lwl:
    assert(false);
h_lw:
    PRE_OP();
    dasm_lw(psx, instr);
    lw(psx, instr);
    POST_OP();
    DISPATCH();
h_lbu:
    assert(false);
h_lhu:
    assert(false);
    PRE_OP();
    lhu(psx, instr);
    POST_OP();
    DISPATCH();
h_lwr:
    assert(false);
h_sb:
    PRE_OP();
    dasm_sb(psx, instr);
    sb(psx, instr);
    POST_OP();
    DISPATCH();
h_sh:
    PRE_OP();
    dasm_sh(psx, instr);
    sh(psx, instr);
    POST_OP();
    DISPATCH();
h_swl:
    assert(false);
h_sw:
    PRE_OP();
    dasm_sw(psx, instr);
    sw(psx, instr);
    POST_OP();
    DISPATCH();
h_swr:
    assert(false);
h_lwc0:
    assert(false);
h_lwc1:
    assert(false);
h_lwc2:
    assert(false);
h_lwc3:
    assert(false);
h_swc0:
    assert(false);
h_swc1:
    assert(false);
h_swc2:
    assert(false);
h_swc3:
    assert(false);
/* special */
h_sll:
    PRE_OP();
    dasm_sll(psx, instr);
    sll(psx, instr);
    POST_OP();
    DISPATCH();
h_srl:
    assert(false);
h_sra:
    assert(false);
h_sllv:
    assert(false);
h_srlv:
    assert(false);
h_srav:
    assert(false);
h_jr:
    PRE_OP();
    dasm_jr(psx, instr);
    jr(psx, instr);
    POST_OP();
    DISPATCH();
h_jalr:
    assert(false);
h_syscall:
    assert(false);
h_break:
    assert(false);
h_mfhi:
    assert(false);
h_mthi:
    assert(false);
h_mflo:
    assert(false);
h_mtlo:
    assert(false);
h_mult:
    assert(false);
h_multu:
    assert(false);
h_div:
    assert(false);
h_divu:
    assert(false);
h_add:
    PRE_OP();
    dasm_add(psx, instr);
    add(psx, instr);
    POST_OP();
    DISPATCH();
h_addu:
    PRE_OP();
    dasm_addu(psx, instr);
    addu(psx, instr);
    POST_OP();
    DISPATCH();
h_sub:
    assert(false);
h_subu:
    assert(false);
h_and:
    PRE_OP();
    dasm_and(psx, instr);
    and(psx, instr);
    POST_OP();
    DISPATCH();
h_or:
    PRE_OP();
    dasm_or(psx, instr);
    or(psx, instr);
    POST_OP();
    DISPATCH();
h_xor:
    assert(false);
h_nor:
    assert(false);
h_slt:
    assert(false);
h_sltu:
    PRE_OP();
    dasm_sltu(psx, instr);
    sltu(psx, instr);
    POST_OP();
    DISPATCH();
h_na:
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
    psx.sr = 0x10900000; /* no clue why I need to do this. */

    start(&psx);
    vm_deallocate(mach_task_self(), (vm_address_t)psx.addr, ADDR_SPACE_SIZE);
    vm_deallocate(mach_task_self(), (vm_address_t)psx.isolated_cache,
                  ADDR_SPACE_SIZE);
}
