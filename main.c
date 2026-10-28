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

bool bullshit = false;

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

typedef int64_t i64;
typedef int32_t i32;
typedef int16_t i16;
typedef int8_t i8;

#define Assert(e, sym)                                                         \
    do {                                                                       \
        if (__builtin_expect(!(e), 0)) {                                       \
            dasm_##sym(psx, instr);                                            \
            assert(e);                                                         \
        }                                                                      \
    } while (0)

/* DEBUG RELATED MACROS */
#define DEBUG 1 // not 0 debug, 0 no debug.

/* always 7 spaces between opcode and register fields in debug format. */
#define OPCODE_ALIGNMENT 7

#define type_last_bit(t) ((t)1 << (8 * sizeof(t) - 1))
#define utype_max(t) ((t)(type_last_bit(t) + (type_last_bit(t) - 1)))
#define BUILD_BUG_ON_ZERO(e) (sizeof(struct { int : -!!(e); }))
#define GENMASK_TYPE(t, h, l)                                                  \
    ((t)(BUILD_BUG_ON_ZERO((l) > (h)) + (utype_max(t) << (l)) &                \
         (utype_max(t) >> (8 * sizeof(t) - 1 - (h)))))

#define GENMASK_U8(h, l) GENMASK_TYPE(u8, h, l)
#define GENMASK_U16(h, l) GENMASK_TYPE(u16, h, l)
#define GENMASK_U32(h, l) GENMASK_TYPE(u32, h, l)
#define GENMASK(h, l) GENMASK_U32(h, l)
/* Expect reg and mask to be unsigned. */
#define FIELD_GET(_mask, _reg)                                                 \
    ((typeof(_mask))(((_reg) & (_mask)) >> __builtin_ctzll(_mask)))

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
#define RESET_VECTOR (KSEG1_START + BASE_BIOS_ADDR)
#define UTLB_MISS_VECTOR 0x80000000
#define COP0_BREAK_VECTOR 0x80000040
#define RAM_GENERAL_EXCEPTION_VECTOR 0x80000080
#define ROM_GENERAL_EXCEPTION_VECTOR 0xbfc00180
#define BASE_IO_PORTS_ADDR 0x1F801000

#define GP1_RESET (0x14802000)
#define GP0 (0x1f801810)
#define GP_CMD (0xff000000)
#define DRAW_MODE_SETTINGS (0xe1 << 24)
#define GP1 (0x1f801814)

/* DMA addresses */
#define DMA0(x) 0x1f80108##x /* MDECin (RAM to MDEC) */
#define DMA1(x) 0x1f80109##x /* MDECout (MDEC to RAM) */
#define DMA2(x) 0x1f8010a##x /* GPU (lists + image data) */
#define DMA3(x) 0x1f8010b##x /* CDROM   (CDROM to RAM) */
#define DMA4(x) 0x1f8010c##x /* SPU */
#define DMA5(x) 0x1f8010d##x /* PIO (Expansion Port) */
#define DMA6(x) 0x1f8010e##x /* OTC (reverse clear OT) (GPU related) */

#define D0_MADR DMA0(0)
#define D1_MADR DMA1(0)
#define D2_MADR DMA2(0)
#define D3_MADR DMA3(0)
#define D4_MADR DMA4(0)
#define D5_MADR DMA5(0)
#define D6_MADR DMA6(0)

#define D0_BCR DMA0(4)
#define D1_BCR DMA1(4)
#define D2_BCR DMA2(4)
#define D3_BCR DMA3(4)
#define D4_BCR DMA4(4)
#define D5_BCR DMA5(4)
#define D6_BCR DMA6(4)

#define D0_CHCR DMA0(8)
#define D1_CHCR DMA1(8)
#define D2_CHCR DMA2(8)
#define D3_CHCR DMA3(8)
#define D4_CHCR DMA4(8)
#define D5_CHCR DMA5(8)
#define D6_CHCR DMA6(8)

#define DPCR 0x1f8010f0
#define DICR 0x1f8010f4

/* DMA Channel Control flags
 * for documentation: https://psx-spx.consoledev.net/ps1/system/dmachannels/ */
enum D_CHCR_FLAGS : u32 {
    TRANSFER_DIRECTION = 1 << 0,
    MADR_INC = 1 << 1,
    MODE_CAPABILITY = 1 << 8,
    SYNC_MODE = GENMASK(10, 9),
    CHOPPING_DMA_WINDOW_SIZE = GENMASK(18, 16),
    CHOPPING_CPU_WINDOW_SIZE = GENMASK(22, 20),
    START_TRANSFER = 1 << 24,
    FORCE_TRANSFER_START = 1 << 28,
    PAUSE_OR_HOLD_FORCED = 1 << 29,
    BUS_SNOOPING = 1 << 30,
};

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

/* assuming machine is little endian */
#define GET_WORD_PTR(u8ptr) ((u32 *)(&(u8ptr)))
#define GET_HALFWORD_PTR(u8ptr) ((u16 *)(&(u8ptr)))

/* COP0 MACROS */
#define SR_GET_ISC(sr) (((sr) >> 16) & 0x1)

#define IS_DCHR(addr)                                                          \
    ({                                                                         \
        bool _val = false;                                                     \
        for (int _n = 0; _n <= 6; ++_n) {                                      \
            _val = _val || (addr == (0x1f801088 + _n * 0x10));                 \
        }                                                                      \
        _val;                                                                  \
    })

#define IS_DMADR(addr)                                                         \
    ({                                                                         \
        bool _val = false;                                                     \
        for (int _n = 0; _n <= 6; ++_n) {                                      \
            _val = _val || (addr == (0x1f801080 + _n * 0x10));                 \
        }                                                                      \
        _val;                                                                  \
    })

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

size_t
get_page_size()
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

/* Descrbies what kind of exception occured, set in cop0.cause. */
enum exc_code : u8 {
    e_INT,  /* External Interrupt */
    e_MOD,  /* TLB modification (none such in PSX) */
    e_TLBL, /* TLB load (none such in PSX) */
    e_TLBS, /* TLB store (none such in PSX) */
    e_AdEL, /* Address error, Data load or Instruction fetch */
    e_AdES, /* Address error, Data store. The address errors occur when
               attempting to read outside of KUseg in user mode and when the
               address is misaligned. (See also: Bad Address register) */
    e_IBE,  /* Bus error on Instruction fetch */
    e_DBE,  /* Bus error on Data load/store */
    e_Sys,  /* Generated unconditionally by syscall instruction */
    e_Bp,   /* Breakpoint - break instruction */
    e_RI,   /* Reserved instruction */
    e_CpU,  /* Coprocessor unusable */
    e_Ovf,  /* Arithmetic overflow */
};

/* GPU version 0 support only so far. */
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

#define TBX GENMASK(3, 0)
#define TBY (1 << 4)
#define ABR GENMASK(6, 5)
#define TPF GENMASK(8, 7)
#define DTD (1 << 9)
#define DFE (1 << 10)
#define PBW (1 << 11)
#define PBC (1 << 12)
#define ODE2 (1 << 13)
#define REV (1 << 14)
#define TBY2 (1 << 15)
#define HDS2 (1 << 16)
#define HDS GENMASK(18, 17)
#define VDS (1 << 19)
#define NPB (1 << 20)
#define LBS (1 << 21)
#define IRS (1 << 22)
#define DMSK (1 << 23)
#define IRQ (1 << 24)
#define DREQ (1 << 25)
#define IDLE (1 << 26)
#define RFFL (1 << 27)
#define WFEP (1 << 28)
#define DMD GENMASK(30, 29)
#define ODE (1 << 31)
    u32 gpustat;

    u32 dma_chcr[7];

    /* dma_madr[i], The memory address the DMA will start reading from/writing
     * to, for channel i.*/
    u32 dma_madr[7];

    u32 dma_bcr[7];

#define DMA2_MASTER (1 << 11)
#define DMA6_MASTER (1 << 27)
    u32 dpcr;

    u32 dicr;

    struct {
        u8 x, y;
    } texture_window_mask;

    struct {
        u8 x, y;
    } texture_window_offset;

    /* The Render commands GP0(20h..7Fh) are automatically clipping any pixels
     * that are outside of the region described by top_left and bottom_right. */
    struct {
        u32 x, y;
    } draw_area_top_left;

    struct {
        u32 x, y;
    } draw_area_bottom_right;

    struct {
        i32 x, y;
    } draw_offset;

    /* The register that the last executed instruction modified,
     * if no registers were modified then 0. This includes delayed modification
     * (i.e load instructions) */
    u8 last_modified_reg;

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

    /* interrupt status/mask register */
#define I_STAT (0x1f801070)
#define I_MASK (0x1f801074)
    u32 i_stat;
    u32 i_mask;

    union {
        u32 cop0[16];
        struct {
            u32 _na[3];
            u32 bpc; /* Breakpoint Program Counter (R/W) */
            u32 _na0;
            u32 bda;    /* Breakpoint Data Address (R/W) */
            u32 target; /* In branch delay slot, this hold the target address of
                           the branch. (R) */
            u32 dcic;   /* Debug and Cache Invalidate Control (R/W) */
            u32 bada;   /* Bad Address (R) */
            u32 bdam;   /* Breakpoint Data Address Mask	(R/W) */
            u32 _na1;
            u32 bpcm; /* Breakpoint Program Counter Mask	(R/W) */
#define IEc (1 << 0)
#define HIm (1 << 10) /* Hardware Interrupt mask) */
            u32 sr;   /* System status register (R/W) */
#define IP (1 << 10)
#define Sw GENMASK(9, 8)
            u32 cause; /* Cause of the last exception (R, except bits 8-9 are
                          R/W) */
            u32 epc;   /* Exception Program Counter (R) */
            u32 prid;  /* Processor Revision Identifier (R) */
        };
    };

    /* if the current instruction follows a branch instruction this is true
     * otherwise false. */
    bool is_branch_delay;

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
struct load_delay_slot
lds_pop(struct load_delay_slot lds[2])
{
    struct load_delay_slot top = lds[0];

    lds[0].reg_index = lds[1].reg_index;
    lds[0].value = lds[1].value;

    lds[1].reg_index = 0;
    lds[1].value = 0;

    return top;
}

/* This is used to handle the scenario where the instruction in the load delay
 * slot is writing to the same register as the load instruction. */
void
lds_sanity_clear(struct load_delay_slot lds[2], u8 last_modified_reg)
{
    if (lds[0].reg_index == last_modified_reg) {
        lds[0].reg_index = 0;
        lds[0].value = 0;
    }
}

/* just push into back load delay slot. */
void
lds_push(struct load_delay_slot lds[2], u32 reg_index, u32 value)
{
    if (reg_index == 0) {
        lds[1].reg_index = 0;
        lds[1].value = 0;
    } else {
        lds[1].reg_index = reg_index;
        lds[1].value = value;
    }
}

static inline void
print_imm16(i16 imm)
{
    if (imm < 0) {
        printf("-0x%04x", -imm);
    } else {
        printf("0x%04x", imm);
    }
}

static inline void
print_word(struct psx *psx, u32 instr)
{
    u32 addr = psx->regs[GET_RS(instr)] + GET_IMM16(instr);
    u32 word = *GET_WORD_PTR(psx->memmap[addr]);
    printf("([%08x] = %08x)", addr, word);
}

static inline void
print_halfword(struct psx *psx, u32 instr)
{
    u32 addr = psx->regs[GET_RS(instr)] + GET_IMM16(instr);
    u16 halfword = *GET_HALFWORD_PTR(psx->memmap[addr]);
    printf("([%08x] = %04x)", addr, halfword);
}

static inline void
print_byte(struct psx *psx, u32 instr)
{
    u32 addr = psx->regs[GET_RS(instr)] + GET_IMM16(instr);
    u8 byte = psx->memmap[addr];
    printf("([%08x] = %02x)", addr, byte);
}

enum dasm_op {
    d_RS,
    d_RT,
    d_RD,
    d_SHAMT,
    d_UIMM16,
    d_IMM16,
    d_IMM20,
    d_WORD,
    d_HALFWORD,
    d_BYTE,
    d_TARGET,
    d_COP0_RD,
    d_BRANCH,
    d_LO,
    d_HI,
    STOP,
};

void
dasm_print(struct psx *psx, u32 instr, const char *sym, ...)
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
        case d_IMM20:
            printf("0x%08x", (instr >> 6) & 0xfffff);
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
        case d_LO:
            printf("$lo(%08x)", psx->lo);
            break;
        case d_HI:
            printf("$hi(%08x)", psx->hi);
            break;
        default:
            assert(false);
        }

        comma = true;
    }

    va_end(args);
    printf("\n");
}

void
dasm_lui(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "lui", d_RT, d_UIMM16, STOP);
}

void
dasm_ori(struct psx *psx, u32 instr)
{
    if (GET_RS(instr) == 0) {
        dasm_print(psx, instr, "li", d_RT, d_UIMM16, STOP);
    } else if (GET_RS(instr) == GET_RT(instr)) {
        dasm_print(psx, instr, "ori", d_RT, d_UIMM16, STOP);
    } else {
        dasm_print(psx, instr, "ori", d_RT, d_RS, d_UIMM16, STOP);
    }
}

void
dasm_andi(struct psx *psx, u32 instr)
{
    if (GET_RS(instr) == GET_RT(instr)) {
        dasm_print(psx, instr, "andi", d_RT, d_UIMM16, STOP);
    } else {
        dasm_print(psx, instr, "andi", d_RT, d_RS, d_UIMM16, STOP);
    }
}

void
dasm_sw(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "sw", d_RT, d_WORD, STOP);
}

void
dasm_sh(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "sh", d_RT, d_HALFWORD, STOP);
}

void
dasm_sb(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "sb", d_RT, d_BYTE, STOP);
}

void
dasm_lw(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "lw", d_RT, d_WORD, STOP);
}

void
dasm_lb(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "lb", d_RT, d_BYTE, STOP);
}

void
dasm_lh(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "lh", d_RT, d_HALFWORD, STOP);
}

void
dasm_lbu(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "lbu", d_RT, d_BYTE, STOP);
}

void
dasm_lhu(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "lhu", d_RT, d_HALFWORD, STOP);
}

void
dasm_sll(struct psx *psx, u32 instr)
{
    if (instr == 0) {
        dasm_print(psx, instr, "nop", STOP);
    } else if (GET_RD(instr) == GET_RT(instr)) {
        dasm_print(psx, instr, "sll", d_RD, d_SHAMT, STOP);
    } else {
        dasm_print(psx, instr, "sll", d_RD, d_RT, d_SHAMT, STOP);
    }
}

void
dasm_sllv(struct psx *psx, u32 instr)
{
    if (GET_RD(instr) == GET_RT(instr)) {
        dasm_print(psx, instr, "sllv", d_RD, d_RS, STOP);
    } else {
        dasm_print(psx, instr, "sllv", d_RD, d_RT, d_RS, STOP);
    }
}

void
dasm_srlv(struct psx *psx, u32 instr)
{
    if (GET_RD(instr) == GET_RT(instr)) {
        dasm_print(psx, instr, "srlv", d_RD, d_RS, STOP);
    } else {
        dasm_print(psx, instr, "srlv", d_RD, d_RT, d_RS, STOP);
    }
}

void
dasm_srav(struct psx *psx, u32 instr)
{
    if (GET_RD(instr) == GET_RT(instr)) {
        dasm_print(psx, instr, "srav", d_RD, d_RS, STOP);
    } else {
        dasm_print(psx, instr, "srav", d_RD, d_RT, d_RS, STOP);
    }
}

void
dasm_srl(struct psx *psx, u32 instr)
{
    if (GET_RD(instr) == GET_RT(instr)) {
        dasm_print(psx, instr, "srl", d_RD, d_SHAMT, STOP);
    } else {
        dasm_print(psx, instr, "srl", d_RD, d_RT, d_SHAMT, STOP);
    }
}

void
dasm_sra(struct psx *psx, u32 instr)
{
    if (GET_RD(instr) == GET_RT(instr)) {
        dasm_print(psx, instr, "sra", d_RD, d_SHAMT, STOP);
    } else {
        dasm_print(psx, instr, "sra", d_RD, d_RT, d_SHAMT, STOP);
    }
}

void
dasm_addiu(struct psx *psx, u32 instr)
{
    if (GET_RS(instr) == 0) {
        dasm_print(psx, instr, "li", d_RT, d_IMM16, STOP);
    } else if (GET_RT(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "addiu", d_RT, d_IMM16, STOP);
    } else {
        dasm_print(psx, instr, "addiu", d_RT, d_RS, d_IMM16, STOP);
    }
}

void
dasm_addi(struct psx *psx, u32 instr)
{
    if (GET_RT(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "addi", d_RT, d_IMM16, STOP);
    } else {
        dasm_print(psx, instr, "addi", d_RT, d_RS, d_IMM16, STOP);
    }
}

void
dasm_j(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "j", d_TARGET, STOP);
}

void
dasm_jr(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "jr", d_RS, STOP);
}

void
dasm_jal(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "jal", d_TARGET, STOP);
}

void
dasm_jalr(struct psx *psx, u32 instr)
{
    if (GET_RD(instr) == 31) {
        dasm_print(psx, instr, "jalr", d_RS, STOP);
    } else {
        dasm_print(psx, instr, "jalr", d_RS, d_RD, STOP);
    }
}

void
dasm_nor(struct psx *psx, u32 instr)
{
    if (GET_RT(instr) == 0) {
        if (GET_RD(instr) != GET_RS(instr)) {
            dasm_print(psx, instr, "not", d_RD, d_RS, STOP);
        } else {
            dasm_print(psx, instr, "not", d_RD, STOP);
        }
    } else if (GET_RS(instr) == 0) {
        if (GET_RD(instr) != GET_RT(instr)) {
            dasm_print(psx, instr, "not", d_RD, d_RT, STOP);
        } else {
            dasm_print(psx, instr, "not", d_RD, STOP);
        }
    } else {
        if (GET_RD(instr) != GET_RS(instr)) {
            dasm_print(psx, instr, "nor", d_RD, d_RS, d_RT, STOP);
        } else {
            dasm_print(psx, instr, "nor", d_RD, d_RT, STOP);
        }
    }
}

void
dasm_or(struct psx *psx, u32 instr)
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

void
dasm_xor(struct psx *psx, u32 instr)
{
    if (GET_RS(instr) != GET_RD(instr)) {
        dasm_print(psx, instr, "xor", d_RD, d_RS, d_RT, STOP);
    } else {
        dasm_print(psx, instr, "xor", d_RD, d_RT, STOP);
    }
}

void
dasm_and(struct psx *psx, u32 instr)
{
    if (GET_RD(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "and", d_RD, d_RT, STOP);
    } else {
        dasm_print(psx, instr, "and", d_RD, d_RS, d_RT, STOP);
    }
}

void
dasm_mtc0(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "mtc0", d_COP0_RD, d_RT, STOP);
}

void
dasm_mfc0(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "mfc0", d_RT, d_COP0_RD, STOP);
}

void
dasm_bne(struct psx *psx, u32 instr)
{
    if (GET_RS(instr) == 0) {
        dasm_print(psx, instr, "bnez", d_RT, d_BRANCH, STOP);
    } else if (GET_RT(instr) == 0) {
        dasm_print(psx, instr, "bnez", d_RS, d_BRANCH, STOP);
    } else {
        dasm_print(psx, instr, "bne", d_RS, d_RT, d_BRANCH, STOP);
    }
}

void
dasm_beq(struct psx *psx, u32 instr)
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

void
dasm_bgtz(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "bgtz", d_RS, d_BRANCH, STOP);
}

void
dasm_blez(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "blez", d_RS, d_BRANCH, STOP);
}

void
dasm_bltz(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "bltz", d_RS, d_BRANCH, STOP);
}

void
dasm_bgez(struct psx *psx, u32 instr)
{
    if (GET_RS(instr) == 0) {
        dasm_print(psx, instr, "b", d_BRANCH, STOP);
    } else {
        dasm_print(psx, instr, "bgez", d_RS, d_BRANCH, STOP);
    }
}

void
dasm_sltu(struct psx *psx, u32 instr)
{
    if (GET_RD(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "sltu", d_RD, d_RT, STOP);
    } else {
        dasm_print(psx, instr, "sltu", d_RD, d_RS, d_RT, STOP);
    }
}

void
dasm_slt(struct psx *psx, u32 instr)
{
    if (GET_RD(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "slt", d_RD, d_RT, STOP);
    } else {
        dasm_print(psx, instr, "slt", d_RD, d_RS, d_RT, STOP);
    }
}

void
dasm_slti(struct psx *psx, u32 instr)
{
    if (GET_RT(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "slti", d_RT, d_IMM16, STOP);
    } else {
        dasm_print(psx, instr, "slti", d_RT, d_RS, d_IMM16, STOP);
    }
}

void
dasm_sltiu(struct psx *psx, u32 instr)
{
    if (GET_RT(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "sltiu", d_RT, d_UIMM16, STOP);
    } else {
        dasm_print(psx, instr, "sltiu", d_RT, d_RS, d_UIMM16, STOP);
    }
}

void
dasm_addu(struct psx *psx, u32 instr)
{
    if (GET_RT(instr) == 0) {
        dasm_print(psx, instr, "move", d_RD, d_RS, STOP);
    } else if (GET_RS(instr) == 0) {
        dasm_print(psx, instr, "move", d_RD, d_RT, STOP);
    } else if (GET_RD(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "addu", d_RD, d_RT, STOP);
    } else {
        dasm_print(psx, instr, "addu", d_RD, d_RS, d_RT, STOP);
    }
}

void
dasm_subu(struct psx *psx, u32 instr)
{
    if (GET_RD(instr) == GET_RS(instr)) {
        dasm_print(psx, instr, "subu", d_RD, d_RT, STOP);
    } else {
        dasm_print(psx, instr, "subu", d_RD, d_RS, d_RT, STOP);
    }
}

void
dasm_add(struct psx *psx, u32 instr)
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

void
dasm_multu(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "multu", d_RS, d_RT, STOP);
}

void
dasm_div(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "div", d_RS, d_RT, STOP);
}

void
dasm_divu(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "divu", d_RS, d_RT, STOP);
}

void
dasm_mflo(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "mflo", d_RD, d_LO, STOP);
}

void
dasm_mtlo(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "mtlo", d_LO, d_RS, STOP);
}

void
dasm_mfhi(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "mfhi", d_RD, d_HI, STOP);
}

void
dasm_mthi(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "mthi", d_HI, d_RS, STOP);
}

void
dasm_syscall(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "syscall", d_IMM20, STOP);
}

void
dasm_rfe(struct psx *psx, u32 instr)
{
    dasm_print(psx, instr, "rfe", STOP);
}

void
exception(struct psx *psx, enum exc_code code)
{
    /* disable interrupts and switch to kernel mode. */
    psx->sr = (psx->sr & ~0x3f) | ((psx->sr & 0xf) << 2);

    psx->cause &= ~GENMASK(31, 30);
    psx->cause |= psx->is_branch_delay ? 1 << 31 : 0;
    psx->cause |= psx->pc != (psx->addr_to_instr + 4) ? 1 << 30 : 0;

    psx->cause &= ~GENMASK(6, 2);
    psx->cause |= code << 2;

    if (psx->is_branch_delay) {
        psx->epc = psx->addr_to_instr - 4;
        psx->target = psx->pc;
    } else {
        psx->epc = psx->addr_to_instr;
    }

    if (psx->sr & (1 << 22)) {
        psx->pc = ROM_GENERAL_EXCEPTION_VECTOR;
    } else {
        psx->pc = RAM_GENERAL_EXCEPTION_VECTOR;
    }

    psx->next_pc = psx->pc + 4;
}

/* Convention: any instrunction (even load instructions)
 * that modifies a general purpose register (gpr) returns the register id that
 * it modifies, if it doesn't modify any gpr it returns 0. */
u8
sll(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] = psx->regs[GET_RT(instr)] << GET_SHAMT(instr);
    return GET_RD(instr);
}

u8
sllv(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] = psx->regs[GET_RT(instr)]
                               << (psx->regs[GET_RS(instr)] & 0x1F);
    return GET_RD(instr);
}

u8
srlv(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] =
        psx->regs[GET_RT(instr)] >> (psx->regs[GET_RS(instr)] & 0x1F);
    return GET_RD(instr);
}

u8
srav(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] =
        ((i32)psx->regs[GET_RT(instr)]) >> (psx->regs[GET_RS(instr)] & 0x1F);
    return GET_RD(instr);
}

u8
sra(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] =
        ((i32)psx->regs[GET_RT(instr)]) >> GET_SHAMT(instr);
    return GET_RD(instr);
}

u8
srl(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] = psx->regs[GET_RT(instr)] >> GET_SHAMT(instr);
    return GET_RD(instr);
}

u8
sltu(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] =
        (psx->regs[GET_RS(instr)] < psx->regs[GET_RT(instr)]);
    return GET_RD(instr);
}

u8
slt(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] =
        (((i32)psx->regs[GET_RS(instr)]) < ((i32)psx->regs[GET_RT(instr)]));
    return GET_RD(instr);
}

u8
slti(struct psx *psx, u32 instr)
{
    psx->regs[GET_RT(instr)] =
        (((i32)(psx->regs[GET_RS(instr)])) < ((i32)GET_IMM16(instr)));
    return GET_RT(instr);
}

u8
sltiu(struct psx *psx, u32 instr)
{
    psx->regs[GET_RT(instr)] =
        psx->regs[GET_RS(instr)] < ((u32)((i32)GET_IMM16(instr)));
    return GET_RT(instr);
}

u8
lui(struct psx *psx, u32 instr)
{
    psx->regs[GET_RT(instr)] = instr << 16;
    return GET_RT(instr);
}

u8
jr(struct psx *psx, u32 instr)
{
    assert(psx->regs[GET_RS(instr)] % 4 == 0 && "unaligned address");
    psx->next_pc = psx->regs[GET_RS(instr)];
    return 0;
}

u8
jalr(struct psx *psx, u32 instr)
{
    assert(psx->regs[GET_RS(instr)] % 4 == 0 && "unaligned address");
    psx->next_pc = psx->regs[GET_RS(instr)];
    psx->regs[GET_RD(instr)] = psx->addr_to_instr + 8;
    return GET_RD(instr);
}

u8
j(struct psx *psx, u32 instr)
{
    psx->next_pc = GET_TARGET(psx->pc, instr);
    return 0;
}

u8
jal(struct psx *psx, u32 instr)
{
    psx->next_pc = GET_TARGET(psx->pc, instr);
    psx->ra = psx->addr_to_instr + 8;
    return 31;
}

u8
beq(struct psx *psx, u32 instr)
{
    if (psx->regs[GET_RS(instr)] == psx->regs[(GET_RT(instr))]) {
        psx->next_pc = GET_BRANCH(psx->addr_to_instr, instr);
    }
    return 0;
}

u8
bne(struct psx *psx, u32 instr)
{
    if (psx->regs[GET_RS(instr)] != psx->regs[(GET_RT(instr))]) {
        psx->next_pc = GET_BRANCH(psx->addr_to_instr, instr);
    }
    return 0;
}

u8
bgtz(struct psx *psx, u32 instr)
{
    if (((i32)(psx->regs[GET_RS(instr)])) > 0) {
        psx->next_pc = GET_BRANCH(psx->addr_to_instr, instr);
    }
    return 0;
}

u8
blez(struct psx *psx, u32 instr)
{
    if (((i32)(psx->regs[GET_RS(instr)])) <= 0) {
        psx->next_pc = GET_BRANCH(psx->addr_to_instr, instr);
    }
    return 0;
}

u8
bltz(struct psx *psx, u32 instr)
{
    if (((i32)(psx->regs[GET_RS(instr)])) < 0) {
        psx->next_pc = GET_BRANCH(psx->addr_to_instr, instr);
    }
    return 0;
}

u8
bgez(struct psx *psx, u32 instr)
{
    if (((i32)(psx->regs[GET_RS(instr)])) >= 0) {
        psx->next_pc = GET_BRANCH(psx->addr_to_instr, instr);
    }
    return 0;
}

u8
mips_multu(struct psx *psx, u32 instr)
{
    u64 hilo = (u64)psx->regs[GET_RS(instr)] * psx->regs[GET_RT(instr)];

    psx->lo = (u32)hilo;
    psx->hi = (u32)(hilo >> 32);
    return 0;
}

u8
mips_div(struct psx *psx, u32 instr)
{
    if (psx->regs[GET_RT(instr)] == 0) {
        psx->lo = psx->regs[GET_RS(instr)] >= 0 ? -1 : 1;
        psx->hi = psx->regs[GET_RS(instr)];
    } else if (((i32)psx->regs[GET_RS(instr)]) == -0x80000000 &&
               ((i32)psx->regs[GET_RT(instr)]) == -1) {
        psx->lo = -0x80000000;
        psx->hi = 0;
    } else {
        psx->lo =
            ((i32)psx->regs[GET_RS(instr)]) / ((i32)psx->regs[GET_RT(instr)]);
        psx->hi =
            ((i32)psx->regs[GET_RS(instr)]) % ((i32)psx->regs[GET_RT(instr)]);
    }
    return 0;
}

u8
mips_divu(struct psx *psx, u32 instr)
{
    if (psx->regs[GET_RT(instr)] == 0) {
        psx->lo = 0xffffffff;
        psx->hi = psx->regs[GET_RS(instr)];
    } else {
        psx->lo = psx->regs[GET_RS(instr)] / psx->regs[GET_RT(instr)];
        psx->hi = psx->regs[GET_RS(instr)] % psx->regs[GET_RT(instr)];
    }
    return 0;
}

u8
mflo(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] = psx->lo;
    return GET_RD(instr);
}

u8
mtlo(struct psx *psx, u32 instr)
{
    psx->lo = psx->regs[GET_RS(instr)];
    return 0;
}

u8
mfhi(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] = psx->hi;
    return GET_RD(instr);
}

u8
mthi(struct psx *psx, u32 instr)
{
    psx->hi = psx->regs[GET_RS(instr)];
    return 0;
}

u8
addiu(struct psx *psx, u32 instr)
{
    psx->regs[GET_RT(instr)] = psx->regs[GET_RS(instr)] + GET_IMM16(instr);
    return GET_RT(instr);
}

u8
addi(struct psx *psx, u32 instr)
{
    i32 val = psx->regs[GET_RS(instr)];
    i16 imm = GET_IMM16(instr);
    i64 sum = (i64)val + imm;

    if (INT_MAX < sum || sum < INT_MIN) {
        assert(false);
    }

    psx->regs[GET_RT(instr)] = sum;
    return GET_RT(instr);
}

u8
add(struct psx *psx, u32 instr)
{
    i32 a = psx->regs[GET_RS(instr)];
    i32 b = psx->regs[GET_RT(instr)];
    i64 sum = (i64)a + b;

    if (INT_MAX < sum || sum < INT_MIN) {
        assert(false);
    }

    psx->regs[GET_RD(instr)] = sum;
    return GET_RD(instr);
}

u8
addu(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] =
        psx->regs[GET_RS(instr)] + psx->regs[GET_RT(instr)];
    return GET_RD(instr);
}

u8
subu(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] =
        psx->regs[GET_RS(instr)] - psx->regs[GET_RT(instr)];
    return GET_RD(instr);
}

u8
ori(struct psx *psx, u32 instr)
{
    psx->regs[GET_RT(instr)] = psx->regs[GET_RS(instr)] | GET_UIMM16(instr);
    return GET_RT(instr);
}

u8
andi(struct psx *psx, u32 instr)
{
    psx->regs[GET_RT(instr)] = psx->regs[GET_RS(instr)] & GET_UIMM16(instr);
    return GET_RT(instr);
}

u8
sb(struct psx *psx, u32 instr)
{
    u32 addr = psx->regs[GET_RS(instr)] + GET_IMM16(instr);
    u8 value = psx->regs[GET_RT(instr)] & 0xFF;

    Assert(addr != D0_MADR, sb);
    Assert(addr != D1_MADR, sb);
    Assert(addr != D2_MADR, sb);
    Assert(addr != D3_MADR, sb);
    Assert(addr != D4_MADR, sb);
    Assert(addr != D5_MADR, sb);
    Assert(addr != D6_MADR, sb);
    Assert(addr != D0_BCR, sb);
    Assert(addr != D1_BCR, sb);
    Assert(addr != D2_BCR, sb);
    Assert(addr != D3_BCR, sb);
    Assert(addr != D4_BCR, sb);
    Assert(addr != D5_BCR, sb);
    Assert(addr != D6_BCR, sb);
    Assert(addr != D0_CHCR, sb);
    Assert(addr != D1_CHCR, sb);
    Assert(addr != D2_CHCR, sb);
    Assert(addr != D3_CHCR, sb);
    Assert(addr != D4_CHCR, sb);
    Assert(addr != D5_CHCR, sb);
    Assert(addr != D6_CHCR, sb);
    Assert(addr != DPCR, sb);
    Assert(addr != DICR, sb);
    Assert(addr != I_STAT, sb);
    Assert(addr != I_MASK, sb);
    Assert(addr != GP0, sb);
    Assert(addr != GP1, sb);

    /* 0x1f802041 is the Post Status byte in the BIOS.
     * An ID byte to say how far along the boot has progressed. */
    if (addr == 0x1f802041) {
        printf("BIOS Trace1: 0x%02x\n", value);
    }

    u8 *byte = &(psx->addr[addr]);
    byte[0] = value;
    return 0;
}

u8
sh(struct psx *psx, u32 instr)
{
    u32 addr = psx->regs[GET_RS(instr)] + GET_IMM16(instr);
    u16 val = psx->regs[GET_RT(instr)] & 0xFFFF;

    /* has to be 2 bytes aligned */
    assert(addr % sizeof(u16) == 0);

    Assert(addr != D0_MADR, sh);
    Assert(addr != D1_MADR, sh);
    Assert(addr != D2_MADR, sh);
    Assert(addr != D3_MADR, sh);
    Assert(addr != D4_MADR, sh);
    Assert(addr != D5_MADR, sh);
    Assert(addr != D6_MADR, sh);
    Assert(addr != D0_BCR, sh);
    Assert(addr != D1_BCR, sh);
    Assert(addr != D2_BCR, sh);
    Assert(addr != D3_BCR, sh);
    Assert(addr != D4_BCR, sh);
    Assert(addr != D5_BCR, sh);
    Assert(addr != D6_BCR, sh);
    Assert(addr != D0_CHCR, sh);
    Assert(addr != D1_CHCR, sh);
    Assert(addr != D2_CHCR, sh);
    Assert(addr != D3_CHCR, sh);
    Assert(addr != D4_CHCR, sh);
    Assert(addr != D5_CHCR, sh);
    Assert(addr != D6_CHCR, sh);
    Assert(addr != DPCR, sh);
    Assert(addr != DICR, sh);
    Assert(addr != GP0, sh);
    Assert(addr != GP1, sh);

    if (addr == I_MASK) {
        psx->i_mask = val;
        Assert((psx->i_mask & GENMASK(10, 0)) == psx->i_mask, sh);
        return 0;
    } else if (addr == I_STAT) {
        psx->i_stat &= val;
        Assert((psx->i_stat & GENMASK(10, 0)) == psx->i_stat, sw);
        return 0;
    }

    u16 *half_word = GET_HALFWORD_PTR(psx->addr[addr]);
    half_word[0] = val;
    return 0;
}

void
interrupt_check(struct psx *psx)
{
    /* This variable is used to make interrupts "delayed". 
     * This is desired since currently when executing something like dma2 channel,
     * the interrupt happens immediately after sw instruction, before pc, next_addr, and addr_to_instr get updated,
     * so epc points to the same sw that caused the exception causing infinite loop.
     * To avoid this use this variable to essentially wait 8 instructions after interrupt conditions are first true.
     * */
    static u32 clock = 0;

    if (psx->i_stat & psx->i_mask) {
        psx->cause |= 1 << 10;
    } else {
        /* cop0r13.bit10 is NOT a latch, ie. it gets automatically cleared as
         * soon as "(I_STAT AND I_MASK)=zero", so there's no need to do an
         * acknowledge at the cop0 side. */
        psx->cause &= ~(1 << 10);
    }

    if (psx->sr & IEc &&
        ((psx->sr & psx->cause & IP) || (psx->cause & psx->sr & Sw))) {
        if (clock == 8) {
            exception(psx, e_INT);
            clock = 0; /* resest clock exception done */
        } else {
            ++clock;
        }
    } else {
        clock = 0; /* if interrupt are no longer happening clock should be reset */
    }
}

void
dma_update_master_interrupt_flag(struct psx *psx)
{
    if ((psx->dicr & (1 << 15)) ||
        ((psx->dicr & (1 << 23)) && (psx->dicr & GENMASK(30, 24)) > 0)) {

        if ((psx->dicr & (1 << 31)) == 0) {
            psx->dicr |= 1 << 31;
            psx->i_stat |= 1 << 3; /* DMA IRQ set */
        }

    } else {
        psx->dicr &= ~(1 << 31);
    }
}

u8
sw(struct psx *psx, u32 instr)
{
    u32 addr = psx->regs[GET_RS(instr)] + GET_IMM16(instr);
    u32 val = psx->regs[GET_RT(instr)];

    Assert(addr != GP1 || (val & GP_CMD) == 0 ||
               (val & GP_CMD) == (0x08 << 24) || (val & GP_CMD) == (0x04 << 24),
           sw);
    Assert(addr != GP0 || (val & GP_CMD) == DRAW_MODE_SETTINGS ||
               (val & GP_CMD) == 0,
           sw);
    Assert(addr % 4 == 0, sw);

    Assert(addr != D0_BCR, sw);
    Assert(addr != D1_BCR, sw);
    Assert(addr != D3_BCR, sw);
    Assert(addr != D4_BCR, sw);
    Assert(addr != D5_BCR, sw);

    Assert(addr != D0_MADR, sw);
    Assert(addr != D1_MADR, sw);
    Assert(addr != D3_MADR, sw);
    Assert(addr != D4_MADR, sw);
    Assert(addr != D5_MADR, sw);

    Assert(addr != D0_CHCR, sw);
    Assert(addr != D1_CHCR, sw);
    Assert(addr != D2_CHCR || (val & START_TRANSFER) == 0 || val == 0x01000401,
           sw);
    Assert(addr != D3_CHCR, sw);
    Assert(addr != D4_CHCR, sw);
    Assert(addr != D5_CHCR, sw);

    if (addr == DPCR) {
        psx->dpcr = val;
        /* TODO: figure out what to do when master is not enabled and channel is
         * started. */
        return 0;
    } else if (addr == I_STAT) {
        psx->i_stat &= val;
        Assert((psx->i_stat & GENMASK(10, 0)) == psx->i_stat, sw);
        return 0;
    } else if (addr == I_MASK) {
        psx->i_mask = val;
        Assert((psx->i_mask & GENMASK(10, 0)) == psx->i_mask, sw);
        return 0;
    } else if (addr == DICR) {
        psx->dicr |= val & GENMASK(23, 0);
        psx->dicr &= ~(val & GENMASK(30, 24));
        dma_update_master_interrupt_flag(psx);
        return 0;
    } else if (addr == GP0 && (val & GP_CMD) == DRAW_MODE_SETTINGS) {
        /* no need to emulate fifo cmd buffer just always say it's empty. */
        /* TODO: DUPLICATED CODE */
        psx->gpustat &= ~(TBY2 | 0x7ff);
        psx->gpustat |= (val & 0x7ff) | ((val & (1 << 11)) << 4);
        return 0;
    } else if (addr == GP0 && (val & GP_CMD) == 0) {
        /* NOP */
        return 0;
    } else if (addr == D2_CHCR && (val & START_TRANSFER) == 0) {
        psx->dma_chcr[2] = val;
        return 0;
    } else if (addr == D2_CHCR && val == 0x01000401) {
        /* RAM to GP0 write FIFO (commands for GP0)
         * MADR +4 increments
         * Linked list mode
         * Start Transfer
         * No force transfer start have to wait for DREQ
         * */
        bullshit = true;
        psx->dma_chcr[2] = val;
        Assert((psx->dpcr & DMA2_MASTER) != 0, sw);
        Assert((psx->gpustat & DREQ) != 0, sw);
        Assert((psx->dma_chcr[2] & FORCE_TRANSFER_START) == 0, sw);
        Assert((psx->dma_chcr[2] & START_TRANSFER) != 0, sw);
        Assert((psx->dicr & (1 << 2)) == 0, sw);
        Assert((psx->dicr & (1 << 23)) != 0, sw);
        Assert((psx->dicr & (1 << 18)) != 0, sw);

        /* Should someday check that this is a ram address */
        u32 node_addr = psx->dma_madr[2];
        while (node_addr != 0xffffff) {
            u32 *node = GET_WORD_PTR(psx->addr[node_addr]);

            const u32 header = *node;

            /* Number of extra commands (not including header) to transfer
             * node */
            const u8 cmd_cnt = (header & ~0xffffff) >> 24;

            /* exclude header word */
            ++node;

            for (int i = 0; i < cmd_cnt; ++i) {
                /* since these are gp0 command currently duplicated code since
                 * can modify through sw and dma. */
                /* GP0 commands */
                const u32 word = node[i];
                switch ((word & GP_CMD) >> 24) {
                default:
                    printf("GP0(%08x)\n", word);
                    Assert(false, sw);
                case 0xe1: /* Duplicate */
                    psx->gpustat &= ~(TBY2 | 0x7ff);
                    psx->gpustat |= (word & 0x7ff) | ((word & (1 << 11)) << 4);
                    break;
                case 0xe2:
                    psx->texture_window_mask.x = word & GENMASK(4, 0);
                    psx->texture_window_mask.y = word & GENMASK(9, 5);
                    psx->texture_window_offset.x = word & GENMASK(14, 10);
                    psx->texture_window_offset.y = word & GENMASK(19, 15);
                    break;
                case 0xe3: /* Set Drawing Area top left */
                    psx->draw_area_top_left.x = word & GENMASK(9, 0);
                    psx->draw_area_top_left.y =
                        FIELD_GET(GENMASK(18, 10), word);
                    break;
                case 0xe4: /* Set Drawing Area bottom right */
                    psx->draw_area_bottom_right.x = word & GENMASK(9, 0);
                    psx->draw_area_bottom_right.y =
                        FIELD_GET(GENMASK(18, 10), word);
                    break;
                case 0xe5:
                    /* x,y can be negative so have
                     * to worry about sign extension */
                    psx->draw_offset.x =
                        (i32)((word & GENMASK(10, 0)) << 21) >> 21;
                    psx->draw_offset.y =
                        (i32)((word & GENMASK(21, 11)) << 10) >> 21;
                    break;
                case 0xe6:
                    psx->gpustat &= ~(PBW | PBC);
                    psx->gpustat |= (word & GENMASK(1, 0)) << 11;
                    break;
                }
            }

            node_addr = header & 0xffffff;
        }

        /* START_TRANSFER cleared on transfer end */
        psx->dma_chcr[2] &= ~START_TRANSFER;

        /* at transfer end contains end marker, technically madr should get
         * updated everytime node_addr gets updated. */
        psx->dma_madr[2] = node_addr;

        /* Something wrong with interrupt. */
        if ((psx->dicr & (1 << 18)) && (psx->dicr & (1 << 23))) {
            psx->dicr |= (1 << 26);
            dma_update_master_interrupt_flag(psx);
        }

        return 0;
    } else if (addr == GP1 && (val & GP_CMD) == 0) {
        psx->gpustat = GP1_RESET;
        return 0;
    } else if (addr == GP1 && (val & GP_CMD) == (0x08 << 24)) {
        psx->gpustat = (psx->gpustat & ~(GENMASK(22, 16) | (1 << 14))) |
                       ((val & GENMASK(5, 0)) << 17) |
                       ((val & (1 << 6)) << 10) | ((val & (1 << 7)) << 7);
        return 0;
    } else if (addr == GP1 && (val & GP_CMD) == (0x04 << 24)) {
        psx->gpustat =
            (psx->gpustat & ~GENMASK(30, 29)) | ((val & GENMASK(1, 0)) << 29);

        if ((val & GENMASK(1, 0)) != 0) {
            Assert((val & GENMASK(1, 0)) == 2, sw);
            psx->gpustat |= DREQ | WFEP;
        } else {
            psx->gpustat &= ~DREQ;
        }

        return 0;
    } else if (addr == D2_MADR) {
        Assert((val & GENMASK(1, 0)) == 0, sw);
        psx->dma_madr[2] = val & GENMASK(23, 0);
        return 0;
    } else if (addr == D2_BCR) {
        psx->dma_bcr[2] = val;
        return 0;
    } else if (addr == D6_MADR) {
        Assert((val & GENMASK(1, 0)) == 0, sw);
        psx->dma_madr[6] = val & GENMASK(23, 0);
        return 0;
    } else if (addr == D6_BCR) {
        psx->dma_bcr[6] = val;
        return 0;
    } else if (addr == D6_CHCR) {
        /* D6_CHCR has only three read/write-able bits: 24,28,30. All other bits
         * are read-only: bit 1 is always 1 (increment=-4), and the other bits
         * are always 0. */
        psx->dma_chcr[6] =
            (val & FORCE_TRANSFER_START & START_TRANSFER & BUS_SNOOPING) |
            (1 << 1);

        if (psx->dma_chcr[6] & START_TRANSFER) {
            Assert((psx->dpcr & DMA6_MASTER) != 0, sw);
            Assert((psx->dma_chcr[6] & FORCE_TRANSFER_START) != 0, sw);
            /* FORCE_TRANSFER_START cleared on transfer start */
            psx->dma_chcr[6] &= ~FORCE_TRANSFER_START;

            /* Ordering table clear */
            const u32 madr = psx->dma_madr[6];
            const u32 n = psx->dma_bcr[6];
            for (int i = 0; i < n - 1; ++i) {
                u32 *word = GET_WORD_PTR(psx->addr[madr - (4 * i)]);
                *word = (madr - 4 * (i + 1)) & 0xffffff;
            }
            u32 *word = GET_WORD_PTR(psx->addr[madr - (4 * (n - 1))]);
            *word = 0xffffff;

            /* START_TRANSFER cleared on transfer end */
            psx->dma_chcr[6] &= ~START_TRANSFER;

            if ((psx->dicr & (1 << 22)) && (psx->dicr & (1 << 23))) {
                psx->dicr |= (1 << 30);
                dma_update_master_interrupt_flag(psx);
            }
        }
        return 0;
    }

    u32 *word = GET_WORD_PTR(psx->addr[addr]);
    word[0] = val;
    return 0;
}

u8
lbu(struct psx *psx, u32 instr)
{
    u32 addr = (i32)GET_IMM16(instr) + psx->regs[GET_RS(instr)];

    Assert(addr != D0_MADR, lbu);
    Assert(addr != D1_MADR, lbu);
    Assert(addr != D2_MADR, lbu);
    Assert(addr != D3_MADR, lbu);
    Assert(addr != D4_MADR, lbu);
    Assert(addr != D5_MADR, lbu);
    Assert(addr != D6_MADR, lbu);
    Assert(addr != D0_BCR, lbu);
    Assert(addr != D1_BCR, lbu);
    Assert(addr != D2_BCR, lbu);
    Assert(addr != D3_BCR, lbu);
    Assert(addr != D4_BCR, lbu);
    Assert(addr != D5_BCR, lbu);
    Assert(addr != D6_BCR, lbu);
    Assert(addr != D0_CHCR, lbu);
    Assert(addr != D1_CHCR, lbu);
    Assert(addr != D2_CHCR, lbu);
    Assert(addr != D3_CHCR, lbu);
    Assert(addr != D4_CHCR, lbu);
    Assert(addr != D5_CHCR, lbu);
    Assert(addr != D6_CHCR, lbu);
    Assert(addr != DPCR, lbu);
    Assert(addr != DICR, lbu);
    Assert(addr != I_STAT, lbu);
    Assert(addr != I_MASK, lbu);
    Assert(addr != GP0, lbu);
    Assert(addr != GP1, lbu);

    u32 byte = psx->addr[addr];
    lds_push(psx->load_delay_slots, GET_RT(instr), byte);
    return GET_RT(instr);
}

u8
lhu(struct psx *psx, u32 instr)
{
    u32 addr = (i32)GET_IMM16(instr) + psx->regs[GET_RS(instr)];

    Assert(addr != D0_MADR, lhu);
    Assert(addr != D1_MADR, lhu);
    Assert(addr != D2_MADR, lhu);
    Assert(addr != D3_MADR, lhu);
    Assert(addr != D4_MADR, lhu);
    Assert(addr != D5_MADR, lhu);
    Assert(addr != D6_MADR, lhu);
    Assert(addr != D0_BCR, lhu);
    Assert(addr != D1_BCR, lhu);
    Assert(addr != D2_BCR, lhu);
    Assert(addr != D3_BCR, lhu);
    Assert(addr != D4_BCR, lhu);
    Assert(addr != D5_BCR, lhu);
    Assert(addr != D6_BCR, lhu);
    Assert(addr != D0_CHCR, lhu);
    Assert(addr != D1_CHCR, lhu);
    Assert(addr != D2_CHCR, lhu);
    Assert(addr != D3_CHCR, lhu);
    Assert(addr != D4_CHCR, lhu);
    Assert(addr != D5_CHCR, lhu);
    Assert(addr != D6_CHCR, lhu);
    Assert(addr != DPCR, lhu);
    Assert(addr != DICR, lhu);
    Assert(addr != GP0, lhu);
    Assert(addr != GP1, lhu);
    assert(addr % 2 == 0);

    if (addr == I_MASK) {
        Assert((psx->i_mask & GENMASK(10, 0)) == psx->i_mask, lhu);
        lds_push(psx->load_delay_slots, GET_RT(instr), psx->i_mask);
        return GET_RT(instr);
    } else if (addr == I_STAT) {
        Assert((psx->i_stat & GENMASK(10, 0)) == psx->i_stat, lhu);
        lds_push(psx->load_delay_slots, GET_RT(instr), psx->i_stat);
        return GET_RT(instr);
    }

    u32 halfword = *GET_HALFWORD_PTR(psx->addr[addr]);
    lds_push(psx->load_delay_slots, GET_RT(instr), halfword);
    return GET_RT(instr);
}

u8
and(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] =
        psx->regs[GET_RS(instr)] & psx->regs[GET_RT(instr)];
    return GET_RD(instr);
}

u8
or(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] =
        psx->regs[GET_RS(instr)] | psx->regs[GET_RT(instr)];
    return GET_RD(instr);
}

u8
xor(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] =
        psx->regs[GET_RS(instr)] ^ psx->regs[GET_RT(instr)];
    return GET_RD(instr);
}

u8
nor(struct psx *psx, u32 instr)
{
    psx->regs[GET_RD(instr)] =
        0xffffffff ^ (psx->regs[GET_RS(instr)] | psx->regs[GET_RT(instr)]);
    return GET_RD(instr);
}

u8
lb(struct psx *psx, u32 instr)
{
    u32 addr = ((i32)GET_IMM16(instr)) + psx->regs[GET_RS(instr)];

    Assert(addr != D0_MADR, lb);
    Assert(addr != D1_MADR, lb);
    Assert(addr != D2_MADR, lb);
    Assert(addr != D3_MADR, lb);
    Assert(addr != D4_MADR, lb);
    Assert(addr != D5_MADR, lb);
    Assert(addr != D6_MADR, lb);
    Assert(addr != D0_BCR, lb);
    Assert(addr != D1_BCR, lb);
    Assert(addr != D2_BCR, lb);
    Assert(addr != D3_BCR, lb);
    Assert(addr != D4_BCR, lb);
    Assert(addr != D5_BCR, lb);
    Assert(addr != D6_BCR, lb);
    Assert(addr != D0_CHCR, lb);
    Assert(addr != D1_CHCR, lb);
    Assert(addr != D2_CHCR, lb);
    Assert(addr != D3_CHCR, lb);
    Assert(addr != D4_CHCR, lb);
    Assert(addr != D5_CHCR, lb);
    Assert(addr != D6_CHCR, lb);
    Assert(addr != DPCR, lb);
    Assert(addr != DICR, lb);
    Assert(addr != I_STAT, lb);
    Assert(addr != I_MASK, lb);
    Assert(addr != GP0, lb);
    Assert(addr != GP1, lb);

    /* i forgot why? */
    if (PADDR(addr) == 0x1f000084 && addr != 0xfffe0130) {
        lds_push(psx->load_delay_slots, GET_RT(instr), (i32)(i8)(0xff));
        return GET_RT(instr);
    }

    u8 byte = psx->addr[addr];

    lds_push(psx->load_delay_slots, GET_RT(instr), (i32)(i8)byte);
    return GET_RT(instr);
}

u8
lh(struct psx *psx, u32 instr)
{
    u32 addr = (i32)GET_IMM16(instr) + psx->regs[GET_RS(instr)];

    Assert(addr != D0_MADR, lh);
    Assert(addr != D1_MADR, lh);
    Assert(addr != D2_MADR, lh);
    Assert(addr != D3_MADR, lh);
    Assert(addr != D4_MADR, lh);
    Assert(addr != D5_MADR, lh);
    Assert(addr != D6_MADR, lh);
    Assert(addr != D0_BCR, lh);
    Assert(addr != D1_BCR, lh);
    Assert(addr != D2_BCR, lh);
    Assert(addr != D3_BCR, lh);
    Assert(addr != D4_BCR, lh);
    Assert(addr != D5_BCR, lh);
    Assert(addr != D6_BCR, lh);
    Assert(addr != D0_CHCR, lh);
    Assert(addr != D1_CHCR, lh);
    Assert(addr != D2_CHCR, lh);
    Assert(addr != D3_CHCR, lh);
    Assert(addr != D4_CHCR, lh);
    Assert(addr != D5_CHCR, lh);
    Assert(addr != D6_CHCR, lh);
    Assert(addr != DPCR, lh);
    Assert(addr != DICR, lh);
    Assert(addr != I_STAT, lh);
    Assert(addr != I_MASK, lh);
    Assert(addr != GP0, lh);
    Assert(addr != GP1, lh);
    assert(addr % 2 == 0);

    u32 halfword = (i32)(i16)(*GET_HALFWORD_PTR(psx->addr[addr]));

    lds_push(psx->load_delay_slots, GET_RT(instr), halfword);
    return GET_RT(instr);
}

u8
lw(struct psx *psx, u32 instr)
{
    u32 addr = GET_IMM16(instr) + psx->regs[GET_RS(instr)];

    Assert(addr != GP0 || GET_RT(instr) == 0, lw);
    Assert(addr % 4 == 0, lw);

    Assert(addr != D0_MADR, lw);
    Assert(addr != D1_MADR, lw);
    Assert(addr != D2_MADR, lw);
    Assert(addr != D3_MADR, lw);
    Assert(addr != D4_MADR, lw);
    Assert(addr != D5_MADR, lw);
    Assert(addr != D6_MADR, lw);
    Assert(addr != D0_BCR, lw);
    Assert(addr != D1_BCR, lw);
    Assert(addr != D2_BCR, lw);
    Assert(addr != D3_BCR, lw);
    Assert(addr != D4_BCR, lw);
    Assert(addr != D5_BCR, lw);
    Assert(addr != D6_BCR, lw);
    Assert(addr != D0_CHCR, lw);
    Assert(addr != D1_CHCR, lw);
    Assert(addr != D3_CHCR, lw);
    Assert(addr != D4_CHCR, lw);
    Assert(addr != D5_CHCR, lw);

    if (addr == DICR) {
        lds_push(psx->load_delay_slots, GET_RT(instr), psx->dicr);
        return GET_RT(instr);
    }

    if (addr == I_STAT) {
        lds_push(psx->load_delay_slots, GET_RT(instr), psx->i_stat);
        return GET_RT(instr);
    }

    if (addr == D6_CHCR) {
        lds_push(psx->load_delay_slots, GET_RT(instr), psx->dma_chcr[6]);
        return GET_RT(instr);
    }

    if (addr == DPCR) {
        lds_push(psx->load_delay_slots, GET_RT(instr), psx->dpcr);
        return GET_RT(instr);
    }

    if (addr == I_MASK) {
        lds_push(psx->load_delay_slots, GET_RT(instr), psx->i_mask);
        return GET_RT(instr);
    }

    if (addr == GP1) {
        lds_push(psx->load_delay_slots, GET_RT(instr), psx->gpustat);
        return GET_RT(instr);
    }

    if (addr == D2_CHCR) {
        lds_push(psx->load_delay_slots, GET_RT(instr), psx->dma_chcr[2]);
        return GET_RT(instr);
    }

    u32 word = *GET_WORD_PTR(psx->addr[addr]);
    lds_push(psx->load_delay_slots, GET_RT(instr), word);
    return GET_RT(instr);
}

u8
mfc0(struct psx *psx, u32 instr)
{
    lds_push(psx->load_delay_slots, GET_RT(instr), psx->cop0[GET_RD(instr)]);
    return GET_RT(instr);
}

u8
mtc0(struct psx *psx, u32 instr)
{
    assert(GET_RD(instr) < (sizeof(psx->cop0) / sizeof(psx->cop0[0])));
    Assert(GET_RD(instr) != 13 || psx->regs[GET_RT(instr)] == 0, mtc0);
    psx->cop0[GET_RD(instr)] = psx->regs[GET_RT(instr)];

    Assert((psx->sr & IEc) == 0, mtc0);

    if (SR_GET_ISC(psx->sr) == 1) {
        psx->addr = psx->isolated_cache;
    } else if (SR_GET_ISC(psx->sr) == 0) {
        psx->addr = psx->memmap;
    } else {
        assert(false);
    }
    return 0;
}

u8
mips_syscall(struct psx *psx)
{
    exception(psx, e_Sys);
    return 0;
}

u8
rfe(struct psx *psx, u32 instr)
{
    psx->sr = (psx->sr & ~0x3f) | ((psx->sr & 0x3f) >> 2);
    return 0;
}

void
bios_functions(struct psx *psx)
{
    static u16 len;
    static char buf[4 * KB];

    u32 addr = PADDR(psx->addr_to_instr);
    u8 func_number = psx->t1 & 0xFF;

    if ((addr == 0xa0 && func_number == 0x3c) ||
        (addr == 0xb0 && func_number == 0x3d)) {
        char c = psx->a0;
        buf[len] = c;
        if (buf[len] == '\n' || len == ((4 * KB) - 2)) {
            buf[len + 1] = 0;
            printf("%s", buf);
            len = 0;
        } else {
            ++len;
        }
    }

    assert(len < 4 * KB);
}

/* convention: any instrunction that modifies a register returns the register id
 * that it modifies, if it doesn't modify any register it returns 0.
 * */
void
exec_loop(struct psx *psx)
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

    static void *bcondz_dispatch_tbl[] = {
        &&h_bltz,   &&h_bgez,   &&h_na, &&h_na, // 00
        &&h_na,     &&h_na,     &&h_na, &&h_na, // 04
        &&h_na,     &&h_na,     &&h_na, &&h_na, // 08
        &&h_na,     &&h_na,     &&h_na, &&h_na, // 0c
        &&h_bltzal, &&h_bgezal, &&h_na, &&h_na, // 10
        &&h_na,     &&h_na,     &&h_na, &&h_na, // 14
        &&h_na,     &&h_na,     &&h_na, &&h_na, // 18
        &&h_na,     &&h_na,     &&h_na, &&h_na, // 1c
    };

    u32 instr = 0;
    psx->pc = RESET_VECTOR;
    psx->next_pc = psx->pc + 4;

#define PRE_OP(cmd)                                                            \
    do {                                                                       \
        assert(psx->r0 == 0);                                                  \
        psx->addr_to_instr = psx->pc;                                          \
        bios_functions(psx);                                                   \
        psx->pc = psx->next_pc;                                                \
        psx->next_pc = psx->pc + 4;                                            \
        if (bullshit) {                                                        \
            dasm_##cmd(psx, instr);                                            \
        }                                                                      \
    } while (0)

#define POST_OP(val)                                                           \
    do {                                                                       \
        psx->is_branch_delay = val;                                            \
        lds_sanity_clear(psx->load_delay_slots, psx->last_modified_reg);       \
        struct load_delay_slot top = lds_pop(psx->load_delay_slots);           \
        psx->regs[top.reg_index] = top.value;                                  \
        interrupt_check(psx);                                                  \
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
    goto *bcondz_dispatch_tbl[GET_RT(instr)];
h_bltz:
    PRE_OP(bltz);
    psx->last_modified_reg = bltz(psx, instr);
    POST_OP(true);
    DISPATCH();
h_bgez:
    PRE_OP(bgez);
    psx->last_modified_reg = bgez(psx, instr);
    POST_OP(true);
    DISPATCH();
h_bltzal:
    assert(false);
h_bgezal:
    assert(false);
h_j:
    PRE_OP(j);
    psx->last_modified_reg = j(psx, instr);
    POST_OP(true);
    DISPATCH();
h_jal:
    PRE_OP(jal);
    psx->last_modified_reg = jal(psx, instr);
    POST_OP(true);
    DISPATCH();
h_beq:
    PRE_OP(beq);
    psx->last_modified_reg = beq(psx, instr);
    POST_OP(true);
    DISPATCH();
h_bne:
    PRE_OP(bne);
    psx->last_modified_reg = bne(psx, instr);
    POST_OP(true);
    DISPATCH();
h_blez:
    PRE_OP(blez);
    psx->last_modified_reg = blez(psx, instr);
    POST_OP(true);
    DISPATCH();
h_bgtz:
    PRE_OP(bgtz);
    psx->last_modified_reg = bgtz(psx, instr);
    POST_OP(true);
    DISPATCH();
h_addi:
    PRE_OP(addi);
    psx->last_modified_reg = addi(psx, instr);
    POST_OP(false);
    DISPATCH();
h_addiu:
    PRE_OP(addiu);
    psx->last_modified_reg = addiu(psx, instr);
    POST_OP(false);
    DISPATCH();
h_slti:
    PRE_OP(slti);
    psx->last_modified_reg = slti(psx, instr);
    POST_OP(false);
    DISPATCH();
h_sltiu:
    PRE_OP(sltiu);
    psx->last_modified_reg = sltiu(psx, instr);
    POST_OP(false);
    DISPATCH();
h_andi:
    PRE_OP(andi);
    psx->last_modified_reg = andi(psx, instr);
    POST_OP(false);
    DISPATCH();
h_ori:
    PRE_OP(ori);
    psx->last_modified_reg = ori(psx, instr);
    POST_OP(false);
    DISPATCH();
h_xori:
    assert(false);
h_lui:
    PRE_OP(lui);
    psx->last_modified_reg = lui(psx, instr);
    POST_OP(false);
    DISPATCH();
h_cop0:
    assert(GET_RS(instr) <
           sizeof(cop0_dispatch_tbl) / sizeof(cop0_dispatch_tbl[0]));
    goto *cop0_dispatch_tbl[GET_RS(instr)];
h_mfc0:
    PRE_OP(mfc0);
    psx->last_modified_reg = mfc0(psx, instr);
    POST_OP(false);
    DISPATCH();
h_cfc0:
    assert(false);
h_mtc0:
    PRE_OP(mtc0);
    psx->last_modified_reg = mtc0(psx, instr);
    POST_OP(false);
    DISPATCH();
h_ctc0:
    assert(false);
h_rfe:
    PRE_OP(rfe);
    psx->last_modified_reg = rfe(psx, instr);
    POST_OP(false);
    DISPATCH();
h_cop1:
    assert(false);
h_cop2:
    assert(false);
h_cop3:
    assert(false);
h_lb:
    PRE_OP(lb);
    psx->last_modified_reg = lb(psx, instr);
    POST_OP(false);
    DISPATCH();
h_lh:
    PRE_OP(lh);
    psx->last_modified_reg = lh(psx, instr);
    POST_OP(false);
    DISPATCH();
h_lwl:
    assert(false);
h_lw:
    PRE_OP(lw);
    psx->last_modified_reg = lw(psx, instr);
    POST_OP(false);
    DISPATCH();
h_lbu:
    PRE_OP(lbu);
    psx->last_modified_reg = lbu(psx, instr);
    POST_OP(false);
    DISPATCH();
h_lhu:
    PRE_OP(lhu);
    psx->last_modified_reg = lhu(psx, instr);
    POST_OP(false);
    DISPATCH();
h_lwr:
    assert(false);
h_sb:
    PRE_OP(sb);
    psx->last_modified_reg = sb(psx, instr);
    POST_OP(false);
    DISPATCH();
h_sh:
    PRE_OP(sh);
    psx->last_modified_reg = sh(psx, instr);
    POST_OP(false);
    DISPATCH();
h_swl:
    assert(false);
h_sw:
    PRE_OP(sw);
    psx->last_modified_reg = sw(psx, instr);
    POST_OP(false);
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
    PRE_OP(sll);
    psx->last_modified_reg = sll(psx, instr);
    POST_OP(false);
    DISPATCH();
h_srl:
    PRE_OP(srl);
    psx->last_modified_reg = srl(psx, instr);
    POST_OP(false);
    DISPATCH();
h_sra:
    PRE_OP(sra);
    psx->last_modified_reg = sra(psx, instr);
    POST_OP(false);
    DISPATCH();
h_sllv:
    PRE_OP(sllv);
    psx->last_modified_reg = sllv(psx, instr);
    POST_OP(false);
    DISPATCH();
h_srlv:
    PRE_OP(srlv);
    psx->last_modified_reg = srlv(psx, instr);
    POST_OP(false);
    DISPATCH();
h_srav:
    PRE_OP(srav);
    psx->last_modified_reg = srav(psx, instr);
    POST_OP(false);
    DISPATCH();
h_jr:
    PRE_OP(jr);
    psx->last_modified_reg = jr(psx, instr);
    POST_OP(true);
    DISPATCH();
h_jalr:
    PRE_OP(jalr);
    psx->last_modified_reg = jalr(psx, instr);
    POST_OP(true);
    DISPATCH();
h_syscall:
    PRE_OP(syscall);
    psx->last_modified_reg = mips_syscall(psx);
    POST_OP(false);
    DISPATCH();
h_break:
    assert(false);
h_mfhi:
    PRE_OP(mfhi);
    psx->last_modified_reg = mfhi(psx, instr);
    POST_OP(false);
    DISPATCH();
h_mthi:
    PRE_OP(mthi);
    psx->last_modified_reg = mthi(psx, instr);
    POST_OP(false);
    DISPATCH();
h_mflo:
    PRE_OP(mflo);
    psx->last_modified_reg = mflo(psx, instr);
    POST_OP(false);
    DISPATCH();
h_mtlo:
    PRE_OP(mtlo);
    psx->last_modified_reg = mtlo(psx, instr);
    POST_OP(false);
    DISPATCH();
h_mult:
    assert(false);
h_multu:
    PRE_OP(multu);
    psx->last_modified_reg = mips_multu(psx, instr);
    POST_OP(false);
    DISPATCH();
h_div:
    PRE_OP(div);
    psx->last_modified_reg = mips_div(psx, instr);
    POST_OP(false);
    DISPATCH();
h_divu:
    PRE_OP(divu);
    psx->last_modified_reg = mips_divu(psx, instr);
    POST_OP(false);
    DISPATCH();
h_add:
    PRE_OP(add);
    psx->last_modified_reg = add(psx, instr);
    POST_OP(false);
    DISPATCH();
h_addu:
    PRE_OP(addu);
    psx->last_modified_reg = addu(psx, instr);
    POST_OP(false);
    DISPATCH();
h_sub:
    assert(false);
h_subu:
    PRE_OP(subu);
    psx->last_modified_reg = subu(psx, instr);
    POST_OP(false);
    DISPATCH();
h_and:
    PRE_OP(and);
    psx->last_modified_reg = and(psx, instr);
    POST_OP(false);
    DISPATCH();
h_or:
    PRE_OP(or);
    psx->last_modified_reg = or(psx, instr);
    POST_OP(false);
    DISPATCH();
h_xor:
    PRE_OP(xor);
    psx->last_modified_reg = xor(psx, instr);
    POST_OP(false);
    DISPATCH();
h_nor:
    PRE_OP(nor);
    psx->last_modified_reg = nor(psx, instr);
    POST_OP(false);
    DISPATCH();
h_slt:
    PRE_OP(slt);
    psx->last_modified_reg = slt(psx, instr);
    POST_OP(false);
    DISPATCH();
h_sltu:
    PRE_OP(sltu);
    psx->last_modified_reg = sltu(psx, instr);
    POST_OP(false);
    DISPATCH();
h_na:
    assert(false);

#undef DISPATCH
}

void
make_kseg(vm_address_t base_kseg, mem_entry_name_port_t shared_ram, int bios_fd,
          mem_entry_name_port_t shared_er1, mem_entry_name_port_t shared_io)
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

void *
make_psx_addr_space()
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

int
main()
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
    psx.sr = 0x10900000;

    /* This is the same value of gpustatus when reset gpu command (GP1(00h)) is
     * ran. */
    psx.gpustat = GP1_RESET;

    exec_loop(&psx);
    vm_deallocate(mach_task_self(), (vm_address_t)psx.addr, ADDR_SPACE_SIZE);
    vm_deallocate(mach_task_self(), (vm_address_t)psx.isolated_cache,
                  ADDR_SPACE_SIZE);
}
