#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
typedef int32_t i32;
typedef int16_t i16;
typedef int8_t i8;

#define RESET_EXCEPTION_VECTOR 0xBFC00000
#define BIOS_PATH "./BIOS/SCPH1001.BIN"
#define GPR_SIZE 32
#define GET_PRIMARY(opcode) ((opcode) >> 26)
#define GET_SECONDARY(opcode) ((opcode) & 0x3F)
#define GET_IMM26(opcode) ((opcode) & 0x3FFFFFF)
#define GET_IMM16(opcode) ((opcode) & 0xFFFF)
#define GET_IMM5(opcode) (((opcode) >> 6) & 0x1F)
#define GET_RD(opcode) (((opcode) >> 11) & 0x1F)
#define GET_RT(opcode) (((opcode) >> 16) & 0x1F)
#define GET_RS(opcode) (((opcode) >> 21) & 0x1F)

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
            u32 s7; /*    Static variables, must be saved by subs */
            u32 t8;
            u32 t9; /* Temporaries, may be changed by subroutines */
            u32 k0;
            u32 k1; /*   Reserved for kernel (destroyed by some IRQ handlers!)
                     */
            u32 gp; /*     Global pointer (rarely used) */
            u32 sp; /* Stack pointer */
            u32 fp; /*  Frame Pointer, or 9th Static variable, must be saved */
            u32 ra; /*    Return address (used so by JAL,BLTZAL,BGEZAL opcodes)
                     */
            u32 pc;
            u32 hi;
            u32 lo; /* these registers are not general purpose (can't be
                       accessed by opcodes) */
        };
    };

    u32 next_pc;

    u8 *addr; /* this is little endian, and m1 chip is also little endian. */
};

// enum psx_error {
//     NONE
// };

// enum psx_error (*actv_sinstr[])(struct psx*) = {
// };

// enum psx_error (*actv_pinstr[])(struct psx*) = {
// };

void sll(struct psx *psx, u32 opcode)
{
    psx->regs[GET_RD(opcode)] = GET_RT(opcode) << GET_IMM5(opcode);
}

void lui(struct psx *psx, u32 opcode)
{
    psx->regs[GET_RT(opcode)] = opcode << 16;
}

void j(struct psx *psx, u32 opcode)
{
    psx->next_pc = (psx->pc & 0xF0000000) + (GET_IMM26(opcode) << 2);
}

void addiu(struct psx *psx, u32 opcode)
{
    psx->regs[GET_RT(opcode)] =
        psx->regs[GET_RS(opcode)] + (i32)GET_IMM16(opcode);
}

void addu(struct psx *psx, u32 opcode)
{
    psx->regs[GET_RD(opcode)] =
        psx->regs[GET_RS(opcode)] + psx->regs[GET_RT(opcode)];
}

void start(struct psx *psx)
{
    /*
     * instead of making seperate dispatch tables I could just come up with a
     * way to merge the primary and secondary opcodes, this would help with
     * increasing branch prediction.
     */
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

#define DISPATCH()                                                             \
    opcode = *((u32 *)&psx->addr[psx->pc]); /* m1 is little endian */            \
    printf("pc: 0x%08x, opcode: 0x%08x, primary: 0x%02x, secondary: 0x%02x\n", \
           psx->pc, opcode, GET_PRIMARY(opcode), GET_SECONDARY(opcode));       \
    assert(GET_PRIMARY(opcode) <                                               \
           sizeof(dispatch_tbl) / sizeof(dispatch_tbl[0]));                    \
    if (GET_PRIMARY(opcode) != 0) {                                            \
        goto *dispatch_tbl[GET_PRIMARY(opcode)];                               \
    } else {                                                                   \
        assert(GET_SECONDARY(opcode) <                                         \
               sizeof(sp_dispatch_tbl) / sizeof(sp_dispatch_tbl[0]));          \
        goto *sp_dispatch_tbl[GET_SECONDARY(opcode)];                          \
    }

    DISPATCH();

h_bcondz:
    assert(false);
h_j:
    assert(false);
    psx->pc = psx->next_pc;
    j(psx, opcode);
    DISPATCH();
h_jal:
    assert(false);
h_beq:
    assert(false);
h_bne:
    assert(false);
h_blez:
    assert(false);
h_bgtz:
    assert(false);
h_addi:
    assert(false);
h_addiu:
    assert(false);
    psx->pc = psx->next_pc;
    psx->next_pc += 4;
    addiu(psx, opcode);
    DISPATCH();
h_slti:
    assert(false);
h_sltiu:
    assert(false);
h_andi:
    assert(false);
h_ori:
    assert(false);
h_xori:
    assert(false);
h_lui:
    psx->pc = psx->next_pc;
    psx->next_pc += 4;
    lui(psx, opcode);
    DISPATCH();
h_cop0:
h_cop1:
h_cop2:
h_cop3:
h_lb:
h_lh:
h_lwl:
h_lw:
h_lbu:
h_lhu:
h_lwr:
h_sb:
h_sh:
h_swl:
h_sw:
h_swr:
h_lwc0:
h_lwc1:
h_lwc2:
h_lwc3:
h_swc0:
h_swc1:
h_swc2:
h_swc3:
    assert(false);
/* special */
h_sll:
    assert(false);
    psx->pc = psx->next_pc;
    psx->next_pc += 4;
    sll(psx, opcode);
    DISPATCH();
h_srl:
h_sra:
h_sllv:
h_srlv:
h_srav:
h_jr:
h_jalr:
h_syscall:
h_break:
h_mfhi:
h_mthi:
h_mflo:
h_mtlo:
h_mult:
h_multu:
h_div:
h_divu:
h_add:
    assert(false);
h_addu:
    assert(false);
    psx->pc = psx->next_pc;
    psx->next_pc += 4;
    addu(psx, opcode);
    DISPATCH();
h_sub:
h_subu:
h_and:
h_or:
h_xor:
h_nor:
h_slt:
h_sltu:
h_na:
    assert(false);
}

int main()
{
    int fd = open(BIOS_PATH, O_RDONLY);

    if (fd < 0) {
        fprintf(stderr, "error opening bios: %d\n", errno);
        exit(EXIT_FAILURE);
    }

    struct stat st;
    if (fstat(fd, &st) < 0) {
        fprintf(stderr, "error stats bios: %d\n", errno);
        exit(EXIT_FAILURE);
    }

    /* allocates entire psx address space */
    u8 *addr = mmap(NULL, 0x100000000, PROT_READ | PROT_WRITE,
                    MAP_PRIVATE | MAP_ANON, -1, 0);

    if (addr == MAP_FAILED) {
        fprintf(stderr, "error mmap address space: %d\n", errno);
        exit(EXIT_FAILURE);
    }

    u8 *bios = mmap(addr + RESET_EXCEPTION_VECTOR, st.st_size, PROT_READ,
                    MAP_PRIVATE | MAP_FIXED, fd, 0);

    if (bios == MAP_FAILED) {
        fprintf(stderr, "error mmap bios: %d\n", errno);
        exit(EXIT_FAILURE);
    }

    struct psx psx;
    psx.addr = addr;
    start(&psx);

    if (munmap(bios, st.st_size) < 0) {
        fprintf(stderr, "error munmap bios: %d\n", errno);
        exit(EXIT_FAILURE);
    }
}
