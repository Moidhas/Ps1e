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
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;

#define RESET_EXCEPTION_VECTOR 0xBFC00000
#define BIOS_PATH "./BIOS/SCPH1001.BIN"
#define GPR_SIZE 32
#define GET_PRIMARY(opcode) ((opcode) >> 26)
#define GET_SECONDARY(opcode) ((opcode) & 0x1F)
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

    u8* bios; /* this is little endian, and m1 chip is also little endian. */
};

// enum psx_error {
//     NONE
// };

// enum psx_error (*actv_sinstr[])(struct psx*) = {
// };

// enum psx_error (*actv_pinstr[])(struct psx*) = {
// };

void sll(struct psx* psx, u32 opcode) {
    assert(GET_RD(opcode) < GPR_SIZE);
    psx->regs[GET_RD(opcode)] = GET_RT(opcode) << GET_IMM5(opcode);
}

void lui(struct psx* psx, u32 opcode) {
    assert(GET_RT(opcode) < GPR_SIZE);
    psx->regs[GET_RT(opcode)] = opcode << 16;
}

void start(struct psx* psx) {
    u32 opcode = ((u32*)psx->bios)[8]; /* macbook m1 chip is little endian */
    printf("0x%08x\n", opcode);
    printf("primary: 0x%02x, secondary: 0x%02x\n", GET_PRIMARY(opcode),
           GET_SECONDARY(opcode));
    assert(false);

    /* 
     * lw:
     *   loadword stuff
     *   load delay slot stuff
     *   branch delay slot stuff
     *   update slot stuff
     *   goto next instruction
     */
}

int main() {
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

    u8* bios = mmap(NULL, st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    close(fd);

    if (bios == MAP_FAILED) {
        fprintf(stderr, "error mmap bios: %d\n", errno);
        exit(EXIT_FAILURE);
    }

    struct psx psx;
    psx.bios = bios;
    start(&psx);

    if (munmap(bios, st.st_size) < 0) {
        fprintf(stderr, "error munmap bios: %d\n", errno);
        exit(EXIT_FAILURE);
    }
}
