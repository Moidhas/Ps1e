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
typedef int32_t i32;
typedef int16_t i16;
typedef int8_t i8;

/* all of these are in bytes. */
#define KB (1 << 10)
#define MB (1 << 20)
#define GB (1 << 30)

size_t get_page_size()
{
    static size_t page_size = 0;
    if (page_size == 0) {
        page_size = sysconf(_SC_PAGESIZE);
    }
    return page_size;
}

/* since the psx uses 32 bit addresses, total address space is 2^32 */
#define ADDR_SPACE_SIZE 0x100000000
#define KSEG0 0x80000000
#define KSEG1 0xA0000000
#define KSEG_SIZE (512 * MB)

#define RESET_EXCEPTION_VECTOR 0xBFC00000
#define BIOS_PATH "./BIOS/SCPH1001.BIN"
#define GPR_SIZE 32
/* for all macros assume opcode is sizeof(u32) bytes */
#define GET_PRIMARY(opcode) ((opcode) >> 26)
#define GET_SECONDARY(opcode) ((opcode) & 0x3F)
#define GET_IMM26(opcode) ((opcode) & 0x3FFFFFF)
#define GET_UIMM16(opcode) ((opcode) & 0xFFFF)      /* Unsigned */
#define GET_SIMM16(opcode) (i16)((opcode) & 0xFFFF) /* Signed */
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
    psx->regs[GET_RT(opcode)] = psx->regs[GET_RS(opcode)] + GET_SIMM16(opcode);
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

void sw(struct psx *psx, u32 opcode)
{
    /* has to be 4 bytes aligned */
    assert((psx->regs[GET_RS(opcode)] + GET_SIMM16(opcode)) % sizeof(u32) == 0);

    printf("\t storing memory at: 0x%08x, value: 0x%08x\n",
           psx->regs[GET_RS(opcode)] + GET_SIMM16(opcode),
           psx->regs[GET_RT(opcode)]);

    /* assuming machine is little endian */
    u32 *addr =
        (u32 *)&psx->addr[psx->regs[GET_RS(opcode)] + GET_SIMM16(opcode)];
    addr[0] = psx->regs[GET_RT(opcode)];
}

void lhu(struct psx *psx, u32 opcode)
{
    /* has to be 2 bytes aligned */
    assert((GET_SIMM16(opcode) + psx->regs[GET_RS(opcode)]) % sizeof(u16) == 0);

    printf("\t loading memory at: 0x%08x, value: 0x%08x\n",
           GET_SIMM16(opcode) + psx->regs[GET_RS(opcode)],
           *(u16 *)&psx->addr[GET_SIMM16(opcode) + psx->regs[GET_RS(opcode)]]);

    /* assuming machine is little endian */
    psx->regs[GET_RT(opcode)] =
        *(u16 *)&psx->addr[GET_SIMM16(opcode) + psx->regs[GET_RS(opcode)]];
}

void or(struct psx *psx, u32 opcode)
{
    psx->regs[GET_RD(opcode)] =
        psx->regs[GET_RS(opcode)] | psx->regs[GET_RT(opcode)];
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

#define DISPATCH()                                                             \
    do {                                                                       \
        assert(psx->zero == 0);                                                \
        opcode = *((u32 *)&psx->addr[psx->pc]); /* m1 is little endian */      \
        printf("pc: 0x%08x, opcode: 0x%08x, primary: 0x%02x, secondary: "      \
               "0x%02x\n",                                                     \
               psx->pc, opcode, GET_PRIMARY(opcode), GET_SECONDARY(opcode));   \
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
    assert(false);
h_j:
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
    psx->pc = psx->next_pc;
    psx->next_pc += 4;
    ori(psx, opcode);
    DISPATCH();
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
    assert(false);
    psx->pc = psx->next_pc;
    psx->next_pc += 4;
    lhu(psx, opcode);
    DISPATCH();
h_lwr:
h_sb:
h_sh:
h_swl:
h_sw:
    psx->pc = psx->next_pc;
    psx->next_pc += 4;
    sw(psx, opcode);
    DISPATCH();
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
    psx->pc = psx->next_pc;
    psx->next_pc += 4;
    or(psx, opcode);
    DISPATCH();
h_xor:
h_nor:
h_slt:
h_sltu:
h_na:
    assert(false);

#undef DISPATCH
}

/* allocates psx address space, setups mirroring and protections. */
void *psx_alloc_addr_space()
{
#define CHECK_ERR(expr, fstr, fmt)                                             \
    do {                                                                       \
        if (expr) {                                                            \
            fprintf(stderr, fstr, fmt);                                        \
            exit(EXIT_FAILURE);                                                \
        }                                                                      \
    } while (0)

    memory_object_size_t size = ADDR_SPACE_SIZE;
    mem_entry_name_port_t shmem_handle = MACH_PORT_NULL;

    kern_return_t ret = mach_make_memory_entry_64(
        mach_task_self(), &size, 0,
        MAP_MEM_NAMED_CREATE | VM_PROT_READ | VM_PROT_WRITE, &shmem_handle,
        MACH_PORT_NULL);
    CHECK_ERR(ret != KERN_SUCCESS, "error making shmem_handle: %d\n", ret);

    vm_address_t addr;
    ret = vm_map(mach_task_self(), &addr, ADDR_SPACE_SIZE, 0, VM_FLAGS_ANYWHERE,
                 shmem_handle, 0, false, VM_PROT_READ | VM_PROT_WRITE,
                 VM_PROT_READ | VM_PROT_WRITE, VM_INHERIT_DEFAULT);
    CHECK_ERR(ret != KERN_SUCCESS, "error allocating addr space: %d\n", ret);

    vm_address_t kseg1 = addr + KSEG1;

    /* try to understand why creating the shared handler first and then mapping
     * to make mirrors works. */
    ret = vm_map(mach_task_self(), &kseg1, KSEG_SIZE, 0,
                 VM_FLAGS_FIXED | VM_FLAGS_OVERWRITE, shmem_handle, KSEG0,
                 false, VM_PROT_READ | VM_PROT_WRITE,
                 VM_PROT_READ | VM_PROT_WRITE, VM_INHERIT_DEFAULT);
    CHECK_ERR(ret != KERN_SUCCESS, "error mirroring kseg1 to kseg0: %d\n", ret);

    int fd = open(BIOS_PATH, O_RDONLY);
    CHECK_ERR(fd < 0, "error opening bios: %d\n", errno);

    struct stat st;
    CHECK_ERR(fstat(fd, &st) < 0, "error stats bios: %d\n", errno);

    assert(((uintptr_t)addr + RESET_EXCEPTION_VECTOR) % get_page_size() == 0);
    assert(st.st_size % get_page_size() == 0);

    u8 *bios = mmap((u8 *)addr + RESET_EXCEPTION_VECTOR, st.st_size, PROT_READ,
                    MAP_PRIVATE | MAP_FIXED, fd, 0);
    CHECK_ERR(bios == MAP_FAILED, "error allocating bios space: %d\n", errno);

    ret = mach_port_deallocate(mach_task_self(), shmem_handle);
    CHECK_ERR(ret != KERN_SUCCESS, "error deallocating shmem_handle: %d\n",
              ret);
    close(fd);
    return (void *)addr;
#undef CHECK_ERR
}

int main()
{
    struct psx psx;
    memset(&psx, 0, sizeof(psx));
    psx.addr = psx_alloc_addr_space();
    start(&psx);
    vm_deallocate(mach_task_self(), (vm_address_t)psx.addr, ADDR_SPACE_SIZE);
}
