#include "../src/BitUtils.hpp"
#include "../src/cpu.hpp"
#include <vector>

using namespace BitUtils;
using namespace Mips;
using namespace std;

/// Number of CPU cycles after which we consider the test to be a
/// failure
const size_t TIMEOUT = 1'000'000;

void write_blob(Cpu &cpu, u32 address, vector<u32> blob) {
    for (u32 i = 0; i < blob.size(); ++i) {
        cpu.mmap->store32(VAddress{address + (i * 4)}, blob[i]);
    }
}

void write(Cpu &cpu, u32 address, u32 v) {
    cpu.mmap->store32(VAddress{address}, v);
}

u32 read(Cpu &cpu, u32 address) {
    return cpu.mmap->load32(VAddress{address});
}

void setRegsZero(Cpu &cpu) {
    for (int i = 0; i < 32; ++i) {
        cpu.reg.gpr[i] = 0;
    }
}

union R_OPCODE {
    u32 data;
    BitField<26, 6, u32> op;
    BitField<21, 5, u32> rs;
    BitField<16, 5, u32> rt;
    BitField<11, 5, u32> rd;
    BitField<6, 5, u32> shamt;
    BitField<0, 6, u32> funct;
};

void test_beq() {
    Cpu cpu;
    setRegsZero(cpu);

    cpu.reg.gpr[1] = 0x1;
    cpu.reg.gpr[2]= 0x2;
    cpu.reg.gpr[3] = static_cast<u32>(-1);
    cpu.reg.gpr[4]= 0xffffffff;

    write_blob(cpu, 0x80100000,
               vector<u32>{0x10220005,
                 0x00000000,
                 0x200a0001,
                 0x10640004,
                 0x00000000,
                 0x200b0001,
                 0x200a0002,
                 0x00000000,
                 0x00000000,
                 0x0bab6fb8,
                 0x00000000});

    cpu.reg.pc() = 0x80100000;
    bool timeout = true;

    for (int i = 0;  i < TIMEOUT; ++i) {
        if ((cpu.reg.pc() & 0x0fffffff) == 0xeadbee0) {
            timeout = false;
            break;
        }
        cpu.runNextInstr();
    }

    assert(timeout == false);
    assert(cpu.reg.gpr[10] == 0x1);
    assert(cpu.reg.gpr[11] == 0);
}

void test_branch_in_branch_delay() {
    Cpu cpu;
    setRegsZero(cpu);


    write_blob(cpu, 0x80100000,
               vector<u32>{0x10000002,
                 0x10000004,
                 0x20030001,
                 0x20010001,
                 0x10000002,
                 0x00000000,
                 0x20020001,
                 0x00000000,
                 0x0bab6fb8,
                 0x00000000});

    cpu.reg.pc() = 0x80100000;

    bool timeout = true;
    for (int i = 0; i < TIMEOUT; ++i) {
        if ((cpu.reg.pc() & 0x0fffffff) == 0xeadbee0) {
            timeout = false;
            break;
        }
        cpu.runNextInstr();
    }

    assert(timeout == false);
    assert(cpu.reg[1] == 0x1);
    assert(cpu.reg[2] == 0);
    assert(cpu.reg[3] == 0);
}

// DONT RUN TEST YET.
void test_lwr_and_lwr_load_delay() {
     Cpu cpu;
     setRegsZero(cpu);

     write(cpu, 0, 0x76543210);
     write(cpu, 0x4, 0xfedcba98);

    write_blob(cpu, 0x80100000,
               vector<u32>{0x2401ffff,
                 0x98010002,
                 0x88010005,
                 0x00201021,
                 0x2403ffff,
                 0x98030002,
                 0x00000000,
                 0x88030005,
                 0x00602021,
                 0x2405ffff,
                 0x88050005,
                 0x00000000,
                 0x98050002,
                 0x00a03021,
                 0x2407ffff,
                 0x8c070004,
                 0x88070002,
                 0x00e04021,
                 0x2409ffff,
                 0x8c090004,
                 0x00000000,
                 0x88090002,
                 0x01205021,
                 0x240bffff,
                 0x8c0b0004,
                 0x980b0002,
                 0x01606021,
                 0x240dffff,
                 0x8c0d0004,
                 0x00000000,
                 0x980d0002,
                 0x01a07021,
                 0x3c0f067e,
                 0x35ef067e,
                 0x488fc800,
                 0x240fffff,
                 0x480fc800,
                 0x880f0001,
                 0x01e08021,
                 0x2411ffff,
                 0x4811c800,
                 0x00000000,
                 0x98110001,
                 0x02209021,
                 0x0bab6fb8,
                 0x00000000});

    cpu.reg.pc() = 0x80100000;

    bool timeout = true;
    for (int i = 0; i < TIMEOUT; ++i) {
        if ((cpu.reg.pc() & 0x0fffffff) == 0xeadbee0) {
            timeout = false;
            break;
        }
        cpu.runNextInstr();
    }

    assert(timeout == false);

    assert(cpu.reg[1] == 0xba987654);
    assert(cpu.reg[2] == 0xffffffff);
    assert(cpu.reg[3] == 0xba987654);
    assert(cpu.reg[4] == 0xffff7654);
    assert(cpu.reg[5] == 0xba987654);
    assert(cpu.reg[6] == 0xba98ffff);
    assert(cpu.reg[7] == 0x54321098);
    assert(cpu.reg[8] == 0xffffffff);
    assert(cpu.reg[9] == 0x54321098);
    assert(cpu.reg[10] == 0xfedcba98);
    assert(cpu.reg[11] == 0xfedc7654);
    assert(cpu.reg[12] == 0xffffffff);
    assert(cpu.reg[13] == 0xfedc7654);
    assert(cpu.reg[14] == 0xfedcba98);
    assert(cpu.reg[15] == 0x3210067e);
    assert(cpu.reg[16] == 0xffffffff);
    assert(cpu.reg[17] == 0x6765432);
    assert(cpu.reg[18] == 0x67e067e);
}

void test_add_1() {
    Cpu cpu;
    setRegsZero(cpu);


    cpu.reg[1] = 0xa;
    cpu.reg[2] = static_cast<u32>(-15);

    write_blob(cpu, 0x80100000,
               vector<u32>{0x00201820,
                 0x00222020,
                 0x00412820,
                 0x00423020,
                 0x0bab6fb8,
                 0x00000000});

    cpu.reg.pc() = 0x80100000;

    bool timeout = true;
    for (int i = 0; i < TIMEOUT; ++i) {
        if ((cpu.reg.pc() & 0x0fffffff) == 0xeadbee0){
            timeout = false;
            break;
        }
        cpu.runNextInstr();
    }

    assert(timeout == false);
    assert(cpu.reg[1] == 0xa);
    assert(cpu.reg[2] == static_cast<u32>(-15));
    assert(cpu.reg[3] == 0xa);
    assert(cpu.reg[4] == static_cast<u32>(-5));
    assert(cpu.reg[5] == -5);
    assert(cpu.reg[6] == -30);
}

void test_arithmetic_branching_test() {
    Cpu cpu;

    setRegsZero(cpu);

    cpu.reg.gpr[2] =  0xdead;
    cpu.reg.gpr[3] =  0;
    cpu.reg.gpr[5] =  0x1;

    write_blob(cpu, 0x80100000,
               vector<u32>{0x00451023,
                 0x24630001,
                 0x1c40fffd,
                 0x00000000,
                 0x0bab6fb8,
                 0x00000000});

    cpu.reg.pc() = 0x80100000;

    bool timeout = true;
    for (int i = 0; i < TIMEOUT; ++i) {
        if ((cpu.reg.pc() & 0x0fffffff) == 0xeadbee0) {
            timeout = false;
            break;
        }
        cpu.runNextInstr();
    }

    assert(timeout == false);
    assert(cpu.reg[2] == 0);
    assert(cpu.reg[3] == 0xdead);
    assert(cpu.reg[5] == 0x1);
}

void test_bltzal_and_bgezal() {
    Cpu cpu;
    setRegsZero(cpu);

    write_blob(cpu, 0x80100000,
               vector<u32>{0x3c05ffff,
                 0x34a5ffff,
                 0x00000821,
                 0x0000f821,
                 0x04100002,
                 0x00000000,
                 0x34010001,
                 0x001f102b,
                 0x3c03ffff,
                 0x3463ffff,
                 0x0000f821,
                 0x04710002,
                 0x00000000,
                 0x34030001,
                 0x001f202b,
                 0x3c05ffff,
                 0x34a5ffff,
                 0x0000f821,
                 0x04b00002,
                 0x00000000,
                 0x34050001,
                 0x001f302b,
                 0x00003821,
                 0x0000f821,
                 0x04110002,
                 0x00000000,
                 0x34070001,
                 0x001f402b,
                 0x0bab6fb8,
                 0x00000000});

    cpu.reg.pc() = (0x80100000);

    bool timeout = true;
    for (int i = 0; i < TIMEOUT; ++i) {
        if ((cpu.reg.pc() & 0x0fffffff) == 0xeadbee0) {
            timeout = false;
            break;
        }
        cpu.runNextInstr();
    }

    assert(timeout == false);
    assert(cpu.reg[1] == 0x1);
    assert(cpu.reg[2] == 0x1);
    assert(cpu.reg[3] == 0x1);
    assert(cpu.reg[4] == 0x1);
    assert(cpu.reg[5] == -1);
    assert(cpu.reg[6] == 0x1);
    assert(cpu.reg[7] == 0);
    assert(cpu.reg[8] == 0x1);
}

void test_unaligned_loads() {
    Cpu cpu;
    setRegsZero(cpu);

    write(cpu, 0xbee0, 0xdeadbeef);
    cpu.reg.gpr[30] =  0xbee1;

    write_blob(cpu, 0x80100000,
               vector<u32>{0x83c10000,
                 0x93c20000,
                 0x0bab6fb8,
                 0x00000000});

    cpu.reg.pc() = 0x80100000;

    bool timeout = true;
    for (int i = 0; i < TIMEOUT; ++i) {
        if ((cpu.reg.pc() & 0x0fffffff) == 0xeadbee0) {
            timeout = false;
            break;
        }
        cpu.runNextInstr();
    }

    assert(timeout == false);
    assert(cpu.reg[1] == -66);
    assert(cpu.reg[2] == 0xbe);
    assert(cpu.reg[3] == 0);
    assert(cpu.reg[4] == 0);
}

void test_load_delay_for_cop() {
    Cpu cpu;
    setRegsZero(cpu);

    cpu.reg.gpr[2] =  0x80110000;
    write(cpu, 0x80110000, 0xdeadbeef);

    write_blob(cpu, 0x80100000,
               vector<u32>{0x8c430000,
                 0x00000000,
                 0x4803c800,
                 0x10600004,
                 0x00000000,
                 0x20010001,
                 0x0804000a,
                 0x00000000,
                 0x20010002,
                 0x0804000a,
                 0x0bab6fb8,
                 0x00000000});

    cpu.reg.pc() = 0x80100000;

    bool timeout = true;
    for (int i = 0; i < TIMEOUT; ++i) {
        if ((cpu.reg.pc() & 0x0fffffff) == 0xeadbee0) {
            timeout = false;
            break;
        }
        cpu.runNextInstr();
    }

    assert(timeout == false);
    assert(cpu.reg[3] == 0);
    assert(cpu.reg[1] == 0x1);
}

void test_swl_and_swr() {
    Cpu cpu;
    setRegsZero(cpu);

    cpu.reg.gpr[1] =  0;
    cpu.reg.gpr[2] =  0x76543210;
    cpu.reg.gpr[3] =  0xfedcba98;

    write_blob(cpu, 0x80100000,
               vector<u32>{0xac220000,
                 0xa8230000,
                 0x24210004,
                 0xac220000,
                 0xa8230001,
                 0x24210004,
                 0xac220000,
                 0xa8230002,
                 0x24210004,
                 0xac220000,
                 0xa8230003,
                 0x24210004,
                 0xac220000,
                 0xb8230000,
                 0x24210004,
                 0xac220000,
                 0xb8230001,
                 0x24210004,
                 0xac220000,
                 0xb8230002,
                 0x24210004,
                 0xac220000,
                 0xb8230003,
                 0x0bab6fb8,
                 0x00000000});

    cpu.reg.pc() = 0x80100000;

    bool timeout = true;
    for (int i = 0; i < TIMEOUT; ++i) {
        if ((cpu.reg.pc() & 0x0fffffff) == 0xeadbee0) {
            timeout = false;
            break;
        }
        cpu.runNextInstr();
    }

    assert(timeout == false);
    assert(read( cpu, 0) == 0x765432fe);
    assert(read( cpu, 0x4) == 0x7654fedc);
    assert(read( cpu, 0x8) == 0x76fedcba);
    assert(read( cpu, 0xc) == 0xfedcba98);
    assert(read( cpu, 0x10) == 0xfedcba98);
    assert(read( cpu, 0x14) == 0xdcba9810);
    assert(read( cpu, 0x18) == 0xba983210);
    assert(read( cpu, 0x1c) == 0x98543210);
}

void test_multiple_load_cancelling() {
    Cpu cpu;
    setRegsZero(cpu);

    write(cpu, 0, 0x7001a7e);
    cpu.reg.gpr[1] =  0x600dc0de;

    write_blob(cpu, 0x80100000,
               vector<u32>{0x40016000,
                 0x8c010000,
                 0x40017800,
                 0x8c010000,
                 0x8c010000,
                 0x00201021,
                 0x0bab6fb8,
                 0x00000000});

    cpu.reg.pc() = 0x80100000;

    bool timeout = true;
    for (int i = 0; i < TIMEOUT; ++i) {
        if ((cpu.reg.pc() & 0x0fffffff) == 0xeadbee0) {
            timeout = false;
            break;
        }
        cpu.runNextInstr();
    }

    assert(timeout == false);
    assert(cpu.reg[1] == 0x7001a7e);
    assert(cpu.reg[2] == 0x600dc0de);
}

void test_lwl_and_lwr() {
    Cpu cpu;
    setRegsZero(cpu);

    write(cpu, 0, 0x76543210);
    write(cpu, 0x4, 0xfedcba98);

    write_blob(cpu, 0x80100000,
               vector<u32>{0x98010000,
                 0x88010003,
                 0x98020001,
                 0x88020004,
                 0x98030002,
                 0x88030005,
                 0x98040003,
                 0x88040006,
                 0x98050004,
                 0x88050007,
                 0x88060003,
                 0x98060000,
                 0x88070004,
                 0x98070001,
                 0x88080005,
                 0x98080002,
                 0x88090006,
                 0x98090003,
                 0x880a0007,
                 0x980a0004,
                 0x240bffff,
                 0x880b0000,
                 0x240cffff,
                 0x980c0000,
                 0x240dffff,
                 0x880d0001,
                 0x240effff,
                 0x980e0001,
                 0x240fffff,
                 0x880f0002,
                 0x2410ffff,
                 0x98100002,
                 0x2411ffff,
                 0x88110003,
                 0x2412ffff,
                 0x98120003,
                 0x0bab6fb8,
                 0x00000000});

    cpu.reg.pc() = 0x80100000;

    bool timeout = true;
    for (int i = 0; i < TIMEOUT; ++i) {
        if ((cpu.reg.pc() & 0x0fffffff) == 0xeadbee0) {
            timeout = false;
            break;
        }
        cpu.runNextInstr();
    }

    assert(timeout == false);
    assert(cpu.reg[1] == 0x76543210);
    assert(cpu.reg[2] == 0x98765432);
    assert(cpu.reg[3] == 0xba987654);
    assert(cpu.reg[4] == 0xdcba9876);
    assert(cpu.reg[5] == 0xfedcba98);
    assert(cpu.reg[6] == 0x76543210);
    assert(cpu.reg[7] == 0x98765432);
    assert(cpu.reg[8] == 0xba987654);
    assert(cpu.reg[9] == 0xdcba9876);
    assert(cpu.reg[10] == 0xfedcba98);
    assert(cpu.reg[11] == 0x10ffffff);
    assert(cpu.reg[12] == 0x76543210);
    assert(cpu.reg[13] == 0x3210ffff);
    assert(cpu.reg[14] == 0xff765432);
    assert(cpu.reg[15] == 0x543210ff);
    assert(cpu.reg[16] == 0xffff7654);
    assert(cpu.reg[17] == 0x76543210);
    assert(cpu.reg[18] == 0xffffff76);
}

void test_lh_and_lb_sign_extension() {
    Cpu cpu;
    setRegsZero(cpu);

    write(cpu, 0, 0x8080);

    write_blob(cpu, 0x80100000,
               vector<u32>{0x84010000,
                 0x94020000,
                 0x80030000,
                 0x90040000,
                 0x00000000,
                 0x0bab6fb8,
                 0x00000000});

    cpu.reg.pc() = 0x80100000;

    bool timeout = true;
    for (int i = 0; i < TIMEOUT; ++i) {
        if ((cpu.reg.pc() & 0x0fffffff) == 0xeadbee0) {
            timeout = false;
            break;
        }
        cpu.runNextInstr();
    }

    assert(timeout == false);
    assert(cpu.reg[1] == 0xffff8080);
    assert(cpu.reg[2] == 0x8080);
    assert(cpu.reg[3] == 0xffffff80);
    assert(cpu.reg[4] == 0x80);
}

int main() {
    // test_add_1();
    // test_arithmetic_branching_test();
    // test_beq();
    // test_lh_and_lb_sign_extension();
    // test_branch_in_branch_delay();
        // test_load_delay_for_cop();
        test_multiple_load_cancelling();
}
