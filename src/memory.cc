#include "memory.hpp"

#include "cop0.hpp"

using namespace BitUtils;
using namespace std;

MemCtrl1DelayReg::MemCtrl1DelayReg(u32 value)
    : writeDelay{static_cast<u8>(extractBits(value, 0, 4))},
      readDelay{static_cast<u8>(extractBits(value, 4, 4))},
      recoveryPeriod{static_cast<u8>(extractBits(value, 8, 1))},
      holdPeriod{static_cast<u8>(extractBits(value, 9, 1))},
      floatingPeriod{static_cast<u8>(extractBits(value, 10, 1))},
      PreStrobePeriod{static_cast<u8>(extractBits(value, 11, 1))},
      dataBusWidth{static_cast<u8>(extractBits(value, 12, 1))},
      autoIncrement{static_cast<u8>(extractBits(value, 13, 1))},
      numberOfAddressBits{static_cast<u8>(extractBits(value, 16, 5))},
      DMATimingOverride{static_cast<u8>(extractBits(value, 24, 4))},
      addressErrorFlag{static_cast<u8>(extractBits(value, 28, 1))},
      DMATimingSelect{static_cast<u8>(extractBits(value, 29, 1))},
      wideDma{static_cast<u8>(extractBits(value, 30, 1))},
      wait{static_cast<u8>(extractBits(value, 31, 1))},
      value{value} {}

MemCtrl1DelayReg::MemCtrl1DelayReg() {}

u32 MMap::getMemCtrl1DelayRegIdx(const PAddress paddr) {
    assert(memCtrl1DelayRegs.range.contains(paddr.m_addr));
    const u32 idx = memCtrl1DelayRegs.range.getOffset(paddr.m_addr) / 4;
    assert(idx < memCtrl1DelayRegs.value.size());
    return idx;
}

u32 MMap::getMemCtrl1Reg(const PAddress paddr) {
    const u32 addr = paddr.m_addr;

    if (e1BaseAddr.range.contains(addr)) {
        return e1BaseAddr.value;
    } else if (e2BaseAddr.range.contains(addr)) {
        return e2BaseAddr.value;
    }

    return memCtrl1DelayRegs.value[getMemCtrl1DelayRegIdx(paddr)].value;
}

u32 MMap::getIoReg(const PAddress pAddr) {
    const u32 addr = pAddr.m_addr;
    if (MEM_CTRL1_RANGE.contains(addr)) {
        return getMemCtrl1Reg(pAddr);
    } else if (ramSize.range.contains(addr)) {
        return ramSize.value;
    } else if (INTERRUPT_CTRL.contains(addr)) {
        assert((addr == 0x1F801070 || addr == 0x1F801074) &&
               "INTERRUPT NOT YET IMPLEMENTED");
        return 0;
    } else if (DMA_IO_RANGE.contains(addr)) {
        // println("DMA Not yet implemeneted");
        return 0;
    } else if (SPU_IO_RANGE.contains(addr)) {
        // println("SPU Not yet implemeneted");
        return 0;
    } else if (GPU_IO_RANGE.contains(addr)) {
        return static_cast<u32>(~0);
    } else {
        println("{:#X}", addr);
        assert(false);
    }
}

void MMap::updateDelayRegState(const DelayIdx idx) {
    array<Range *, 6> dict = {&E1_RANGE,      &E3_RANGE,    &biosBuffer.range,
                              &SPU_RAM_RANGE, &CDROM_RANGE, &E2_RANGE};

    assert(static_cast<u32>(idx) < dict.size());

    dict[static_cast<u32>(idx)]->byteLength =
        (1
         << memCtrl1DelayRegs.value[static_cast<u32>(idx)].numberOfAddressBits);
}

void MMap::writeMemCtrl1Register(const PAddress paddr, const u32 value) {
    const u32 addr = paddr.m_addr;
    if (e1BaseAddr.range.contains(addr)) {
        assert(extractBits(value, 24, 8) == 0x1F);
        assert(value == 0x1F000000);
        // println("e1BaseAddr");
        e1BaseAddr.range.start = value;
    } else if (e2BaseAddr.range.contains(addr)) {
        assert(extractBits(value, 24, 8) == 0x1F);
        assert(value == 0x1F802000);
        // println("e2BaseAddr");
        e2BaseAddr.range.start = value;
    } else if (memCtrl1DelayRegs.range.contains(paddr.m_addr)) {
        const u32 idx = getMemCtrl1DelayRegIdx(paddr);
        memCtrl1DelayRegs.value[idx] = value;
        updateDelayRegState(static_cast<DelayIdx>(idx));
    } else if (comDelay.range.contains(addr)) {
        // println("comDelay");
        comDelay.value = value;
    } else {
        assert(false);
    }
}

void MMap::writeIoReg(const PAddress pAddr, const u32 value) {
    const u32 addr = pAddr.m_addr;

    if (MEM_CTRL1_RANGE.contains(addr)) {
        writeMemCtrl1Register(pAddr, value);
    } else if (ramSize.range.contains(addr)) {
        const bitset<32> bits = value;
        const u8 ramRangeChoice = (bits[11] << 1) | bits[9];
        const array<u8, 4> ramSizes{1, 4, 2, 8};
        ramBuffer.range.byteLength = ramSizes[ramRangeChoice] * MB;
        ramSize.value = value;
    } else if (SPU_IO_RANGE.contains(addr)) {
        // println("SPU Not yet implemeneted");
    } else if (INTERRUPT_CTRL.contains(addr)) {
        // println("write to INTERRUPT_CTRL");
        // assert(false && "write to INTERRUPT_CTRL");
    } else if (timers.range.contains(addr)) {
        if (value != 0) {
            // println("{:#X}, {:#X}", addr, value);
        }
        // println("TIMERS NOT YET IMPLEMENTED");
    } else if (DMA_IO_RANGE.contains(addr)) {
        // println("DMA Not yet implemeneted");
        return;
    } else if (GPU_IO_RANGE.contains(addr)) {
        return;
    } else {
        println("{:#X}", pAddr.m_addr);
        assert(false);
    }
}

PAddress MMap::getPAddr(const VAddress vAddr) {
    const u32 addr = vAddr.m_addr;
    const bool condition = KSEG1_RANGE.contains(addr) ||
                           KSEG0_RANGE.contains(addr) ||
                           KUSEG_RANGE.contains(addr);
    PAddress paddr;
    if (condition) {
        const u32 m_addr{~0xE0000000 & addr};
        assert(
            !(KSEG1_RANGE.contains(addr) && scratchpadBuffer.range.contains(m_addr)));
        paddr.m_addr = m_addr;
    } else if (KSEG2_RANGE.contains(addr))
        paddr.m_addr = addr;
    else {
        // I should be throwing AdEL, or AdES
        assert(false);
    }

    return paddr;
}

u32 MMap::handleBufferRead(const AddressedValue<span<u8>> &addressedBuffer,
                           const PAddress paddr, const u8 numOfBytes) {
    const u32 addr = paddr.m_addr;
    assert(addr % numOfBytes == 0);

    const u32 idx = addressedBuffer.range.getOffset(addr);
    assert(addressedBuffer.value.size() >= idx + numOfBytes);
    u32 accBytes{0};
    for (int i = 0; i < numOfBytes; ++i) {
        const u32 byte = addressedBuffer.value[idx + i] << (8 * i);
        accBytes |= byte;
    }

    return accBytes;
}

u32 MMap::handleRead(const PAddress paddr, const u8 numOfBytes) {
    assert(numOfBytes <= 4);
    const u32 addr = paddr.m_addr;

    assert(isc == 1 || isc == 0);
    assert(addr % numOfBytes == 0);
    if (addr % numOfBytes != 0) throw COP0::MipsException{COP0::ExcCode::AdEL};

    if (ramBuffer.range.contains(addr)) {
        if (isc == 1) {
            const u32 idx = ramBuffer.range.getOffset(addr);
            assert(idx < scratchpadBuffer.value.size());
            assert(idx == scratchpadBuffer.range.getOffset(addr));
            return handleBufferRead({span{scratchpadBuffer.value}, scratchpadBuffer.range}, paddr, numOfBytes);
        }
        const u32 value = handleBufferRead(
            {span{ramBuffer.value}, ramBuffer.range}, paddr, numOfBytes);
        return value;
    } else if (E1_RANGE.contains(addr)) {
        // println("E1");
        // assert(false);
        return 0;
    } else if (scratchpadBuffer.range.contains(addr)) {
        return handleBufferRead({span{scratchpadBuffer.value}, scratchpadBuffer.range}, paddr, numOfBytes);
    } else if (IO_RANGE.contains(addr)) {
        // println("IO_RANGE");
        return getIoReg(PAddress{addr});
    } else if (E2_RANGE.contains(addr)) {
        // println("E2_RANGE");
        assert(false);
    } else if (E3_RANGE.contains(addr)) {
        // println("E3_RANGE");
        assert(false);
    } else if (biosBuffer.range.contains(addr)) {
        return handleBufferRead({span{biosBuffer.value}, biosBuffer.range},
                                paddr, numOfBytes);
    } else if (cacheCtrlReg.range.contains(addr)) {
        // println("CacheCtrl");
        return cacheCtrlReg.value;
    } else {
        println("{:#X}", addr);
        assert(false);
    }
}

void MMap::handleBufferWrite(AddressedValue<span<u8>> addressedBuffer,
                             const PAddress paddr, const u8 numOfBytes,
                             const u32 value) {
    const u32 addr = paddr.m_addr;
    assert(addr % numOfBytes == 0);

    const u32 idx = addressedBuffer.range.getOffset(addr);
    assert(addressedBuffer.value.size() >= idx + numOfBytes);
    for (int i = 0; i < numOfBytes; ++i) {
        const u8 byte = (value & (0xFF << (8 * i))) >> (8 * i);
        addressedBuffer.value[idx + i] = byte;
    }

    // println("[{:#X}] <- {:#X}", addr, value);
}

void MMap::handleWrite(const PAddress paddr, const u32 value,
                       const u8 numOfBytes) {
    assert(numOfBytes <= 4);
    const u32 addr = paddr.m_addr;

    assert(isc == 1 || isc == 0);
    assert(addr % numOfBytes == 0);
    if (addr % numOfBytes != 0) throw COP0::MipsException{COP0::ExcCode::AdES};

    if (ramBuffer.range.contains(addr)) {
        if (isc == 1) {
            const u32 idx = ramBuffer.range.getOffset(addr);
            assert(idx < scratchpadBuffer.value.size());
            assert(idx == scratchpadBuffer.range.getOffset(addr));
            handleBufferWrite({span{scratchpadBuffer.value}, scratchpadBuffer.range}, paddr, numOfBytes, value);
            return;
        }
        handleBufferWrite({span{ramBuffer.value}, ramBuffer.range}, paddr,
                          numOfBytes, value);
    } else if (E1_RANGE.contains(addr)) {
        // println("E1");
        assert(false);
    } else if (scratchpadBuffer.range.contains(addr)) {
        assert(false);
    } else if (IO_RANGE.contains(addr)) {
        writeIoReg(paddr, value);
    } else if (E2_RANGE.contains(addr)) {
        assert(addr == 0x1F802041);
    } else if (E3_RANGE.contains(addr)) {
        // println("E3_RANGE");
        assert(false);
    } else if (biosBuffer.range.contains(addr)) {
        assert(false);
    } else if (cacheCtrlReg.range.contains(addr)) {
        cacheCtrlReg.value = value;
    } else {
        println("{}", addr);
        assert(false);
    }
}


u32 bufrd(const span<u8> bufr, u32 idx, const u8 numOfBytes) {
    u32 accBytes{0};
    assert(idx + numOfBytes <= bufr.size());

    for (int i = 0; i < numOfBytes; ++i) {
        const u32 byte = bufr[idx + i] << (8 * i);
        accBytes |= byte;
    }

    return accBytes;
}


HeaderReg MMap::sideload() {
    struct stat info;
    stat("./tests/psxtest_cpu.exe", &info);
    ifstream exe{"./tests/psxtest_cpu.exe", ios::binary | ios::in};

    array<u8, 2*KB> header;
    exe.read((char *)header.data(), header.size());
    u32 pc = bufrd(header, 0x10, 4);
    u32 gp = bufrd(header, 0x14, 4);
    u32 dstAddr = getPAddr(VAddress{bufrd(header, 0x18, 4)}).m_addr;
    u32 filesize = bufrd(header, 0x1C, 4);
    u32 sp = bufrd(header, 0x30, 4);

    assert((filesize % (2 * KB)) == 0);
    assert((filesize + (2 * KB)) == info.st_size);

    u32 idx = ramBuffer.range.getOffset(dstAddr);
    exe.read((char *)&ramBuffer.value[idx], filesize);
    return {pc, gp, sp};
}


MMap::MMap() {
    struct stat info;
    stat("./BIOS/SCPH1001.BIN", &info);
    if (biosBuffer.value.size() != info.st_size)
        throw runtime_error{"size of BIOS is not 512KB"};

    ifstream bios{"./BIOS/SCPH1001.BIN", ios::binary | ios::in};
    bios.read((char *)biosBuffer.value.data(), biosBuffer.value.size());
}

// need to refactor the way shits read/stored;
u32 MMap::load32(VAddress vAddr) {
    const PAddress paddr = getPAddr(vAddr);
    return handleRead(paddr, 4);
}

u16 MMap::load16(VAddress vAddr) {
    const PAddress paddr = getPAddr(vAddr);
    const u32 value = handleRead(paddr, 2);
    assert(value <= NumLimit<u16>());
    return value;
}

u8 MMap::load8(VAddress vAddr) {
    const PAddress paddr = getPAddr(vAddr);
    const u32 value = handleRead(paddr, 1);
    assert(value <= NumLimit<u8>());
    return value;
}

void MMap::store32(const VAddress vAddr, const u32 value) {
    const PAddress pAddr = getPAddr(vAddr);
    handleWrite(pAddr, value, 4);
}

void MMap::store16(const VAddress vAddr, const u32 value) {
    const PAddress pAddr = getPAddr(vAddr);
    handleWrite(pAddr, value, 2);
}

void MMap::store8(const VAddress vAddr, const u32 value) {
    const PAddress pAddr = getPAddr(vAddr);
    handleWrite(pAddr, value, 1);
}
