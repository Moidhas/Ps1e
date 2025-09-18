#include "memory.hpp"

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
    } else {
        assert(false);
    }
}

void MMap::updateDelayRegState(const DelayIdx idx) {
    array<Range *, 6> dict = {&E1_RANGE,  &E3_RANGE,    &biosBuffer.range,
                              &SPU_RANGE, &CDROM_RANGE, &E2_RANGE};

    assert(static_cast<u32>(idx) < dict.size());

    dict[static_cast<u32>(idx)]->byteLength =
        (1
         << memCtrl1DelayRegs.value[static_cast<u32>(idx)].numberOfAddressBits);

    const u32 byteLength = dict[static_cast<u32>(idx)]->byteLength;
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
    } else {
        // println("{:#X}", pAddr.m_addr);
        // assert(false);
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
            !(KSEG1_RANGE.contains(addr) && SCRATCHPAD_RANGE.contains(m_addr)));
        paddr.m_addr = m_addr;
    } else if (KSEG2_RANGE.contains(addr))
        paddr.m_addr = addr;
    else
        // BUS ERROR
        assert(false);

    return paddr;
}

u32 MMap::handleBufferRead(const AddressedValue<span<u8>> &addressedBuffer,
                     const PAddress paddr, const u8 numOfBytes) {
    const u32 addr = paddr.m_addr;
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

    if (paddr.m_addr == 0XB0) {
    }

    if (ramBuffer.range.contains(addr)) {
        const u32 value = handleBufferRead(
            {span{ramBuffer.value}, ramBuffer.range}, paddr, numOfBytes);

        if (addr == 0xB0) {
            println("**** **** Value reading: {:#X}", value);
        }

        return value;
    } else if (E1_RANGE.contains(addr)) {
        println("E1");
        // assert(false);
        return 0;
    } else if (SCRATCHPAD_RANGE.contains(addr)) {
        // println("SCRATCHPAD_RANGE");
        assert(false);
    } else if (IO_RANGE.contains(addr)) {
        // println("IO_RANGE");
        return getIoReg(PAddress{addr});
    } else if (E2_RANGE.contains(addr)) {
        println("E2_RANGE");
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
    const u32 idx = addressedBuffer.range.getOffset(addr);
    assert(addressedBuffer.value.size() >= idx + numOfBytes);
    for (int i = 0; i < numOfBytes; ++i) {
        const u8 byte = (value & (0xFF << (8 * i))) >> (8 * i);
        addressedBuffer.value[idx + i] = byte;
    }
}

void MMap::handleWrite(const PAddress paddr, const u32 value, const u8 numOfBytes) {
    assert(numOfBytes <= 4);
    const u32 addr = paddr.m_addr;

    if (ramBuffer.range.contains(addr)) {
        if (addr == 0xB0) {
            println("**** **** Value writing: {:#X}", value);
        }
        handleBufferWrite({span{ramBuffer.value}, ramBuffer.range}, paddr,
                          numOfBytes, value);
    } else if (E1_RANGE.contains(addr)) {
        println("E1");
        assert(false);
    } else if (SCRATCHPAD_RANGE.contains(addr)) {
        println("SCRATCHPAD_RANGE");
        assert(false);
    } else if (IO_RANGE.contains(addr)) {
        writeIoReg(paddr, value);
    } else if (E2_RANGE.contains(addr)) {
        assert(addr == 0x1F802041);
    } else if (E3_RANGE.contains(addr)) {
        println("E3_RANGE");
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
