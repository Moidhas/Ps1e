#ifndef MEMORY_H
#define MEMORY_H

#include <sys/stat.h>
#include <array>
#include <bitset>
#include <cassert>
#include <fstream>
#include <print>
#include <span>
#include <stdexcept>

#include "BitUtils.hpp"
#include "types.hpp"

// TODO: Combine the range with buffer.
struct Range {
    u32 start;
    u32 byteLength;
    const u32 mirrorMask;

    constexpr Range(const u32 start, const u32 byteLength,
                    const u32 mirrorMask = 0xFFFF'FFFF)
        : start{start}, byteLength{byteLength}, mirrorMask{mirrorMask} {}

    constexpr bool contains(const u32 addr) const {
        if (byteLength == 0) return false;
        return start <= addr && addr <= start + (byteLength - 1);
    }

    constexpr u32 getOffset(const u32 addr) const {
        assert(contains(addr));
        return (addr & mirrorMask) - start;
    }
};

struct MemCtrl1DelayReg {
    u8 writeDelay;
    u8 readDelay;
    u8 recoveryPeriod;
    u8 holdPeriod;
    u8 floatingPeriod;
    u8 PreStrobePeriod;
    u8 dataBusWidth;
    u8 autoIncrement;
    u8 numberOfAddressBits;
    u8 DMATimingOverride;
    u8 addressErrorFlag;
    u8 DMATimingSelect;
    u8 wideDma;
    u8 wait;
    u32 value;

    MemCtrl1DelayReg(u32 value);
    MemCtrl1DelayReg();
};

template <typename T>
struct AddressedValue {
    T value;
    Range range;

    AddressedValue(const T &value, const Range &range)
        : value{value}, range{range} {}
    AddressedValue(const Range &range) : range{range} {}
};

struct VAddress {
    u32 m_addr;
};
struct PAddress {
    u32 m_addr;
};

// TODO: Change the code so that the order of an enum never matters.
// TODO: Change the code so that if ranges get changed by bios, the addresses
// are some sort of not_initliazed value.
// TODO: Implement caching.
struct MMap {
    // User Memory: KUSEG is intended to contain 2GB virtual memory (on extended
    // MIPS processors), the PSX doesn't support virtual memory, and KUSEG
    // simply contains a mirror of KSEG0/KSEG1 (in the first 512MB) (trying to
    // access memory in the remaining 1.5GB causes an exception).
    // source: https://psx-spx.consoledev.net/memorymap/#write-queue
    static constexpr Range KUSEG_RANGE{0x00000000, 512 * MB};
    static constexpr Range KSEG0_RANGE{0x80000000, 512 * MB};
    static constexpr Range KSEG1_RANGE{0xA0000000, 512 * MB};
    static constexpr Range KSEG2_RANGE{0xC0000000, GB};
    static constexpr Range EXCEPTION_HANDLER{0x00000080, 0x10};

    AddressedValue<std::array<u8, 2 * MB>> ramBuffer{
        Range{0x00000000, 2 * MB, 2 * MB - 1}};

    Range E1_RANGE{0x1F000000, 512 * KB};
    std::array<u8, 512 * KB> e1Buffer;

    static constexpr Range SCRATCHPAD_RANGE{0x1F800000, KB};
    std::array<u8, SCRATCHPAD_RANGE.byteLength> scratchpadBuffer;

    static constexpr Range IO_RANGE{0x1F801000, 4 * KB};
    static constexpr Range MEM_CTRL1_RANGE{0x1F801000, 36};
    static constexpr Range INTERRUPT_CTRL{0x1F801070, 8};
    static constexpr Range SPU_IO_RANGE{0x1F801C00, 640};
    static constexpr Range DMA_IO_RANGE{0x1F801080, 0x80};
    static constexpr Range GPU_IO_RANGE{0x1F801810, 8};

    Range SPU_RAM_RANGE{0, 0};
    Range CDROM_RANGE{0x1F801800, 0};

    AddressedValue<std::array<u32, 9>> timers{Range{0x1F801100, 0x32}};
    AddressedValue<std::array<MemCtrl1DelayReg, 6>> memCtrl1DelayRegs{
        Range{0x1F801008, 6 * 4}};

    AddressedValue<u32> e1BaseAddr{0, Range{0x1F801000, 1}};
    AddressedValue<u32> e2BaseAddr{0, Range{0x1F801004, 1}};

    AddressedValue<u32> ramSize{0, Range{0x1F801060, 4}};

    Range E2_RANGE{0x1F802000, 8 * KB};

    // technically not used by psx.
    Range E3_RANGE{0x1FA00000, 2 * MB};

    AddressedValue<std::array<u8, 512 * KB>> biosBuffer{
        Range{0x1FC00000, 512 * KB}};
    AddressedValue<u32> cacheCtrlReg{0, Range{0xFFFE0130, 4}};
    AddressedValue<u32> comDelay{0, Range{0x1F801020, 4}};

    // TODO: make it so that this is somehow with resepct to the addresses.
    enum class DelayIdx {
        E1_DELAY,
        E3_DELAY,
        BIOS_DELAY,
        SPU_DELAY,
        CDROM_DELAY,
        E2_DELAY
    };

    u32 getMemCtrl1DelayRegIdx(const PAddress paddr);

    u32 getMemCtrl1Reg(const PAddress paddr);

    u32 getIoReg(const PAddress pAddr);

    void updateDelayRegState(const DelayIdx idx);

    void writeMemCtrl1Register(const PAddress paddr, const u32 value);

    void writeIoReg(const PAddress pAddr, const u32 value);

    PAddress getPAddr(const VAddress vAddr);

    u32 handleBufferRead(const AddressedValue<std::span<u8>> &addressedBuffer,
                         const PAddress paddr, const u8 numOfBytes);

    u32 handleRead(const PAddress paddr, const u8 numOfBytes);

    void handleBufferWrite(AddressedValue<std::span<u8>> addressedBuffer,
                           const PAddress paddr, const u8 numOfBytes,
                           const u32 value);

    void handleWrite(const PAddress paddr, const u32 value,
                     const u8 numOfBytes);

    MMap();

    // need to refactor the way shits read/stored;
    u32 load32(VAddress vAddr);

    u16 load16(VAddress vAddr);

    u8 load8(VAddress vAddr);

    void store32(const VAddress vAddr, const u32 value);

    void store16(const VAddress vAddr, const u32 value);

    void store8(const VAddress vAddr, const u32 value);
};

#endif
