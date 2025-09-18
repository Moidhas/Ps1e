#ifndef TYPES_H
#define TYPES_H
#include <cstdint>

using s8 = int8_t;
using s16 = int16_t;
using s32 = int32_t;
using s64 = int64_t;

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

constexpr u32 KB = (1 << 10);
constexpr u32 MB = (1 << 20);
constexpr u32 GB = (1 << 30);
#endif
