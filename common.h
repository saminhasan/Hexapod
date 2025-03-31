#ifndef COMMON_H
#define COMMON_H

#include <FastCRC.h>

inline uint32_t calcCRC32(const uint8_t* data, uint32_t length) {
    FastCRC32 CRC32;
    return CRC32.crc32(data, length);
}

#endif // COMMON_H
