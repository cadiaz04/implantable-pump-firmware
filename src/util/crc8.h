#pragma once
// CRC-8 as used by Sensirion sensors: polynomial 0x31 (x^8 + x^5 + x^4 + 1),
// init 0xFF, no input/output reflection, no final XOR.
#include <cstddef>
#include <cstdint>

uint8_t crc8Sensirion(const uint8_t* data, size_t len);
