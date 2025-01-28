#include "common.h"

uint16_t calc_crc16(uint8_t *data, uint32_t length) {

    uint16_t crc = 0xFFFF;

    for (uint32_t i = 0; i < length; i++) {

        crc ^= (uint16_t)data[i];
        for (uint8_t bit = 0; bit < 8; bit++)  {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0x8005;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}