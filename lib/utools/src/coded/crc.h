#ifndef __UTOOLS_CRC_H__
#define __UTOOLS_CRC_H__

#include "../utools_cfg.h"

#if UTOOLS_CODE_CRC_ENABLE == UTOOLS_ENABLE

#include <stdint.h>
#include <stddef.h>

namespace utools::code
{
    const uint8_t crc4_itu(const void *data, size_t length);
    const uint8_t crc5_epc(const void *data, size_t length);
    const uint8_t crc5_itu(const void *data, size_t length);
    const uint8_t crc5_usb(const void *data, size_t length);

    const uint8_t crc6_itu(const void *data, size_t length);

    const uint8_t crc7_mmc(const void *data, size_t length);

    const uint8_t crc8(const void *data, size_t length);
    const uint8_t crc8_itu(const void *data, size_t length);
    const uint8_t crc8_rohc(const void *data, size_t length);
    const uint8_t crc8_maxim(const void *data, size_t length);

    const uint16_t crc16_ibm(const void *data, size_t length);
    const uint16_t crc16_maxim(const void *data, size_t length);
    const uint16_t crc16_usb(const void *data, size_t length);
    const uint16_t crc16_modbus(const void *data, size_t length);
    const uint16_t crc16_ccitt(const void *data, size_t length);
    const uint16_t crc16_ccitt_false(const void *data, size_t length);
    const uint16_t crc16_x25(const void *data, size_t length);
    const uint16_t crc16_xmodem(const void *data, size_t length);
    const uint16_t crc16_dnp(const void *data, size_t length);

    const uint32_t crc32(const void *data, size_t length);
    const uint32_t crc32_mpeg2(const void *data, size_t length);
}

#endif // UTOOLS_CODE_CRC_ENABLE

#endif // __UTOOLS_CRC_H__