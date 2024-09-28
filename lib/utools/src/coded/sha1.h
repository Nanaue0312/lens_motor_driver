#ifndef __UTOOLS_SHA1_H__
#define __UTOOLS_SHA1_H__

#include "../utools_cfg.h"

#if UTOOLS_CODE_SHA1_ENABLE == UTOOLS_ENABLE

#include <cstdint>

namespace utools::code
{
    void sha1(uint8_t *input, uint32_t inputlen, uint8_t digest[20]);

    // NOTE: if outputlen > 40: output[40] = '\0'
    void sha1_hex(uint8_t *input, uint32_t inputlen, char *output, uint32_t outputlen);
}

#endif // UTOOLS_CODE_SHA1_ENABLE

#endif // __UTOOLS_SHA1_H__
