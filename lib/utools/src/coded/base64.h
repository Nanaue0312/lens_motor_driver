#ifndef __UTOOLS_BASE64_H__
#define __UTOOLS_BASE64_H__

#include "../utools_cfg.h"

#if UTOOLS_CODE_BASE64_ENABLE == UTOOLS_ENABLE

#define BASE64_ENCODE_OUT_SIZE(s) (((s) + 2) / 3 * 4)
#define BASE64_DECODE_OUT_SIZE(s) (((s)) / 4 * 3)

#include <cstring>
#include <string>
#include <vector>
#include <cstdint>

namespace utools::code
{

    /// @brief 编码 base64
    /// @param in 输入数据
    /// @param inlen 输入数据长度
    /// @param out 输出数据
    /// @return 编码后的数据长度
    int base64_encode(const unsigned char *in, unsigned int inlen, char *out);

    /// @brief 解码 base64
    /// @param in 输入数据
    /// @param inlen 输入数据长度
    /// @param out 输出数据
    /// @return 解码后的数据长度
    int base64_decode(const char *in, unsigned int inlen, unsigned char *out);

    const std::string base64_encode(const unsigned char *data, unsigned int len);

    const std::string base64_decode(const char *str, unsigned int len = 0);

    const std::string base64_encode(const std::vector<uint8_t> data);

    const std::string base64_decode(std::string str);
}

#endif // UTOOLS_CODE_BASE64_ENABLE

#endif // __UTOOLS_BASE64_H__
