#ifndef __UTOOLS_MD5_H__
#define __UTOOLS_MD5_H__

#include "../utools_cfg.h"

#if UTOOLS_CODE_MD5_ENABLE == 1

#include <vector>
#include <string>
#include <cstdint>

namespace utools::code
{
    /// @brief 计算md5值
    /// @param input 输入数据
    /// @param inputlen 输入数据长度
    /// @param digest 输出数据
    void md5(uint8_t *input, uint32_t inputlen, uint8_t digest[16]);

    /// @brief 计算md5值并转为16进制字符串
    /// @param input 输入数据
    /// @param inputlen 输入数据长度
    /// @param output 输出数据
    /// @param outputlen 输出数据长度
    /// @note if outputlen > 32: output[32] = '\0'
    void md5_hex(uint8_t *input, uint32_t inputlen, char *output, uint32_t outputlen);

    const std::vector<uint8_t> md5(std::vector<uint8_t> input);

    const std::string md5_hex(std::vector<uint8_t> input);
}

#endif // UTOOLS_CODE_MD5_ENABLE

#endif // __UTOOLS_MD5_H__
