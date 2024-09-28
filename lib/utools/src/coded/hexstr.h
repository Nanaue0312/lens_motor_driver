#ifndef __HEXSTR_HPP__
#define __HEXSTR_HPP__

#include "../utools_cfg.h"

#if UTOOLS_CODE_HEXSTR_ENABLE == UTOOLS_ENABLE

#include <string>
#include <vector>
#include <array>
#include <cstdint>
#include <cctype>

namespace utools::code
{
    /// @brief 转换成byte字符串
    /// @tparam _UPPERCASE 是否转换为大写
    /// @tparam _EN_DELIMITER 是否添加分割符
    /// @param data_ptr 输入数据
    /// @param size 数据长度
    /// @param delimiter 分割符
    /// @return 形如“aa bb cc 00 01 ff”的数据
    template <bool _UPPERCASE = false, bool _EN_DELIMITER = true>
    const std::string to_hex(const void *data_ptr, const size_t size, const std::string &delimiter = " ")
    {
        if (data_ptr == nullptr || size == 0)
        {
            return "";
        }

        static const char hexme_uppercase[]{"0123456789ABCDEF"};
        static const char hexme_lowercase[]{"0123456789abcdef"};
        auto &hexme = _UPPERCASE ? hexme_uppercase : hexme_lowercase;
        auto str_ptr = static_cast<const unsigned char *>(data_ptr);

        std::string result;
        result.reserve(size * 2 + (_EN_DELIMITER ? ((size - 1) * delimiter.length()) : 0));
        size_t pos{0};
        for (; pos < size - 1; ++pos)
        {
            result.push_back(hexme[(str_ptr[pos] & 0xF0) >> 4]);
            result.push_back(hexme[str_ptr[pos] & 0x0F]);
            if (_EN_DELIMITER)
            {
                result += delimiter;
            }
        }
        result.push_back(hexme[(str_ptr[pos] & 0xF0) >> 4]);
        result.push_back(hexme[str_ptr[pos] & 0x0F]);
        return result;
    }

    template <typename _ELE_TYP, bool _UPPERCASE = false, bool _EN_DELIMITER = true,
              typename = std::enable_if<std::is_integral<_ELE_TYP>::value || std::is_floating_point<_ELE_TYP>::value>>
    const std::string to_hex(const _ELE_TYP num, const std::string &delimiter = " ")
    {
        return to_hex<_UPPERCASE, _EN_DELIMITER>(reinterpret_cast<const void *>(&num), sizeof(num), delimiter);
    }

    template <typename _ELE_TYP, bool _UPPERCASE = false, bool _EN_DELIMITER = true>
    const std::string to_hex(const std::vector<_ELE_TYP> &buffer, const std::string &delimiter = " ")
    {
        return to_hex<_UPPERCASE, _EN_DELIMITER>(buffer.data(), buffer.size() * sizeof(_ELE_TYP), delimiter);
    }

    template <std::size_t _ELE_CNT, typename _ELE_TYP, bool _UPPERCASE = false, bool _EN_DELIMITER = true>
    const std::string to_hex(const std::array<_ELE_TYP, _ELE_CNT> &buffer, const std::string &delimiter = " ")
    {
        return to_hex<_UPPERCASE, _EN_DELIMITER>(buffer.data(), buffer.size() * sizeof(_ELE_TYP), delimiter);
    }

    /// @brief 将十六进制字符串转换为字节数组
    /// @tparam _HAS_DELIMITER 是否包含分隔符
    /// @param hex_str 十六进制字符串
    /// @param delimiter 分隔符
    /// @return 字节数组
    /// @note 十六进制字符串中，每个字节用空格分隔
    /// @note 如果十六进制字符串中包含分隔符，则需要指定分隔符
    /// @note std::string hex_str_with_delimiter = "4A 6F 68 6E 20 44 6F 65";
    //  @note std::string hex_str_without_delimiter = "4A6F686E20446F65";
    //  @note auto vec_with_delimiter = hex_to_vec<true>(hex_str_with_delimiter);
    //  @note auto vec_without_delimiter = hex_to_vec<false>(hex_str_without_delimiter);
    template <bool _HAS_DELIMITER = true>
    const std::vector<uint8_t> hex_to_vec(const std::string &hex_str, const std::string &delimiter = " ")
    {
        std::vector<uint8_t> result;

        auto hex_char_to_int = [](char c) -> int
        {
            if (c >= '0' && c <= '9')
                return c - '0';
            if (c >= 'a' && c <= 'f')
                return c - 'a' + 10;
            if (c >= 'A' && c <= 'F')
                return c - 'A' + 10;
            return -1;
        };

        // 估算需要分配的空间大小
        size_t estimated_size = hex_str.length() / 2;
        if (_HAS_DELIMITER)
        {
            estimated_size = hex_str.length() / (2 + delimiter.length());
        }
        result.reserve(estimated_size); // 预先分配空间

#if __cplusplus >= 201703L
        if constexpr (_HAS_DELIMITER)
#else
        if (_HAS_DELIMITER)
#endif
        {
            // 有分隔符的情况
            size_t start = 0;
            size_t end = hex_str.find(delimiter);

            while (end != std::string::npos)
            {
                std::string byteString = hex_str.substr(start, end - start);
                if (byteString.length() == 2)
                {
                    int high = hex_char_to_int(byteString[0]);
                    int low = hex_char_to_int(byteString[1]);
                    if (high != -1 && low != -1)
                    {
                        result.push_back(static_cast<uint8_t>((high << 4) | low));
                    }
                }
                start = end + delimiter.length();
                end = hex_str.find(delimiter, start);
            }

            // 处理最后一个分隔符后的部分
            std::string lastByteString = hex_str.substr(start);
            if (lastByteString.length() == 2)
            {
                int high = hex_char_to_int(lastByteString[0]);
                int low = hex_char_to_int(lastByteString[1]);
                if (high != -1 && low != -1)
                {
                    result.push_back(static_cast<uint8_t>((high << 4) | low));
                }
            }
        }
        else
        {
            // 没有分隔符的情况
            for (size_t i = 0; i < hex_str.length(); i += 2)
            {
                std::string byteString = hex_str.substr(i, 2);
                int high = hex_char_to_int(byteString[0]);
                int low = hex_char_to_int(byteString[1]);
                if (high != -1 && low != -1)
                {
                    result.push_back(static_cast<uint8_t>((high << 4) | low));
                }
            }
        }

        return result;
    }

    /// @brief 转换成byte字符串
    /// @tparam _EN_CRTL_SYMBOLS 是否将可见字符（如回车、空格等不显示）以16进制形式显示
    /// @tparam _UPPERCASE 是否转换为大写
    /// @param data_ptr 输入数据
    /// @param size 数据长度
    /// @return 形如“abc\x00\x01”的数据
    template <bool _EN_CRTL_SYMBOLS = true, bool _UPPERCASE = false>
    const std::string to_bstr(const void *data_ptr, const size_t size)
    {
        static const char hexme_uppercase[]{"0123456789ABCDEF"};
        static const char hexme_lowercase[]{"0123456789abcdef"};
        auto &hexme = _UPPERCASE ? hexme_uppercase : hexme_lowercase;
        auto str_ptr = static_cast<const unsigned char *>(data_ptr);
#if __cplusplus >= 201703L
        constexpr auto is_to_bstr{_EN_CRTL_SYMBOLS ? isprint : [](int c) -> int
                                  { return (unsigned)(c) < 0x80; }};
#else
        auto is_to_bstr{_EN_CRTL_SYMBOLS ? isprint : [](int c) -> int
                        { return (unsigned)(c) < 0x80; }};
#endif
        std::string result;
        result.reserve(size * 2);

        for (size_t pos = 0; pos < size; ++pos)
        {
            if (is_to_bstr(str_ptr[pos]))
            {
                result.push_back(str_ptr[pos]);
            }
            else
            {
                result.push_back('\\');
                result.push_back('x');
                result.push_back(hexme[(str_ptr[pos] & 0xF0) >> 4]);
                result.push_back(hexme[str_ptr[pos] & 0x0F]);
            }
        }
        return result;
    }

    template <typename _ELE_TYP, bool _EN_CRTL_SYMBOLS = true, bool _UPPERCASE = false>
    const std::string to_bstr(const std::vector<_ELE_TYP> &buffer)
    {
        return to_bstr<_EN_CRTL_SYMBOLS, _UPPERCASE>(buffer.data(), buffer.size() * sizeof(_ELE_TYP));
    }

    template <std::size_t _ELE_CNT, typename _ELE_TYP, bool _EN_CRTL_SYMBOLS = true, bool _UPPERCASE = false>
    const std::string to_bstr(const std::array<_ELE_TYP, _ELE_CNT> &buffer)
    {
        return to_bstr<_EN_CRTL_SYMBOLS, _UPPERCASE>(buffer.data(), buffer.size() * sizeof(_ELE_TYP));
    }

    /// @brief 转换成byte字符串转换成vector类型的数据
    /// @param bstr 需要转换的数据
    /// @return 形如“abc\x00\x01”的数据
    const std::vector<uint8_t> bstr_to_vec(const std::string &bstr);
}

#endif // UTOOLS_CODE_HEXSTR_ENABLE

#endif // __HEXSTR_HPP__
