#include "hexstr.h"

#if UTOOLS_CODE_HEXSTR_ENABLE == UTOOLS_ENABLE

namespace utools::code
{
    const std::vector<uint8_t> bstr_to_vec(const std::string &bstr)
    {
        std::vector<uint8_t> result;
        auto is_hex_char = [](char c) -> bool
        {
            return (c >= '0' && c <= '9') ||
                   (c >= 'a' && c <= 'f') ||
                   (c >= 'A' && c <= 'F');
        };

        auto hex_char_to_byte = [](char c) -> uint8_t
        {
            if (c >= '0' && c <= '9')
                return c - '0';
            if (c >= 'a' && c <= 'f')
                return c - 'a' + 10;
            if (c >= 'A' && c <= 'F')
                return c - 'A' + 10;
            return 0;
        };

        size_t i = 0;
        while (i < bstr.size())
        {
            if (bstr[i] == '\\' && i + 3 < bstr.size() && bstr[i + 1] == 'x' &&
                is_hex_char(bstr[i + 2]) && is_hex_char(bstr[i + 3]))
            {
                uint8_t byte = (hex_char_to_byte(bstr[i + 2]) << 4) | hex_char_to_byte(bstr[i + 3]);
                result.push_back(byte);
                i += 4; // 跳过 \xHH
            }
            else
            {
                // 处理打印的字符
                result.push_back(static_cast<uint8_t>(bstr[i]));
                ++i;
            }
        }
        return result;
    }
}
#endif // UTOOLS_ENABLE
