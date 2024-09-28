#ifndef __UTOOLS_UUID_HPP__
#define __UTOOLS_UUID_HPP__

#include "../utools_cfg.h"

#if UTOOLS_CODE_UUID_ENABLE == UTOOLS_ENABLE

#include <string>
#include <chrono>
#include <cstdint>

namespace utools
{
    class uuid
    {
    private:
        static uint64_t xorshift128plus(uint64_t *s)
        {
            // http://xorshift.di.unimi.it/xorshift128plus.c
            uint64_t s1 = s[0];
            const uint64_t s0 = s[1];
            s[0] = s0;
            s1 ^= s1 << 23;
            s[1] = s1 ^ s0 ^ (s1 >> 18) ^ (s0 >> 5);
            return s[1] + s0;
        }

    public:
        /// @brief 生成uuid
        /// @param format 指定uuid的样式
        /// @param uuid_bits 指定uuid长度，最小为128
        /// @param rand
        /// @return uuid字符串
        static std::string generate(const std::string &format, const unsigned int uuid_bits, unsigned int rand_seed)
        {
            unsigned int uint64bytes_len{uuid_bits >> 7};
            uint64bytes_len += uint64bytes_len & 0x01; // 转换成偶数

            // 获取随机数种子
            uint64_t seed[2];
            unsigned char *tmp_seed = reinterpret_cast<unsigned char *>(seed);
            srand(rand_seed);
            for (int i = 0; i < sizeof(uint64_t) * 2; ++i)
            {
                tmp_seed[i] = rand() & 0xFF;
            }
            // 获取随机数
            uint64_t *uint64bytes{new uint64_t[uint64bytes_len]};
            for (int i = 0; i < uint64bytes_len; ++i)
            {
                uint64bytes[i] = xorshift128plus(seed);
            }
            // 生成uuid
            std::string result;
            int temp_count{0};
            const char *chars{"0123456789abcdef"};
            unsigned int i{0}, n{0};
            unsigned char *b = reinterpret_cast<unsigned char *>(uint64bytes);
            for (auto onechar : format)
            {
                if (onechar >= '0' && onechar <= '9')
                {
                    temp_count = temp_count * 10 + onechar - '0';
                    continue;
                }
                do
                {
                    n = b[i >> 1];
                    n = (i & 1) ? (n >> 4) : (n & 0xf);

                    switch (onechar)
                    {
                    case 'x':
                    case 'X':
                        i++;
                        result.push_back(chars[n]);
                        break;
                    case 'y':
                    case 'Y':
                        result.push_back(chars[(n & 0x03) + 8]);
                        i++;
                        break;
                    default:
                        result.push_back(onechar);
                        break;
                    }
                } while (temp_count-- > 1);
                temp_count = 0;
            }
            delete[] uint64bytes;
            return result;
        }

        /// @brief 生成uuid
        /// @param format 指定uuid的样式
        /// @param uuid_bits 指定uuid长度，最小为128
        /// @return uuid字符串
        static std::string generate(const std::string &format, const unsigned int uuid_bits)
        {
            return generate(format, uuid_bits, static_cast<unsigned int>(std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count()));
        }

        /// @brief 生成uuid，默认支持128bit的编码
        /// @brief format 指定uuid的样式
        /// @return uuid字符串
        static std::string generate(const std::string &format)
        {
            return generate(format, 128);
        }

        /// @brief 生成uuid
        ///        xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx
        /// @return uuid字符串
        static std::string generate()
        {
            return generate("8x-4x-4x-4y-12x", 128);
        }
    };
}
#endif // !UTOOLS_CODE_UUID_ENABLE

#endif // !UUID_H
