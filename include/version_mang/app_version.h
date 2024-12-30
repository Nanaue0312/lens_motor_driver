#ifndef __APP_VERSION_H__
#define __APP_VERSION_H__

#include <string>
#include "stm32f3xx.h"
#include "utools.h"
#include "device_type_def.h"

class AppVersion
{
private:
public:
    AppVersion() = default;
    ~AppVersion() = default;

    /// @brief 获取版本号
    /// @return 版本号
    static uint32_t mac()
    {
        static uint32_t mac{0};
        if (mac != 0)
        {
            return mac;
        }

        volatile uint32_t uid[3]{0};
        uid[0] = *(volatile uint32_t *)(0x1FFFF7AC);
        uid[1] = *(volatile uint32_t *)(0x1FFFF7AC + 4);
        uid[2] = *(volatile uint32_t *)(0x1FFFF7AC + 8);

        uint8_t digest[16]{0};
        utcode::md5((uint8_t *)uid, 12, digest);

        mac = static_cast<uint32_t>(DeviceType::FOLLOW_FOCUS) << 24 |
              (uint32_t)digest[0] << 16 |
              (uint32_t)digest[1] << 8 |
              (uint32_t)digest[2];

        return mac;
    }

    /// @brief 获取版本号字符串
    /// @return 版本号字符串
    static std::string &mac_str()
    {
        static std::string mac_str;
        if (!mac_str.empty())
        {
            return mac_str;
        }
        for (int i = 3; i > 0; i--)
        {
            mac_str += std::to_string(static_cast<uint8_t>((mac() >> (i * 8)) & 0xFF)) + ":";
        }
        mac_str += std::to_string(mac() & 0xFF);
        return mac_str;
    }

    /// @brief 获取版本号字符串
    /// @return 版本号字符串
    static std::string mac_hex()
    {
        static std::string mac_hex;
        if (!mac_hex.empty())
        {
            return mac_hex;
        }
        for (int i = 3; i > 0; i--)
        {
            mac_hex += utcode::to_hex(static_cast<uint8_t>((mac() >> (i * 8)) & 0xFF)) + ":";
        }
        mac_hex += utcode::to_hex(static_cast<uint8_t>(mac() & 0xFF));
        return mac_hex;
    }

    /// @brief 获取版本号
    /// @return 版本号
    static uint32_t int_ver()
    {
        return utabout::int_ver();
    }

    /// @brief 获取版本号字符串
    /// @return 版本号字符串
    static std::string str_ver()
    {
        return utabout::str_ver();
    }

    /// @brief 获取版本号字符串
    /// @return 版本号字符串
    static std::string full_ver()
    {
        return utabout::full_ver();
    }
};

#endif // __APP_VERSION_H__
