#include "crc.h"

#if UTOOLS_CODE_CRC_ENABLE == 1

#include <tuple>

namespace utools::code
{
    namespace crc
    {
        /// @brief 支持的CRC校验方法
        enum class mode_t
        {
            CRC4_ITU = 0,
            CRC5_EPC,
            CRC5_ITU,
            CRC5_USB,
            CRC6_ITU,
            CRC7_MMC,
            CRC8,
            CRC8_ITU,
            CRC8_ROHC,
            CRC8_MAXIM,
            CRC16_IBM,
            CRC16_MAXIM,
            CRC16_USB,
            CRC16_MODBUS,
            CRC16_CCITT,
            CRC16_CCITT_FALSE,
            CRC16_X25,
            CRC16_XMODEM,
            CRC16_DNP,
            CRC32,
            CRC32_MPEG2
        };

        /// @brief crc类型相关的描述参数
        template <typename RESULT_TYPE>
        struct crc_type_t
        {
            using result_type = RESULT_TYPE; // 返回值类型

            mode_t mode; // crc模式
            uint8_t width;
            uint32_t poly;
            uint32_t init;
            bool refIn;
            bool refOut;
            uint32_t xorOut;

            crc_type_t(mode_t mode, uint8_t width, uint32_t poly, uint32_t init, bool refIn, bool refOut, uint32_t xorOut)
                : mode(mode), width(width), poly(poly), init(init), refIn(refIn), refOut(refOut), xorOut(xorOut) {}
        };

        /// @brief crc类型
        static auto crc_mode_tpl{
            std::make_tuple(
                crc_type_t<uint8_t>{mode_t::CRC4_ITU, 4, 0x03, 0x00, true, true, 0x00},
                crc_type_t<uint8_t>{mode_t::CRC5_EPC, 5, 0x09, 0x09, false, false, 0x00},
                crc_type_t<uint8_t>{mode_t::CRC5_ITU, 5, 0x15, 0x00, true, true, 0x00},
                crc_type_t<uint8_t>{mode_t::CRC5_USB, 5, 0x05, 0x1f, true, true, 0x1f},
                crc_type_t<uint8_t>{mode_t::CRC6_ITU, 6, 0x03, 0x00, true, true, 0x00},
                crc_type_t<uint8_t>{mode_t::CRC7_MMC, 7, 0x09, 0x00, false, false, 0x00},
                crc_type_t<uint8_t>{mode_t::CRC8, 8, 0x07, 0x00, false, false, 0x00},
                crc_type_t<uint8_t>{mode_t::CRC8_ITU, 8, 0x07, 0x00, false, false, 0x55},
                crc_type_t<uint8_t>{mode_t::CRC8_ROHC, 8, 0x07, 0xff, true, true, 0x00},
                crc_type_t<uint8_t>{mode_t::CRC8_MAXIM, 8, 0x31, 0x00, true, true, 0x00},
                crc_type_t<uint16_t>{mode_t::CRC16_IBM, 16, 0x8005, 0x0000, true, true, 0x0000},
                crc_type_t<uint16_t>{mode_t::CRC16_MAXIM, 16, 0x8005, 0x0000, true, true, 0xffff},
                crc_type_t<uint16_t>{mode_t::CRC16_USB, 16, 0x8005, 0xffff, true, true, 0xffff},
                crc_type_t<uint16_t>{mode_t::CRC16_MODBUS, 16, 0x8005, 0xffff, true, true, 0x0000},
                crc_type_t<uint16_t>{mode_t::CRC16_CCITT, 16, 0x1021, 0x0000, true, true, 0x0000},
                crc_type_t<uint16_t>{mode_t::CRC16_CCITT_FALSE, 16, 0x1021, 0xffff, false, false, 0x0000},
                crc_type_t<uint16_t>{mode_t::CRC16_X25, 16, 0x1021, 0xffff, true, true, 0xffff},
                crc_type_t<uint16_t>{mode_t::CRC16_XMODEM, 16, 0x1021, 0x0000, false, false, 0x0000},
                crc_type_t<uint16_t>{mode_t::CRC16_DNP, 16, 0x3D65, 0x0000, true, true, 0xffff},
                crc_type_t<uint32_t>{mode_t::CRC32, 32, 0x04c11db7, 0xffffffff, true, true, 0xffffffff},
                crc_type_t<uint32_t>{mode_t::CRC32_MPEG2, 32, 0x4c11db7, 0xffffffff, false, false, 0x00000000})};

        struct reflected_mode_t
        {
            enum
            {
                REF_4BIT = 4,
                REF_5BIT = 5,
                REF_6BIT = 6,
                REF_7BIT = 7,
                REF_8BIT = 8,
                REF_16BIT = 16,
                REF_32BIT = 32
            };
        };

        inline uint32_t __reflected_data(uint32_t data, uint8_t ref_mode)
        {
            data = ((data & 0xffff0000) >> 16) | ((data & 0x0000ffff) << 16);
            data = ((data & 0xff00ff00) >> 8) | ((data & 0x00ff00ff) << 8);
            data = ((data & 0xf0f0f0f0) >> 4) | ((data & 0x0f0f0f0f) << 4);
            data = ((data & 0xcccccccc) >> 2) | ((data & 0x33333333) << 2);
            data = ((data & 0xaaaaaaaa) >> 1) | ((data & 0x55555555) << 1);

            switch (ref_mode)
            {
            case reflected_mode_t::REF_32BIT:
                return data;
            case reflected_mode_t::REF_16BIT:
                return (data >> 16) & 0xffff;
            case reflected_mode_t::REF_8BIT:
                return (data >> 24) & 0xff;
            case reflected_mode_t::REF_7BIT:
                return (data >> 25) & 0x7f;
            case reflected_mode_t::REF_6BIT:
                return (data >> 26) & 0x3f;
            case reflected_mode_t::REF_5BIT:
                return (data >> 27) & 0x1f;
            case reflected_mode_t::REF_4BIT:
                return (data >> 28) & 0x0f;
            }
            return 0;
        }

        static uint8_t check_crc4(uint8_t poly, uint8_t init, bool refIn, bool refOut, uint8_t xorOut,
                                  const uint8_t *buffer, uint32_t length)
        {
            uint8_t i;
            uint8_t crc;

            if (refIn == true)
            {
                crc = init;
                poly = __reflected_data(poly, reflected_mode_t::REF_4BIT);

                while (length--)
                {
                    crc ^= *buffer++;
                    for (i = 0; i < 8; i++)
                    {
                        if (crc & 0x01)
                        {
                            crc >>= 1;
                            crc ^= poly;
                        }
                        else
                        {
                            crc >>= 1;
                        }
                    }
                }

                return crc ^ xorOut;
            }
            else
            {
                crc = init << 4;
                poly <<= 4;

                while (length--)
                {
                    crc ^= *buffer++;
                    for (i = 0; i < 8; i++)
                    {
                        if (crc & 0x80)
                        {
                            crc <<= 1;
                            crc ^= poly;
                        }
                        else
                        {
                            crc <<= 1;
                        }
                    }
                }

                return (crc >> 4) ^ xorOut;
            }
        }

        static uint8_t check_crc5(uint8_t poly, uint8_t init, bool refIn, bool refOut, uint8_t xorOut,
                                  const uint8_t *buffer, uint32_t length)
        {
            uint8_t i;
            uint8_t crc;

            if (refIn == true)
            {
                crc = init;
                poly = __reflected_data(poly, reflected_mode_t::REF_5BIT);

                while (length--)
                {
                    crc ^= *buffer++;
                    for (i = 0; i < 8; i++)
                    {
                        if (crc & 0x01)
                        {
                            crc >>= 1;
                            crc ^= poly;
                        }
                        else
                        {
                            crc >>= 1;
                        }
                    }
                }

                return crc ^ xorOut;
            }
            else
            {
                crc = init << 3;
                poly <<= 3;

                while (length--)
                {
                    crc ^= *buffer++;
                    for (i = 0; i < 8; i++)
                    {
                        if (crc & 0x80)
                        {
                            crc <<= 1;
                            crc ^= poly;
                        }
                        else
                        {
                            crc <<= 1;
                        }
                    }
                }

                return (crc >> 3) ^ xorOut;
            }
        }

        static uint8_t check_crc6(uint8_t poly, uint8_t init, bool refIn, bool refOut, uint8_t xorOut,
                                  const uint8_t *buffer, uint32_t length)
        {
            uint8_t i;
            uint8_t crc;

            if (refIn == true)
            {
                crc = init;
                poly = __reflected_data(poly, reflected_mode_t::REF_6BIT);

                while (length--)
                {
                    crc ^= *buffer++;
                    for (i = 0; i < 8; i++)
                    {
                        if (crc & 0x01)
                        {
                            crc >>= 1;
                            crc ^= poly;
                        }
                        else
                        {
                            crc >>= 1;
                        }
                    }
                }

                return crc ^ xorOut;
            }
            else
            {
                crc = init << 2;
                poly <<= 2;

                while (length--)
                {
                    crc ^= *buffer++;
                    for (i = 0; i < 8; i++)
                    {
                        if (crc & 0x80)
                        {
                            crc <<= 1;
                            crc ^= poly;
                        }
                        else
                        {
                            crc <<= 1;
                        }
                    }
                }

                return (crc >> 2) ^ xorOut;
            }
        }

        static uint8_t check_crc7(uint8_t poly, uint8_t init, bool refIn, bool refOut, uint8_t xorOut,
                                  const uint8_t *buffer, uint32_t length)
        {
            uint8_t i;
            uint8_t crc;

            if (refIn == true)
            {
                crc = init;
                poly = __reflected_data(poly, reflected_mode_t::REF_7BIT);

                while (length--)
                {
                    crc ^= *buffer++;
                    for (i = 0; i < 8; i++)
                    {
                        if (crc & 0x01)
                        {
                            crc >>= 1;
                            crc ^= poly;
                        }
                        else
                        {
                            crc >>= 1;
                        }
                    }
                }

                return crc ^ xorOut;
            }
            else
            {
                crc = init << 1;
                poly <<= 1;

                while (length--)
                {
                    crc ^= *buffer++;
                    for (i = 0; i < 8; i++)
                    {
                        if (crc & 0x80)
                        {
                            crc <<= 1;
                            crc ^= poly;
                        }
                        else
                        {
                            crc <<= 1;
                        }
                    }
                }

                return (crc >> 1) ^ xorOut;
            }
        }

        static uint8_t check_crc8(uint8_t poly, uint8_t init, bool refIn, bool refOut, uint8_t xorOut,
                                  const uint8_t *buffer, uint32_t length)
        {
            uint32_t i = 0;
            uint8_t crc = init;

            while (length--)
            {
                if (refIn == true)
                {
                    crc ^= __reflected_data(*buffer++, reflected_mode_t::REF_8BIT);
                }
                else
                {
                    crc ^= *buffer++;
                }

                for (i = 0; i < 8; i++)
                {
                    if (crc & 0x80)
                    {
                        crc <<= 1;
                        crc ^= poly;
                    }
                    else
                    {
                        crc <<= 1;
                    }
                }
            }

            if (refOut == true)
            {
                crc = __reflected_data(crc, reflected_mode_t::REF_8BIT);
            }

            return crc ^ xorOut;
        }

        static uint16_t check_crc16(uint16_t poly, uint16_t init, bool refIn, bool refOut, uint16_t xorOut,
                                    const uint8_t *buffer, uint32_t length)
        {
            uint32_t i = 0;
            uint16_t crc = init;

            while (length--)
            {
                if (refIn == true)
                {
                    crc ^= __reflected_data(*buffer++, reflected_mode_t::REF_8BIT) << 8;
                }
                else
                {
                    crc ^= (*buffer++) << 8;
                }

                for (i = 0; i < 8; i++)
                {
                    if (crc & 0x8000)
                    {
                        crc <<= 1;
                        crc ^= poly;
                    }
                    else
                    {
                        crc <<= 1;
                    }
                }
            }

            if (refOut == true)
            {
                crc = __reflected_data(crc, reflected_mode_t::REF_16BIT);
            }

            return crc ^ xorOut;
        }

        static uint32_t check_crc32(uint32_t poly, uint32_t init, bool refIn, bool refOut, uint32_t xorOut,
                                    const uint8_t *buffer, uint32_t length)
        {
            uint32_t i = 0;
            uint32_t crc = init;

            while (length--)
            {
                if (refIn == true)
                {
                    crc ^= __reflected_data(*buffer++, reflected_mode_t::REF_8BIT) << 24;
                }
                else
                {
                    crc ^= (*buffer++) << 24;
                }

                for (i = 0; i < 8; i++)
                {
                    if (crc & 0x80000000)
                    {
                        crc <<= 1;
                        crc ^= poly;
                    }
                    else
                    {
                        crc <<= 1;
                    }
                }
            }

            if (refOut == true)
            {
                crc = __reflected_data(crc, reflected_mode_t::REF_32BIT);
            }

            return crc ^ xorOut;
        }
    }

    const uint8_t crc4_itu(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC4_ITU)>(crc::crc_mode_tpl)};
        return crc::check_crc4(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint8_t crc5_epc(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC5_EPC)>(crc::crc_mode_tpl)};
        return crc::check_crc5(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint8_t crc5_itu(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC5_ITU)>(crc::crc_mode_tpl)};
        return crc::check_crc5(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint8_t crc5_usb(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC5_USB)>(crc::crc_mode_tpl)};
        return crc::check_crc5(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint8_t crc6_itu(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC6_ITU)>(crc::crc_mode_tpl)};
        return crc::check_crc6(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint8_t crc7_mmc(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC7_MMC)>(crc::crc_mode_tpl)};
        return crc::check_crc7(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint8_t crc8(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC8)>(crc::crc_mode_tpl)};
        return crc::check_crc8(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint8_t crc8_itu(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC8_ITU)>(crc::crc_mode_tpl)};
        return crc::check_crc8(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint8_t crc8_rohc(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC8_ROHC)>(crc::crc_mode_tpl)};
        return crc::check_crc8(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint8_t crc8_maxim(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC8_MAXIM)>(crc::crc_mode_tpl)};
        return crc::check_crc8(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint16_t crc16_ibm(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC16_IBM)>(crc::crc_mode_tpl)};
        return crc::check_crc16(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint16_t crc16_maxim(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC16_MAXIM)>(crc::crc_mode_tpl)};
        return crc::check_crc16(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint16_t crc16_usb(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC16_USB)>(crc::crc_mode_tpl)};
        return crc::check_crc16(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint16_t crc16_modbus(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC16_MODBUS)>(crc::crc_mode_tpl)};
        return crc::check_crc16(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint16_t crc16_ccitt(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC16_CCITT)>(crc::crc_mode_tpl)};
        return crc::check_crc16(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint16_t crc16_ccitt_false(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC16_CCITT_FALSE)>(crc::crc_mode_tpl)};
        return crc::check_crc16(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint16_t crc16_x25(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC16_X25)>(crc::crc_mode_tpl)};
        return crc::check_crc16(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint16_t crc16_xmodem(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC16_XMODEM)>(crc::crc_mode_tpl)};
        return crc::check_crc16(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint16_t crc16_dnp(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC16_DNP)>(crc::crc_mode_tpl)};
        return crc::check_crc16(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint32_t crc32(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC32)>(crc::crc_mode_tpl)};
        return crc::check_crc32(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }

    const uint32_t crc32_mpeg2(const void *data, size_t length)
    {
        static const auto cfg{std::get<static_cast<int32_t>(crc::mode_t::CRC32_MPEG2)>(crc::crc_mode_tpl)};
        return crc::check_crc32(cfg.poly, cfg.init, cfg.refIn, cfg.refOut, cfg.xorOut, static_cast<const uint8_t *>(data), length);
    }
}

#endif // UTOOLS_CODE_CRC_ENABLE