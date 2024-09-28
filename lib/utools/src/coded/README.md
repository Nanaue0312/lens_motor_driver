## 目录结构

```
.
├── hexstr.h        HEX编解码
├── base64.h        BASE64编解码
├── md5.h           MD5数字摘要
├── sha1.h          SHA1安全散列算法
├── bit_packer.h    bit打包器
└── byte_packer.h   byte打包器
```


BitPacker使用方法
```C++

#include "utools.h"

int main() {
    using MyPacker = utools::code::BitPacker<uint32_t, 8, 8, 4, 8, 4>;
    uint32_t packed_data = MyPacker::pack(0x12, 0x34, 0x5, 0x67, 0x8);
    std::cout << std::hex << packed_data << std::endl;  // 输出打包后的数据
    return 0;
}

int main() {
    using MyPacker = BitPacker<uint32_t, 8, 8, 8, 8>;  // Define a BitPacker with 4 fields each 8 bits
    using MyUnpacker = BitUnpacker<uint32_t, 8, 8, 8, 8>;  // Define a corresponding BitUnpacker

    // Pack some values
    uint32_t packed_value = MyPacker::pack(0x12, 0x34, 0x56, 0x78);

    std::cout << "Packed Value: " << std::hex << packed_value << std::endl;

    // Unpack the packed value
    auto unpacked_values = MyUnpacker::unpack(packed_value);

    std::cout << "Unpacked Values: ";
    std::apply([](auto&&... args) { ((std::cout << std::hex << args << " "), ...); }, unpacked_values);
    std::cout << std::endl;

    return 0;
}

```



BytePacker使用方法
```C++
#include <iostream>
#include "utools.h"

int main() {
    using MyPacker = utools::code::BytePacker<uint8_t, uint16_t, float, uint32_t, uint8_t>;  // Define a BytePacker for different types including float
    using MyUnpacker = utools::code::ByteUnpacker<uint8_t, uint16_t, float, uint32_t, uint8_t>;  // Define a corresponding ByteUnpacker

    // Pack some values
    auto packed_data = MyPacker::pack(0x12, 0x3456, 3.14f, 0x789ABCDE, 0xFF);

    // Print packed data
    std::cout << "Packed Data: ";
    for (auto byte : packed_data) {
        std::cout << std::hex << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;

    // Unpack the packed data
    auto unpacked_data = MyUnpacker::unpack(packed_data);

    // Print unpacked data
    std::cout << "Unpacked Data: ";
    std::apply([](auto&&... args) { ((std::cout << args << " "), ...); }, unpacked_data);
    std::cout << std::endl;

    return 0;
}
```