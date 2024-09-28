# utools库的配置文件

## 1 介绍
提供给MCU等小行系统或裸机系统使用

## 2 使用方法

### 2.1 配置文件
utools库的配置文件为utools_cfg.h，该文件位于utools/src目录下，该文件中定义了utools库中各个功能模块的开关宏，用户可以根据自己的需求，打开或关闭相应的功能模块。

配置文件的内容如下：
```c++
#ifndef __UTOOLS_CFG_TEMPLATE_H__
#define __UTOOLS_CFG_TEMPLATE_H__

// 如果用户自己定义了相关的配置文件，直接引入
#ifdef UTOOLS_USER_CONFIG_H
#include UTOOLS_USER_CONFIG_H
#endif // UTOOLS_USER_CONFIG_H

/// ... 其它内容 ...

#endif // __UTOOLS_CFG_TEMPLATE_H__
```

#### 2.1.2 生成自己的配置文件
用户功能配置文件，用户可以根据自己的需求修改此文件，以实现不同的功能。
使用时可以创建一个自己的文件，如utools_usr_cfg.h，内容如下所示：

```c++
#ifndef __UTOOLS_USR_CFG_H__
#define __UTOOLS_USR_CFG_H__

#define UTOOLS_LOGGER_ENABLE 1
#define UTOOLS_COLLAB_SYNC_QUEUE_ENABLE 1
#define UTOOLS_COLLAB_SYNC_PARTNER_ENABLE 1

#endif // __UTOOLS_USR_CFG_H__
```

#### 2.1.3 引入utools库
在需要使用utools库的文件中，需要包含utools.h头文件，如下所示：
```c++
#include "utools.h"
```

#### 2.1.4 编译配置
编译时，如用户需要指定自己的功能配置文件，可以直接在编译时指定其UTOOLS_USER_CONFIG_H所在的位置，
如下所示：
```shell
-DUTOOLS_USER_CONFIG_H=\"../../../include/utools_usr_cfg.h\"	; 文件所在位置，不相对于utools/src目录的路径
```

### 2.2 常见问处理

#### 2.2.1 找不到usleep函数

如果在编译时提示找不到usleep函数，需要重新实现相关函数
在freertos的开发环境中，可以通过配置下面的宏实现相碰功能#

```c++
#define UTOOLS_FREERTOS_STD_SLEEP_FUNC_ENABLE
```

## 3 功能模块

### 3.1 日志模块 (utools::logger)

#### 3.1.1 功能介绍
日志模块提供了一种简单、灵活的日志记录方式，可以方便地记录程序的运行状态和调试信息。用户可以根据自己的需求，选择不同的日志级别和输出方式，以满足不同的需求。

#### 3.1.2 使用方法


### 3.2 编码模块 (utools::code)

### 3.3 假数据生成模块 (utools::fake_data)

### 3.4 时间模块 (utools::time)

### 3.5 池化模块 (utools::pool)

### 3.6 缓存管理模块 (utools::mem)

#### 3.6.1 功能介绍

#### 3.6.2 使用方法

#### 3.6.3 任意类型缓存（utools::mem::AnyMap）

```c++
#include "utools.h"

int main() {
    utools::mem::AnyMap store;

    // 使用不同类型的键存储不同类型的值
    store.set(1, "integer key");
    store.set(std::string("string key"), 3.14);
    store.set(2.5, std::string("double key"));

    try {
        // 检索和打印值
        std::string strVal1 = store.get<std::string>(1);
        double doubleVal = store.get<double>("string key");
        std::string strVal2 = store.get<std::string>(2.5);

        utools::logger_info("Value for integer key: ", strVal1);
        utools::logger_info("Value for string key: ", doubleVal);
        utools::logger_info("Value for double key: ", strVal2);

        // 判断是否存在数据
        utools::logger_info("key in stort:", store.has_key(1));

        // 删除值
        store.remove(1);

        // 尝试检索已删除的值
        strVal1 = store.get<std::string>(1); // 引发一个异常
        auto strVal2 = store.get<std::string>(1, "hello"); // 返回"hello"
    } catch (const std::exception& e) {
        utools::logger_error(e.what());
    }

    return 0;
}
```

### 3.7 任务协作模块 (utools::collab)

#### 3.7.1 功能介绍

#### 3.7.2 使用方法

#### 3.7.3 任务分配器（utools::collab::dispatcher）

任务分析器主要简化不同类型参数，需要调用特定的参数进行处理的需求，设计了任务分配器。
通过注册key和调用函数对应的类型，实现不同类型的参数调用不同的处理函数。
使用方法如下：

```c++
#include "utools.h"

class CommandPrase
{
public:
    void __default_handler(const uint8_t &key, std::vector<uint8_t> &frm_data)
    {
        UTINFO("default handler, data:", utcode::to_hex(frm_data));
    }

    void __focus_control(const uint8_t &key, std::vector<uint8_t> &frm_data)
    {
        uint16_t focus = *(uint16_t *)frm_data.data();
    }

    void __zoom_control(const uint8_t &key, std::vector<uint8_t> &frm_data)
    {
        uint16_t zoom = *(uint16_t *)frm_data.data();
    }

    void __focus_zoom_control(const uint8_t &key, std::vector<uint8_t> &frm_data)
    {
        uint16_t focus = *(uint16_t *)frm_data.data();
        uint16_t zoom = *(uint16_t *)(frm_data.data() + 2);
    }

    void __uv_gain_control(const uint8_t &key, std::vector<uint8_t> &frm_data)
    {
        uint16_t uv_gain = *(uint16_t *)frm_data.data();
    }

    void __request(const uint8_t &key, std::vector<uint8_t> &frm_data)
    {
        uint16_t request = *(uint16_t *)frm_data.data();
    }
};

int main()
{
    CommandPrase command_prase;
    utcollab::Dispatcher<uint8_t, void(const uint8_t &, std::vector<uint8_t> &)> __dispatcher;

    // 添加任务
    __dispatcher.map(0, &CommandPrase::__default_handler, &command_prase)
            .map(1, &CommandPrase::__focus_control,  &command_prase)
            .map(2, &CommandPrase::__zoom_control,  &command_prase)
            .map(3, &CommandPrase::__focus_zoom_control,  &command_prase)
            .map(4, &CommandPrase::__uv_gain_control,  &command_prase)
            .map(10, &CommandPrase::__request,  &command_prase);

    while(true)
    {
        // 模拟任务
        uint8_t key = 0;
        std::vector<uint8_t> frm_data = {0x01, 0x02, 0x03, 0x04, 0x05};
        __dispatcher.dispatch(key, frm_data);
    }
}

```


### 3.8 数学函数功能模块 (utools::math)

#### 3.8.1 误差判断函数（utools::math::is_close）

用于判断两个浮点数是否在误差范围内相等。本模块目前提供了三种不同的实现，分别是

```c++
// 同时参考相对误差和绝对误差，优化绝对误差判断
utools::math::is_close(a, b, ref_tol, abs_tol);

// 仅参考绝对误差判断
utools::math::is_close_abs(a, b, abs_tol);

// 仅参考相对误差判断
utools::math::is_close_ref(a, b, ref_tol);

```
- a ：必需，数字。如果 x 不是一个数字，返回 TypeError。如果值为 0 或负数，则返回 ValueError。
- b ：必需，数字。如果 x 不是一个数字，返回 TypeError。如果值为 0 或负数，则返回 ValueError。
- rel_tol：是相对容差，它是 a 和 b 之间允许的最大差值，相对于 a 或 b 的较大绝对值。例如，要设置5％的容差，请传递 rel_tol=0.05 。默认容差为系统的最小精度值，确保两个值在大约9位十进制数字内相同。 rel_tol 必须大于零。
- abs_tol：是最小绝对容差，对于接近零的比较很有用。 abs_tol 必须至少为零。

使用方法如下：

```cpp
#include "utools.h"

int main() {
    double a = 0.15 + 0.15;
    double b = 0.1 + 0.2;
    int x = 10;
    int y = 10;

    // 比较浮点数
    utools::logger_info("is_close(a, b): ", utools::math::is_close(a, b));
    // 比较整数
    utools::logger_info("is_close(x, y): ", utools::math::is_close(x, y));
    // 比较不同类型的数字
    utools::logger_info("is_close(x, b): ", utools::math::is_close(x, b));

    return 0;
}

```


### 3.9 数字滤波器模块 (utools::fliter)

### 3.10 字符串处理模块 (utools::fmt)

### 3.11 关于模块 (utools::about)

#### 3.11.1 功能介绍
关于模块提供了一种简单、灵活的方式来展示应用程序的版本信息、版权信息、作者信息等。用户可以根据自己的需求，选择不同的展示方式，以满足不同的需求。

#### 3.11.2 使用方法

##### 1) 开启相关功能
配置文件，在utools的自定义文件（utools_usr_cfg.h）中打开相应的开关，配置内容如：

```c++
#define UTOOLS_ABOUT_ENABLE 1   // 打开about功能
#define UTOOLS_ABOUT_INCLUDE "../../include/about.h"  // 自定义用户自己的配置文件目录，如果需要时，不是必须的
```

##### 2) 使用宏定义参数
如果使用了```UTOOLS_ABOUT_INCLUDE```宏，需要在```about.h```文件中进行相关的配置，配置内容如下：

```c++
#ifndef __UTOOLS_ABOUT_H__
#define __UTOOLS_ABOUT_H__

// 产品名称
#define UTOOLS_VERSION_MANG_NAME "utools"
// CPU型号
#define UTOOLS_VERSION_MANG_CPU "ESP32-S3"
// CPU架构
#define UTOOLS_VERSION_MANG_ARCH "RISCV"
// ARM大小
#define UTOOLS_VERSION_MANG_ARM "320kB"
// 操作系统
#define UTOOLS_VERSION_MANG_OS "FreeRTOS V8.4.3"
// 编译器
#define UTOOLS_VERSION_MANG_COMPILER "ESP-IDF"
// 主版本号
#define UTOOLS_VERSION_MANG_MAJOR 1
// 次版本号
#define UTOOLS_VERSION_MANG_MINOR 0
// 修订号
#define UTOOLS_VERSION_MANG_PATCH 0
// 描述信息
#define UTOOLS_VERSION_MANG_DESC "Wooden Heart Bird"
// 作者
#define UTOOLS_VERSION_MANG_AUTHOR "Wooden Heart Bird"
// 公司
#define UTOOLS_VERSION_MANG_COMPANY "Wooden Heart Bird"
// 网址
#define UTOOLS_VERSION_MANG_URL "http://ver.cn"
// 版权信息
#define UTOOLS_VERSION_MANG_COPYRIGHT "Copyright © 1998 - 2024 Wooden Heart Bird. All Rights Reserved."
// 作者邮箱
#define UTOOLS_VERSION_MANG_AUTHOR_EMAIL "lingeasy@126.com"
// 许可证
#define UTOOLS_VERSION_MANG_LICENSE "MIT"
// 硬件识别码
#define UTOOLS_VERSION_MANG_HARDWARE_ID "0000000000000000"
// 软件识别码
#define UTOOLS_VERSION_MANG_SOFTWARE_ID "0000000000000000"
// 系统识别码，独立于硬件和软件识别，可以认为是是硬件和软件的指纹
#define UTOOLS_VERSION_MANG_IDENTIFY "0000000000000000"

#endif // __UTOOLS_ABOUT_H__

```

*注：不是所有的宏都要定义，只需要定义自己需要的，其它不用的参数可以不写*

##### 3) 在程序中定义

本模块不是必须使用宏定义的，也可以在程序使用相关信息前，在程序中进行初始化，具体的方式如下：

```c++
#include "utools.h"

void setup()
{
    utools::about::name = "app name";
    utools::about::cpu = "stm32";
    // ... 其它需要配置的内容
    utools::about::identify = "123456789";
}

int main() {
    setup();
    return 0;
}
```

注：系统中```date```、```time```、```build```三个参数不可以动态配置，会在编译时自动生成

##### 4) 使用方法

在应用程序中，可以通过调用utools::about模块的变量和函数来展示应用程序的版本信息、版权信息、作者信息等。例如：

```c++
#include "utools.h"

int main() {
    utools::about::int_ver();   // 获取整数版本号
    utools::about::str_ver();   // 获取字符串版本号
    utools::logger_info(utools::about::name);      // 获取应用程序名称
    utools::logger_info(utools::about::author);    // 获取作者名称
    utools::logger_info(utools::about::email);     // 获取作者邮箱
    utools::logger_info(utools::about::license);   // 获取许可证
    utools::logger_info(utools::about::date);      // 获取编译日期
    utools::logger_info(utools::about::time);      // 获取编译时间
    utools::logger_info(utools::about::build);     // 获取编译版本
    utools::logger_info(utools::about::identify);  // 获取系统识别码
    utools::logger_info(utools::about::cpu);       // 获取CPU型号
    // ... 其它需要使用的信息
    return 0;
}
```

##### 5) 可用功能清单

具体可用的函数和变量定义如下：

```c++
    /// @brief 获取版本号
    /// @return 输出“20230115153045”类型的字符串
    const std::string format_build_date_time_without_separators();

    /// @brief 获取版本号
    /// @return uint32_t类型的版本号，可用于比较大小等操作
    const uint32_t int_ver();

    /// @brief 获取版本号
    /// @return std::string类型的版本号，如“1.2.3”
    const std::string str_ver();

    /// @brief 获取版本号
    /// @return std::string类型的版本号，如“1.2.3 build 20230115153045”
    const std::string full_ver();

    /// @brief 产品相关信息
    std::string name;      // 产品名称
    std::string cpu;       // CPU型号
    std::string arch;      // 架构
    std::string arm;       // ARM架构
    std::string os;        // 操作系统
    std::string compiler;  // 编译器

    /// @brief 版本号相关
    uint32_t major;           // 主版本号
    uint32_t minor;           // 次版本号
    uint32_t patch;           // 修订号
    const std::string date;   // 编译日期
    const std::string time;   // 编译时间
    const std::string build;  // 构建时间

    /// @brief 软件相关相关信息
    std::string desc;      // 描述
    std::string author;    // 作者
    std::string company;   // 公司
    std::string copyright; // 版权
    std::string url;       // 网址
    std::string email;     // 邮箱
    std::string license;   // 许可证

    /// @brief 设置识别信息
    std::string hardware_id; // 设备（硬件）唯一标识
    std::string software_id; // 软件唯一标识
    std::string identify;    // 全局识别信息

    /// @brief 用户保存的其它版本信息
    utools::mem::AnyMap info;

    /// @brief 设置一个指定的值
    /// @tparam T 值类型
    /// @tparam K 键类型
    /// @param key 键
    template <typename T, typename K>
    utools::mem::AnyMap &set(const K &key, const T &value)
    {
        return info.set(key, value);
    }

    /// @brief 获取一个指定的值
    /// @tparam T 值类型
    /// @tparam K 键类型
    /// @param key 健值
    /// @return 指定key对应的值，如果没有找到会报错
    template <typename T, typename K>
    T get(const K &key)
    {
        return info.get<T>(key);
    }

    /// @brief 获取指定key对应的值，如果key不存在，则返回默认值
    /// @tparam T 值类型
    /// @tparam K 键类型
    /// @param key 键
    /// @param defaule_val 默认值
    /// @return 指定key对应的值，如果key不存在，则返回默认值
    template <typename T, typename K>
    T get(const K &key, const T &defaule_val)
    {
        return info.get<T>(key, defaule_val);
    }
```

*注：usr_info是提供给用户使用的一个接口，用户可以根据自己的需要，写入一些独特的参数，便于同意管理*

### 3.12 设计模式模块 (utools::pattern)

#### 3.11.1 功能介绍
将常用的设计模式进行整理,提高开发效率
*注：设计模式模块，目前只实现了单例模式，后续会根据需求继续扩展*

#### 3.11.2 使用方法

#### 3.11.3 单例模式(Singleton)
使用方法如下:
```cpp
#include "utools.h"

// 示例类
class MyClass : public utools::pattern::Singleton<MyClass> {
public:
    MyClass() {
        UTINFO("MyClass Constructor");
    }

    void doSomething() {
        UTINFO("Doing something");
    }
};

int main() {
    MyClass& instance = MyClass::instance();
    instance.doSomething();

    return 0;
}
```

#### 3.11.4 状态机模式(StateMachine )
使用方法如下:
```cpp
#include "utools.h"

// 示例使用
enum class MyState { Start, Process, End };
enum class MyEvent { StartProcess, Complete };

void onStartProcess() {
    UTINFO("Processing started!");
}

void onComplete() {
    UTINFO("Processing completed!");
}

int main() {
    utools::pattern::StateMachine<MyState, MyEvent> sm;

    // 定义状态转换
    sm.add_transition(MyState::Start, MyEvent::StartProcess, MyState::Process, onStartProcess);
    sm.add_transition(MyState::Process, MyEvent::Complete, MyState::End, onComplete);

    // 设置初始状态
    sm.set_initial_state(MyState::Start);

    // 处理事件
    sm.handle_event(MyEvent::StartProcess);
    sm.handle_event(MyEvent::Complete);

    return 0;
}
```

#### 3.11.5 观察者模式(observe)
使用方法如下:
```cpp
#include "utools.h"

// 示例被观察者类
class WeatherStation : public utools::pattern::observe::Subject<float> {
public:
    void setTemperature(float temp) {
        temperature = temp;
        notify(temperature); // 通知观察者
    }

private:
    float temperature = 0.0f; // 温度
};

// 示例观察者类
class Display : public utools::pattern::observe::Observer<float> {
public:
    void update(const float& data) override {
        std::cout << "Temperature updated: " << data << "°C" << std::endl;
    }
};

// 示例代码
int main() {
    WeatherStation weatherStation;

    // 创建观察者
    auto display1 = Display();
    auto display2 = Display();

    // 注册观察者
    weatherStation.attach(&display1);
    weatherStation.attach(&display2);

    // 更新温度
    weatherStation.setTemperature(25.0f);
    weatherStation.setTemperature(30.0f);

    // 注销观察者
    weatherStation.detach(&display1);
    weatherStation.setTemperature(20.0f); // 只有 display2 会收到更新

    return 0;
}
```
