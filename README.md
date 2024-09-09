# 跟焦器电机控制器

## 1 简介

本项目为基于STM32F103C8T6的电机控制器，用于控制步进电机，实现步进电机的精确控制。

## 2 部署项目

### 2.1 克隆代码

通过如下指令获取项目代码：
```sh
git clone --recurse-submodules https://codeup.aliyun.com/629f6370487c500c27f60603/versatile/focus_motor_driver.git
```

### 2.2 安装STM32支持库

在```res/board_support```目录下找到```F302C(B-C)T```目录，将其复制到```C:\Users\OSD\.platformio\packages\framework-arduinoststm32\variants\```目录下，替换原有文件。

*注：OSC需要替换成自己的用户名。*

### 2.3 安装配制文件

在```res/board_support```目录下找到```genericSTM32F302CB.json```文件，将其复制到```C:\Users\OSD\.platformio\platforms\ststm32\boards```目录下。

*注：OSC需要替换成自己的用户名。*

### 2.4 修改SimpleFOC配置

测试发现，目前的电机控制代码，只有在使用2.3.3版本时才能正常工作，主要是因为电流检测部分代码初始化失败的问题。
修改```platformio.ini```文件，将SimpleFOC版本修改为2.3.3：
```ini
; 其它配置不变
lib_deps =
  askuric/Simple FOC@2.3.3
; 其它配置不变
```


将```.pio\libdeps\genericSTM32F302CB\Simple FOC\src\drivers\hardware_specific\generic_mcu.cpp```文件中的```_configure6PWM```函数修改注释掉：

```c++
// Configuring PWM frequency, resolution and alignment
// - BLDC driver - 6PWM setting
// - hardware specific
__attribute__((weak)) void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  _UNUSED(pwm_frequency);
  _UNUSED(dead_zone);
  _UNUSED(pinA_h);
  _UNUSED(pinA_l);
  _UNUSED(pinB_h);
  _UNUSED(pinB_l);
  _UNUSED(pinC_h);
  _UNUSED(pinC_l);

  return SIMPLEFOC_DRIVER_INIT_FAILED;
}
```

注释后的代码如下：
```c++
// Configuring PWM frequency, resolution and alignment
// - BLDC driver - 6PWM setting
// - hardware specific
// __attribute__((weak)) void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
//   _UNUSED(pwm_frequency);
//   _UNUSED(dead_zone);
//   _UNUSED(pinA_h);
//   _UNUSED(pinA_l);
//   _UNUSED(pinB_h);
//   _UNUSED(pinB_l);
//   _UNUSED(pinC_h);
//   _UNUSED(pinC_l);

//   return SIMPLEFOC_DRIVER_INIT_FAILED;
// }
```

*注：测试发现，上面的代码会屏蔽掉需要调用的代码。*

## 3 使用方法

### 3.1 电机驱动库

[SimpleFOC参考文档](https://docs.simplefoc.com/drivers_library)