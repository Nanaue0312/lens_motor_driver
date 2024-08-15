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

## 3 使用方法

### 3.1 电机驱动库

[SimpleFOC参考文档](https://docs.simplefoc.com/drivers_library)