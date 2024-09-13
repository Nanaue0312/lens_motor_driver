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
修改```platformio.ini```文件，将SimpleFOC版本修改为2.3.3。

在编译时还可能遇到PWM初始化失败的问题，需要修改```platformio.ini```文件，将```lib_archive```选项设置为false。

基于以上配置，需要在将```platformio.ini```文件修改为：

```ini
; 其它配置不变
lib_deps =
  askuric/Simple FOC@2.3.3
lib_archive = false ; 此选项必须为false，否则电机PWM初始化失败
; 其它配置不变
```

## 3 使用方法

### 3.1 电机驱动库

[SimpleFOC参考文档](https://docs.simplefoc.com/drivers_library)

## 4 设备状态

### 4.1 设备状态清单

电机共有如下几种状态：

* ```INIT```：电机初始化状态；
* ```PAIRING```：电机配对状态；
* ```CALIBRATION```：电机校准状态；
* ```RUNNING```：电机运行状态；
* ```ERROR```：电机故障状态；

### 4.2 LED指标灯

设备正面从上到下依次为，1号到3号LED。

颜色分别是BLUE、GREEN、RED。

#### 4.2.1 ```INIT```状态

只有3号LED亮起，表示设备正在初始化中。

#### 4.2.2 ```PAIRING```状态

配对状态，时状态3个LED依次闪烁，表示设备正在配对中。

闪烁顺序为：BLUE -> GREEN -> RED -> BLUE -> GREEN -> RED -> ...

闪烁频率为1Hz，每两个灯之间间隔约333ms。

#### 4.2.3 ```CALIBRATION```状态

校准状态，时状态3个LED依次闪烁，表示设备正在校准中。

此时```FOCUS```或```ZOOM```或```IRIS```的LED灯会闪烁，每500ms翻转一次。

#### 4.2.4 ```RUNNING```状态

运行时，3个LED灯只会有一个亮起，表示设备正在运行中，不同灯分别表示驱动不同的设备。

* 1号LED：工作时为```跟焦电机(Focus Motor)```状态指示灯；
* 2号LED：工作时为```焦距电机(Zoom Motor)```状态指标灯；
* 3号LED：工作时为```光圈电机(IRIS Motor)```状态指标灯。

#### 4.2.5 ```ERROR```状态

当设备运行出现不可恢复的错误时，3个LED灯会同时亮起，表示设备出现错误。

### 4.3 状态切换

设备的状态切换来源于以下几种情况：

* 上电自动进入```INIT```状态，初始化状态时，不接收任何指令，直到初始化完成，怎么切换到```RUNNING```状态，或初始化失败，则切换到```ERROR```状态；
* ```RUNNING```状态时，通过按键长按（>1s）切换到```PAIRING```状态，此时设备会开始配对，配对成功后，切换到```RUNNING```状态；
* ```RUNNING```状态时，通过按键 长按（>3S）切换到```CALIBRATION```状态，此时设备会开始校准，校准成功后，切换到```RUNNING```状态；
* ```PAIRING```状态时，通过按键短按（>1s）切换到```RUNNING```状态，此时设备会停止配对，并切换到```RUNNING```状态；
* ```CALIBRATION```状态时，通过按键短按（>3S）切换到```RUNNING```状态，此时设备会停止校准，并切换到```RUNNING```状态，；
* ```ERROR```状态时，通过按键短按（>1s）切换到```INIT```状态，此时设备会重新初始化，并切换到```INIT```状态。
