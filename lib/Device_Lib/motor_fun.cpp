#include "motor_fun.h"
#include "crc16.h"
#include "bsp_internal_flash.h"
#include "utools.h"

int16_t current_location_save[1] = {0};
int16_t dataToSave[4] = {0, 0, 0, 0};
// 最大角度
float max_angle = 45;
// 最小角度
float min_angle = 0;
float current_angle = 0;
// voltage set point variable
float target_angle = 0;
float target_angle_rec_uart = 0;
bool calibrating = false; // 校准状态标志
bool calibration_flag = false;
// BLDC driver instance
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PA11, PA9, PA12, PA10, PB1);

BLDCMotor motor = BLDCMotor(7, 10, 180);

// LowsideCurrentSense构造函数
//-分路电阻器-分路电阻器值
//   - gain  - current-sense op-amp gain
//   - phA   - A phase adc pin
//   - phB   - B phase adc pin
//   - phC   - C phase adc pin (optional)
LowsideCurrentSense current_sense = LowsideCurrentSense(0.001, 11, PA6, PA7, PA5); // when measuring A and B phase currents and not measuring C
// LowsideCurrentSense current_sense = LowsideCurrentSense(0.001, 11, PA6, PA7, _NC); // when measuring A and B phase currents and not measuring C

GenericSensor sensor = GenericSensor(kth7812_read_angle, kth7812_init);
// voltage set point variable
// Commander command = Commander(Serial);
// void doTarget(char *cmd) { command.scalar(&target_angle_rec_uart, cmd); }
// void doMotor(char *cmd) { command.motor(&motor, cmd); }

// instantiate the commander
void motor_init()
{
    // 传感器初始化
    sensor.init();
    UTINFO("kth7812 sensor ready.");

    // 将电机连接到传感器
    motor.linkSensor(&sensor);
    UTINFO("motor linked to sensor.");

    // 驱动程序配置
    // 重要！
    // 确保设置正确的电源电压[V]
    // 电源电压[V]
    // dead_zone [0,1] - default 0.02 - 2%
    driver.voltage_power_supply = 12;
    driver.voltage_limit = 24;
    driver.pwm_frequency = 20000;
    auto re_dr = driver.init();
    if (re_dr != 1)
    {
        UTERROR("driver init failed. status:", re_dr);
        return;
    }
    UTINFO("driver ready.");
    // link driver
    motor.linkDriver(&driver);

    current_sense.linkDriver(&driver);
    auto re_cs = current_sense.init();
    if (re_cs != 1)
    {
        UTERROR("current sense init failed. status:", re_cs);
        return;
    }
    UTINFO("current sense initialized success.");

    // enable monitoring functionality
    // motor.useMonitoring(Serial);
    motor.linkCurrentSense(&current_sense);
    UTINFO("motor linked to current sense.");

    // set torque mode:
    motor.torque_controller = TorqueControlType::voltage;
    // set motion control loop to be used
    motor.controller = MotionControlType::angle;
    motor.motion_downsample = 0; // 每调用10次 motor.move() 才执行一次运动控制

    // // foc current control parameters (Arduino UNO/Mega)
    // motor.PID_current_q.P = 1;
    // motor.PID_current_q.I = 0;
    // motor.PID_current_d.P = 1;
    // motor.PID_current_d.I = 0;
    // motor.LPF_current_q.Tf = 1;
    // motor.LPF_current_d.Tf = 1;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    // motor.target = 2;
    // velocity loop PID
    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 0.2;
    motor.PID_velocity.D = 0.001;
    motor.PID_velocity.output_ramp = 1000.0;
    motor.PID_velocity.limit = 1.5;
    // Low pass filtering time constant
    motor.LPF_velocity.Tf = 0.2;

    // angle loop PID
    motor.P_angle.P = 10.0;
    motor.P_angle.I = 0.0;
    motor.P_angle.D = 0.0;
    motor.P_angle.output_ramp = 10000.0;
    motor.P_angle.limit = 200.0;
    // Low pass filtering time constant
    motor.LPF_angle.Tf = 0.0;

    // setting the limits
    //  maximal velocity of the position control
    motor.velocity_limit = 200; // rad/s - default 20
    motor.voltage_limit = 10.0;
    motor.current_limit = 1.5;
    // sensor zero offset - home position
    // motor.sensor_offset = 20.013;
    motor.phase_resistance = 5;
    motor.modulation_centered = 1.0;
    // use monitoring with serial
    // Serial.begin(115200);
    // comment out if not needed
    // motor.useMonitoring(Serial);

    // initialize motor
    motor.init();
    // align sensor and start FOC
    // motor.zero_electric_angle = 2.15;       // rad
    // motor.sensor_direction = Direction::CW; // CW or CCW
    // current_sense.skip_align = true;        // default false
    motor.initFOC();
    // motor.monitor_start_char = '\0'; //!< monitor starting character
    // motor.monitor_end_char = '\n';   //!< monitor outputs ending character
    // motor.monitor_separator = ',';   //!< monitor outputs separation character
    // add target command T
    // command.add('T', doTarget, "target current");
    // command.add('M', doMotor, "motor control");
    // Serial.println(F("Motor ready."));
    // Serial.println(F("Set the target current using serial terminal:"));
    UTINFO("Motor ready.");

    // TODO:读取EEPROM
    // uint16_t readData[2];
    // FLASH_ReadData(FLASH_SAVE_ADDR, readData, sizeof(readData) / sizeof(uint16_t));
    //     max_angle = readData[0] / 100;
    //     min_angle = readData[1] / 100;
    //     // Serial.printf("max:%f,min:%f\r\n", max_angle, min_angle);
    //     Serial.println(max_angle);
    // Serial.println(min_angle);
    _delay(1000);
    byte addr_rvf[] = {0xCC, 0xCD, 0xAA, 0xAA};
    UTINFO("send addr: ", utcode::to_hex(addr_rvf, 4));
}
float sigmoid(float x, float k)
{
    return 1.0 / (1.0 + exp(-k * x));
} // 双曲正切函数，用于平滑调整速度
float tanh_scale(float x, float k)
{
    return 200 * tanh(k * x) + 10; // 调整输出范围以适应速度限制
}
void motor_work(float target_angle_rec, float slope)
{
    // main FOC algorithm function
    motor.loopFOC();
    dataToSave[2] = motor.shaftAngle();
    float target_angle_new = min_angle + (target_angle_rec * (max_angle - min_angle));

    if (calibrating)
    {
        if (target_angle_new > max_angle || target_angle_new < min_angle)
        {
            return;
        }
        else
        {
            motor.move(target_angle_new);
        }
    }
    else
        motor.move(target_angle);
    // sensor.getAngle();
    // Serial.println(sensor.getAngle());
    // user communication
    // command.run();
    // motor.monitor();
}

void TaskCalibration(void *pvParameters)
{
    // Serial.println("Start calibration");
    UTINFO("Start calibration");
    calibration_flag = true;
    target_angle = 0;
    int last_time = 0;
    float last_Angle = 0;
    bool flag_direction = true;
    motor.velocity_limit = 20;
    while (1)
    {

        if (flag_direction)
        {
            target_angle += 20;
        }
        else
        {
            target_angle -= 20;
        }
        int current_time = millis();
        if (current_time - last_time > 300)
        {
            last_time = current_time;

            if (abs(last_Angle - sensor.getAngle()) < 0.1)
            {

                if (flag_direction)
                {

                    max_angle = sensor.getAngle() - 1.5;
                    // Serial.println(max_angle);
                    UTTRACE("max angle:", max_angle);
                    flag_direction = false;
                    target_angle = max_angle - 30;
                }
                else
                {

                    min_angle = sensor.getAngle() + 1.5;
                    // Serial.println(min_angle);
                    UTTRACE("min angle:", min_angle);
                    motor.velocity_limit = 300;
                    calibrating = true;
                    vTaskDelay(100);
                    calibration_flag = false;
                    // TODO:保存数据到EEPROM
                    dataToSave[0] = {(int16_t)(max_angle * 100.0f)};
                    dataToSave[1] = {(int16_t)(min_angle * 100.0f)}; // 将浮点数转换为整数保存

                    // FLASH_WriteData(FLASH_SAVE_ADDR, dataToSave, 2);
                    break;
                }
            }
            else
            {
                last_Angle = sensor.getAngle();
            }
        }
       
        vTaskDelay(25);
    }
    UTINFO("TaskCalibration exit");
    vTaskDelete(NULL);
}

float received_frm_data;
uint16_t received_slope_f;

void TaskRecUart(void *pvParameters)
{
    // Serial1.begin(115200);
    while (1)
    {
        // Serial.println(sensor.getAngle());
        int len = Serial.available();
        if (len >= 8)
        { // 确保至少有8个字节可读 (2字节头 + 4字节float + 2字节CRC)
            uint8_t buf[12];
            memset(buf, 0, 12);

            // 读取8个字节的数据
            Serial.readBytes(buf, 12);
            Serial.print("received data:");
            for (int i = 0; i < 12; i++)
            {
                Serial.print(buf[i], HEX);
                Serial.print(" ");
            }
            // 检查头部信息是否匹配
            if (buf[0] == 0xAA && buf[1] == 0xAB)
            {

                unsigned short received_crc = (buf[10] << 8) | buf[11];

                // // 计算数据部分的CRC
                unsigned short calculated_crc = crc16(buf, 10);
                unsigned short adjusted_crc = (calculated_crc >> 8) | (calculated_crc << 8);

                // printf("Calculated CRC: 0x%04X\n", adjusted_crc);

                // 校验CRC
                if (received_crc == adjusted_crc)
                {
                    // 解析四个字节的浮点数数据
                    if (*(float *)(&buf[2]) > 0)
                    {
                        memcpy(&received_frm_data, &buf[2], sizeof(float)); // 从第3个字节开始，读取4字节
                    }
                    memcpy(&received_slope_f, &buf[6], sizeof(uint16_t)); // 从第7个字节开始，读取4字节
                    float target_angle_new = min_angle + (received_frm_data * (max_angle - min_angle));
                    Serial.print("received_frm_data");
                    Serial.println(received_slope_f);
                    if (received_slope_f == 0x0a)
                    {
                        printf("calibration ready\r\n");
                        calibrating = false;
                        if (!calibration_flag)
                        {
                            xTaskCreate(TaskCalibration, (const portCHAR *)"Calibration", 128, NULL, 1, NULL);
                        }
                    }
                    else if (received_slope_f == 0x09)
                    {
                        Serial.println("Wall collision point setting");
                    }
                    else
                    {
                        // 本来是处理斜率的  现在放弃了
                    }
                }

                else
                {
                    // CRC校验失败，处理错误
                    Serial.println("CRC check failed!");
                }
            }
        }

        // UTINFO(sensor.getAngle());  // 串口打印角度
        utcollab::Task::sleep_for(5);
    }
}
