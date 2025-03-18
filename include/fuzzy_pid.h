#ifndef FUZZY_PID_H
#define FUZZY_H
#include <SimpleFOC.h>

#include "Arduino.h"

class FuzzyPID : public PIDController {
public:
    FuzzyPID(float P, float I, float D, float ramp, float limit) : PIDController(P, I, D, ramp, limit) {
    }

    /**
     * @brief 设置论域
     *
     * @param min 最小值
     * @param max 最大值
     */
    void set_area(int16_t min, int16_t max) {
        std::get<0>(area) = min;
        std::get<1>(area) = max;
        NB = min + (max - min) / 6 * 0;  // 分割论域为6个区域
        NM = min + (max - min) / 6 * 1;
        NS = min + (max - min) / 6 * 2;
        ZO = min + (max - min) / 6 * 3;
        PS = min + (max - min) / 6 * 4;
        PM = min + (max - min) / 6 * 5;
        PB = min + (max - min) / 6 * 6;
        e_membership_values = {NB, NM, NS, ZO, PS, PM, PB};  // 根据论域分配不同的隶属值
        ec_membership_values = {NB, NM, NS, ZO, PS, PM, PB};
        kp_menbership_values = {NB, NM, NS, ZO, PS, PM, PB};
        ki_menbership_values = {NB, NM, NS, ZO, PS, PM, PB};
        kd_menbership_values = {NB, NM, NS, ZO, PS, PM, PB};
        fuzzyoutput_menbership_values = {NB, NM, NS, ZO, PS, PM, PB};
    }

    /**
     * @brief 获取e、ec的隶属度
     * 
     * @param erro 
     * @param erro_c 
     */
    void get_grad_membership(float erro, float erro_c) {
        if (erro > e_membership_values[0] && erro < e_membership_values[6]) {
            for (uint8_t i = 0; i < num_area - 2; i++) {
                if (erro >= e_membership_values[i] && erro <= e_membership_values[i + 1]) {
                    e_gradmembership[0] =
                        -(erro - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
                    e_gradmembership[1] =
                        1 + (erro - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
                    e_grad_index[0] = i;
                    e_grad_index[1] = i + 1;
                    break;
                }
            }
        } else {
            if (erro <= e_membership_values[0]) {
                e_gradmembership[0] = 1;
                e_gradmembership[1] = 0;
                e_grad_index[0] = 0;
                e_grad_index[1] = -1;
            } else if (erro >= e_membership_values[6]) {
                e_gradmembership[0] = 1;
                e_gradmembership[1] = 0;
                e_grad_index[0] = 6;
                e_grad_index[1] = -1;
            }
        }

        if (erro_c > ec_membership_values[0] && erro_c < ec_membership_values[6]) {
            for (uint8_t i = 0; i < num_area - 2; i++) {
                if (erro_c >= ec_membership_values[i] && erro_c <= ec_membership_values[i + 1]) {
                    ec_gradmembership[0] = -(erro_c - ec_membership_values[i + 1])
                                           / (ec_membership_values[i + 1] - ec_membership_values[i]);
                    ec_gradmembership[1] = 1
                                           + (erro_c - ec_membership_values[i + 1])
                                                 / (ec_membership_values[i + 1] - ec_membership_values[i]);
                    ec_grad_index[0] = i;
                    ec_grad_index[1] = i + 1;
                    break;
                }
            }
        } else {
            if (erro_c <= ec_membership_values[0]) {
                ec_gradmembership[0] = 1;
                ec_gradmembership[1] = 0;
                ec_grad_index[0] = 0;
                ec_grad_index[1] = -1;
            } else if (erro_c >= ec_membership_values[6]) {
                ec_gradmembership[0] = 1;
                ec_gradmembership[1] = 0;
                ec_grad_index[0] = 6;
                ec_grad_index[1] = -1;
            }
        }
    }

private:
    int Kp_rule_list[7][7] = {{PB, PB, PM, PM, PS, ZO, ZO},  // kp规则表
                              {PB, PB, PM, PS, PS, ZO, NS}, {PM, PM, PM, PS, ZO, NS, NS}, {PM, PM, PS, ZO, NS, NM, NM},
                              {PS, PS, ZO, NS, NS, NM, NM}, {PS, ZO, NS, NM, NM, NM, NB}, {ZO, ZO, NM, NM, NM, NB, NB}};

    int Ki_rule_list[7][7] = {{NB, NB, NM, NM, NS, ZO, ZO},  // ki规则表
                              {NB, NB, NM, NS, NS, ZO, ZO}, {NB, NM, NS, NS, ZO, PS, PS}, {NM, NM, NS, ZO, PS, PM, PM},
                              {NM, NS, ZO, PS, PS, PM, PB}, {ZO, ZO, PS, PS, PM, PB, PB}, {ZO, ZO, PS, PM, PM, PB, PB}};

    int Kd_rule_list[7][7] = {{PS, NS, NB, NB, NB, NM, PS},  // kd规则表
                              {PS, NS, NB, NM, NM, NS, ZO}, {ZO, NS, NM, NM, NS, NS, ZO}, {ZO, NS, NS, NS, NS, NS, ZO},
                              {ZO, ZO, ZO, ZO, ZO, ZO, ZO}, {PB, NS, PS, PS, PS, PS, PB}, {PB, PM, PM, PM, PS, PS, PB}};

    int Fuzzy_rule_list[7][7] = {{PB, PB, PB, PB, PM, ZO, ZO}, {PB, PB, PB, PM, PM, ZO, ZO},
                                 {PB, PM, PM, PS, ZO, NS, NM}, {PM, PM, PS, ZO, NS, NM, NM},
                                 {PS, PS, ZO, NM, NM, NM, NB}, {ZO, ZO, ZO, NM, NB, NB, NB},
                                 {ZO, NS, NB, NB, NB, NB, NB}};

    uint16_t NB, NM, NS, ZO, PS, PM, PB;
    std::tuple<int16_t, int16_t> area;

    std::array<uint16_t, 7> e_membership_values;
    std::array<uint16_t, 7> ec_membership_values;
    std::array<uint16_t, 7> kp_menbership_values;
    std::array<uint16_t, 7> ki_menbership_values;
    std::array<uint16_t, 7> kd_menbership_values;
    std::array<uint16_t, 7> fuzzyoutput_menbership_values;
    float qdetail_kp;  // 增量kp对应论域中的值
    float qdetail_ki;  // 增量ki对应论域中的值
    float qdetail_kd;  // 增量kd对应论域中的值
    float qfuzzy_output;

    float detail_kp;  // 输出增量kp
    float detail_ki;  // 输出增量ki
    float detail_kd;  // 输出增量kd
    float fuzzy_output;
    float qerro;    // 输入e对应论域中的值
    float qerro_c;  // 输入de/dt对应论域中的值
    float errosum;
    float e_gradmembership[2];   // 输入e的隶属度
    float ec_gradmembership[2];  // 输入de/dt的隶属度
    int e_grad_index[2];         // 输入e隶属度在论域分区的索引
    int ec_grad_index[2];        // 输入de/dt隶属度在论域分区的索引
    float gradSums[7] = {0, 0, 0, 0, 0, 0, 0};
    float KpgradSums[7] = {0, 0, 0, 0, 0, 0, 0};  // 输出增量kp总的隶属度
    float KigradSums[7] = {0, 0, 0, 0, 0, 0, 0};  // 输出增量ki总的隶属度
    float KdgradSums[7] = {0, 0, 0, 0, 0, 0, 0};  // 输出增量kd总的隶属度
    constexpr static uint8_t num_area{8};                 // 分区数
};

#endif /* FUZZY_PID_H */
