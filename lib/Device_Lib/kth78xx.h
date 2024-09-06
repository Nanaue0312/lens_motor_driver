#ifndef __KTH78XX_H
#define __KTH78XX_H
#include "Arduino.h"
#ifdef __cplusplus
extern "C"
{
#endif
#define B_Pin GPIO_PIN_0
#define B_GPIO_Port GPIOA
#define A_Pin GPIO_PIN_1
#define A_GPIO_Port GPIOA
#define A_EXTI_IRQn EXTI1_IRQn
#define LED_R_Pin GPIO_PIN_4
#define LED_R_GPIO_Port GPIOA
#define Z_Pin GPIO_PIN_10
#define Z_GPIO_Port GPIOB
#define NSS_POS_Pin GPIO_PIN_12
#define NSS_POS_GPIO_Port GPIOB
#define SCLK_POS_Pin GPIO_PIN_13
#define SCLK_POS_GPIO_Port GPIOB
#define MISO_POS_Pin GPIO_PIN_14
#define MISO_POS_GPIO_Port GPIOB
#define MOSI_POS_Pin GPIO_PIN_15
#define MOSI_POS_GPIO_Port GPIOB
#define U_HIN_Pin GPIO_PIN_8
#define U_HIN_GPIO_Port GPIOA
#define V_HIN_Pin GPIO_PIN_9
#define V_HIN_GPIO_Port GPIOA
#define V_LIN_Pin GPIO_PIN_10
#define V_LIN_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_12
#define LED_B_GPIO_Port GPIOA
#define U_LIN_Pin GPIO_PIN_3
#define U_LIN_GPIO_Port GPIOB
    /* Includes ------------------------------------------------------------------*/
    extern SPI_HandleTypeDef hspi2;
    void MX_GPIO_Init(void);
    void MX_SPI2_Init(void);
    void MX_DMA_Init(void);

#define KTH78_READ_ANGLE 0x00
#define KTH78_READ_REG (0x01 << 6)
#define KTH78_WRITE_REG (0x03 << 6)
#define KTH78_WRITE_MTP (0x02 << 6)

#define KTH78_NON_DATA 0x00
    float kth7812_read_angle();
    void kth7812_init();
    // uint16_t KTH78_ReadAngle(void);
    // uint8_t KTH78_ReadReg(uint8_t addr);
    // uint8_t KTH78_WriteReg(uint8_t addr, uint8_t data);
    // uint8_t KTH78_WriteMTP(uint8_t addr, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __KTH78XX_H */
