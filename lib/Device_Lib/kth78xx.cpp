/**************************************************
** Copyright (c) 2016-202X 昆泰芯微电子科技有限公司
** 文件名: kth78xx.c
** 作者: liujunbo
** 日期: 2023.08.16
** 描述: KTH78xx芯片相关文件，存放用来操作KTH78xx芯片的函数
**
**************************************************/

#include "kth78xx.h"

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LED_R_Pin | LED_G_Pin | LED_B_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(NSS_POS_GPIO_Port, NSS_POS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(A_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PAPin PAPin PAPin */
    GPIO_InitStruct.Pin = LED_R_Pin | LED_G_Pin | LED_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = Z_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Z_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = NSS_POS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(NSS_POS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PB4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    /* DMA1_Channel5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 7;
    hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI2_Init 2 */

    /* USER CODE END SPI2_Init 2 */
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (spiHandle->Instance == SPI2)
    {
        /* USER CODE BEGIN SPI2_MspInit 0 */

        /* USER CODE END SPI2_MspInit 0 */
        /* SPI2 clock enable */
        __HAL_RCC_SPI2_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**SPI2 GPIO Configuration
        PB13     ------> SPI2_SCK
        PB14     ------> SPI2_MISO
        PB15     ------> SPI2_MOSI
        */
        GPIO_InitStruct.Pin = SCLK_POS_Pin | MISO_POS_Pin | MOSI_POS_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* SPI2 DMA Init */
        /* SPI2_RX Init */
        hdma_spi2_rx.Instance = DMA1_Channel4;
        hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi2_rx.Init.Mode = DMA_NORMAL;
        hdma_spi2_rx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(spiHandle, hdmarx, hdma_spi2_rx);

        /* SPI2_TX Init */
        hdma_spi2_tx.Instance = DMA1_Channel5;
        hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi2_tx.Init.Mode = DMA_NORMAL;
        hdma_spi2_tx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(spiHandle, hdmatx, hdma_spi2_tx);

        /* USER CODE BEGIN SPI2_MspInit 1 */

        /* USER CODE END SPI2_MspInit 1 */
    }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle)
{

    if (spiHandle->Instance == SPI2)
    {
        /* USER CODE BEGIN SPI2_MspDeInit 0 */

        /* USER CODE END SPI2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_SPI2_CLK_DISABLE();

        /**SPI2 GPIO Configuration
        PB13     ------> SPI2_SCK
        PB14     ------> SPI2_MISO
        PB15     ------> SPI2_MOSI
        */
        HAL_GPIO_DeInit(GPIOB, SCLK_POS_Pin | MISO_POS_Pin | MOSI_POS_Pin);

        /* SPI2 DMA DeInit */
        HAL_DMA_DeInit(spiHandle->hdmarx);
        HAL_DMA_DeInit(spiHandle->hdmatx);
        /* USER CODE BEGIN SPI2_MspDeInit 1 */

        /* USER CODE END SPI2_MspDeInit 1 */
    }
}

static uint8_t KTH78_SPI_TransmitReceiveBytes(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
    HAL_GPIO_WritePin(NSS_POS_GPIO_Port, NSS_POS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, pTxData, pRxData, 2, 0xffff);
    while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)
        ;
    HAL_GPIO_WritePin(NSS_POS_GPIO_Port, NSS_POS_Pin, GPIO_PIN_SET);
    return 0;
}

// 读取角度值
uint16_t KTH78_ReadAngle(void)
{
    uint8_t databack[2];
    uint8_t cmd[2];

    cmd[0] = KTH78_READ_ANGLE;
    cmd[1] = KTH78_NON_DATA;

    KTH78_SPI_TransmitReceiveBytes(&hspi2, cmd, databack, 2);

    return (uint16_t)(databack[0] << 8) + databack[1];
}

// 读取寄存器数据
uint8_t KTH78_ReadReg(uint8_t addr)
{
    uint8_t databack[2];
    uint8_t cmd[2];

    cmd[0] = KTH78_READ_REG + addr;
    cmd[1] = KTH78_NON_DATA;

    KTH78_SPI_TransmitReceiveBytes(&hspi2, cmd, databack, 2);

    return databack[0];
}

// 写寄存器数据
uint8_t KTH78_WriteReg(uint8_t addr, uint8_t data)
{
    uint8_t databack[2];
    uint8_t cmd[2];

    cmd[0] = KTH78_WRITE_REG + addr;
    cmd[1] = data;

    KTH78_SPI_TransmitReceiveBytes(&hspi2, cmd, databack, 2);

    return databack[0];
}

// 向MTP中写入配置
uint8_t KTH78_WriteMTP(uint8_t addr, uint8_t data)
{
    uint8_t databack[2];
    uint8_t cmd[2];

    cmd[0] = KTH78_WRITE_MTP + addr;
    cmd[1] = data;

    KTH78_SPI_TransmitReceiveBytes(&hspi2, cmd, databack, 2);

    return databack[0];
}

float convert_to_radians_0_to_2pi(uint16_t angle_value)
{
    // 计算每个单位对应的角度
    float degrees_per_unit = 360.0 / 65536.0;

    // 将返回的值转换为度数
    float degrees = angle_value * degrees_per_unit;

    // 将度数转换为弧度
    float radians = degrees * (PI / 180.0);

    // 调整弧度值，使其在0到2π之间
    while (radians < 0)
    {
        radians += 2 * PI;
    }
    while (radians >= 2 * PI)
    {
        radians -= 2 * PI;
    }

    return radians;
}

float kth7812_read_angle()
{
    uint16_t angle_value = KTH78_ReadAngle(); // 假设这是你的函数调用
    return convert_to_radians_0_to_2pi(angle_value);
}

void kth7812_init()
{
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI2_Init();

    HAL_GPIO_WritePin(NSS_POS_GPIO_Port, NSS_POS_Pin, GPIO_PIN_SET);
    // gRegValue = KTH78_ReadReg(0x02);
    // gRegValue = KTH78_ReadReg(0x02);
    // HAL_Delay(10);
    // KTH78_WriteReg(0x02, 0x80);
    // HAL_Delay(25);
    // gRegValue = KTH78_ReadReg(0x02);
    // gRegValue = KTH78_ReadReg(0x02);
    // printf("0x07:%d\r\n", KTH78_ReadReg(0x07));
    // // printf("write:%d\r\n", KTH78_WriteReg(0x07, 0xc0));
    // printf("0x07:%d\r\n", KTH78_ReadReg(0x07));
    // printf("Sensor ready\r\n");
}
