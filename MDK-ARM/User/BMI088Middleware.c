/**
 ******************************************************************************
 * @file    BMI088Middleware.c
 * @brief   BMI088 硬件中间件实现（BSP 层）
 * @note    职责边界：
 *            直接操作 STM32 外设寄存器（通过 HAL），屏蔽硬件细节。
 *            【禁止】在本文件中出现任何 BMI088 寄存器地址或传感器协议。
 *
 *          延时实现说明：
 *            BMI088_delay_us() 使用 SysTick 倒计数实现精确微秒延时，
 *            ticks = us × 168（对应 168MHz 主频），如主频不同请修改此系数。
 *            BMI088_delay_ms() 通过循环调用 delay_us(1000) 实现。
 *
 *          SPI 说明：
 *            使用 hspi1，超时时间固定为 1000ms。
 *            TODO: 若 SPI 总线存在竞争，需加互斥保护。
 *
 * @version 1.0
 * @date    2022-9-3
 * @author  DJI
 ******************************************************************************
 */

#include "BMI088Middleware.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

void BMI088_GPIO_init(void)
{

}

void BMI088_com_init(void)
{


}

void BMI088_delay_ms(uint16_t ms)
{
    while(ms--)
    {
        BMI088_delay_us(1000);
    }
}

void BMI088_delay_us(uint16_t us)
{

    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 168;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }


}




void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

