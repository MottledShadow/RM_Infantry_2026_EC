/**
 ******************************************************************************
 * @file    BMI088Middleware.h
 * @brief   BMI088 硬件中间件头文件（BSP 层）
 * @note    层级说明：
 *            本文件属于【BSP 层】，是 BMI088 驱动的最底层硬件抽象。
 *            直接操作 SPI 收发、GPIO 片选、SysTick 延时，
 *            所有操作均基于 HAL 库原语，不包含任何传感器协议逻辑。
 *
 *          对外提供的硬件原语：
 *            - SPI 单字节收发：BMI088_read_write_byte()
 *            - 加速度计片选：BMI088_ACCEL_NS_L/H()
 *            - 陀螺仪片选：  BMI088_GYRO_NS_L/H()
 *            - 精确延时：    BMI088_delay_us() / BMI088_delay_ms()
 *            - 接口初始化：  BMI088_GPIO_init() / BMI088_com_init()
 *
 *          若需切换为 IIC 接口，取消注释 BMI088_USE_IIC 并实现对应函数体。
 *
 * @version 1.0
 * @date    2022-9-3
 * @author  DJI
 ******************************************************************************
 */

#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "struct_typedef.h"
#include "main.h"

#define BMI088_USE_SPI
//#define BMI088_USE_IIC
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GRYO_Pin GPIO_PIN_5
#define INT1_GRYO_GPIO_Port GPIOC
#define INT1_GRYO_EXTI_IRQn EXTI9_5_IRQn
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

#elif defined(BMI088_USE_IIC)

#endif

#endif
