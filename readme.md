##特别声明： 
 - 此教程基于官方 C 板教程，缺少的相关知识参考：《RoboMaster 开发板 C 型嵌入式软 件教程文档.pdf》
 - 内容也基本来自C板教程，仅供于学习。
## 概述
### 软件环境
 - Toolchain/IDE : MDK-ARM V5.32
 - STM32F4xx_DFP Packs:2.13.0
 - STM32CubeMx:6.5.0
 - package version: STM32Cube FW_F4 V1.27.1
 - FreeRTOS version: 10.0.1
 - CMSIS-RTOS version: 1.02
 
### 配套例程文档
[RoboMaster开发板A型嵌入式软件教程文档.pdf]
### 配套例程
* [0.cubemx新工程](0BASE)
* [1.点亮LED](1LED)
* [2.闪烁LED](2FLASH_LED)
* [3.定时器闪烁LED](3TIM_LED)
* [4.PWM控制LED亮度:呼吸灯](4PWM_LED)
* [5.蜂鸣器](5PWM_BUZZER)
* [6.舵机](6PWM_SERVO_MOTOR)
* [7.外部中断](7KEY_EXIT)
* [8.串口中断](8USART)
* [9.串口DMA中断（遥控器）](9RC_USART)
* [10.传感器数据读取](10I2C_SPI_READ)
* [11.IMU温控PID](11IMU_TEMP_CONTROL)
* [12.IMU姿态解算](12IMU_INS)
* [13.CAN控制RM电机](13CAN_MOTOR)
* [14.FreeRTOS闪烁LED](14FREERTOS_LED)