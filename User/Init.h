#ifndef INIT_H
#define INIT_H

#include "Tool.h"

void BSP(void);

void ClockInit(void);		//使能所有需要用到的外设的时钟
void BlueToothInit(void);	//蓝牙初始化，包括配置PB6与PB7为复用模式,复用至USART1,配置并启用USART1
void GY87Init(void);		/*包含I2C1和MPU6050的初始化,配置PB8和PB9分别作为I2C1的SCL和SDA,
							 *此函数调用I2C1_Init()与MPU6050_Init()函数实现初始化功能
							 */
void MotorInit(void);		//初始化给电机提供PWM信号的PA8，并配置和启动TIM1
void GY87TimerStart(void);	//采样信号量和任务就绪后，启动TIM3的100Hz更新中断
void ReceiverInit(void);	//初始化PA0作为接收机信号引脚，初始化TIM2，配置TIM2通道1为输入捕获，并启动定时器
//		|					/*Attention:
//		└------------------> *该函数内部已经设置了优先级分组为2抢占优先级、2子优先级
//							 *并且设置了TIM2_IRQn优先级为抢占1,子优先级0
//							 */
void GY87TimerStart(void);	//初始化TIM3，作为GY87数据采集的频率控制

void I2C1_Init(void);
void MPU6050_Init(void);
void HMC5883L_Init(void);

#endif
