#ifndef TOOL_H
#define TOOL_H

#include "Cfg.h"

/*以下为与I2C相关的工具函数*/
void I2C_Start(void);							//发送起始位
void I2C_Stop(void);							//发送停止位
void I2C_Send_Addr(uint8_t Addr,uint8_t RW);	/*发送地址,Addr代表地址,注意地址是7位的，但是Addr是8位的,
												 *有效地址需要放在高7位，例如若地址为0b1111111,那么Addr的
												 *值应该是0xFE;RW代表读或者写,读为1,写为0
												 */
void I2C_Send_Byte(uint8_t data);				//发送字节,data为要发送的数据
uint8_t I2C_Receive_Byte(void);					/*接收字节数据,作为返回值,要求ACK已使能,在
												 *本项目中,ACK已经在I2C1_Init()中使能
												 */
uint8_t I2C_Receive_Byte_NACK(void);			/*接收最后一个字节需发送NACK,所以单独使用一个函数,该函数
												 *接收一个字节的数据并发送NACK
												 */
void Read_MPU6050_Gyro_Values(int16_t* x,int16_t* y,int16_t* z);			/*读取MPU6050陀螺仪数据到内存
																			 *指针x、y、z指向的变量分别用
																			 *于接收X、Y、Z轴的数据
																			 */
void Read_MPU6050_Accel_Values(int16_t* x,int16_t* y,int16_t* z);			/*读取MPU6050加速度计数据到内存
																			 *指针x、y、z指向的变量分别用
																			 *于接收X、Y、Z轴的数据
																			 */

/*以下为与延时函数相关的工具函数*/
void Delay(uint32_t time);

/*以下为与蓝牙发送相关的工具函数*/
uint32_t Serial_Pow(uint32_t x,uint32_t y);					//计算x^y
void Serial_SendByte(uint8_t Byte);							//通过蓝牙发送一个字节的数据
void Serial_SendNumber(uint32_t Number,uint8_t Length) ;	//通过蓝牙发送一个Length位的数字Number
void Serial_SendString(char *String);						//通过蓝牙发送一个字符串

/*以下为与设置电机转速相关的工具函数*/
void ESCSetThrottle(uint16_t PA8throttle,uint16_t PA9throttle,uint16_t PA10throttle,uint16_t PA11throttle);

/*以下为验证设置的油门信号是否合法的函数*/
void AssertThrottle(uint16_t* throttle);

#endif
