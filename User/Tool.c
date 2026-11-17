#include "stm32f4xx.h"                  // Device header
#include "Tool.h"

void I2C_Start(void)
{
	I2C1->CR1|=(1<<8);
	while(!(I2C1->SR1&0x01));	//等待起始位发送
}

void I2C_Stop(void)
{
	I2C1->CR1|=(1<<9);
}

void I2C_Send_Addr(uint8_t Addr,uint8_t RW)
{
	I2C1->DR=Addr|RW;
	while(!(I2C1->SR1&0x02));	//等待地址发送
	I2C1->SR2;	//前面已经读过SR1了，现在通过读SR2清除SR1中的ADDR位
}

void I2C_Send_Byte(uint8_t data)
{
	I2C1->DR=data;
	while(!(I2C1->SR1&0x04));	//等待数据发送
}

uint8_t I2C_Receive_Byte(void)
{
	while(!(I2C1->SR1&0x40));	//等待RxNE置1
	return I2C1->DR;			//返回读取到的数据
}

uint8_t I2C_Receive_Byte_NACK(void)
{
	I2C1->CR1&=(~0x0400);		//接收数据前清除ACK位
	while(!(I2C1->SR1&0x40));	//等待RxNE置1
	uint8_t tmp=I2C1->DR;		//保存返回值
	I2C1->CR1|=0x0400;			//重新使能ACK以备后续使用
	return tmp;					//返回读取到的数据
}

void Read_MPU6050_Gyro_Values(int16_t* x,int16_t* y,int16_t* z)
{
	uint8_t tmp;
	I2C_Start();
	I2C_Send_Addr(0xD0,0);		//发送设备地址,写模式
	I2C_Send_Byte(0x43);		//发送寄存器地址,GYRO_XOUT_H
	
	/*重新启动为读模式
	 */
	I2C_Start();										//重新发送起始信号
	I2C_Send_Addr(0xD0,1);								//发送设备地址,读模式
	tmp=I2C_Receive_Byte();								//GYRO_XOUT_H
	*x=(int16_t)((tmp<<8)|I2C_Receive_Byte());			//(GYRO_XOUT_H<<8)|GYRO_XOUT_L
	tmp=I2C_Receive_Byte();								//GYRO_YOUT_H
	*y=(int16_t)((tmp<<8)|I2C_Receive_Byte());			//(GYRO_YOUT_H<<8)|GYRO_YOUT_L
	tmp=I2C_Receive_Byte();								//GYRO_ZOUT_H
	*z=(int16_t)((tmp<<8)|I2C_Receive_Byte_NACK());		//(GYRO_ZOUT_H<<8)|GYRO_ZOUT_L
	I2C_Stop();
}

void Read_MPU6050_Accel_Values(int16_t* x,int16_t* y,int16_t* z)
{
	uint8_t tmp;
	I2C_Start();
	I2C_Send_Addr(0xD0,0);								//发送设备地址,写模式
	I2C_Send_Byte(0x3B);								//发送寄存器地址,ACCEL_XOUT_H
	
	/*重新启动为读模式
	 */
	I2C_Start();
	I2C_Send_Addr(0xD0,1);								//发送设备地址,读模式
	tmp=I2C_Receive_Byte();								//ACCEL_XOUT_H
	*x=(int16_t)((tmp<<8)|I2C_Receive_Byte());			//(ACCEL_XOUT_H<<8)|ACCEL_XOUT_L
	tmp=I2C_Receive_Byte();								//ACCEL_YOUT_H
	*y=(int16_t)((tmp<<8)|I2C_Receive_Byte());			//(ACCEL_YOUT_H<<8)|ACCEL_YOUT_L
	tmp=I2C_Receive_Byte();								//ACCEL_ZOUT_H
	*z=(int16_t)((tmp<<8)|I2C_Receive_Byte_NACK());		//(ACCEL_ZOUT_H<<8)|ACCEL_ZOUT_L
	I2C_Stop();
}

void Delay(uint32_t time)
{
	int i=time;
	while(i--);
}

uint32_t Serial_Pow(uint32_t x,uint32_t y)
{
    uint32_t result=1;
    while(y--)
    {
		result *=x;
    }
    return result;
}

void Serial_SendByte(uint8_t Byte)
{
    while (!(USART1->SR&USART_SR_TXE));	//等待发送数据寄存器为空(TXE=1)
    USART1->DR=Byte;					//写入数据
}

void Serial_SendNumber(uint32_t Number,uint8_t Length)
{
    uint8_t i;
    for(i=0;i<Length;i++)
    {
		Serial_SendByte(Number/Serial_Pow(10,Length-i-1)%10+0x30);
    }
}

void Serial_SendString(char *String)
{
	uint8_t i;
    for(i=0;String[i]!='\0';i++)
    {
		Serial_SendByte(String[i]);
    }
}

void ESCSetThrottle(uint16_t PA8throttle,uint16_t PA9throttle,uint16_t PA10throttle,uint16_t PA11throttle)
{
	AssertThrottle(&PA8throttle);
	AssertThrottle(&PA9throttle);
	AssertThrottle(&PA10throttle);
	AssertThrottle(&PA11throttle);
    TIM1->CCR1=PA8throttle;
	TIM1->CCR2=PA9throttle;
	TIM1->CCR3=PA10throttle;
	TIM1->CCR4=PA11throttle;
}

void AssertThrottle(uint16_t* throttle)
{
#ifdef OptimizedThrottleSet
	if(*throttle<ESC_PULSE_MIN||*throttle>ESC_PULSE_SAFE_MAX)
	{
		*throttle=ESC_PULSE_MIN;
	}
	else if(*throttle>ESC_PULSE_MAX&&*throttle<=ESC_PULSE_SAFE_MAX)
	{
		*throttle=ESC_PULSE_MAX;
	}
#else
	if(*throttle<ESC_PULSE_MIN||*throttle>ESC_PULSE_MAX)
	{
		*throttle=ESC_PULSE_MIN;
	}
#endif
}
