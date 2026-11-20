#include "stm32f4xx.h"                  // Device header
#include "Tool.h"
#include "Init.h"
#include <math.h>
#include <stdio.h>

static const float accel_bias[3]=ACCEL_BIAS_INIT;
static const float accel_scale[3]=ACCEL_SCALE_INIT;
static const float gyro_bias[3]=GYRO_BIAS_INIT;
static const float mag_bias[3]=MAG_BIAS_INIT;
static const float mag_matrix[3][3]=MAG_MATRIX_INIT;

static const int8_t frd_raw[3][3]=FRD_RAW;

void I2C_Start(void)
{
	I2C1->CR1|=(1<<8);
	while(!(I2C1->SR1&0x01));	//等待起始位发送
}

void I2C_Stop(void)
{
	I2C1->CR1|=(1<<9);
	while(I2C1->CR1&I2C_CR1_STOP) ;
}

void I2C_Send_Addr(uint8_t Addr,uint8_t RW)
{
	Addr=Addr<<1;
	I2C1->DR=Addr|RW;
	while(!(I2C1->SR1&0x02));	//等待地址发送
	I2C1->SR2;	//前面已经读过SR1了，现在通过读SR2清除SR1中的ADDR位
}

void I2C_Send_Byte(uint8_t data)
{
	I2C1->DR=data;
	while(!(I2C1->SR1&0x04));	//等待数据发送
}

static uint8_t I2C1_ReadWaitFlag(uint32_t flag)
{
    uint32_t n;
    uint32_t sr1;

    for(n=0;n<I2C1_READ_POLL_LIMIT;++n)
    {
        sr1=I2C1->SR1;
		if (sr1&I2C1_READ_ERRORS)
            return 0x00;
        if (sr1&I2C_SR1_BERR)
            I2C1->SR1=I2C1_READ_ERRORS|I2C_SR1_PECERR|I2C_SR1_SMBALERT;
        if (sr1&flag)
            return 0x01;
    }
    return 0x00;
}

static uint8_t I2C1_ReadWaitIdle(void)
{
    uint32_t n;

    for(n=0;n<I2C1_READ_POLL_LIMIT;++n)
    {
        if(!(I2C1->CR1&(I2C_CR1_START|I2C_CR1_STOP))&&!(I2C1->SR2 & I2C_SR2_BUSY))
            return 0x01;
    }
    return 0x00;
}

static void I2C1_ReadClearAddr(void)
{
    (void)I2C1->SR1;
    (void)I2C1->SR2;
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

uint8_t I2C_Read_Registers(uint8_t addr7,uint8_t reg,uint8_t* buf,uint8_t len)
{
    uint32_t primask;
    uint32_t sr1,sr2;

    if(addr7>0x7F||buf==0||len==0||len==2)
        return 0x00;
    if(!I2C1_ReadWaitIdle())
        return 0x00;
    I2C1->CR1=(I2C1->CR1&~I2C_CR1_POS)|I2C_CR1_ACK|I2C_CR1_START;
    if(!I2C1_ReadWaitFlag(I2C_SR1_SB)) goto fail;
    I2C1->DR=(uint32_t)addr7<<1;
    if(!I2C1_ReadWaitFlag(I2C_SR1_ADDR)) goto fail;
    I2C1_ReadClearAddr();
    if(!I2C1_ReadWaitFlag(I2C_SR1_TXE)) goto fail;
    I2C1->DR=reg;
    if(!I2C1_ReadWaitFlag(I2C_SR1_BTF)) goto fail;
    I2C1->CR1|=I2C_CR1_START;
    if(!I2C1_ReadWaitFlag(I2C_SR1_SB)) goto fail;
    I2C1->DR=((uint32_t)addr7<<1)|1U;
    if(!I2C1_ReadWaitFlag(I2C_SR1_ADDR)) goto fail;
    if(len==1)
    {
        /*关ACK→清ADDR→请求STOP→等待并读取*/
        primask=__get_PRIMASK();
        __disable_irq();
        I2C1->CR1&=~I2C_CR1_ACK;
        I2C1_ReadClearAddr();
        I2C1->CR1|=I2C_CR1_STOP;
        __set_PRIMASK(primask);
        if(!I2C1_ReadWaitFlag(I2C_SR1_RXNE)) goto fail;
        buf[0]=(uint8_t)I2C1->DR;
    }
    else
    {
        I2C1_ReadClearAddr();
        while(len>3)
        {
            if(!I2C1_ReadWaitFlag(I2C_SR1_RXNE)) goto fail;
            *buf++=(uint8_t)I2C1->DR;
            --len;
        }

        /*剩三字节：等BTF，关ACK，读倒数第三个*/
        if(!I2C1_ReadWaitFlag(I2C_SR1_BTF)) goto fail;
        primask=__get_PRIMASK();
        __disable_irq();
        I2C1->CR1&=~I2C_CR1_ACK;
        *buf++=(uint8_t)I2C1->DR;
        __set_PRIMASK(primask);
        /*再等BTF，先请求STOP，再读最后两个*/
        if(!I2C1_ReadWaitFlag(I2C_SR1_BTF)) goto fail;
        primask=__get_PRIMASK();
        __disable_irq();
        I2C1->CR1|=I2C_CR1_STOP;
        *buf++=(uint8_t)I2C1->DR;
        *buf=(uint8_t)I2C1->DR;
        __set_PRIMASK(primask);
    }
    if(!I2C1_ReadWaitIdle()) goto fail;
    I2C1->CR1|=I2C_CR1_ACK;
    return 0x01;

fail:
    /*若仍是主机且没有挂起的START/STOP，先尝试正常结束*/
    sr1=I2C1->SR1;
    sr2=I2C1->SR2;
    if(!(sr1&I2C_SR1_ARLO)&&(sr2&I2C_SR2_MSL)&&!(I2C1->CR1&(I2C_CR1_START|I2C_CR1_STOP)))
    {
        I2C1->CR1|=I2C_CR1_STOP;
        (void)I2C1_ReadWaitIdle();
    }
    /*复位STM32的I2C1外设，丢弃残留状态，再恢复原配置*/
    RCC->APB1RSTR|=RCC_APB1RSTR_I2C1RST;
    (void)RCC->APB1RSTR;
    RCC->APB1RSTR&=~RCC_APB1RSTR_I2C1RST;
    I2C1_Init();
    return 0x00;
}

void Read_MPU6050_Gyro_Values(int16_t* x,int16_t* y,int16_t* z)
{
	uint8_t tmp[6];
	
	if(!I2C_Read_Registers(0x68,0x43,tmp,6))
	{
		ErrorHandle();
	}
	
	*x=(int16_t)((tmp[0]<<8)|tmp[1]);
	*y=(int16_t)((tmp[2]<<8)|tmp[3]);
	*z=(int16_t)((tmp[4]<<8)|tmp[5]);
}

void Read_MPU6050_Accel_Values(int16_t* x,int16_t* y,int16_t* z)
{
	uint8_t tmp[6];
	
	if(!I2C_Read_Registers(0x68,0x3B,tmp,6))
	{
		ErrorHandle();
	}
	
	*x=(int16_t)((tmp[0]<<8)|tmp[1]);
	*y=(int16_t)((tmp[2]<<8)|tmp[3]);
	*z=(int16_t)((tmp[4]<<8)|tmp[5]);
}

uint8_t Read_HMC5883L_Values(int16_t* x,int16_t* y,int16_t* z)
{
	uint8_t tmp[6];
	uint8_t result;
	
	if(!I2C_Read_Registers(0x1E,0x09,&result,1))
	{
		ErrorHandle();
	}
	if(!(result&0x01))
	{
		return 0x00;		//数据未更新
	}
	if(!I2C_Read_Registers(0x1E,0x03,tmp,6))
	{
		ErrorHandle();
	}
	
	*x=(int16_t)((tmp[0]<<8)|tmp[1]);
	*y=(int16_t)((tmp[4]<<8)|tmp[5]);
	*z=(int16_t)((tmp[2]<<8)|tmp[3]);
	return 0x01;
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

void Serial_SendFloat(float value)
{
    char buffer[64];
    snprintf(buffer,sizeof(buffer),"%.6f",(double)value);
    Serial_SendString(buffer);
}

void Serial_SendString(char *String)
{
	uint8_t i;
    for(i=0;String[i]!='\0';i++)
    {
		Serial_SendByte(String[i]);
    }
}

void Serial_SendInt16(int16_t num)
{
	if(num<0)
	{
		Serial_SendByte('-');
		Serial_SendNumber(-num,5);
	}
	else
	{
		Serial_SendNumber(num,5);
	}
}

void DataTransfer(RawSensorFrame rawdata,SensorFrame data,Quaternion q,EulerAngles e)
{
#if UseCalibrationMode==1
	/*校准模式数据发送的格式：timestamp,mag_timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z
	 *每行一帧，每行的行首有固定前缀"CalibrationMode:"
	 */
	Serial_SendString("CalibrationMode:");
	
	Serial_SendNumber(rawdata.timestamp,10);
	Serial_SendByte(',');
	
	Serial_SendNumber(rawdata.mag_timestamp,10);
	Serial_SendByte(',');
	
	Serial_SendInt16(rawdata.accel[0]);
	Serial_SendByte(',');
	Serial_SendInt16(rawdata.accel[1]);
	Serial_SendByte(',');
	Serial_SendInt16(rawdata.accel[2]);
	Serial_SendByte(',');
	
	Serial_SendInt16(rawdata.gyro[0]);
	Serial_SendByte(',');
	Serial_SendInt16(rawdata.gyro[1]);
	Serial_SendByte(',');
	Serial_SendInt16(rawdata.gyro[2]);
	Serial_SendByte(',');
	
	Serial_SendInt16(rawdata.mag[0]);
	Serial_SendByte(',');
	Serial_SendInt16(rawdata.mag[1]);
	Serial_SendByte(',');
	Serial_SendInt16(rawdata.mag[2]);
	Serial_SendByte('\n');
#endif

#if UseWorkMode==1
	Serial_SendString("-------------------WorkMode-------------------\n\n");
	
	Serial_SendString("a_x:\t");
	Serial_SendFloat(data.accel[0]);
	Serial_SendByte('\n');
	Serial_SendString("a_y:\t");
	Serial_SendFloat(data.accel[1]);
	Serial_SendByte('\n');
	Serial_SendString("a_z:\t");
	Serial_SendFloat(data.accel[2]);
	Serial_SendString("\n\n");
	
	Serial_SendString("g_x:\t");
	Serial_SendFloat(data.gyro[0]);
	Serial_SendByte('\n');
	Serial_SendString("g_y:\t");
	Serial_SendFloat(data.gyro[1]);
	Serial_SendByte('\n');
	Serial_SendString("g_z:\t");
	Serial_SendFloat(data.gyro[2]);
	Serial_SendString("\n\n");
	
	Serial_SendString("m_x:\t");
	Serial_SendFloat(data.mag[0]);
	Serial_SendByte('\n');
	Serial_SendString("m_y:\t");
	Serial_SendFloat(data.mag[1]);
	Serial_SendByte('\n');
	Serial_SendString("m_z:\t");
	Serial_SendFloat(data.mag[2]);
	Serial_SendString("\n\n");
	
	Serial_SendString("Quaternion:\t[");
	Serial_SendFloat(q.w);
	Serial_SendByte(',');
	Serial_SendFloat(q.x);
	Serial_SendByte(',');
	Serial_SendFloat(q.y);
	Serial_SendByte(',');
	Serial_SendFloat(q.z);
	Serial_SendString("]\n\n");
	
	Serial_SendString("roll:\t");
	Serial_SendFloat(e.roll);
	Serial_SendByte('\n');
	Serial_SendString("pitch:\t");
	Serial_SendFloat(e.pitch);
	Serial_SendByte('\n');
	Serial_SendString("yaw:\t");
	Serial_SendFloat(e.yaw);
	Serial_SendString("\n\n\n");
#endif
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

void ErrorHandle(void)
{
	//当前暂时不添加具体处理，后续再完善
	while(1) ;
}

void Cal_3_3_plus_3_1(float A[3][3],float b[3],float result[3])
{
	int i=0,j=0;
	float tmp=0.0f;
	for(i=0;i<3;i++)
	{
		tmp=0.0f;
		for(j=0;j<3;j++)
		{
			tmp+=A[i][j]*b[j];
		}
		result[i]=tmp;
	}
}

void Transpose(float m[3][3],float result[3][3])
{
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			result[j][i]=m[i][j];
		}
	}
}

void CalibrationAndUnitConversion(RawSensorFrame raw,SensorFrame* corrected)
{
	corrected->timestamp=raw.timestamp;
	corrected->mag_timestamp=raw.mag_timestamp;
	
	int i=0,j=0;
	for(i=0;i<3;i++)
	{
		corrected->accel[i]=accel_scale[i]*((float)raw.accel[i]/16384.0f-accel_bias[i]);
	}
	for(i=0;i<3;i++)
	{
		corrected->gyro[i]=(((float)raw.gyro[i]-gyro_bias[i])*PI)/(131.0f*180.0f);
	}
	float tmp[3];
	for(i=0;i<3;i++)
	{
		tmp[i]=(float)raw.mag[i]-mag_bias[i];
	}
	float tmp_mag_matrix[3][3];
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			tmp_mag_matrix[i][j]=mag_matrix[i][j];
		}
	}
	Cal_3_3_plus_3_1(tmp_mag_matrix,tmp,corrected->mag);
}

void RawToFRD(SensorFrame* raw,SensorFrame* result)
{
	result->timestamp=raw->timestamp;
	result->mag_timestamp=raw->mag_timestamp;
	
	int i=0;
	for(i=0;i<3;i++)
	{
		result->accel[i]=frd_raw[2][i]*raw->accel[frd_raw[1][i]];
	}
	for(i=0;i<3;i++)
	{
		result->gyro[i]=frd_raw[2][i]*raw->gyro[frd_raw[1][i]];
	}
	for(i=0;i<3;i++)
	{
		result->mag[i]=frd_raw[2][i]*raw->mag[frd_raw[1][i]];
	}
}

void InitializeQuaternion(Quaternion* q,SensorFrame data)
{
	float u_b[3],m_lev[3],n_b[3],w_b[3],C[4];	//"_b"代表是机体系下的坐标
	float R[3][3];
	uint8_t index=0;
	Quaternion tmp;
	if(!NormVector(data.accel,u_b))
	{
		ErrorHandle();
	}
	float m_u_b=0.0f;
	m_u_b=VectorDotProduct(data.mag,u_b);
	for(int i=0;i<3;i++)
	{
		m_lev[i]=data.mag[i]-m_u_b*u_b[i];
	}
	if(!NormVector(m_lev,n_b))
	{
		ErrorHandle();
	}
	VectorCrossProduct(u_b,n_b,w_b);
	if(!NormVector(w_b,w_b))
	{
		ErrorHandle();
	}
	VectorCrossProduct(w_b,u_b,n_b);
	for(int i=0;i<3;i++)
	{
		R[0][i]=n_b[i];
	}
	for(int i=0;i<3;i++)
	{
		R[1][i]=-w_b[i];
	}
	for(int i=0;i<3;i++)
	{
		R[2][i]=-u_b[i];
	}
	C[0]=1+R[0][0]+R[1][1]+R[2][2];
	C[1]=1+R[0][0]-R[1][1]-R[2][2];
	C[2]=1-R[0][0]+R[1][1]-R[2][2];
	C[3]=1-R[0][0]-R[1][1]+R[2][2];
	index=MaxValueIndex(C,4);
	switch(index)
	{
		case 0:
			tmp.w=sqrtf(C[0])/2.0f;
			tmp.x=(R[2][1]-R[1][2])/(4.0f*tmp.w);
			tmp.y=(R[0][2]-R[2][0])/(4.0f*tmp.w);
			tmp.z=(R[1][0]-R[0][1])/(4.0f*tmp.w);
			break;
		case 1:
			tmp.x=sqrtf(C[1])/2.0f;
			tmp.w=(R[2][1]-R[1][2])/(4.0f*tmp.x);
			tmp.y=(R[0][1]+R[1][0])/(4.0f*tmp.x);
			tmp.z=(R[0][2]+R[2][0])/(4.0f*tmp.x);
			break;
		case 2:
			tmp.y=sqrtf(C[2])/2.0f;
			tmp.w=(R[0][2]-R[2][0])/(4.0f*tmp.y);
			tmp.x=(R[0][1]+R[1][0])/(4.0f*tmp.y);
			tmp.z=(R[1][2]+R[2][1])/(4.0f*tmp.y);
			break;
		case 3:
			tmp.z=sqrtf(C[3])/2.0f;
			tmp.w=(R[1][0]-R[0][1])/(4.0f*tmp.z);
			tmp.x=(R[0][2]+R[2][0])/(4.0f*tmp.z);
			tmp.y=(R[1][2]+R[2][1])/(4.0f*tmp.z);
			break;
		default:
			ErrorHandle();
			break;
	}
	if(!NormQuaternion(&tmp,q))
	{
		ErrorHandle();
	}
}

void QuaternionMulC(Quaternion* a,float c,Quaternion* result)
{
	float w,x,y,z;
	w=a->w*c;
	x=a->x*c;
	y=a->y*c;
	z=a->z*c;
	result->w=w;
	result->x=x;
	result->y=y;
	result->z=z;
}

void QuaternionMul(Quaternion* a,Quaternion* b,Quaternion* result)
{
	float w,x,y,z;
	w=a->w*b->w-a->x*b->x-a->y*b->y-a->z*b->z;
	x=a->w*b->x+a->x*b->w+a->y*b->z-a->z*b->y;
	y=a->w*b->y+a->y*b->w+a->z*b->x-a->x*b->z;
	z=a->w*b->z+a->z*b->w+a->x*b->y-a->y*b->x;
	result->w=w;
	result->x=x;
	result->y=y;
	result->z=z;
}

void MatrixOfQuaternion(Quaternion* q,float Rq[3][3])
{
	Rq[0][0]=1-2*(q->y*q->y+q->z*q->z);
	Rq[0][1]=2*(q->x*q->y-q->w*q->z);
	Rq[0][2]=2*(q->x*q->z+q->w*q->y);
	Rq[1][0]=2*(q->x*q->y+q->w*q->z);
	Rq[1][1]=1-2*(q->x*q->x+q->z*q->z);
	Rq[1][2]=2*(q->y*q->z-q->w*q->x);
	Rq[2][0]=2*(q->x*q->z-q->w*q->y);
	Rq[2][1]=2*(q->y*q->z+q->w*q->x);
	Rq[2][2]=1-2*(q->x*q->x+q->y*q->y);
}

float VectorDotProduct(float a[3],float b[3])
{
	float result=0.0f;
	for(int i=0;i<3;i++)
	{
		result+=a[i]*b[i];
	}
	return result;
}

void VectorCrossProduct(float a[3],float b[3],float result[3])
{
	float t1,t2,t3;
	t1=a[1]*b[2]-a[2]*b[1];
	t2=a[2]*b[0]-a[0]*b[2];
	t3=a[0]*b[1]-a[1]*b[0];
	result[0]=t1;
	result[1]=t2;
	result[2]=t3;
}

uint8_t NormVector(float m[3],float result[3])
{
	float tmp;
	tmp=m[0]*m[0]+m[1]*m[1]+m[2]*m[2];
	tmp=sqrtf(tmp);
	if(tmp==0.0f)
	{
		return 0x00;	//模长为0
	}
	for(int i=0;i<3;i++)
	{
		result[i]=m[i]/tmp;
	}
	return 0x01;
}

uint8_t NormQuaternion(Quaternion* q,Quaternion* result)
{
	float tmp;
	tmp=sqrtf(q->w*q->w+q->x*q->x+q->y*q->y+q->z*q->z);
	if(tmp==0.0f)
	{
		return 0x00;	//模长为0
	}
	result->w=q->w/tmp;
	result->x=q->x/tmp;
	result->y=q->y/tmp;
	result->z=q->z/tmp;
	return 0x01;
}

void QuaternionToEulerAngles(float Rq[3][3],EulerAngles* e)
{
	if(sqrtf(Rq[0][0]*Rq[0][0]+Rq[1][0]*Rq[1][0])<EPSILON)
	{
		e->roll=0.0f;
		e->yaw=atan2f(-Rq[0][1],Rq[1][1]);
	}
	else
	{
		e->roll=atan2f(Rq[2][1],Rq[2][2]);
		e->yaw=atan2f(Rq[1][0],Rq[0][0]);
	}
	e->pitch=asinf(Clip(-Rq[2][0],-1.0f,1.0f));
}

uint8_t MaxValueIndex(float* arr,uint8_t len)
{
	uint8_t result=len-1;
	float maxvalue=arr[len-1];
	len--;
	while(len--)
	{
		if(arr[len]>maxvalue)
		{
			result=len;
			maxvalue=arr[len];
		}
	}
	return result;
}

void UpdateUnitvectorMag(float Rq[3][3],float mag[3],float result[3])
{
	float h[3];
	Cal_3_3_plus_3_1(Rq,mag,h);
	result[0]=sqrtf(h[0]*h[0]+h[1]*h[1]);
	result[1]=0.0f;
	result[2]=h[2];
}

void CalJAccel(Quaternion* q,float J[3][4])
{
	J[0][0]=2.0f*q->y;
	J[0][1]=-2.0f*q->z;
	J[0][2]=2.0f*q->w;
	J[0][3]=-2.0f*q->x;
	J[1][0]=-2.0f*q->x;
	J[1][1]=-2.0f*q->w;
	J[1][2]=-2.0f*q->z;
	J[1][3]=-2.0f*q->y;
	J[2][0]=0.0f;
	J[2][1]=4.0f*q->x;
	J[2][2]=4.0f*q->y;
	J[2][3]=0.0f;
}

void CalJMag(Quaternion* q,float b[3],float J[3][4])
{
	J[0][0]=-2.0f*b[2]*q->y;
	J[0][1]=2.0f*b[2]*q->z;
	J[0][2]=-4.0f*b[0]*q->y-2.0f*b[2]*q->w;
	J[0][3]=-4.0f*b[0]*q->z+2.0f*b[2]*q->x;
	J[1][0]=-2.0f*b[0]*q->z+2.0f*b[2]*q->x;
	J[1][1]=2.0f*b[0]*q->y+2.0f*b[2]*q->w;
	J[1][2]=2.0f*b[0]*q->x+2.0f*b[2]*q->z;
	J[1][3]=-2.0f*b[0]*q->w+2.0f*b[2]*q->y;
	J[2][0]=2.0f*b[0]*q->y;
	J[2][1]=2.0f*b[0]*q->z-4.0f*b[2]*q->x;
	J[2][2]=2.0f*b[0]*q->w-4.0f*b[2]*q->y;
	J[2][3]=2.0f*b[0]*q->x;
}

float Clip(float a,float leftclip,float rightclip)
{
	if(a<leftclip) return leftclip;
	else if(a>rightclip) return rightclip;
	else return a;
}

#if UseTheHostComputerToVerifyAttitudeCalculation==1
void HostComputerInit(void)
{
    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR|=RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;

    /*PA2为USART2_TX，PA3为USART2_RX，默认由板载ST-LINK连接到USB虚拟串口。*/
    GPIOA->MODER&=~((3UL<<4)|(3UL<<6));
    GPIOA->MODER|=(2UL<<4)|(2UL<<6);
    GPIOA->OTYPER&=~((1UL<<2)|(1UL<<3));
    GPIOA->OSPEEDR&=~((3UL<<4)|(3UL<<6));
    GPIOA->OSPEEDR|=(2UL<<4)|(2UL<<6);
    GPIOA->PUPDR&=~((3UL<<4)|(3UL<<6));
    GPIOA->PUPDR|=(1UL<<4)|(1UL<<6);
    GPIOA->AFR[0]&=~((0xFUL<<8)|(0xFUL<<12));
    GPIOA->AFR[0]|=(7UL<<8)|(7UL<<12);

    USART2->CR1=0;
    USART2->CR2=0;
    USART2->CR3=0;
    USART2->BRR=365;                    //APB1为42MHz，42MHz/115200取最近整数
    USART2->CR1=USART_CR1_TE|USART_CR1_UE;//8位数据、无校验、1位停止位，启用发送
}

void HostComputerSendByte(uint8_t data)
{
    while(!(USART2->SR&USART_SR_TXE));
    USART2->DR=data;
}

void HostComputerSendAttitude(EulerAngles attitude)
{
    /*V7.3内嵌协议V7.16：AA FF 03 07 ROL PIT YAW FUSION_STA SC AC。*/
    uint8_t frame[13]={0xAA,0xFF,0x03,0x07};
    float angles[3]={attitude.roll,attitude.pitch,attitude.yaw};
    uint8_t sumcheck=0,addcheck=0;

    for(uint8_t i=0;i<3;i++)
    {
        float angle=angles[i]*(18000.0f/PI); //rad转为0.01度
        int16_t angle_int;
        uint16_t angle_bits;

        if(!isfinite(angle)) return;
        angle=Clip(angle,-32768.0f,32767.0f);
        angle_int=(int16_t)(angle+(angle>=0.0f?0.5f:-0.5f));
        angle_bits=(uint16_t)angle_int;
        frame[4+2*i]=(uint8_t)angle_bits;    //协议规定低字节在前
        frame[5+2*i]=(uint8_t)(angle_bits>>8);
    }
    frame[10]=0; //协议未定义FUSION_STA的取值，本项目填0占位

    for(uint8_t i=0;i<11;i++)
    {
        sumcheck+=frame[i];
        addcheck+=sumcheck;
    }
    frame[11]=sumcheck;
    frame[12]=addcheck;
    for(uint8_t i=0;i<13;i++)
    {
        HostComputerSendByte(frame[i]);
    }
}
#endif
