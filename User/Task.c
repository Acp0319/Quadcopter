#include "stm32f4xx.h"                  // Device header
#include "ucos_ii.h"
#include "Tool.h"

//共享变量声明
int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;

volatile uint16_t PPMChannels[8]={0};		/*通道1:副翼;通道2:升降;通道3:油门;通道4:方向;
											 *通道5~8:辅助通道
											 */

//测试通道
uint8_t channel=2;

//信号量定义
extern OS_EVENT *Sem_BlueTooth;		//GY86数据就绪后，通知蓝牙任务发送数据
extern OS_EVENT *Sem_Receiver;		//每次PPM信号到来时通知ReceiverTask处理帧信号
extern OS_EVENT *Sem_FrameReady;	//当一次完整的PPM信号接收完毕后，通知MotorTask处理

void BlueToothTask(void *p_arg)
{
	#if OS_CRITICAL_METHOD==3u
	OS_CPU_SR  cpu_sr = 0u;
	#endif
	
	(void)p_arg;
	INT8U err;
	int16_t a_x, a_y, a_z;
	int16_t g_x, g_y, g_z;
	
	while(1)
	{
		OSSemPend(Sem_BlueTooth,0,&err);
		if(err==OS_ERR_NONE)
		{
			//进入临界区获取读到的数据
			OS_ENTER_CRITICAL();
			a_x=accel_x;a_y=accel_y;a_z=accel_z;
			g_x=gyro_x;g_y=gyro_y;g_z=gyro_z;
			OS_EXIT_CRITICAL();
			
			//发送数据
			Serial_SendString("----------Accelerometer----------");
			Serial_SendByte('\n');
			Serial_SendString("X: ");
			if (a_x<0){
				Serial_SendString("-");
				Serial_SendNumber(-a_x,5);
			}else{
				Serial_SendNumber(a_x,5);
			}
			Serial_SendString("Y: ");
			if (a_y<0){
				Serial_SendString("-");
				Serial_SendNumber(-a_y,5);
			}else{
				Serial_SendNumber(a_y,5);
			}
			Serial_SendString("Z: ");
			if (a_z<0){
				Serial_SendString("-");
				Serial_SendNumber(-a_z,5);
			}else{
				Serial_SendNumber(a_z,5);
			}
			Serial_SendString("\n");

			Serial_SendString("------------Gyroscope------------");
			Serial_SendByte('\n');
			Serial_SendString("X: ");
			if (g_x<0){
				Serial_SendString("-");
				Serial_SendNumber(-g_x,5);
			}else{
				Serial_SendNumber(g_x,5);
			}
			Serial_SendString("Y: ");
			if (g_y<0){
				Serial_SendString("-");
				Serial_SendNumber(-g_y,5);
			}else{
				Serial_SendNumber(g_y,5);
			}
			Serial_SendString("Z: ");
			if (g_z<0){
				Serial_SendString("-");
				Serial_SendNumber(-g_z,5);
			}else{
				Serial_SendNumber(g_z,5);
			}
			Serial_SendString("\n\n\n");
		}
	}
}

void GY86Task(void *p_arg)
{
	(void)p_arg;
	INT8U err;
	
	err=err;
	
	#if OS_CRITICAL_METHOD==3u
	OS_CPU_SR  cpu_sr = 0u;
	#endif
	
	while(1)
	{
		OS_ENTER_CRITICAL();
		Read_MPU6050_Accel_Values(&accel_x, &accel_y, &accel_z);
		Read_MPU6050_Gyro_Values(&gyro_x, &gyro_y, &gyro_z);
		OS_EXIT_CRITICAL();
		OSSemPost(Sem_BlueTooth);
		OSTimeDly(30);		//每0.3s读取一次
	}
}

void ReceiverTask(void *p_arg)
{
	(void)p_arg;
	INT8U err;
	
	#if OS_CRITICAL_METHOD==3u
	OS_CPU_SR  cpu_sr = 0u;
	#endif
	
	static uint32_t LastCapture=0;		//上一次捕获到的计数值
    static uint8_t ChannelIndex=0;		//指示下一个要写入PulseWidth的通道
    uint32_t CurrentCapture;			//本次捕获到的计数值
	
	while(1)
	{
		OSSemPend(Sem_Receiver,0,&err);
		if(err==OS_ERR_NONE)
		{
			CurrentCapture=TIM2->CCR1;
			uint32_t PulseWidth;			//本次脉冲的宽度，通过计数值相减获取<--┐
			if(CurrentCapture>LastCapture)								//--┐      |
			{															//  |      |
				PulseWidth=CurrentCapture-LastCapture;					//  |      |
			}															//  |------┘
			else														//  |
			{															//  |
				PulseWidth=(0xFFFF-LastCapture)+CurrentCapture;			//  |
			}															//--┘
			//严格的脉冲宽度验证
			if(PulseWidth>2500)
			{
				//识别为帧同步脉冲，识别一帧的结束
				if(ChannelIndex>=4)	//至少要有4个通道才认为是有效帧
				{
					OSSemPost(Sem_FrameReady);
				}
				ChannelIndex=0;
			} 
			//通道脉冲范围检查
			else if(PulseWidth>=900&&PulseWidth<=2100&&ChannelIndex<8)
			{
				OS_ENTER_CRITICAL();
				PPMChannels[ChannelIndex]=PulseWidth; //将脉冲宽度存入数组
				OS_EXIT_CRITICAL();
				ChannelIndex++;
			}
			//无效脉冲，重置通道计数（可能是噪声）
			else
			{
				ChannelIndex=0;//遇到无效脉冲，重新开始
			}
			LastCapture=CurrentCapture;
		}
	}
}

void MotorTask(void *p_arg)
{
	#if OS_CRITICAL_METHOD==3u
	OS_CPU_SR  cpu_sr = 0u;
	#endif
	
	(void)p_arg;
	INT8U err;
	volatile uint16_t PPMChannelsSaved[8]={0};
	
	while(1)
	{
		OSSemPend(Sem_FrameReady,6,&err);			//第二个参数设置为6，代表6个tick没有等待到信号量就会直接返回继续执行，而不是继续等待，返回的
													//err是OS_ERR_TIMEOUT。该设置用于信号丢失检测。
		if(err==OS_ERR_NONE)
		{
			OS_ENTER_CRITICAL();
			for(int i=0;i<8;i++)
			{
				PPMChannelsSaved[i]=PPMChannels[i];
			}
			OS_EXIT_CRITICAL();
			uint16_t throttle=PPMChannelsSaved[channel];
			//严格的信号验证
            //if(throttle>=1000&&throttle<=2000)
			//{
                //有效信号,设置油门
                ESCSetThrottle(throttle,throttle,throttle,throttle);
            //}
			//else
			//{
                //无效信号,安全停止
                //ESCSetThrottle(1000,1000,1000,1000);
            //}
		}
		else if(err==OS_ERR_TIMEOUT)
		{
			ESCSetThrottle(1000,1000,1000,1000);
		}
	}
}
