#include "stm32f4xx.h"                  // Device header
#include "ucos_ii.h"
#include "Tool.h"

//共享变量
RawSensorFrame raw_sensorframe;				//从传感器读取到的原始数据
SensorFrame corrected_raw_sensorframe;		//经过校准之后的数据，但还没有转换到FRD坐标系
SensorFrame sensorframe;					//转换到FRD后的数据

Quaternion quaternion;			//四元数
EulerAngles eulerangles;		//欧拉角
float RotationMatrix[3][3];		//旋转矩阵

#if UseTheHostComputerToVerifyAttitudeCalculation==1
static uint8_t host_attitude_ready=0;
#endif

volatile uint16_t PPMChannels[8]={0};		/*通道1:副翼;通道2:升降;通道3:油门;通道4:方向;
											 *通道5~8:辅助通道
											 */

//测试通道
uint8_t channel=2;

//信号量定义
extern OS_EVENT *Sem_BlueTooth;		//GY87数据就绪后，通知蓝牙任务发送数据
extern OS_EVENT *Sem_GY87;		//TIM3通知GY87Task采集数据
extern OS_EVENT *Sem_Receiver;		//每次PPM信号到来时通知ReceiverTask处理帧信号
extern OS_EVENT *Sem_FrameReady;	//当一次完整的PPM信号接收完毕后，通知MotorTask处理

void BlueToothTask(void *p_arg)
{
	#if OS_CRITICAL_METHOD==3u
	OS_CPU_SR  cpu_sr = 0u;
	#endif
	
	(void)p_arg;
	INT8U err;
	
	RawSensorFrame bluetooth_raw_sensorframe;
	SensorFrame bluetooth_sensorframe;
	Quaternion bluetooth_quaternion;
	EulerAngles bluetooth_eulerangles;
	
#if UseTheHostComputerToVerifyAttitudeCalculation==1
	uint8_t bluetooth_attitude_ready;
#endif
	
	while(1)
	{
		OSSemPend(Sem_BlueTooth,0,&err);
		if(err==OS_ERR_NONE)
		{
			//进入临界区获取读到的数据
			OS_ENTER_CRITICAL();
			bluetooth_raw_sensorframe=raw_sensorframe;
			bluetooth_sensorframe=sensorframe;
			bluetooth_quaternion=quaternion;
			bluetooth_eulerangles=eulerangles;
			
#if UseTheHostComputerToVerifyAttitudeCalculation==1
			bluetooth_attitude_ready=host_attitude_ready;
#endif
			
			OS_EXIT_CRITICAL();
			
			//发送数据
			DataTransfer(bluetooth_raw_sensorframe,bluetooth_sensorframe,bluetooth_quaternion,bluetooth_eulerangles);
			
#if UseTheHostComputerToVerifyAttitudeCalculation==1
			if(bluetooth_attitude_ready)
			{
				HostComputerSendAttitude(bluetooth_eulerangles);
			}
#endif
			
		}
	}
}

void GY87Task(void *p_arg)
{
	(void)p_arg;
	INT8U err;
	
	uint8_t firstexec=1;
	uint8_t frames_since_send=0;
	RawSensorFrame gy87_raw_sensorframe;
	SensorFrame gy87_corrected_raw_sensorframe;
	SensorFrame gy87_sensorframe;
	SensorFrame last_gy87_sensorframe;
	float dt=0.0f;
	uint32_t maxcount=0xFFFFFFFF;	//DWT->CYCCNT的最大值
	Quaternion dq_w;
	Quaternion dq;
	Quaternion s;
	Quaternion Omega;
	Quaternion gy87_quaternion;				//四元数
	EulerAngles gy87_eulerangles;			//欧拉角
	float gy87_RotationMatrix[3][3];		//旋转矩阵
	float normed_accel[3],normed_mag[3],unitvector_up[3]={0.0f,0.0f,-1.0f},unitvector_mag[3],predicted_accel[3],predicted_mag[3],r[6];
	float transposed_q[3][3],J_accel[3][4],J_mag[3][4];
	uint8_t mag_readed=0;
	uint8_t mag_updated=0;
	
	Omega.w=0.0f;
	
	#if OS_CRITICAL_METHOD==3u
	OS_CPU_SR  cpu_sr = 0u;
	#endif
	
	while(1)
	{
		OSSemPend(Sem_GY87,0,&err);
		if(err!=OS_ERR_NONE) continue;
		//合并积压的定时通知，只采集当前一帧。
		OSSemSet(Sem_GY87,0,&err);
		if(err!=OS_ERR_NONE) continue;

		gy87_raw_sensorframe.timestamp=DWT->CYCCNT;
		Read_MPU6050_Accel_Values(&gy87_raw_sensorframe.accel[0], &gy87_raw_sensorframe.accel[1], &gy87_raw_sensorframe.accel[2]);
		Read_MPU6050_Gyro_Values(&gy87_raw_sensorframe.gyro[0], &gy87_raw_sensorframe.gyro[1], &gy87_raw_sensorframe.gyro[2]);
		if(Read_HMC5883L_Values(&gy87_raw_sensorframe.mag[0],&gy87_raw_sensorframe.mag[1],&gy87_raw_sensorframe.mag[2])==0x01)
		{
			mag_readed=1;
			gy87_raw_sensorframe.mag_timestamp=DWT->CYCCNT;
		}
		CalibrationAndUnitConversion(gy87_raw_sensorframe,&gy87_corrected_raw_sensorframe);
		
		last_gy87_sensorframe=gy87_sensorframe;
		
		RawToFRD(&gy87_corrected_raw_sensorframe,&gy87_sensorframe);
		OS_ENTER_CRITICAL();
		raw_sensorframe=gy87_raw_sensorframe;
		corrected_raw_sensorframe=gy87_corrected_raw_sensorframe;
		sensorframe=gy87_sensorframe;
		OS_EXIT_CRITICAL();
		
		if(firstexec)
		{
			if(mag_readed)
			{
				InitializeQuaternion(&gy87_quaternion,gy87_sensorframe);
				MatrixOfQuaternion(&gy87_quaternion,gy87_RotationMatrix);
				Transpose(gy87_RotationMatrix,transposed_q);
				QuaternionToEulerAngles(gy87_RotationMatrix,&gy87_eulerangles);
				OS_ENTER_CRITICAL();
				quaternion=gy87_quaternion;
				for(int i=0;i<3;i++)
				{
					for(int j=0;j<3;j++)
					{
						RotationMatrix[i][j]=gy87_RotationMatrix[i][j];
					}
				}
				eulerangles=gy87_eulerangles;
#if UseTheHostComputerToVerifyAttitudeCalculation==1
				host_attitude_ready=1;
#endif
				OS_EXIT_CRITICAL();
				firstexec=0;
			}
		}
		else
		{
			if(last_gy87_sensorframe.mag_timestamp==gy87_sensorframe.mag_timestamp)
			{
				mag_updated=0;
			}
			else
			{
				mag_updated=1;
			}
			if(last_gy87_sensorframe.timestamp>gy87_sensorframe.timestamp)
			{
				dt=((float)(maxcount-last_gy87_sensorframe.timestamp+gy87_sensorframe.timestamp+1))/84000000.0f;
			}
			else
			{
				dt=((float)(gy87_sensorframe.timestamp-last_gy87_sensorframe.timestamp))/84000000.0f;
			}
			Omega.x=gy87_sensorframe.gyro[0];
			Omega.y=gy87_sensorframe.gyro[1];
			Omega.z=gy87_sensorframe.gyro[2];
			QuaternionMul(&gy87_quaternion,&Omega,&dq_w);
			QuaternionMulC(&dq_w,0.5f,&dq_w);
			if(!NormVector(gy87_sensorframe.accel,normed_accel))
			{
				QuaternionMulC(&dq_w,dt,&dq_w);
				gy87_quaternion.w=gy87_quaternion.w+dq_w.w;
				gy87_quaternion.x=gy87_quaternion.x+dq_w.x;
				gy87_quaternion.y=gy87_quaternion.y+dq_w.y;
				gy87_quaternion.z=gy87_quaternion.z+dq_w.z;
				if(!NormQuaternion(&gy87_quaternion,&gy87_quaternion))
				{
					ErrorHandle();
				}
				MatrixOfQuaternion(&gy87_quaternion,gy87_RotationMatrix);
				Transpose(gy87_RotationMatrix,transposed_q);
				QuaternionToEulerAngles(gy87_RotationMatrix,&gy87_eulerangles);
				
				OS_ENTER_CRITICAL();
				quaternion=gy87_quaternion;
				for(int i=0;i<3;i++)
				{
					for(int j=0;j<3;j++)
					{
						RotationMatrix[i][j]=gy87_RotationMatrix[i][j];
					}
				}
				eulerangles=gy87_eulerangles;
				OS_EXIT_CRITICAL();
				continue;
			}
			Cal_3_3_plus_3_1(transposed_q,unitvector_up,predicted_accel);
			for(int i=0;i<3;i++)
			{
				r[i]=predicted_accel[i]-normed_accel[i];
			}
			CalJAccel(&gy87_quaternion,J_accel);
			s.w=0.0f;
			s.x=0.0f;
			s.y=0.0f;
			s.z=0.0f;
			for(int i=0;i<4;i++)
			{
				for(int j=0;j<3;j++)
				{
					if(i==0) s.w+=J_accel[j][i]*r[j];
					if(i==1) s.x+=J_accel[j][i]*r[j];
					if(i==2) s.y+=J_accel[j][i]*r[j];
					if(i==3) s.z+=J_accel[j][i]*r[j];
				}
			}
			if(mag_updated)
			{
				if(NormVector(gy87_sensorframe.mag,normed_mag))
				{
					UpdateUnitvectorMag(gy87_RotationMatrix,normed_mag,unitvector_mag);
					Cal_3_3_plus_3_1(transposed_q,unitvector_mag,predicted_mag);
					for(int i=3;i<6;i++)
					{
						r[i]=predicted_mag[i-3]-normed_mag[i-3];
					}
					CalJMag(&gy87_quaternion,unitvector_mag,J_mag);
					for(int i=0;i<4;i++)
					{
						for(int j=0;j<3;j++)
						{
							if(i==0) s.w+=J_mag[j][i]*r[j+3];
							if(i==1) s.x+=J_mag[j][i]*r[j+3];
							if(i==2) s.y+=J_mag[j][i]*r[j+3];
							if(i==3) s.z+=J_mag[j][i]*r[j+3];
						}
					}
				}
			}
			if(!NormQuaternion(&s,&s))
			{
				s.w=0.0f;
				s.x=0.0f;
				s.y=0.0f;
				s.z=0.0f;
			}
			QuaternionMulC(&s,BETA,&s);
			dq.w=dq_w.w-s.w;
			dq.x=dq_w.x-s.x;
			dq.y=dq_w.y-s.y;
			dq.z=dq_w.z-s.z;
			QuaternionMulC(&dq,dt,&dq);
			dq.w=gy87_quaternion.w+dq.w;
			dq.x=gy87_quaternion.x+dq.x;
			dq.y=gy87_quaternion.y+dq.y;
			dq.z=gy87_quaternion.z+dq.z;
			if(!NormQuaternion(&dq,&gy87_quaternion))
			{
				ErrorHandle();
			}
			MatrixOfQuaternion(&gy87_quaternion,gy87_RotationMatrix);
			Transpose(gy87_RotationMatrix,transposed_q);
			QuaternionToEulerAngles(gy87_RotationMatrix,&gy87_eulerangles);
		}
		
		if(!firstexec)
		{
			OS_ENTER_CRITICAL();
			quaternion=gy87_quaternion;
			for(int i=0;i<3;i++)
			{
				for(int j=0;j<3;j++)
				{
					RotationMatrix[i][j]=gy87_RotationMatrix[i][j];
				}
			}
			eulerangles=gy87_eulerangles;
			OS_EXIT_CRITICAL();
		}
		
		//每采集10帧通知一次，100Hz采集时蓝牙约10Hz
		if(++frames_since_send==10U)
		{
			frames_since_send=0;
			OSSemPost(Sem_BlueTooth);
		}
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
