#include "stm32f4xx.h"                  // Device header
#include "ucos_ii.h"
#include "Init.h"
#include "Tool.h"
#include "Task.h"
#include "Cfg.h"
#include <os_trace.h>

/*******************************************
 *										   *
 *      参考系为磁NED系，机体系为FRD系     *
 *										   *
 *******************************************
 */

//任务栈定义
OS_STK StartStk[StartStkSize];
OS_STK BlueToothStk[BlueToothStkSize];
OS_STK GY87Stk[GY87StkSize];
OS_STK ReceiverStk[ReceiverStkSize];
OS_STK MotorStk[MotorStkSize];

//信号量定义
OS_EVENT *Sem_GY87;			//TIM3每10ms通知GY87任务采集
OS_EVENT *Sem_BlueTooth;	//GY87数据就绪后，通知蓝牙任务发送数据，蓝牙发送频率为10Hz，即采集10次数据后才发送一次
OS_EVENT *Sem_Receiver;		//每次PPM信号到来时通知ReceiverTask处理帧信号
OS_EVENT *Sem_FrameReady;	//当一次完整的PPM信号接收完毕后，通知MotorTask处理

extern void  OS_CPU_SysTickInitFreq (INT32U  cpu_freq);

void StartTask(void *p_arg);

int main()
{
	INT8U err;
	BSP();
	OSInit();
	OSTaskCreate(StartTask,(void *)0,(OS_STK *)&StartStk[StartStkSize-1],StartPrio);
	OSTaskNameSet(StartPrio,(INT8U *)"StartTask",&err);
	OSStart();
	while(1)
	{
		;
	}
}

void StartTask(void *p_arg)
{
	(void)p_arg;
	INT8U err;
	
	OS_CPU_SysTickInitFreq(84000000);
	OS_TRACE_INIT();
	
	Sem_GY87=OSSemCreate(0);
	Sem_BlueTooth=OSSemCreate(0);
	Sem_Receiver=OSSemCreate(0);
	Sem_FrameReady=OSSemCreate(0);
	
	OSTaskCreate(BlueToothTask,(void *)0,(OS_STK *)&BlueToothStk[BlueToothStkSize-1],BlueToothPrio);
	OSTaskNameSet(BlueToothPrio,(INT8U *)"BlueToothTask",&err);
	OSTaskCreate(ReceiverTask,(void *)0,(OS_STK *)&ReceiverStk[ReceiverStkSize-1],ReceiverPrio);
	OSTaskNameSet(ReceiverPrio,(INT8U *)"ReceiverTask",&err);
	OSTaskCreate(MotorTask,(void *)0,(OS_STK *)&MotorStk[MotorStkSize-1],MotorPrio);
	OSTaskNameSet(MotorPrio,(INT8U *)"MotorTask",&err);
	
	ESCSetThrottle(1000,1000,1000,1000);
	
	if(Sem_GY87!=0&&OSTaskCreate(GY87Task,(void *)0,(OS_STK *)&GY87Stk[GY87StkSize-1],GY87Prio)==OS_ERR_NONE)
	{
		OSTaskNameSet(GY87Prio,(INT8U *)"GY87Task",&err);
		GY87TimerStart();
	}

	OSTaskDel(OS_PRIO_SELF);
}
