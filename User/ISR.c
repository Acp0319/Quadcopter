#include "stm32f4xx.h"                  // Device header
#include "ucos_ii.h"

extern OS_EVENT *Sem_Receiver;
extern OS_EVENT *Sem_GY87;


void TIM2_IRQHandler(void)
{
	OSIntEnter();  //进入中断
	if((TIM2->SR&TIM_SR_CC1IF)&&(TIM2->DIER&TIM_DIER_CC1IE))
	{
		OSSemPost(Sem_Receiver);
		TIM2->SR&=~TIM_SR_CC1IF;
	}
	OSIntExit();   //退出中断
}

void TIM3_IRQHandler(void)
{
#if OS_CRITICAL_METHOD==3u
	OS_CPU_SR cpu_sr=0u;
#endif

	if((TIM3->SR&TIM_SR_UIF)&&(TIM3->DIER&TIM_DIER_UIE))
	{
		TIM3->SR=0;
		OS_ENTER_CRITICAL();
		OSIntEnter();
		OS_EXIT_CRITICAL();
		OSSemPost(Sem_GY87);
		OSIntExit();
	}
}
