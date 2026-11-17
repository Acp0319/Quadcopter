#include "stm32f4xx.h"                  // Device header
#include "ucos_ii.h"

extern OS_EVENT *Sem_Receiver;


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
