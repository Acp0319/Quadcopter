#define   OS_CPU_GLOBALS

#include "os_cpu.h" 
#include "ucos_ii.h"
	
#define OS_CPU_CM_SYST_CSR         (*((volatile INT32U *)0xE000E010uL))
#define OS_CPU_CM_SYST_RVR         (*((volatile INT32U *)0xE000E014uL))
#define OS_CPU_CM_SCB_SHPRI3       (*((volatile INT32U *)0xE000ED20uL))
#define OS_CPU_CM_SYST_CSR_CLKSOURCE                     0x00000004uL
#define OS_CPU_CM_SYST_CSR_TICKINT                       0x00000002uL
#define OS_CPU_CM_SYST_CSR_ENABLE                        0x00000001uL

INT32U  OS_KA_BASEPRI_Boundary;

// 定义钩子函数 
void OSInitHookBegin()
{
    INT32U   size;
    OS_STK  *pstk;
    pstk = &OS_CPU_ExceptStk[0];
    size = OS_CPU_EXCEPT_STK_SIZE;
    while (size > 0u) {
        size--;
       *pstk++ = (OS_STK)0;
    }
    OS_CPU_ExceptStkBase = (OS_STK *)&OS_CPU_ExceptStk[OS_CPU_EXCEPT_STK_SIZE];
    OS_CPU_ExceptStkBase = (OS_STK *)((OS_STK)(OS_CPU_ExceptStkBase) & 0xFFFFFFF8);
    OS_KA_BASEPRI_Boundary = (INT32U)(CPU_CFG_KA_IPL_BOUNDARY << (8u - CPU_CFG_NVIC_PRIO_BITS));
} 
void OSInitHookEnd(){} 
void OSTCBInitHook(OS_TCB* ptcb){} 
void OSTaskCreateHook(OS_TCB* ptcb){} 
void OSTaskIdleHook(){} 
void OSTaskReturnHook(OS_TCB* ptcb){} 
void OSTaskSwHook(){} 

OS_STK *OSTaskStkInit(void (*task)(void *p_arg), void *p_arg, OS_STK *ptos, INT16U opt)
{ 
	opt=opt;
	OS_STK *stk;
	stk=ptos+1u;
	stk=(OS_STK *)((OS_STK)(stk)&0xFFFFFFF8u);
	*(--stk)=0x01000000uL;	//xPSR
	*(--stk)=(OS_STK)task;	//PC
	*(--stk)=(OS_STK)OS_TaskReturn;	//LR1,*(--stk)=(OS_STK)OS_TaskReturn;,OS_TaskReturn是os_task.c里面的一个函数
	*(--stk)=0x00000000uL;	//R12
	*(--stk)=0x00000000uL;	//R3
	*(--stk)=0x00000000uL;	//R2
	*(--stk)=0x00000000uL;	//R1
	*(--stk)=(OS_STK)p_arg;	//R0
	
	*(--stk)=0xFFFFFFFDuL;	//LR2,EXC_RETURN,使用了浮点运算单元的话要改一下
	*(--stk)=0x00000000uL;	//R11
	*(--stk)=0x00000000uL;	//R10
	*(--stk)=0x00000000uL;	//R9
	*(--stk)=0x00000000uL;	//R8
	*(--stk)=0x00000000uL;	//R7
	*(--stk)=0x00000000uL;	//R6
	*(--stk)=0x00000000uL;	//R5
	*(--stk)=0x00000000uL;	//R4
	return stk; 
} 

void OS_CPU_SysTickHandler(void)
{
	#if OS_CRITICAL_METHOD==3
	OS_CPU_SR cpu_sr;
	#endif
	
	OS_ENTER_CRITICAL();
	OSIntEnter();
	OS_EXIT_CRITICAL();
	
	OSTimeTick();
	
	OSIntExit();
}

void  OS_CPU_SysTickInit (INT32U  cnts)
{
    INT32U  prio;
    INT32U  basepri;

    basepri               = (INT32U)(CPU_CFG_KA_IPL_BOUNDARY << (8u - CPU_CFG_NVIC_PRIO_BITS));
    OS_CPU_CM_SYST_RVR    = cnts - 1u;
    prio                  =  OS_CPU_CM_SCB_SHPRI3;
    prio                 &=  0x00FFFFFFu;
    prio                 |= (basepri << 24u);
    OS_CPU_CM_SCB_SHPRI3  = prio;
    OS_CPU_CM_SYST_CSR   |= OS_CPU_CM_SYST_CSR_CLKSOURCE |
                            OS_CPU_CM_SYST_CSR_ENABLE;
    OS_CPU_CM_SYST_CSR   |= OS_CPU_CM_SYST_CSR_TICKINT;
}

void  OS_CPU_SysTickInitFreq (INT32U  cpu_freq)
{
    INT32U  cnts;
    cnts = (cpu_freq/(INT32U)OS_TICKS_PER_SEC);
    OS_CPU_SysTickInit(cnts);
}
