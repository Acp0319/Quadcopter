#ifndef OS_CPU_H 
#define OS_CPU_H 

#ifdef   OS_CPU_GLOBALS
#define  OS_CPU_EXT
#else
#define  OS_CPU_EXT  extern
#endif

//-------------------------------------------------------------------------------------------------------
#ifndef  OS_CPU_EXCEPT_STK_SIZE
#define  OS_CPU_EXCEPT_STK_SIZE      256u        /* Default exception stack size is 256 OS_STK entries */
#endif

//数据类型 
typedef unsigned char BOOLEAN; // 定义布尔类型 
typedef unsigned char INT8U; // 无符号8位整数 
typedef signed char INT8S; // 有符号8位整数 
typedef unsigned short INT16U; // 无符号16位整数 
typedef signed short INT16S; // 有符号16位整数 
typedef unsigned int INT32U; // 无符号32位整数 
typedef signed int INT32S; // 有符号32位整数 
typedef float FP32; // 单精度浮点数 
typedef double FP64; // 双精度浮点数 
typedef unsigned int OS_STK; // 每个栈条为32位宽 
typedef unsigned int OS_CPU_SR;// 定义CPU状态寄存器的大小（PSR = 32位） 

// 启用第三种临界区保护方法 
#define OS_CRITICAL_METHOD 3u 

#if OS_CRITICAL_METHOD ==3u 

// 声明保存和恢复CPU状态寄存器的函数 
OS_CPU_SR  OS_CPU_SR_Save   (OS_CPU_SR  new_basepri);
void       OS_CPU_SR_Restore(OS_CPU_SR  cpu_sr);

// 定义进入和退出临界区的宏，保存状态并关中断和恢复原始中断状态 
#define  OS_ENTER_CRITICAL()  do { cpu_sr = OS_CPU_SR_Save(CPU_CFG_KA_IPL_BOUNDARY << (8u - CPU_CFG_NVIC_PRIO_BITS));} while (0)
#define  OS_EXIT_CRITICAL()   do { OS_CPU_SR_Restore(cpu_sr);} while (0)
#endif 


//任务切换函数 
#define OS_TASK_SW() OSCtxSw() 

//堆栈增长方式为从高到低（1：满递减方式） 
#define OS_STK_GROWTH 1u 

//------------------------------------------------------------------------------
OS_CPU_EXT  OS_STK   OS_CPU_ExceptStk[OS_CPU_EXCEPT_STK_SIZE];
OS_CPU_EXT  OS_STK  *OS_CPU_ExceptStkBase;

//函数声明 
void OSCtxSw(void); 
void OSIntCtxSw(void); 
void OSStartHighRdy(void); 
#endif 
