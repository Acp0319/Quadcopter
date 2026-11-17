#ifndef CFG_H
#define CFG_H

/*是否使用优化后的电调阈值设定，若使用则解注释，否则保持注释*/
#define OptimizedThrottleSet

/*电调阈值*/
#define ESC_PULSE_MIN  1000
#define ESC_PULSE_MAX  2000

#ifdef OptimizedThrottleSet
	#define AcceptableError 100										//可容忍的误差值
	#define ESC_PULSE_SAFE_MAX  (ESC_PULSE_MAX+AcceptableError)		//认为数据有效的最大阈值
#endif

/*优先级定义*/
#define StartPrio  		6
#define BlueToothPrio	15
#define GY86Prio		14
#define ReceiverPrio	12
#define MotorPrio		13

/*任务栈大小定义*/
#define StartStkSize		128
#define BlueToothStkSize	128
#define GY86StkSize			128
#define ReceiverStkSize		128
#define MotorStkSize		128

#endif
