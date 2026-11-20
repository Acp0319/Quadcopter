#ifndef CFG_H
#define CFG_H

/*是否使用优化后的电调阈值设定，若使用则解注释，否则保持注释*/
#define OptimizedThrottleSet

/*选择使用校准模式还是工作模式*/
#define UseCalibrationMode	0
#define UseWorkMode			(UseCalibrationMode==1?0:1)

/*选择是否使用上位机验证姿态解算*/
#define UseTheHostComputerToVerifyAttitudeCalculation	1	//值为1时使用，否则不使用

/*数据校准的参数*/
#define ACCEL_BIAS_INIT		{0.002564f,-0.006268f,-0.015593f}
#define ACCEL_SCALE_INIT	{0.999281f,0.996111f,1.005028f}

#define	GYRO_BIAS_INIT		{-14.03f,-351.07f,103.68f}

#define MAG_BIAS_INIT		{-9.923814351e+01f,-7.050314769e+01f,-1.045750005e+01f}
#define MAG_MATRIX_INIT		{{1.825178090e-03f,-4.431658472e-05f,2.658000751e-05f},\
							 {-4.431658472e-05f,1.859377188e-03f,1.727562822e-05f},\
							 {2.658000751e-05f,1.727562822e-05f,2.194813414e-03f}}

/*坐标转换*/
#define FRD_RAW {{0,1,2},{1,0,2},{1,1,-1}}		//FRD[[0,i]]=[2,i]*RAW[[1,i]]

/*圆周率*/
#define PI 3.14159265358979323846f

/*Madgwick参数*/
#define BETA 0.1f		//β控制纠偏强度

/*四元数转欧拉角时的奇异值判定阈值*/
#define EPSILON 1.0e-5f

/*电调阈值*/
#define ESC_PULSE_MIN  1000
#define ESC_PULSE_MAX  2000

#ifdef OptimizedThrottleSet
	#define AcceptableError 100										//可容忍的误差值
	#define ESC_PULSE_SAFE_MAX  (ESC_PULSE_MAX+AcceptableError)		//认为数据有效的最大阈值
#endif

/*IIC读取相关配置*/
#define I2C1_READ_POLL_LIMIT  100000UL		//每次等待的轮询次数上限
#define I2C1_READ_ERRORS      (I2C_SR1_AF|I2C_SR1_ARLO|I2C_SR1_OVR|I2C_SR1_TIMEOUT)

/*优先级定义*/
#define StartPrio  		6
#define BlueToothPrio	15
#define GY87Prio		14
#define ReceiverPrio	12
#define MotorPrio		13

/*任务栈大小定义*/
#define StartStkSize		128
#define BlueToothStkSize	128
#define GY87StkSize			256
#define ReceiverStkSize		128
#define MotorStkSize		128

#endif
