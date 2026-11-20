#ifndef TOOL_H
#define TOOL_H

#include "Cfg.h"

//存储传感器原始数据的结构体定义
typedef struct
{
	uint32_t timestamp;		//采集到数据的时间戳，其值为DWT->CYCCNT
	uint32_t mag_timestamp;	//磁力计数据有更新时更新此时间戳
	int16_t accel[3];		//加速度计数据
	int16_t gyro[3];		//陀螺仪数据
	int16_t mag[3];			//磁力计数据
} RawSensorFrame;

//存储传感器数据的结构体定义
typedef struct
{
	uint32_t timestamp;		//采集到数据的时间戳，其值为DWT->CYCCNT
	uint32_t mag_timestamp;	//磁力计数据有更新时更新此时间戳
	float accel[3];		//加速度计数据
	float gyro[3];		//陀螺仪数据
	float mag[3];			//磁力计数据
} SensorFrame;

//四元数
typedef struct
{
	float w;
	float x;
	float y;
	float z;
} Quaternion;

//欧拉角
typedef struct
{
	float roll;
	float pitch;
	float yaw;
} EulerAngles;

/*以下为与I2C相关的工具函数*/
void I2C_Start(void);							//发送起始位
void I2C_Stop(void);							//发送停止位
void I2C_Send_Addr(uint8_t Addr,uint8_t RW);	/*发送地址,Addr代表7位地址;RW代表读或者写,读为1,写为0
												 */
void I2C_Send_Byte(uint8_t data);				//发送字节,data为要发送的数据
static uint8_t I2C1_ReadWaitFlag(uint32_t flag);
static uint8_t I2C1_ReadWaitIdle(void);
static void I2C1_ReadClearAddr(void);
uint8_t I2C_Receive_Byte(void);					/*接收字节数据,作为返回值,要求ACK已使能,在
												 *本项目中,ACK已经在I2C1_Init()中使能
												 */
uint8_t I2C_Receive_Byte_NACK(void);			/*接收最后一个字节需发送NACK,所以单独使用一个函数,该函数
												 *接收一个字节的数据并发送NACK
												 */
uint8_t I2C_Read_Registers(uint8_t addr7,uint8_t reg,uint8_t* buf,uint8_t len);/*读取寄存器数据，可以处理长度为1B或大于等于3B的数据读取。
																			 *addr7为7位设备地址，reg为要读取数据的起始寄存器地址，
																			 *buf为数据读取后的存放位置，len表示读取len B的数据
																			 */
void Read_MPU6050_Gyro_Values(int16_t* x,int16_t* y,int16_t* z);			/*读取MPU6050陀螺仪数据到内存
																			 *指针x、y、z指向的变量分别用
																			 *于接收X、Y、Z轴的数据
																			 */
void Read_MPU6050_Accel_Values(int16_t* x,int16_t* y,int16_t* z);			/*读取MPU6050加速度计数据到内存
																			 *指针x、y、z指向的变量分别用
																			 *于接收X、Y、Z轴的数据
																			 */
uint8_t Read_HMC5883L_Values(int16_t* x,int16_t* y,int16_t* z);				/*读取HMC5883L磁力计数据到内存
																			 *指针x、y、z指向的变量分别用
																			 *于接收X、Y、Z轴的数据。
																			 *返回值：
																			 *0x01:磁力计数据已更新，读取成功
																			 *0x00:磁力计数据未更新，没有读取
																			 */

/*以下为与延时函数相关的工具函数*/
void Delay(uint32_t time);

/*以下为与蓝牙发送相关的工具函数*/
uint32_t Serial_Pow(uint32_t x,uint32_t y);					//计算x^y
void Serial_SendByte(uint8_t Byte);							//通过蓝牙发送一个字节的数据
void Serial_SendNumber(uint32_t Number,uint8_t Length) ;	//通过蓝牙发送一个Length位的数字Number
void Serial_SendFloat(float value);							//通过蓝牙发送一个浮点数
void Serial_SendString(char *String);						//通过蓝牙发送一个字符串
void Serial_SendInt16(int16_t num);							//通过蓝牙发送一个int16_t的数据
void DataTransfer(RawSensorFrame rawdata,SensorFrame data,Quaternion q,EulerAngles e);		/*通过蓝牙发送(原始)数据
																							 *rawdata是包含要发送的原始数据的结构体
																							 *data是包含要发送的数据的结构体
																							 */

/*以下为与设置电机转速相关的工具函数*/
void ESCSetThrottle(uint16_t PA8throttle,uint16_t PA9throttle,uint16_t PA10throttle,uint16_t PA11throttle);

/*以下为验证设置的油门信号是否合法的函数*/
void AssertThrottle(uint16_t* throttle);

/*以下为错误处理函数*/
void ErrorHandle(void);

/*以下为与矩阵运算相关的函数*/
void Cal_3_3_plus_3_1(float A[3][3],float b[3],float result[3]);			/*计算3*3矩阵与3*1矩阵的乘积
																			 *参数说明：
																			 *A:参与计算的3*3矩阵
																			 *b:参与计算的3*1矩阵
																			 *result:计算结果
																			 */
void Transpose(float m[3][3],float result[3][3]);							/*计算矩阵的转置矩阵
																			 *参数说明：
																			 *m:指向待转置的矩阵
																			 *result:指向转置后的矩阵
																			 *
																			 *Attention:
																			 *本函数不支持m与result相等，不能实现原地转置
																			 *
																			 */

/*以下为校准数据及单位换算的函数*/
void CalibrationAndUnitConversion(RawSensorFrame raw,SensorFrame* corrected);	/*完成数据校准以及单位换算
																			 *参数说明：
																			 *raw:待操作的数据
																			 *corrected:指向修正后的数据
																			 */
																			 
/*以下为与坐标转换相关的函数*/
void RawToFRD(SensorFrame* raw,SensorFrame* result);			/*将原始传感器坐标转换到FRD坐标
																 *参数说明：
																 *raw:指向包含转换前的坐标的数据结构的指针
																 *result:指向包含转换后的坐标的数据结构的指针
																 */
																 
/*以下为与四元数计算相关的函数*/
void InitializeQuaternion(Quaternion* q,SensorFrame data);		/*初始化四元数
																 *参数说明：
																 *q:指向存放四元数值的指针
																 *data:包含传感器数据的结构体
																 */
void QuaternionMulC(Quaternion* a,float c,Quaternion* result);		/*计算常量与四元数的积
																	 *参数说明：
																	 *a:指向参与计算的四元数的指针
																	 *c:参与计算的常量
																	 *result:指向结果四元数的指针
																	 *
																	 *Attention:
																	 *本函数支持a与result相等，以实现原地计算
																	 *
																	 */
void QuaternionMul(Quaternion* a,Quaternion* b,Quaternion* result);		/*计算四元数乘法
																		 *参数说明：
																		 *a:指向参与计算的第一个四元数的指针
																		 *b:指向参与计算的第二个四元数的指针
																		 *result:指向计算结果的指针
																		 *
																		 *Attention:
																		 *1.a和b的位置一般不可交换
																		 *2.本函数支持result与a或b二者之一相等
																		 *
																		 */
void MatrixOfQuaternion(Quaternion* q,float Rq[3][3]);					/*计算四元数对应的旋转矩阵
																		 *参数说明：
																		 *q:欲计算旋转矩阵的四元数
																		 *Rq:指向计算完成的旋转矩阵的指针
																		 */
																		 
/*以下为与向量或四元数操作相关的函数*/
float VectorDotProduct(float a[3],float b[3]);					/*计算两个向量的内积
																 *参数说明：
																 *a:指向参与计算的向量a
																 *b:指向参与计算的向量b
																 *返回值:内积结果
																 */
void VectorCrossProduct(float a[3],float b[3],float result[3]);	/*计算两个向量的外积
																 *参数说明：
																 *a:指向参与计算的向量a
																 *b:指向参与计算的向量b
																 *result:指向外积结果的指针
																 *
																 *Attention:
																 *本函数支持result与a或b二者之一相等
																 *
																 */
uint8_t NormVector(float m[3],float result[3]);					/*归一化向量
																 *参数说明：
																 *m:指向待归一化向量的指针
																 *result:指向归一化之后的向量的指针
																 *返回值:0x00代表模长为0，出错；0x01代表计算正常
																 *
																 *Attention:
																 *本函数支持m与result相等，以实现原地修改
																 *
																 */
uint8_t NormQuaternion(Quaternion* q,Quaternion* result);		/*归一化四元数
																 *参数说明：
																 *q:指向待归一化的四元数的指针
																 *result:指向归一化之后的四元数的指针
																 *
																 *Attention:
																 *本函数支持q与result相等，以实现原地修改
																 *返回值:0x00代表模长为0，出错；0x01代表计算正常
																 *
																 */
																 
/*以下为与欧拉角相关的工具函数*/
void QuaternionToEulerAngles(float Rq[3][3],EulerAngles* e);	/*四元数转欧拉角
																 *参数说明：
																 *Rq:指向旋转矩阵，该旋转矩阵与欧拉角对应
																 *e:指向计算出的欧拉角
																 */
																 
/*以下为其它工具函数*/
uint8_t MaxValueIndex(float* arr,uint8_t len);		/*返回一个数组中最大值的下标
													 *参数说明：
													 *arr:指向这个数组的指针
													 *len:数组长度
													 */
void UpdateUnitvectorMag(float Rq[3][3],float mag[3],float result[3]);		/*更新参考磁场向量b
																			 *参数说明：
																			 *Rq:旋转矩阵，上一时刻四元数对应的旋转矩阵
																			 *data:指向包含传感器数据的指针
																			 *result:指向更新后的b
																			 */
void CalJAccel(Quaternion* q,float J[3][4]);		/*计算加速度计数据对应的雅可比矩阵
													 *参数说明：
													 *q:指向四元数的指针
													 *J:指向计算后的雅可比矩阵
													 */
void CalJMag(Quaternion* q,float b[3],float J[3][4]);		/*计算磁力计数据对应的雅可比矩阵
															 *参数说明：
															 *q:指向四元数的指针
															 *b:参考磁场方向
															 *J:指向计算后的雅可比矩阵
															 */
float Clip(float a,float leftclip,float rightclip);			/*数据范围限制
															 *若a<leftclip，则返回leftclip；
															 *若a>rightclip，则返回rightclip；
															 *否则返回a
															 */
															 
/*以下为与上位机通信相关的函数*/
#if UseTheHostComputerToVerifyAttitudeCalculation==1
void HostComputerInit(void);                          //USART2：PA2/PA3，115200波特率，8N1
void HostComputerSendByte(uint8_t data);              //向ST-LINK虚拟串口发送一个字节
void HostComputerSendAttitude(EulerAngles attitude);  /*发送匿名V7的0x03欧拉角帧
                                                       *attitude为磁NED参考系、FRD机体系下的ZYX欧拉角，单位rad
                                                       *调用前须已完成姿态初始化；帧内角度转换为0.01度整数
                                                       */
#endif
#endif
