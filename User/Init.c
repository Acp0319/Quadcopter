#include "stm32f4xx.h"                  // Device header
#include "Init.h"

void ClockInit(void)
{
	RCC->AHB1ENR|=0x00000003;			//使能GPIOA和GPIOB的时钟
	RCC->APB1ENR|=0x00200001;			//使能TIM2和I2C1的时钟
	RCC->APB2ENR|=0x00000011;			//使能TIM1和USART1的时钟
}

void BlueToothInit(void)
{
	GPIOB->MODER&=~((3UL<<12)|(3UL<<14));		//配置PB6、PB7为复用功能模式
	GPIOB->MODER|=((2UL<<12)|(2UL<<14));

	GPIOB->OTYPER&=~((1UL<<6)|(1UL<<7));		//配置输出类型为推挽输出

	GPIOB->OSPEEDR&=~((3UL<<12)|(3UL<<14));		//配置输出速度50MHz
	GPIOB->OSPEEDR|=((2UL<<12)|(2UL<<14));

	GPIOB->PUPDR&=~((3UL<<12)|(3UL<<14));		//配置上拉
	GPIOB->PUPDR|=((1UL<<12)|(1UL<<14));

	GPIOB->AFR[0]&=~((0xFUL<<24)|(0xFUL<<28));	//配置复用功能为USART1(AF7)
	GPIOB->AFR[0]|=((7UL<<24)|(7UL<<28));
	
	USART1->CR1&=~(1<<13);						//禁用USART

	USART1->BRR=0x88C;							//配置波特率

	USART1->CR2=0;								//配置CR2(停止位)

	USART1->CR3=0;								//配置CR3(硬件流控制)

	USART1->CR1=0;								//配置CR1(字长、校验等)
	USART1->CR1&=~(1 << 12);
	USART1->CR1&=~(1 << 10);

	USART1->CR1|=(1<<3);						//一次性启用所有功能
	USART1->CR1|=(1<<2);
	USART1->CR1|=(1<<13);
}

void GY86Init(void)
{
	I2C1_Init();
	MPU6050_Init();
}

void MotorInit(void)
{
    GPIOA->MODER&=~(3UL<<16);				//配置PA8为复用功能模式
    GPIOA->MODER|=(2UL<<16);
	GPIOA->MODER&=~(3UL<<18);				//配置PA9为复用功能模式
    GPIOA->MODER|=(2UL<<18);
    GPIOA->MODER&=~(3UL<<20);				//配置PA10为复用功能模式
    GPIOA->MODER|=(2UL<<20);
    GPIOA->MODER&=~(3UL<<22);				//配置PA11为复用功能模式
    GPIOA->MODER|=(2UL<<22);

    GPIOA->OSPEEDR&=~(3UL<<16);				//PA8输出速度：100MHz
    GPIOA->OSPEEDR|=(3UL<<16);
	GPIOA->OSPEEDR&=~(3UL<<18);				//PA9输出速度：100MHz
    GPIOA->OSPEEDR|=(3UL<<18);
    GPIOA->OSPEEDR&=~(3UL<<20);				//PA10输出速度：100MHz
    GPIOA->OSPEEDR|=(3UL<<20);
    GPIOA->OSPEEDR&=~(3UL<<22);				//PA11输出速度：100MHz
    GPIOA->OSPEEDR|=(3UL<<22);

    GPIOA->OTYPER&=~(1UL<<8);				//PA8输出类型：推挽
	GPIOA->OTYPER&=~(1UL<<9);				//PA9输出类型：推挽
    GPIOA->OTYPER&=~(1UL<<10);				//PA10输出类型：推挽
    GPIOA->OTYPER&=~(1UL<<11);				//PA11输出类型：推挽

    GPIOA->PUPDR&=~(3UL<<16);				//PA8上下拉：无上拉/下拉
	GPIOA->PUPDR&=~(3UL<<18);				//PA9上下拉：无上拉/下拉
    GPIOA->PUPDR&=~(3UL<<20);				//PA10上下拉：无上拉/下拉
    GPIOA->PUPDR&=~(3UL<<22);				//PA11上下拉：无上拉/下拉

    GPIOA->AFR[1]&=~(0xFUL<<0);			//PA8复用功能选择：AF1
    GPIOA->AFR[1]|=(1UL<<0);
	GPIOA->AFR[1]&=~(0xFUL<<4);			//PA9复用功能选择：AF1
    GPIOA->AFR[1]|=(1UL<<4);
    GPIOA->AFR[1]&=~(0xFUL<<8);			//PA10复用功能选择：AF1
    GPIOA->AFR[1]|=(1UL<<8);
    GPIOA->AFR[1]&=~(0xFUL<<12);		//PA11复用功能选择：AF1
    GPIOA->AFR[1]|=(1UL<<12);

    TIM1->CR1&=~TIM_CR1_CEN;				//禁用定时器

    TIM1->PSC=83;							//预分频器值
    TIM1->ARR=19999;						//自动重装载值(20ms周期)
    TIM1->RCR=0;							//重复计数器

    TIM1->CR1&=~(TIM_CR1_CKD|TIM_CR1_DIR);	//清除时钟分频CKD(bit8-9)和方向DIR(bit4)，向上计数、无分频
    TIM1->CR1|=TIM_CR1_ARPE;				//使能ARR预装载(ARPE)

    TIM1->EGR=TIM_EGR_UG;					//生成更新事件，立即将PSC和ARR加载到影子寄存器 (标准库行为)

    /*配置四个通道为PWM模式1*/
    /*通道1和通道2在CCMR1*/
    uint32_t tmp=TIM1->CCMR1;
    tmp&=~(TIM_CCMR1_OC1M|TIM_CCMR1_OC1PE);	//清除CH1相关位(OC1M,OC1PE)
    tmp|=(6UL<<4)|TIM_CCMR1_OC1PE;			//OC1M=110(PWM模式1),OC1PE=1
    tmp&=~(TIM_CCMR1_OC2M|TIM_CCMR1_OC2PE);	//清除CH2相关位(OC2M,OC2PE)
    tmp|=(6UL<<12)|TIM_CCMR1_OC2PE;			//OC2M=110,OC2PE=1
    TIM1->CCMR1=tmp;

    /*通道3和通道4在CCMR2*/
    tmp=TIM1->CCMR2;
    tmp&=~(TIM_CCMR2_OC3M|TIM_CCMR2_OC3PE);	//清除CH3相关位(OC3M,OC3PE)
    tmp|=(6UL<<4)|TIM_CCMR2_OC3PE;			//OC3M=110,OC3PE=1
    tmp&=~(TIM_CCMR2_OC4M|TIM_CCMR2_OC4PE);	//清除CH4相关位(OC4M,OC4PE)
    tmp|=(6UL<<12)|TIM_CCMR2_OC4PE;			//OC4M=110,OC4PE=1
    TIM1->CCMR2=tmp;

    TIM1->CCR1=1000;						//PA8脉宽初始值：1000(1ms高电平)
	TIM1->CCR2=1000;						//PA9脉宽初始值：1000(1ms高电平)
    TIM1->CCR3=1000;						//PA10脉宽初始值：1000(1ms高电平)
    TIM1->CCR4=1000;						//PA11脉宽初始值：1000(1ms高电平)

    tmp=TIM1->CCER;							//CCER: 通道1输出使能，极性高 (CC1P=0)

    /*清除各通道使能位和极性位(确保极性为高有效)*/
    tmp&=~(TIM_CCER_CC1E|TIM_CCER_CC1P|
           TIM_CCER_CC2E|TIM_CCER_CC2P|
           TIM_CCER_CC3E|TIM_CCER_CC3P|
           TIM_CCER_CC4E|TIM_CCER_CC4P);
    /*使能所有通道(CCxE=1,CCxP=0)*/
    tmp|=TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E;
    TIM1->CCER=tmp;

    TIM1->BDTR|=TIM_BDTR_MOE;				//主输出使能(MOE,高级定时器必需)

    TIM1->CR1|=TIM_CR1_CEN;					//启动定时器(CEN=1)
}

void ReceiverInit(void)
{
	//配置PA0为复用功能模式
    GPIOA->MODER&=~(3UL<<0);
    GPIOA->MODER|=(2UL<<0);

	//100MHz
    GPIOA->OSPEEDR&=~(3UL<<0);
    GPIOA->OSPEEDR|=(3UL<<0);

	//推挽输出
    GPIOA->OTYPER&=~(1UL<<0);

	//上拉
    GPIOA->PUPDR&=~(3UL<<0);
    GPIOA->PUPDR|=(1UL<<0);

    //PA0复用功能选择:AF1
    GPIOA->AFR[0]&=~(0xFUL<<0);
    GPIOA->AFR[0]|=(1UL<<0);

    /*配置 TIM2 时基:
     *    - 预分频器: 83 → 定时器计数频率 1 MHz (84 MHz / (83+1))
     *    - 自动重载: 0xFFFF (最大周期 65.536 ms)
     *    - 向上计数, 无时钟分频, ARPE 默认关闭 (不使能预装载)
     */
    TIM2->CR1&=~(TIM_CR1_DIR|TIM_CR1_CKD);//向上计数, 无分频
    TIM2->PSC=83;
    TIM2->ARR=0xFFFF;

    /*可选: 生成一次更新事件, 将影子寄存器立即加载 (等效标准外设库行为)*/
    TIM2->EGR=TIM_EGR_UG;

    /*配置 TIM2 通道1为输入捕获:
     *    - 输入直接映射到 TI1 (CC1S=01)
     *    - 无预分频 (IC1PSC=00)
     *    - 无滤波 (IC1F=0000)
     */
    uint32_t ccmr1=TIM2->CCMR1;
    ccmr1&=~(TIM_CCMR1_CC1S |TIM_CCMR1_IC1PSC|TIM_CCMR1_IC1F);
    ccmr1|=TIM_CCMR1_CC1S_0;                //CC1S=01(直接连接到TI1)
    TIM2->CCMR1=ccmr1;

    /*配置捕获极性: 上升沿 (CC1P=0, CC1NP=0), 并使能捕获 */
    TIM2->CCER&=~(TIM_CCER_CC1P|TIM_CCER_CC1NP); // 确保为上升沿
    TIM2->CCER|=TIM_CCER_CC1E;                      // 使能通道1捕获

    /*使能捕获/比较1中断(CC1IE)*/
    TIM2->DIER|=TIM_DIER_CC1IE;

    /*配置 NVIC: 抢占优先级0, 子优先级1
     *    首先设置优先级分组为2 (2位抢占, 2位子优先级)
     *    TIM2_IRQn = 28
     */
    SCB->AIRCR=(0x5FAUL<<16)|(5UL<<8);//分组2: 抢占2位 [7:6], 子优先级2位 [5:4]

    /*优先级的实际值: (抢占0 << 6) | (子1 << 4) = 0x10, 直接写入IP寄存器 */
    NVIC->IP[28]=(0UL<<6)|(1UL<<4);// 抢占0, 子优先级1
    NVIC->ISER[0]=(1UL<<28);               //使能TIM2中断

    /*启动定时器*/
    TIM2->CR1|=TIM_CR1_CEN;
}

void BSP(void)
{
	ClockInit();
	BlueToothInit();
	GY86Init();
	MotorInit();
	ReceiverInit();
	
	//使能DWT周期计数器（用于SystemView时间戳）
	//使能内核调试跟踪单元（解锁DWT）
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	//清零计数器
	DWT->CYCCNT = 0;
	//使能CYCCNT开始计数
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void I2C1_Init(void)
{
	/*配置PB8(SCL), PB9(SDA)
	 */
	GPIOB->MODER&=(~0x0F0000);
	GPIOB->MODER|=0x0A0000;			//设置PB8和PB9为AF模式
	
	GPIOB->OTYPER|=0x0300;			//设置PB8和PB9为开漏输出
	
	GPIOB->OSPEEDR|=0x0F0000;		//设置PB8和PB9为高速模式
	
	GPIOB->PUPDR&=(~0x0F0000);
	GPIOB->PUPDR|=0x050000;			//设置PB8和PB9为上拉模式
	
	GPIOB->AFR[1]&=(~0xFF);
	GPIOB->AFR[1]|=0x44;			//设置PB8和PB9为AF4
	
	/*配置I2C1(100kHz标准模式)
	 */
	I2C1->CR1=0;		//先禁用I2C
	I2C1->CR2=42;		//设置I2C时钟频率为42MHz
	I2C1->CCR=210;		/*设置CCR寄存器的值为210,具体计算方法:
						 *CCR=42,000,000/(2×100,000)=210
						 *详情参见数据手册
						 */
	I2C1->TRISE=43;		/*设置TRISE寄存器的值，具体计算方法:
						 *TRISE=(Max_SCL_Rise_Time/APB1_Period)+1
						 *标准模式最大上升时间=1000ns
						 *TRISE=(42×1000/1000)+1=43
						 *详情参见数据手册
						 */
	I2C1->CR1|=0x0401;	//使能I2C1并使能ACK
}

void MPU6050_Init(void)
{
	/*唤醒MPU6050&时钟源
	 */
	I2C_Start();
	I2C_Send_Addr(0xD0,0);		//发送设备地址,写模式
	I2C_Send_Byte(0x6B);		//发送寄存器地址
	I2C_Send_Byte(0x01);		//发送寄存器值
	I2C_Stop();
	
	/*配置陀螺仪量程
	 */
	I2C_Start();
	I2C_Send_Addr(0xD0,0);		//发送设备地址,写模式
	I2C_Send_Byte(0x1B);
	I2C_Send_Byte(0x00);
	I2C_Stop();
	
	/*配置加速度计量程
	 */
	I2C_Start();
	I2C_Send_Addr(0xD0,0);		//发送设备地址,写模式
	I2C_Send_Byte(0x1C);
	I2C_Send_Byte(0x00);
	I2C_Stop();
}
