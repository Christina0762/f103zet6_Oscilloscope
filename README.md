# 简介![在这里插入图片描述](https://img-blog.csdnimg.cn/20200218215022673.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQ0ODk3MTk0,size_16,color_FFFFFF,t_70)
此项案例是基于正点原子精英板制作的一个**简易示波器**，可以读取**信号的频率和幅值**，并可以通过按键**改变采样频率和控制屏幕的更新暂停**。
（输入最大3.3V，由ADC参考电压决定）

将PA6与PA4相连，可观察到正弦波。
将PA6与PA5相连，可观察到三角波/噪声(默认三角波)。
KEY_UP控制波形的更新和暂停。
KEY_1降低采样率。
KEY_0提高采样率。

思路借鉴了此位博主的方案（其代码为寄存器版）：
[基于STM32的示波器（寄存器）](https://blog.csdn.net/dedell/article/details/96626143)

下面将介绍所用到的基本原理以及相关代码的展示。
# 一.信号的采集
信号的采集主要是依靠**ADC（通过定时器触发采样，与在定时器中断中开启一次采样的效果类似，以此来控制采样的间隔时间相同）**，然后**通过DMA将所采集的数据从ADC的DR寄存器转移到一个变量中**，此时完成一次采样。

由于设定采集一次完整的波形需要1024个点，即需要**连续采集1024次才算一次完整的波形采样**（需要采集1024个点的原因在后面会提到）。因此我们还需创建一个数组用于存储这些数据，并**在DMA中断中，将成功转移到变量中的数据依次存储进数组**（注意此数组中存入的数据是12位的数字量，还未做回归处理），完成1024个数据的采样和储存，用于后续在LCD上进行波形的显示和相关参数的处理。

此案例用到的是ADC1的通道6（即PA6口）进行数据的采样，主要需注意将**ADC转换的触发方式改为定时器触发**（我用的是定时器2的通道2进行触发，由于STM32手册提示只有在**上升沿时可以触发ADC**，因此我们需要让定时器2的通道2每隔固定的时间产生一个上升沿）。将定时器2设置成**PWM模式**，即可令ADC1在定时器2的通道2每产生一次上升沿时触发采样，后续即可通过**改变PWM的频率**（即定时器的溢出频率），便可**控制采样的频率**。

更为具体的原理解释可参考此位博主：
[【STM32】定时器TIM触发ADC采样，DMA搬运到内存（超详细讲解）](https://blog.csdn.net/qq_38410730/article/details/89921413)

### 1.ADC的配置：
```c
/**********************************************************
简介：ADC1-CH6初始化函数
***********************************************************/															   
void  Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1, ENABLE );	  //使能ADC1通道时钟
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	//PA6 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在非连续转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;	//转换由定时器2的通道2触发（只有在上升沿时可以触发）
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_DMACmd(ADC1, ENABLE);	//ADC的DMA功能使能
	
	ADC_ResetCalibration(ADC1);	//使能复位校准  
	 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_1Cycles5 );//ADC1通道6,采样时间为239.5周期	 
	 
	ADC_ResetCalibration(ADC1);//复位较准寄存器
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能

}			
```

### 2.定时器的配置：

```c
/******************************************************************
函数名称:TIM2_PWM_Init(u16 arr,u16 psc)
函数功能:定时器3，PWM输出模式初始化函数
参数说明:arr：重装载值
		 psc：预分频值
备    注:通过TIM2-CH2的PWM输出触发ADC采样
*******************************************************************/ 	
void TIM2_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//使能定时器2时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
 
   //设置该引脚为复用输出功能,输出TIM2 CH2的PWM脉冲波形	GPIOA.1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
 
   //初始化TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM2 Channel2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_Pulse=1000;	//发生反转时的计数器数值，用于改变占空比
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM2

	TIM_CtrlPWMOutputs(TIM2, ENABLE);//使能PWM输出
	
	TIM_Cmd(TIM2, ENABLE);  //使能TIM2
}
```
### 3.DMA配置：

```c
/******************************************************************
函数名称:MYDMA1_Config()
函数功能:DMA1初始化配置
参数说明:DMA_CHx：DMA通道选择
		 cpar：DMA外设ADC基地址
		 cmar：DMA内存基地址
		 cndtrDMA通道的DMA缓存的大小
备    注:
*******************************************************************/
void MYDMA1_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
	
    DMA_DeInit(DMA_CHx);   //将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从外设读取发送到内存//
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //数据宽度为16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //工作在循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA通道 x拥有高优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //ADC1匹配DMA通道1
	
	DMA_ITConfig(DMA1_Channel1,DMA1_IT_TC1,ENABLE);	//使能DMA传输中断	
	
	//配置中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	

	DMA_Cmd(DMA1_Channel1,ENABLE);//使能DMA通道
}
```

**注：
1.由于在设置PWM时将TIM_Pulse默认设置为1000，因此在初始化定时器2时，TIM_Period的值不能小于该值，可自行修改。TIM_Pulse的值并不会影响采样频率。
2.采样频率= 定时器2溢出频率=SYSCLK/预分频值/溢出值。因此如果将TIM_Pulse设为1，TIM_Period设为2，TIM_Prescaler设为1，理论上采样频率最高可达36Mhz。**
# 二.数据的处理
数据的处理主要是要求出**信号的频率和幅值等相关参数**。幅值可以通过找出之前存储1024个点的数组中最大最小值，回归处理过后算出差值。难点主要在于频率的求取。一个信号中可能**包含多种频率成分**，而我显示的是**幅值最大的频率分量**（当然其他频率也可获得）。这里便用到了**STM32提供的DSP库中的FFT**（快速傅里叶变换），DSP库在最后的源码中有。

**需要采样1024个点的原因**：FFT算法要求样本数为2的n次方，而DSP库中提供了64，256和1024样本数对应的库函数，因此选用1024最大样本数可以使频率分辨率最小，更加精确。（定义频率分辨率f0=fs/N，其中fs等于采样率，N为采样点数）

**需注意**：FFT后的输出不是实际的信号频率，需要经过转换。f(k)=k*(fs/N)，其中
f(k)是实际频率，k是实际信号的最大幅度频率所对应的数。**（详见下面代码，分享的源代码中公式有误，未重新上传）**

详细资料可借鉴：
[如何使用DSP库进行FFT](https://www.cnblogs.com/menlsh/p/4154070.html)
[如何将FFT的输出转为信号的实际频率](http://www.elecfans.com/d/781486.html)

获取频率的函数：

```c
#define NPT 1024//一次完整采集的采样点数

/******************************************************************
函数名称:GetPowerMag()
函数功能:计算各次谐波幅值
参数说明:
备　　注:先将lBufOutArray分解成实部(X)和虚部(Y)，然后计算幅值(sqrt(X*X+Y*Y)
*******************************************************************/
void GetPowerMag(void)
{
    float X,Y,Mag,magmax;//实部，虚部，各频率幅值，最大幅值
    u16 i;
	
	//调用自cr4_fft_1024_stm32
	cr4_fft_1024_stm32(fftout, fftin, NPT);	
	//fftin为傅里叶输入序列数组，ffout为傅里叶输出序列数组
	
    for(i=1; i<NPT/2; i++)
    {
		X = (fftout[i] << 16) >> 16;
		Y = (fftout[i] >> 16);
		
		Mag = sqrt(X * X + Y * Y); 
		FFT_Mag[i]=Mag;//存入缓存，用于输出查验
		//获取最大频率分量及其幅值
		if(Mag > magmax)
		{
			magmax = Mag;
			temp = i;
		}
    }
	F=(u16)(temp*(fre*1.0/NPT));//源代码中此公式有误，将此复制进去
	
	LCD_ShowNum(280,180,F,5,16);
}					
```
# 三.模拟正弦波输出
此正弦波输出是用于**调试示波器**，观察显示和实际是否相同。主要利用**DAC输出**，在定时器3的中断中不断改变DAC的输出值，产生一个正弦波。因此改变正弦波的频率可以通过**更改定时器3的溢出频率**。（采用的PA4口进行输出）

在初始化时，我将定时器3的重装载值设置为40，预分频值设置为72，正弦波输出频率为72Mhz/40/72/1024≈24.5Hz（**1024是因为将一个周期正弦波均分成1024个输出点，详见下面函数InitBufInArray()**）。经采样处理后显示为24-25Hz，与实际值接近。（但是当采样频率提高到最大3.6kHz时，频率显示为32Hz左右，原因未知）

下面是相关代码：

```c
u16 magout[NPT];
/******************************************************************
函数名称:InitBufInArray()
函数功能:正弦波值初始化，将正弦波各点的值存入magout[]数组中
参数说明:
备    注:
*******************************************************************/
void InitBufInArray(void)
{
    u16 i;
    float fx;
    for(i=0; i<NPT; i++)
    {
        fx = sin((PI2*i)/NPT);
        magout[i] = (u16)(2048+2048*fx);
    }
}

/******************************************************************
函数名称:sinout()
函数功能:正弦波输出
参数说明:
备    注:将此函数置于定时器中断中，可模拟输出正弦波
*******************************************************************/
void sinout(void)
{
	static u16 i=0;
	DAC_SetChannel1Data(DAC_Align_12b_R,magout[i]);
	i++;
	if(i>=NPT)
		i=0;
}
```
# 四.模拟噪声或三角波输出
模拟噪声或三角波输出可直接通过配置DAC，利用芯片内部的发生器产生。**DAC2的转换由定时器4的TRGO触发（事件触发）**。同时需要注意设置TRGO由更新事件产生。
**若为三角波输出，频率=72Mhz/定时器重装载值/预分频系数/幅值/2；**
例如：初始化定时器的重装载值为2，预分频系数为36，幅值为最大（4096），即Freq=72Mhz/2/36/4096/2≈122Hz；
```c
void Dac2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );	  //使能PORTA通道时钟
   	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );	  //使能DAC通道时钟 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 // 端口配置
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 		 //模拟输入
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
					
	DAC_InitType.DAC_Trigger=DAC_Trigger_T4_TRGO;	//定时器4触发
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_Noise;//产生噪声
	//DAC_WaveGeneration_Triangle产生三角波
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_TriangleAmplitude_4095;//幅值设置为最大，即3.3V
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1输出缓存关闭 BOFF1=1
    DAC_Init(DAC_Channel_2,&DAC_InitType);	 //初始化DAC通道2

	DAC_Cmd(DAC_Channel_2, ENABLE);  //使能DAC-CH2
	
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12位右对齐数据格式设置DAC值	
}
```

```c
void TIM4_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Update);//触发外设方式为更新触发
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIMx外设
							 
}
```

# 五.其余显示和按键控制等
1.显示波形只需将所获得的1024个采样数据选择一部分进行显示
大致思路即：

```c
u16 pre_vol;//当前电压值对应点的纵坐标
u16 past_vol;//前一个电压值对应点的纵坐标
//adcx[]数组及通过DMA存入的1024个原始数据
pre_vol = 50+adcx[x]/4096.0*100;
LCD_DrawLine(x,past_vol,x+1,pre_vol);//根据实际，打点位置可进行相应更改
past_vol = pre_vol;
```
2.按键的控制是在外部中断中进行（正点原子资料中提供相应参考代码）
比较重要的是改变采样频率。
可通过函数`void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode)`进行定时器2的预分频值更改。

第一次写文章，先写这么多，诸多地方还待完善，代码也有一些缺陷。希望和各位交流，相互学习。

[CSDN下载源码请点击此处](https://download.csdn.net/download/qq_44897194/12169167)或者评论区发邮箱（记得点赞哦！）。

**2022年7月16日记：**
由于需求人数较多，邮件发送较为麻烦。现将程序放在Gitee上，供大家免费下载。
链接地址：[源码链接（Gitee）](https://gitee.com/silent-rookie/Simple-Oscilloscope)
**注：本历程使用时由于直接通过ST-Link烧录程序，因此没有生成最新的hex文件。若出现程序无法正常运行的情况，请在工程中生成最新的hex再烧录！**
