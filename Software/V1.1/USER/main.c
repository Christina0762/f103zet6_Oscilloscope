/**********************************************************
简易示波器，PA6采样输入
ADC(TIME触发)->DMA
PA4通过DAC通道1输出正弦波，频率由TIME3分频值控制
PA5通过DAC通道2输出三角波或噪声，频率由TIME4控制
KEY_WAKE控制更新与暂停
KEY0增加采样频率
KEY1降低采样频率
定时器4触发DAC通道2输出三角波和噪声
定时器3产生中断DAC通道1输出正弦波

问题：三角波的输出频率有问题，与理论值不符 
***********************************************************/
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "adc.h"
#include "dma.h"
#include "timer.h"
#include "table_fft.h"
#include "stm32_dsp.h"
#include "math.h"
#include "key.h"
#include "BEEP.h"
#include "dac.h"
#include "exti.h"

#define NPT 1024 //采样次数
#define PI2 6.28318530717959

void lcd_huadian(u16 a,u16 b,u16 color);//画点函数
void lcd_huaxian(u16 x1,u16 y1,u16 x2,u16 y2);//画线函数
void window(void);//主界面
void clear_point(u16 mode);//更新显示屏当前列
void InitBufInArray(void);//正弦波输出缓存
void sinout(void);//正弦波输出
void GetPowerMag(void);//FFT变换，输出频率

int long fftin [NPT];//FFT输入
int long fftout[NPT];//FFT输出
u32 FFT_Mag[NPT/2]={0};//幅频特性
u16 magout[NPT];//模拟正弦波输出缓存区

u16 table[15] ={16,32,48,64,80,96,112,128,144,160,176,192,208,224,240};//标点横坐标
u16 currentadc;//实时采样数据
u16 adcx[NPT];//adc数值缓存
u32 adcmax;//采样最大值和最小值
u32 adcmin;
u8 adc_flag=0;//采样结束标志
u8 key_flag=0;//按键扫描标志
u8 show_flag=1;//更新暂停标志
u16 T=2000;//定时器2重载值，不能小于PWM的Pluse值
u16 pre=36;//定时器2预分频值
u32 fre;//采样频率 kHz
u16 F;//波形频率

u16 V=660;//纵坐标单位刻度 mv/div
u16 temp=0;//幅值最大的频率成分
u16 t=0;
u16 key;//按键值

int main()
{
	u16 i;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	MYDMA1_Config(DMA1_Channel1,(u32)&ADC1->DR,(u32)&currentadc,1);
	uart_init(115200);
	delay_init();
	LED_Init();
	EXTIX_Init();
	LCD_Init();
	Adc_Init();
	BEEP_Init();
	InitBufInArray();
	TIM4_Int_Init(1,35);	//三角波和噪声频率控制，
	TIM3_Int_Init(39,71);	//72MHz/40/72=25kHz   25kHz/1024≈25Hz 正弦波频率约为24.5Hz
	TIM2_PWM_Init(T-1,pre-1);	//最大频率72000000/1/2000=3.6KHz
	Dac1_Init();
	Dac2_Init();
	LCD_Clear(BLACK);
	window();
	while(1)
	{
		//等待采样完成
		while(adc_flag==0)
		{
			LED1=!LED1;
			delay_ms(100);
		}
		adc_flag=0;
		
		//获取最大最小值
		adcmax=adcx[1];
		adcmin=adcx[1];
		for(i=0;i<NPT;i++)
		{
			fftin[i] = 0;
			fftin[i] = adcx[i] << 16;
			
			if(adcx[i] >= adcmax)
			{			
				adcmax = adcx[i];
			}			
			if(adcx[i] <= adcmin)
			{			
				adcmin = adcx[i];
			}						
		}
		
		POINT_COLOR=BLUE;
		GetPowerMag();
		
		adcmax=adcmax*0.8;   //0.8 ≈ 3300/4096	
		adcmin=adcmin*0.8;
		
		LCD_ShowNum(270,25,adcmax,4,16);	//显示最大值
		LCD_ShowNum(270,65,adcmin,4,16);	//显示最小值
		LCD_ShowNum(270,105,adcmax-adcmin,4,16);	//显示幅值
		
		if(show_flag==1)
		{
			clear_point(1);    //更新显示屏当前列，采用连线绘制
		}	
		
		LED0=!LED0;
		DMA_Cmd(DMA1_Channel1,ENABLE);//使能DMA1-CH1
		delay_ms(100);
	}
}

/**********************************************************
简介：画点函数，反转Y坐标
***********************************************************/
void lcd_huadian(u16 a,u16 b,u16 color)
{							    
	LCD_Fast_DrawPoint(a,240-b,color);
}

/**********************************************************
简介：画线函数，反转Y坐标
***********************************************************/
void lcd_huaxian(u16 x1,u16 y1,u16 x2,u16 y2)
{
	LCD_DrawLine(x1,240-y1,x2,240-y2);
}

/**********************************************************
简介：主界面绘制
***********************************************************/
void window(void)
{
	u16 x,i;
	static u16 h; 
	
	POINT_COLOR=GREEN;	  
	LCD_ShowString(5,8,200,24,24,"OSC-DWY");
	
	POINT_COLOR=GRAY;
	LCD_ShowString(190,13,200,16,16,"mV/div");	
	LCD_ShowString(260,5,200,16,16,"max(mv):");
	LCD_ShowString(260,45,200,16,16,"min(mv):");
	LCD_ShowString(260,85,200,16,16,"vpp(mv):");
	LCD_ShowString(260,165,200,16,16,"f(Hz):");
	LCD_ShowString(260,200,200,16,16,"OSR:");  //采样率	
	LCD_ShowString(304,220,200,16,16,"Hz");
	LCD_ShowString(304,250,200,16,16,"F1-F2=-1982hz");

	POINT_COLOR=BRRED;
	LCD_ShowString(100,13,200,16,16,"IN:PA6");
	
	POINT_COLOR=BLUE;
	LCD_ShowNum(150,13,V,4,16);//mv/div
	
	POINT_COLOR=WHITE;			
	lcd_huaxian(0,0,0,200);
	lcd_huaxian(256,0,256,200);
	lcd_huaxian(0,0,256,0);		
	lcd_huaxian(0,200,256,200);
	
	for(x=0;x<256;x++)
	{
		lcd_huadian(x,100,WHITE);
		if(x == table[h])	
		{
			lcd_huaxian(x,1,x,3);
			lcd_huaxian(x,101,x,103);
			lcd_huaxian(x,199,x,197);
			h++;
			if(h>=16) h=0;
		}	
		if(x==128) 
		{
			lcd_huaxian(x,1,x,199);
			for(i=10;i<200;i+=10)
			{
				lcd_huaxian(125,i,127,i);
			}
		}
	}
	
	POINT_COLOR=MAGENTA;	
	LCD_ShowString(260,128,200,16,16,"ing...");
}

/******************************************************************
函数名称:clear_point()
函数功能:循环更新波形
参数说明:mode 波形模式选择 1——连线模式，0——打点模式
备    注:波形的显示可采用打点方式和绘制线方式
*******************************************************************/
void clear_point(u16 mode)
{
	u16 x,i,past_vol,pre_vol;
	static u16 h; 
	
	POINT_COLOR=BLUE;
	fre=72000000/T/pre;//更新采样频率
	LCD_ShowNum(261,220,fre,5,16);//更新采样率显示
	
	//清除旧波形与绘制坐标轴
	for(x=0;x<256;x++)
	{	
		POINT_COLOR=BLACK;	//按列清除
		if(x!=128)	//y轴不进行列清除
			lcd_huaxian(x,4,x,196);
		
		//绘制坐标轴
		POINT_COLOR=WHITE;
		lcd_huaxian(0,0,0,200);
		lcd_huadian(x,100,WHITE);

		// 动态刻度标记（基于table数组）
		if(x == table[h])	
		{
			lcd_huaxian(x,101,x,103);
			h++;
			if(h>=15) h=0;
		}	
		
		// 绘制Y轴及刻度
		if(x==128) 
		{
			lcd_huaxian(x,1,x,199);
			for(i=10;i<200;i+=10)
			{
				lcd_huaxian(125,i,127,i);
			}
		}
		
		pre_vol = 50+adcx[x]/4096.0*100; // 将ADC值转换为垂直坐标

		//波形更新
		if(mode==1)
		{
			POINT_COLOR=YELLOW;
			if(x>0&&x<255&&x!=128)	//去除第一个，最后一个以及y轴上点的绘制
				lcd_huaxian(x,past_vol,x+1,pre_vol);// 连接当前点与下一点
		}
		else
			lcd_huadian(x,pre_vol,YELLOW); // 绘制离散点

		past_vol = pre_vol;// 保存当前点坐标，供下次连线使用
	}
	
}

/******************************************************************
函数名称:InitBufInArray()
函数功能:正弦波值初始化，将正弦波各点的值存入magout[]数组中
参数说明:
备    注:需要拔掉WIFI模块，否则输出电压出错
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

/******************************************************************
函数名称:GetPowerMag()
函数功能:计算各次谐波幅值
参数说明:
备　　注:先将lBufOutArray分解成实部(X)和虚部(Y)，然后计算幅值(sqrt(X*X+Y*Y)
*******************************************************************/
void GetPowerMag(void)
{
    float X,Y,Mag,magmax=0;//实部，虚部，各频率幅值，最大幅值
    u16 i;
	
	//调用自cr4_fft_1024_stm32
	cr4_fft_1024_stm32(fftout, fftin, NPT);	
	
    for(i=1; i<NPT/2; i++)
    {
		X = (fftout[i] << 16) >> 16;	//低16位存实部
		Y = (fftout[i] >> 16);			//高16位存虚部
		
		Mag = sqrt(X * X + Y * Y); 		//计算模值
		FFT_Mag[i]=Mag;					//存入缓存，用于输出查验
		//获取最大频率分量及其幅值
		if(Mag > magmax)
		{
			magmax = Mag; // 记录最大幅值
			temp = i; 	  // 记录对应频点序号
		}
    }
	// 采样频率fre越高，可检测频率f范围越大，但频率分辨率下降，误差就越大（频率分辨率=采样频率fre/采样点数NPT）
	F=(u16)(temp*(fre*1.0/NPT));	//FFT所得实际频率f=最大幅值索引 temp ×(采样频率 fre / 总采样点数 NPT)
		
//	if(T==1000)		F=(u32)((double)temp/NPT*1000  );	
//	if(T==100)		F=(u32)((double)temp/NPT*10010 );
//	if(T==10)		F=(u32)((double)temp/NPT*100200);
//	if(T==2)		F=(u32)((double)temp/NPT*249760);
	
	LCD_ShowNum(260,180,F*10,5,16);	
	
//		LCD_ShowNum(280,200,temp,4,16);					
//		LCD_ShowNum(280,220,(u32)(magmax*2.95),5,16);			
}

/******************************************************************
简介：DMA中断用于完整采样一次（采样1024次），
	  并将其存储于adcx[]缓存数组中，等待后续数据处理
*******************************************************************/	
void DMA1_Channel1_IRQHandler(void) 
{
	if(DMA_GetITStatus(DMA1_IT_TC1)!=RESET)
	{
		adcx[t]=currentadc;  // 将DMA缓冲区currentadc中的ADC值存入数组
		t++;                 // 存储位置索引自增
		if(t==NPT)           // 当存满1024个样本(NPT=1024)
		{
			t=0;            // 重置索引
			adc_flag=1;     // 置位采样完成标志
			DMA_Cmd(DMA1_Channel1, DISABLE); // 关闭DMA通道
		}
	}
	DMA_ClearITPendingBit(DMA1_IT_TC1);
}

/******************************************************************
简介：定时器3中断服务函数，用于正弦波输出
	  每进入一次中断改变一次DCA输出值
*******************************************************************/
void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
			sinout();
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源 
		}
}

/******************************************************************
简介：三个外部中断用于按键的读取
WK_UP按键控制波形显示的更新和暂停
KEY1按键降低采样频率
KEY0按键增加采样频率
*******************************************************************/
void EXTI0_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(WK_UP==1)	 	 //WK_UP按键
	{	
		BEEP=1;
		delay_ms(50);
		BEEP=0;
		show_flag=!show_flag;
		
		POINT_COLOR=MAGENTA;
		if(show_flag)
			LCD_ShowString(260,128,200,16,16,"ing...");
		else
			LCD_ShowString(260,128,200,16,16,"stop");
	}
	EXTI_ClearITPendingBit(EXTI_Line0); //清除LINE0上的中断标志位  
}
 
void EXTI3_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(KEY1==0)	 //按键KEY1
	{	
		BEEP=1;
		delay_ms(50);
		BEEP=0;
		pre=pre+5;
		if(pre>72)
		{
			pre=1;
		}
		TIM_PrescalerConfig(TIM2,pre-1,TIM_PSCReloadMode_Immediate);
	}		 
	EXTI_ClearITPendingBit(EXTI_Line3);  //清除LINE3上的中断标志位  
}

void EXTI4_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(KEY0==0)	 //按键KEY0
	{
		BEEP=1;
		delay_ms(50);
		BEEP=0;
		pre=pre-5;
		if(pre<=1)
		{
			pre=1;
		}
		TIM_PrescalerConfig(TIM2,pre-1,TIM_PSCReloadMode_Immediate);
	}		 
	EXTI_ClearITPendingBit(EXTI_Line4);  //清除LINE4上的中断标志位  
}
