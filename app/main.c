/*
 * @file       main.c
 * @brief      主函数
 * @author     lzh
 */
/*************
舵机的基础知识：
舵机的控制脉冲宽从0.5ms-2.5ms，分别对应-90度到+90度的位置。
1.5ms 为舵机归0点（置中）。接上摆臂后由于机械安装的偏差，
1.5ms的脉宽可能使摆臂不能回中，
这个时候我们可以通过实验得到使得摆臂回中的脉宽。
**************/
/*************
主板电机驱动的基础知识：
电机驱动信号为PWM，占空比为0-100%，占空比为0 电机停止。占空比为100% 电机满转
驱动电压一定的情况下，控制占空比可以控制转速。
一路电机由2路PWM波控制,假设记为AB两路。当A输出PWM，B不输出（占空比为0）时，电机正转
当B输出PWM，A不输出（占空比为0）时，电机反转
**************/
/*************
oled的基础知识
我们采用的字符是8*6大小，即一个字符占8行 6列的空间
OLED一共有64行，但是由于显示一个字符占用8行，所以实际有8行空间可以显示
*************/
#include "include.h"
#include "common.h"
#define N 1  //滑动滤波窗口
#define n 0 //异常值个数剔除

void all_init(void);//把初始化写在一起
void Motor_Out(void);//电机控制输出函数
void PIT_IRQHandler();
int motor_duty_Limit(int duty);//防止速度过大
int sever_duty_Limit(int duty);//防止舵机角度过大
void getBuf();
void bubbleSort() ;
void getValue();
void PID_motor();//电机PID

int SIGN(int a); 
int prv_ABS(int a);

/**定时器相关**/
int32 time_count=0;
unsigned char flag_5ms=0;
unsigned char flag_20ms=0;
unsigned char flag_100ms=0;

/**舵机相关B**/
int sever_middle=120;  //舵机摆臂回正的脉宽，需要根据实际情况修改，现在是(155/1000)*10ms=1.55ms 1000是脉冲精度 
int sever_range=20;    //限制一下舵机摆动的幅度，防止打死造成机械损坏（大约正负25度，根据实际情况修改）
int sever_duty=0;//舵机占空比变化值

/**驱动电机相关QAQ**/
float motor1_out,motor2_out;
int motor_duty=0;//驱动电机占空比变化值
int motor_duty_error=0;//用于调节两个轮子的PID  大于0向左偏 小于0向右偏
int motor_range=40;//驱动电机占空比限制在40%内，防止输出过大 也就是0~40

int16 pulse=0;		//脉冲数
int16 pulse_last=0;
int16 Realspeed=0;		//脉冲差数
int16 speed_set=0; 	//速度设定     motor-0.5---speed-1000
int16 speedmemo=0;  //记录速度值
int16 distmemo=10000; //记录距离值

//电机pid
int16 Kp_motor=100; // 除以1000
int16 Ki_motor=0;   //除以1000
int16 Kd_motor=0; //除以1000
int16 errorMot=0;
int16 errorMot_last=0;
int16 errorMot_i=0;
int16 errorMot_d=0;

/**电磁传感器相关**/
int16 Value1_SystemError=0;
int16 Value2_SystemError=35;
int16 value1,value2; //电感值 直接读取
int16 value11,value12,value13,value14,value15,value16,value17,value18;
int16 value21,value22,value23,value24,value25,value26,value27,value28;
int16 Value1=0,Value2=0;  // 滤波之后的电感值
int16 value1_buf[N]; 
int16 value2_buf[N];
int16 sort1_buf[N];
int16 sort2_buf[N];
int16 Value_average;

/**道路相关**/
unsigned char outload_flag=0;//冲出场外
/**OLED相关**/
char buff[20];

void  main(void)
{ 
  all_init();  //初始化
  OLED_Draw_Logo();
  DELAY_MS(2000);
  OLED_CLS();
  OLED_P6x8Str(0,0,"DaJaV"); //第0行第0列开始显示
  OLED_P6x8Str(6*6,0,"QAQ"); //第0行第30列开始显示

  OLED_P6x8Str(20,2,"Current_Road=");
  OLED_P6x8Str(20,3,"motor_duty=");
  OLED_P6x8Str(20,4,"sever_duty=");
  OLED_P6x8Str(20,5,"Value1= ");
  OLED_P6x8Str(20,6,"Value2= ");
  OLED_P6x8Str(20,7,"C_ERROR= ");
  EnableInterrupts; //打开中断 
  enable_irq (PIT0_IRQn); //使能中断
  while(1) 
    {
		if ((Value1<=2 || Value2<=2)&& Value_average<=20)//安全保护
			{
        outload_flag=1;
			}
  if(BT_YES_IN==0)//BT1 重启按钮
       {
          DELAY_MS(10); //延时10ms 消抖
            if(BT_YES_IN==0){
               outload_flag=0;//回到场地
            }  
       }
if(outload_flag==1){//冲出场外
	    //电机停转
			motor1_out=0; //转化为实际占空比
			motor2_out=0;  
			Motor_Out();
      sever_duty=0;
			FTM_PWM_Duty(FTM1,FTM_CH0,sever_middle+sever_duty);    //舵机控制输出
}
else{
  if(flag_5ms){
          flag_5ms=0;
          //滑动滤波
          getBuf();
          bubbleSort();
          getValue();
          Value_average=(Value1+Value2)/2;

      }
  if(flag_20ms){//用于电机输出
          flag_20ms=0;
        //控制编码器
			pulse=FTM_QUAD_get(FTM2);	//获取FTM2模块的读数
			Realspeed=pulse-pulse_last;
			pulse_last=pulse;                
      PID_motor();//pid控制一下duty_motor
      motor_duty=motor_duty_Limit(motor_duty);
      motor1_out=(motor_duty)*0.01; //转化为实际占空比
      motor2_out=(motor_duty)*0.01;  
      Motor_Out();//驱动电机控制输出

//      sever_duty=sever_duty_Limit(sever_duty);
//      FTM_PWM_Duty(FTM1,FTM_CH0,sever_middle+sever_duty);    //舵机控制输出
      }
  if(flag_100ms){//用于液晶显示屏刷新

          flag_100ms=0;
    sprintf(buff,"%d",motor_duty);  //将读数motor_duty转换为字符串 存在buff 里面 不懂的百度 sprintf 函数
    OLED_P6x8Str(20+66,3,buff); //将数值显示在液晶屏幕上11*6
    OLED_P6x8Char(' ');         //末尾放个空格防止显示错误（末尾不刷新）
    sprintf(buff,"%d",sever_duty);  //将读数sever_duty转换为字符串 存在buff 里面 不懂的百度 sprintf 函数
    OLED_P6x8Str(20+66,4,buff); //将数值显示在液晶屏幕上11*6
    OLED_P6x8Char(' ');         //末尾放个空格防止显示错误（末尾不刷新）
    sprintf(buff,"%d",Value1);  //将读数Value1转换为字符串 存在buff 里面 不懂的百度 sprintf 函数
    OLED_P6x8Str(20+66,5,buff); //将数值显示在液晶屏幕上11*6
    OLED_P6x8Char(' ');         //末尾放个空格防止显示错误（末尾不刷新）
    sprintf(buff,"%d",Value2);  //将读数Value2转换为字符串 存在buff 里面 不懂的百度 sprintf 函数
    OLED_P6x8Str(20+66,6,buff); //将数值显示在液晶屏幕上11*6
    OLED_P6x8Char(' ');         //末尾放个空格防止显示错误（末尾不刷新）    
      }
  }
    }
}

void all_init(void)
{
button_init();//初始化蜂鸣器
//BEEP_ON;//介意的可以把蜂鸣器关掉 打开打开 一定要打开
led_init();  //初始化LED   
OLED_Init(); //初始化显示屏
pit_init_ms(PIT0,5); //5ms定时中断 
set_vector_handler(PIT0_VECTORn ,PIT_IRQHandler);//中断向量表


FTM_PWM_init(FTM1,FTM_CH0,100,sever_middle);   //舵机 PWM  PTA12输出，频率为100hz,周期为10ms

FTM_PWM_init(FTM0,FTM_CH0,10*1000,0);          // 电机 PWM  PTC1输出，频率为10Khz、
FTM_PWM_init(FTM0,FTM_CH1,10*1000,0);          // 电机 PWM  PTC2输出，频率为10Khz、
FTM_PWM_init(FTM0,FTM_CH2,10*1000,0);          // 电机 PWM  PTC3输出，频率为10Khz、
FTM_PWM_init(FTM0,FTM_CH3,10*1000,0);          // 电机 PWM  PTC4输出，频率为10Khz、

   adc_init(ADC0_DP1);  //初始化AD模块
   adc_init(ADC0_DM1);  
   adc_init(ADC1_DP1);  
   adc_init(ADC1_DM1);

//   FTM_QUAD_Init(FTM1); //初始化正交解码计数通道
   FTM_QUAD_Init(FTM2); //初始化正交解码计数通道
}

void Motor_Out(void) //电机控制输出函数
{
  
 motor1_out=LIMIT(motor1_out,0.99,-0.99);  //电机的输出值限制在-1~1 之间，-1代表反转最快，1代表正转最快；
 motor2_out=LIMIT(motor2_out,0.99,-0.99);
  //电机1
  if(motor1_out>=0) //占空比为正，正转
  {
     FTM_PWM_Duty(FTM0,FTM_CH0,(int)(motor1_out*10000));//占空比精度为10000 ，占空比*占空比精度
     FTM_PWM_Duty(FTM0,FTM_CH2,0);
  }
  else   //为负就反转
  {
     FTM_PWM_Duty(FTM0,FTM_CH0,0);
     FTM_PWM_Duty(FTM0,FTM_CH2,(int)(-motor1_out*10000));
  }
  
  //电机2
    if(motor2_out>=0) //占空比为正，正转
  {
     FTM_PWM_Duty(FTM0,FTM_CH1,(int)(motor2_out*10000)); 
     FTM_PWM_Duty(FTM0,FTM_CH3,0);
  }
  else   //反转
  {
     FTM_PWM_Duty(FTM0,FTM_CH1,0);
     FTM_PWM_Duty(FTM0,FTM_CH3,(int)(-motor2_out*10000));
  }
}
void PID_motor()//电机PID
{
	  errorMot=speed_set-Realspeed;
        if (errorMot<=200 && abs(errorMot)<=20000 )
        {
          errorMot_i+=errorMot;
	      }
        errorMot_d=errorMot-errorMot_last;
        errorMot_last=errorMot;
        motor_duty=Kp_motor*errorMot+Kd_motor*errorMot_d+Ki_motor*errorMot_i;
        motor_duty/=1000;
	        //motor_duty=-motor_duty;
}

int motor_duty_Limit(int duty)//防止速度过大
{
      if (duty>=motor_range)  //如果超出了范围 占空比限制在40%内，防止输出过大
      duty=motor_range;
      if(duty<=-motor_range)
      duty=-motor_range;
      return duty;
}
int sever_duty_Limit(int duty)//防止舵机角度过大
{
      if (duty>=sever_range)  //如果超出了范围 占空比限制在20%内，防止输出过大
      duty=sever_range+1;
      if(duty<=-sever_range)
      duty=-sever_range;
      return duty;
}

void PIT_IRQHandler()  //5ms一次中断
{
    PIT_Flag_Clear(PIT0);       //清中断标志位
    time_count++;//每5ms加一次，并且不会造成数据溢出
                          flag_5ms=1;
    if(time_count%4==0)   flag_20ms=1;
    if(time_count%20==0)  flag_100ms=1;
}




int SIGN(int a){
  if(a>0) return 1;
  if(a<0) return -1;
  return 0;
}
int prv_ABS(int a)
{
  if(a>0) return a;
  else return -a;
}
void getBuf() //建立一个N长的窗口，把数据存储到value*_buf里面
{ 
      int16 count; 
      
      value1=adc_once(ADC0_DP1,ADC_12bit);  //获取电感模块的读数
      value2=adc_once(ADC0_DM1,ADC_12bit);  

      for ( count=0;count<N-1;count++) 
      {
        value1_buf[count]=value1_buf[count+1];
      }
      value1_buf[N-1]=value1;
      for ( count=0;count<N-1;count++) 
      {
        value2_buf[count]=value2_buf[count+1];
      }
      value2_buf[N-1]=value2;
}
//排序到 sort*_buf里面
void bubbleSort() 
{
	
    int i, j;
	int16 temp;

	//复制数组
	for(i=0; i<N;i++)
		{

		sort1_buf[i]=value1_buf[i];
		sort2_buf[i]=value2_buf[i];
		}
	//对数组进行排序
    for (i = 0; i < N - 1; i++)
		{
        for (j = 0; j < N - 1 - i; j++)
        	{
			if (sort1_buf[j] > sort1_buf[j + 1]) 
				{
                temp = sort1_buf[j];
                sort1_buf[j] = sort1_buf[j + 1];
                sort1_buf[j + 1] = temp;
        	    }
			if (sort2_buf[j] > sort2_buf[j + 1]) 
				{
                temp = sort2_buf[j];
                sort2_buf[j] = sort2_buf[j + 1];
                sort2_buf[j + 1] = temp;
        	    }
			
		}
	}
}
//去掉N-3个极大，和N-3个极小。剩下地取平均
void getValue()
{
	int64  sum1=0; 
	int64 sum2=0;

	int i=N;
	for (i=n;i<N-n;i++)
		{

		sum1+=sort1_buf[i];
		sum2+=sort2_buf[i];
		}
	Value1=sum1/(N-2*n);
	Value2=sum2/(N-2*n);
  Value2-=Value2_SystemError; //Value2的静差
}