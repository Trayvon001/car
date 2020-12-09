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

void all_init(void);//把初始化写在一起
void Motor_Out(void);//电机控制输出函数
void PIT_IRQHandler();
int motor_duty_Limit(int duty);//防止速度过大
int sever_duty_Limit(int duty);//防止舵机角度过大
void LOAD(void);//整个道路情况

/**舵机相关B**/
int sever_middle=120  //舵机摆臂回正的脉宽，需要根据实际情况修改，现在是(155/1000)*10ms=1.55ms 1000是脉冲精度 
int sever_range=13;    //限制一下舵机摆动的幅度，防止打死造成机械损坏（大约正负25度，根据实际情况修改）
int sever_duty=0;//舵机占空比变化值
/**驱动电机相关QAQ**/
float motor1_out,motor2_out;
int motor_duty=0;//驱动电机占空比变化值
int motor_range=40;//驱动电机占空比限制在40%内，防止输出过大 也就是0~40

/**电磁传感器相关**/
int16 Value1,Value2;
int16 Value1_SystemError=???;
int16 Value2_SystemError=???;
int error_stack[100];
unsigned char current_error=0;
unsigned char error_delay=10;

/**道路相关**/
#define load0_straight 0//直线
#define load1_curve 1   //大弯道
#define load2_roundabout 2 //环岛1
#define load3_curve 3      //小弯道
#define load4_roundabout 4 //环岛2
#define load5_curve 5   //小弯道
#define load6_curve 6   //大弯道
#define load7_straight 7   //直线
#define stop 10   //停止
unsigned char current_load_state=load0_straight;


char buff[20];
void  main(void)
{ 
  all_init();  //初始化
  OLED_Draw_Logo();
  DELAY_MS(2000);
  OLED_CLS();
  OLED_P6x8Str(0,0,"DaJaV"); //第0行第0列开始显示
  OLED_P6x8Str(5*6,0,"QAQ"); //第0行第30列开始显示

  OLED_P6x8Str(20,4,"Current_Road=");
  OLED_P6x8Str(20,5,"motor_duty=");
  OLED_P6x8Str(20,6,"sever_duty=");


  EnableInterrupts; //打开中断 
  enable_irq (PIT0_IRQn); //使能中断
  while(1) 
    {
 



      LOAD();





      motor_duty=motor_duty_Limit(motor_duty);
      motor1_out=motor_duty*0.01; //转化为实际占空比
      motor2_out=motor_duty*0.01;  
      Motor_Out();//驱动电机控制输出

      sever_duty=sever_duty_Limit(sever_duty);
      FTM_PWM_Duty(FTM1,FTM_CH0,sever_middle+sever_duty);    //舵机控制输出

    sprintf(buff,"%d",current_load_state);  //将读数Value1转换为字符串 存在buff 里面 不懂的百度 sprintf 函数
    OLED_P6x8Str(20+78,4,buff); //将数值显示在液晶屏幕上 13*6
    OLED_P6x8Char(' ');         //末尾放个空格防止显示错误（末尾不刷新）
    sprintf(buff,"%d",motor_duty);  //将读数Value2转换为字符串 存在buff 里面 不懂的百度 sprintf 函数
    OLED_P6x8Str(20+66,5,buff); //将数值显示在液晶屏幕上11*6
    OLED_P6x8Char(' ');         //末尾放个空格防止显示错误（末尾不刷新）
    sprintf(buff,"%d",sever_duty);  //将读数Value2转换为字符串 存在buff 里面 不懂的百度 sprintf 函数
    OLED_P6x8Str(20+66,6,buff); //将数值显示在液晶屏幕上11*6
    OLED_P6x8Char(' ');         //末尾放个空格防止显示错误（末尾不刷新）

      DELAY_MS(90); //延时90ms
    }
    
}

void all_init(void)
{
led_init();  //初始化LED
OLED_Init(); 
pit_init_ms(PIT0,10); //10ms定时中断 用于传感器获取读数
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
int motor_duty_Limit(int duty)//防止速度过大
{
      if (duty>=motor_range)  //如果超出了范围 占空比限制在40%内，防止输出过大
      duty=40;
      if(duty<=-motor_range)
      duty=-motor_range;
      return duty;
}
int sever_duty_Limit(int duty)//防止舵机角度过大
{
      if (duty>=sever_range)  //如果超出了范围 占空比限制在20%内，防止输出过大
      duty=sever_range;
      if(duty<=-sever_range)
      duty=-sever_range;
      return duty;
}

void PIT_IRQHandler()  //10ms一次中断
{
   static unsigned char i=0;
   static int16 Value1_cal[5],Value2_cal[5];
    PIT_Flag_Clear(PIT0);       //清中断标志位
    if(i<5)
    {
      Value1=adc_once(ADC0_DP1,ADC_12bit)-Value1_SystemError;  //获取电感模块的读数
      Value2=adc_once(ADC0_DM1,ADC_12bit)-Value2_SystemError;
      if(Value1+Value2<???) current_load_state=stop;//确保安全
      else
      {
      Value1_cal[i]=Value1;
      Value2_cal[i]=Value2;
      i++;
      }
    }
   else
   {
     error_stack[(current_error+error_delay)%100] =(Value1_cal[0]+Value1_cal[1]+Value1_cal[2]+Value1_cal[3]+Value1_cal[4])/5-(Value2_cal[0]+Value2_cal[1]+Value2_cal[2]+Value2_cal[3]+Value2_cal[4])/5;
     i=0;
   }

}


void LOAD(void)
{



switch(current_load_state)
{
case load0_straight:
motor_duty=10;
/*if(ABS(error_stack[current_error])>???)   current_load_state=load1_curve;
else 
{
sever_duty= sever_PID(error_stack[current_error]);
}*/
sever_duty= sever_PID(error_stack[current_error]);
current_error=(current_error+1)%100;
break;
case load1_curve:


break;
case load2_roundabout:

break;
case load3_curve:

break;
case load4_roundabout:

break;
case load5_curve:

break;
case load6_curve:

break;
case load7_straight:

break;
case stop:
motor_duty=0;
sever_duty=0;
break;
}
}