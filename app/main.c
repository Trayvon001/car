/*
 * @file       main.c
 * @brief      ������
 * @author     lzh
 */
/*************
����Ļ���֪ʶ��
����Ŀ���������0.5ms-2.5ms���ֱ��Ӧ-90�ȵ�+90�ȵ�λ�á�
1.5ms Ϊ�����0�㣨���У������ϰڱۺ����ڻ�е��װ��ƫ�
1.5ms���������ʹ�ڱ۲��ܻ��У�
���ʱ�����ǿ���ͨ��ʵ��õ�ʹ�ðڱۻ��е�����
**************/
/*************
�����������Ļ���֪ʶ��
��������ź�ΪPWM��ռ�ձ�Ϊ0-100%��ռ�ձ�Ϊ0 ���ֹͣ��ռ�ձ�Ϊ100% �����ת
������ѹһ��������£�����ռ�ձȿ��Կ���ת�١�
һ·�����2·PWM������,�����ΪAB��·����A���PWM��B�������ռ�ձ�Ϊ0��ʱ�������ת
��B���PWM��A�������ռ�ձ�Ϊ0��ʱ�������ת
**************/
/*************
oled�Ļ���֪ʶ
���ǲ��õ��ַ���8*6��С����һ���ַ�ռ8�� 6�еĿռ�
OLEDһ����64�У�����������ʾһ���ַ�ռ��8�У�����ʵ����8�пռ������ʾ
*************/
#include "include.h"
#include "common.h"

void all_init(void);//�ѳ�ʼ��д��һ��
void Motor_Out(void);//��������������
void PIT_IRQHandler();
int motor_duty_Limit(int duty);//��ֹ�ٶȹ���
int sever_duty_Limit(int duty);//��ֹ����Ƕȹ���
void LOAD(void);//������·���

/**������B**/
int sever_middle=120  //����ڱۻ�����������Ҫ����ʵ������޸ģ�������(155/1000)*10ms=1.55ms 1000�����徫�� 
int sever_range=13;    //����һ�¶���ڶ��ķ��ȣ���ֹ������ɻ�е�𻵣���Լ����25�ȣ�����ʵ������޸ģ�
int sever_duty=0;//���ռ�ձȱ仯ֵ
/**����������QAQ**/
float motor1_out,motor2_out;
int motor_duty=0;//�������ռ�ձȱ仯ֵ
int motor_range=40;//�������ռ�ձ�������40%�ڣ���ֹ������� Ҳ����0~40

/**��Ŵ��������**/
int16 Value1,Value2;
int16 Value1_SystemError=???;
int16 Value2_SystemError=???;
int error_stack[100];
unsigned char current_error=0;
unsigned char error_delay=10;

/**��·���**/
#define load0_straight 0//ֱ��
#define load1_curve 1   //�����
#define load2_roundabout 2 //����1
#define load3_curve 3      //С���
#define load4_roundabout 4 //����2
#define load5_curve 5   //С���
#define load6_curve 6   //�����
#define load7_straight 7   //ֱ��
#define stop 10   //ֹͣ
unsigned char current_load_state=load0_straight;


char buff[20];
void  main(void)
{ 
  all_init();  //��ʼ��
  OLED_Draw_Logo();
  DELAY_MS(2000);
  OLED_CLS();
  OLED_P6x8Str(0,0,"DaJaV"); //��0�е�0�п�ʼ��ʾ
  OLED_P6x8Str(5*6,0,"QAQ"); //��0�е�30�п�ʼ��ʾ

  OLED_P6x8Str(20,4,"Current_Road=");
  OLED_P6x8Str(20,5,"motor_duty=");
  OLED_P6x8Str(20,6,"sever_duty=");


  EnableInterrupts; //���ж� 
  enable_irq (PIT0_IRQn); //ʹ���ж�
  while(1) 
    {
 



      LOAD();





      motor_duty=motor_duty_Limit(motor_duty);
      motor1_out=motor_duty*0.01; //ת��Ϊʵ��ռ�ձ�
      motor2_out=motor_duty*0.01;  
      Motor_Out();//��������������

      sever_duty=sever_duty_Limit(sever_duty);
      FTM_PWM_Duty(FTM1,FTM_CH0,sever_middle+sever_duty);    //����������

    sprintf(buff,"%d",current_load_state);  //������Value1ת��Ϊ�ַ��� ����buff ���� �����İٶ� sprintf ����
    OLED_P6x8Str(20+78,4,buff); //����ֵ��ʾ��Һ����Ļ�� 13*6
    OLED_P6x8Char(' ');         //ĩβ�Ÿ��ո��ֹ��ʾ����ĩβ��ˢ�£�
    sprintf(buff,"%d",motor_duty);  //������Value2ת��Ϊ�ַ��� ����buff ���� �����İٶ� sprintf ����
    OLED_P6x8Str(20+66,5,buff); //����ֵ��ʾ��Һ����Ļ��11*6
    OLED_P6x8Char(' ');         //ĩβ�Ÿ��ո��ֹ��ʾ����ĩβ��ˢ�£�
    sprintf(buff,"%d",sever_duty);  //������Value2ת��Ϊ�ַ��� ����buff ���� �����İٶ� sprintf ����
    OLED_P6x8Str(20+66,6,buff); //����ֵ��ʾ��Һ����Ļ��11*6
    OLED_P6x8Char(' ');         //ĩβ�Ÿ��ո��ֹ��ʾ����ĩβ��ˢ�£�

      DELAY_MS(90); //��ʱ90ms
    }
    
}

void all_init(void)
{
led_init();  //��ʼ��LED
OLED_Init(); 
pit_init_ms(PIT0,10); //10ms��ʱ�ж� ���ڴ�������ȡ����
set_vector_handler(PIT0_VECTORn ,PIT_IRQHandler);//�ж�������

FTM_PWM_init(FTM1,FTM_CH0,100,sever_middle);   //��� PWM  PTA12�����Ƶ��Ϊ100hz,����Ϊ10ms

FTM_PWM_init(FTM0,FTM_CH0,10*1000,0);          // ��� PWM  PTC1�����Ƶ��Ϊ10Khz��
FTM_PWM_init(FTM0,FTM_CH1,10*1000,0);          // ��� PWM  PTC2�����Ƶ��Ϊ10Khz��
FTM_PWM_init(FTM0,FTM_CH2,10*1000,0);          // ��� PWM  PTC3�����Ƶ��Ϊ10Khz��
FTM_PWM_init(FTM0,FTM_CH3,10*1000,0);          // ��� PWM  PTC4�����Ƶ��Ϊ10Khz��

   adc_init(ADC0_DP1);  //��ʼ��ADģ��
   adc_init(ADC0_DM1);  
   adc_init(ADC1_DP1);  
   adc_init(ADC1_DM1);

//   FTM_QUAD_Init(FTM1); //��ʼ�������������ͨ��
   FTM_QUAD_Init(FTM2); //��ʼ�������������ͨ��
}
void Motor_Out(void) //��������������
{
  
 motor1_out=LIMIT(motor1_out,0.99,-0.99);  //��������ֵ������-1~1 ֮�䣬-1����ת��죬1������ת��죻
 motor2_out=LIMIT(motor2_out,0.99,-0.99);
  //���1
  if(motor1_out>=0) //ռ�ձ�Ϊ������ת
  {
     FTM_PWM_Duty(FTM0,FTM_CH0,(int)(motor1_out*10000));//ռ�ձȾ���Ϊ10000 ��ռ�ձ�*ռ�ձȾ���
     FTM_PWM_Duty(FTM0,FTM_CH2,0);
  }
  else   //Ϊ���ͷ�ת
  {
     FTM_PWM_Duty(FTM0,FTM_CH0,0);
     FTM_PWM_Duty(FTM0,FTM_CH2,(int)(-motor1_out*10000));
  }
  
  //���2
    if(motor2_out>=0) //ռ�ձ�Ϊ������ת
  {
     FTM_PWM_Duty(FTM0,FTM_CH1,(int)(motor2_out*10000)); 
     FTM_PWM_Duty(FTM0,FTM_CH3,0);
  }
  else   //��ת
  {
     FTM_PWM_Duty(FTM0,FTM_CH1,0);
     FTM_PWM_Duty(FTM0,FTM_CH3,(int)(-motor2_out*10000));
  }
}
int motor_duty_Limit(int duty)//��ֹ�ٶȹ���
{
      if (duty>=motor_range)  //��������˷�Χ ռ�ձ�������40%�ڣ���ֹ�������
      duty=40;
      if(duty<=-motor_range)
      duty=-motor_range;
      return duty;
}
int sever_duty_Limit(int duty)//��ֹ����Ƕȹ���
{
      if (duty>=sever_range)  //��������˷�Χ ռ�ձ�������20%�ڣ���ֹ�������
      duty=sever_range;
      if(duty<=-sever_range)
      duty=-sever_range;
      return duty;
}

void PIT_IRQHandler()  //10msһ���ж�
{
   static unsigned char i=0;
   static int16 Value1_cal[5],Value2_cal[5];
    PIT_Flag_Clear(PIT0);       //���жϱ�־λ
    if(i<5)
    {
      Value1=adc_once(ADC0_DP1,ADC_12bit)-Value1_SystemError;  //��ȡ���ģ��Ķ���
      Value2=adc_once(ADC0_DM1,ADC_12bit)-Value2_SystemError;
      if(Value1+Value2<???) current_load_state=stop;//ȷ����ȫ
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