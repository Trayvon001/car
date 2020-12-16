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
#define N 1  //�����˲�����
#define n 0 //�쳣ֵ�����޳�

void all_init(void);//�ѳ�ʼ��д��һ��
void Motor_Out(void);//��������������
void PIT_IRQHandler();
int motor_duty_Limit(int duty);//��ֹ�ٶȹ���
int sever_duty_Limit(int duty);//��ֹ����Ƕȹ���
void getBuf();
void bubbleSort() ;
void getValue();
void PID_motor();//���PID

int SIGN(int a); 
int prv_ABS(int a);

/**��ʱ�����**/
int32 time_count=0;
unsigned char flag_5ms=0;
unsigned char flag_20ms=0;
unsigned char flag_100ms=0;

/**������B**/
int sever_middle=120;  //����ڱۻ�����������Ҫ����ʵ������޸ģ�������(155/1000)*10ms=1.55ms 1000�����徫�� 
int sever_range=20;    //����һ�¶���ڶ��ķ��ȣ���ֹ������ɻ�е�𻵣���Լ����25�ȣ�����ʵ������޸ģ�
int sever_duty=0;//���ռ�ձȱ仯ֵ

/**����������QAQ**/
float motor1_out,motor2_out;
int motor_duty=0;//�������ռ�ձȱ仯ֵ
int motor_duty_error=0;//���ڵ����������ӵ�PID  ����0����ƫ С��0����ƫ
int motor_range=40;//�������ռ�ձ�������40%�ڣ���ֹ������� Ҳ����0~40

int16 pulse=0;		//������
int16 pulse_last=0;
int16 Realspeed=0;		//�������
int16 speed_set=0; 	//�ٶ��趨     motor-0.5---speed-1000
int16 speedmemo=0;  //��¼�ٶ�ֵ
int16 distmemo=10000; //��¼����ֵ

//���pid
int16 Kp_motor=100; // ����1000
int16 Ki_motor=0;   //����1000
int16 Kd_motor=0; //����1000
int16 errorMot=0;
int16 errorMot_last=0;
int16 errorMot_i=0;
int16 errorMot_d=0;

/**��Ŵ��������**/
int16 Value1_SystemError=0;
int16 Value2_SystemError=35;
int16 value1,value2; //���ֵ ֱ�Ӷ�ȡ
int16 value11,value12,value13,value14,value15,value16,value17,value18;
int16 value21,value22,value23,value24,value25,value26,value27,value28;
int16 Value1=0,Value2=0;  // �˲�֮��ĵ��ֵ
int16 value1_buf[N]; 
int16 value2_buf[N];
int16 sort1_buf[N];
int16 sort2_buf[N];
int16 Value_average;

/**��·���**/
unsigned char outload_flag=0;//�������
/**OLED���**/
char buff[20];

void  main(void)
{ 
  all_init();  //��ʼ��
  OLED_Draw_Logo();
  DELAY_MS(2000);
  OLED_CLS();
  OLED_P6x8Str(0,0,"DaJaV"); //��0�е�0�п�ʼ��ʾ
  OLED_P6x8Str(6*6,0,"QAQ"); //��0�е�30�п�ʼ��ʾ

  OLED_P6x8Str(20,2,"Current_Road=");
  OLED_P6x8Str(20,3,"motor_duty=");
  OLED_P6x8Str(20,4,"sever_duty=");
  OLED_P6x8Str(20,5,"Value1= ");
  OLED_P6x8Str(20,6,"Value2= ");
  OLED_P6x8Str(20,7,"C_ERROR= ");
  EnableInterrupts; //���ж� 
  enable_irq (PIT0_IRQn); //ʹ���ж�
  while(1) 
    {
		if ((Value1<=2 || Value2<=2)&& Value_average<=20)//��ȫ����
			{
        outload_flag=1;
			}
  if(BT_YES_IN==0)//BT1 ������ť
       {
          DELAY_MS(10); //��ʱ10ms ����
            if(BT_YES_IN==0){
               outload_flag=0;//�ص�����
            }  
       }
if(outload_flag==1){//�������
	    //���ͣת
			motor1_out=0; //ת��Ϊʵ��ռ�ձ�
			motor2_out=0;  
			Motor_Out();
      sever_duty=0;
			FTM_PWM_Duty(FTM1,FTM_CH0,sever_middle+sever_duty);    //����������
}
else{
  if(flag_5ms){
          flag_5ms=0;
          //�����˲�
          getBuf();
          bubbleSort();
          getValue();
          Value_average=(Value1+Value2)/2;

      }
  if(flag_20ms){//���ڵ�����
          flag_20ms=0;
        //���Ʊ�����
			pulse=FTM_QUAD_get(FTM2);	//��ȡFTM2ģ��Ķ���
			Realspeed=pulse-pulse_last;
			pulse_last=pulse;                
      PID_motor();//pid����һ��duty_motor
      motor_duty=motor_duty_Limit(motor_duty);
      motor1_out=(motor_duty)*0.01; //ת��Ϊʵ��ռ�ձ�
      motor2_out=(motor_duty)*0.01;  
      Motor_Out();//��������������

//      sever_duty=sever_duty_Limit(sever_duty);
//      FTM_PWM_Duty(FTM1,FTM_CH0,sever_middle+sever_duty);    //����������
      }
  if(flag_100ms){//����Һ����ʾ��ˢ��

          flag_100ms=0;
    sprintf(buff,"%d",motor_duty);  //������motor_dutyת��Ϊ�ַ��� ����buff ���� �����İٶ� sprintf ����
    OLED_P6x8Str(20+66,3,buff); //����ֵ��ʾ��Һ����Ļ��11*6
    OLED_P6x8Char(' ');         //ĩβ�Ÿ��ո��ֹ��ʾ����ĩβ��ˢ�£�
    sprintf(buff,"%d",sever_duty);  //������sever_dutyת��Ϊ�ַ��� ����buff ���� �����İٶ� sprintf ����
    OLED_P6x8Str(20+66,4,buff); //����ֵ��ʾ��Һ����Ļ��11*6
    OLED_P6x8Char(' ');         //ĩβ�Ÿ��ո��ֹ��ʾ����ĩβ��ˢ�£�
    sprintf(buff,"%d",Value1);  //������Value1ת��Ϊ�ַ��� ����buff ���� �����İٶ� sprintf ����
    OLED_P6x8Str(20+66,5,buff); //����ֵ��ʾ��Һ����Ļ��11*6
    OLED_P6x8Char(' ');         //ĩβ�Ÿ��ո��ֹ��ʾ����ĩβ��ˢ�£�
    sprintf(buff,"%d",Value2);  //������Value2ת��Ϊ�ַ��� ����buff ���� �����İٶ� sprintf ����
    OLED_P6x8Str(20+66,6,buff); //����ֵ��ʾ��Һ����Ļ��11*6
    OLED_P6x8Char(' ');         //ĩβ�Ÿ��ո��ֹ��ʾ����ĩβ��ˢ�£�    
      }
  }
    }
}

void all_init(void)
{
button_init();//��ʼ��������
//BEEP_ON;//����Ŀ��԰ѷ������ص� �򿪴� һ��Ҫ��
led_init();  //��ʼ��LED   
OLED_Init(); //��ʼ����ʾ��
pit_init_ms(PIT0,5); //5ms��ʱ�ж� 
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
void PID_motor()//���PID
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

int motor_duty_Limit(int duty)//��ֹ�ٶȹ���
{
      if (duty>=motor_range)  //��������˷�Χ ռ�ձ�������40%�ڣ���ֹ�������
      duty=motor_range;
      if(duty<=-motor_range)
      duty=-motor_range;
      return duty;
}
int sever_duty_Limit(int duty)//��ֹ����Ƕȹ���
{
      if (duty>=sever_range)  //��������˷�Χ ռ�ձ�������20%�ڣ���ֹ�������
      duty=sever_range+1;
      if(duty<=-sever_range)
      duty=-sever_range;
      return duty;
}

void PIT_IRQHandler()  //5msһ���ж�
{
    PIT_Flag_Clear(PIT0);       //���жϱ�־λ
    time_count++;//ÿ5ms��һ�Σ����Ҳ�������������
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
void getBuf() //����һ��N���Ĵ��ڣ������ݴ洢��value*_buf����
{ 
      int16 count; 
      
      value1=adc_once(ADC0_DP1,ADC_12bit);  //��ȡ���ģ��Ķ���
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
//���� sort*_buf����
void bubbleSort() 
{
	
    int i, j;
	int16 temp;

	//��������
	for(i=0; i<N;i++)
		{

		sort1_buf[i]=value1_buf[i];
		sort2_buf[i]=value2_buf[i];
		}
	//�������������
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
//ȥ��N-3�����󣬺�N-3����С��ʣ�µ�ȡƽ��
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
  Value2-=Value2_SystemError; //Value2�ľ���
}