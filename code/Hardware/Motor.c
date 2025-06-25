#include "stm32f10x.h"                  // Device header
#include "PWM.h"

/**
  * ��    ����ֱ�������ʼ��
  * ��    ������
  * �� �� ֵ����
  */
void Motor_Init(void)
{
	/*����ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//����GPIOA��ʱ��
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_0 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);						//��PA4��PA5���ų�ʼ��Ϊ�������	
	
	PWM_Init();													//��ʼ��ֱ������ĵײ�PWM
}

/**
  * ��    ����ֱ����������ٶ�1
  * ��    ����Speed Ҫ���õ��ٶȣ���Χ��-100~100
  * �� �� ֵ����
  */
void MotorA_SetSpeed(int8_t Speed)
{
	if (Speed >= 0)							//���������ת���ٶ�ֵ
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_4);	//PA4�øߵ�ƽ
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);	//PA5�õ͵�ƽ�����÷���Ϊ��ת
		PWM_SetCompare2(Speed);				//PWM����Ϊ�ٶ�ֵ
	}
	else									//���򣬼����÷�ת���ٶ�ֵ
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);	//PA4�õ͵�ƽ
		GPIO_SetBits(GPIOA, GPIO_Pin_5);	//PA5�øߵ�ƽ�����÷���Ϊ��ת
		PWM_SetCompare2(-Speed);			//PWM����Ϊ�����ٶ�ֵ����Ϊ��ʱ�ٶ�ֵΪ��������PWMֻ�ܸ�����
	}
}

/**
  * ��    ����ֱ����������ٶ�2
  * ��    ����Speed Ҫ���õ��ٶȣ���Χ��-100~100
  * �� �� ֵ����
  */
void MotorB_SetSpeed(int8_t Speed)
{
	if (Speed >= 0)							//���������ת���ٶ�ֵ
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_0);	//PA0�øߵ�ƽ
		GPIO_SetBits(GPIOA, GPIO_Pin_3);	//PA3�õ͵�ƽ�����÷���Ϊ��ת
		PWM_SetCompare3(Speed);				//PWM����Ϊ�ٶ�ֵ
	}
	else									//���򣬼����÷�ת���ٶ�ֵ
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_0);	//PA0�õ͵�ƽ
		GPIO_ResetBits(GPIOA, GPIO_Pin_3);	//PA3�øߵ�ƽ�����÷���Ϊ��ת
		PWM_SetCompare3(-Speed);			//PWM����Ϊ�����ٶ�ֵ����Ϊ��ʱ�ٶ�ֵΪ��������PWMֻ�ܸ�����
	}
}
/**
  * ��    ����PDƽ��
  * ��    ����Speed Ҫ���õ��ٶȣ���Χ��-100~100
  * �� �� ֵ����
  */
void PD_SetSpeed(int8_t Speed)
{
	MotorA_SetSpeed(Speed);
	MotorB_SetSpeed(Speed);
}
