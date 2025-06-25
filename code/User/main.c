#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "Motor.h"
#include "PID.h"
#include "TIM.h"

/* PID�������� */
#define KP1 3.4f       // С�Ƕȷ�Χ������
#define KI1 0.014f      // С�Ƕȷ�Χ�����������
#define KD1 0.007f     // С�Ƕȷ�Χ΢����
#define ANGLE 6.0f     // �Ƕ��л���ֵ
#define KP2 4.8f       // ��Ƕȷ�Χ������
#define KI2 0.02f      // ��Ƕȷ�Χ�����������
#define KD2 0.005f     // ��Ƕȷ�Χ΢����

/* ������� */
#define OUTPOT_MAX 100.0f
#define OUTPUT_MIN -100.0f
#define SetPoint 0.0f

/* �˲�ϵ�� */
#define FILTER_ALPHA_DT 0.8f    // ʱ�䲽���˲�ϵ��
#define KD_FILTER_ALPHA 0.8f    // ΢�����˲�ϵ��
#define KP_FILTER_ALPHA 0.3f    // ����������˲�ϵ��

/* ȫ�ֱ��� */
float Pitch, Roll, Yaw;
PID_BalanceLoop PD_Balance;
float output_Balance = 0.0f;
uint32_t dt = 0;
float filtered_dt = 0;

/**
  * @brief  ���ֵ���ʹ�����
  */
int8_t saturate_int8(float value) {
    return (value > 100) ? 100 : ((value < -100) ? -100 : (int8_t)value);
}

int main(void) {
    /* Ӳ����ʼ��������Ki������ */
    init_PID_BalanceLoop(&PD_Balance, KP1, KI1, KD1, OUTPOT_MAX, OUTPUT_MIN);
    MPU6050_Init();
    MPU6050_DMP_Init();
    Motor_Init();
    TIM1_Init(71, 0xFFFF); 
    delay_ms(500);

    while (1) {
        Measure_Start();  // ��ʼ��ʱ
        
        /* ���ݲɼ� */
        MPU6050_DMP_Get_Data(&Pitch, &Roll, &Yaw);

        /* ƽ����ƣ��Ƕȳ�����0.5��ʱ������ */
        if (!(Pitch > -0.5 && Pitch < 0.5)) {
            /* ��������Ӧ�л�������Ki� */
            PD_Balance.Kp = (Pitch < ANGLE && Pitch > -ANGLE) ? KP1 : KP2;
            PD_Balance.Ki = (Pitch < ANGLE && Pitch > -ANGLE) ? KI1 : KI2;
            PD_Balance.Kd = (Pitch < ANGLE && Pitch > -ANGLE) ? KD1 : KD2;
            
            /* PID���㣨ע�⺯������ΪPID_Calculate�� */
            output_Balance = PID_Calculate(&PD_Balance, SetPoint, Pitch, 
                                         filtered_dt, KD_FILTER_ALPHA, KP_FILTER_ALPHA);
            
            /* ������� */
            PD_SetSpeed(saturate_int8(output_Balance));
        }
        
        /* ʱ�䴦�� */
        dt = Measure_End(72000000, 71);  // ��ȡʱ���(us)
        filtered_dt = FILTER_ALPHA_DT * filtered_dt + (1 - FILTER_ALPHA_DT) * (dt / 1000000.0f);
    }
}
