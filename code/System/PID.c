#include "PID.h"

/**
  * @brief  ��ʼ��PID����������
  */
void init_PID_BalanceLoop(PID_BalanceLoop *PD, float Kp, float Ki, float Kd, 
                         float outPut_max, float outPut_min)
{
    PD->Kp = Kp;
    PD->Ki = Ki;  // ��ʼ������ϵ��
    PD->Kd = Kd;
    PD->outPut_max = outPut_max;
    PD->outPut_min = outPut_min;
    PD->prev_Error = 0;
    PD->integral = 0;           // �������ʼ��Ϊ0
    PD->filtered_derivative = 0;
    PD->filtered_error = 0;
}

/**
  * @brief  PID���������㺯��������PIDʵ�֣�
  */
float PID_Calculate(PID_BalanceLoop *PD, float set_point, float measure_value, 
                   float dt, float alpha_d, float alpha_p)
{
    /* 1. ���㵱ǰ��� */
    float error = set_point - measure_value;
    
    /* 2. �������о�ֵ�˲�������Kp� */
    PD->filtered_error = alpha_p * PD->filtered_error + (1 - alpha_p) * error;
    
    /* 3. ����������뿹���ʹ��������� */
    PD->integral += error * dt;
    // �����޷���ֹ����
    if(PD->integral > PD->outPut_max/PD->Ki) 
        PD->integral = PD->outPut_max/PD->Ki;
    else if(PD->integral < PD->outPut_min/PD->Ki) 
        PD->integral = PD->outPut_min/PD->Ki;
    
    /* 4. ����ԭʼ΢���� */
    float derivative = (error - PD->prev_Error) / dt;
    
    /* 5. ��΢������е�ͨ�˲� */
    PD->filtered_derivative = alpha_d * PD->filtered_derivative + (1 - alpha_d) * derivative;
    
    /* 6. ����PID���������+����+΢�֣� */
    float output = PD->Kp * PD->filtered_error    // ������
                 + PD->Ki * PD->integral         // �����������
                 + PD->Kd * PD->filtered_derivative; // ΢����
    
    /* 7. ���������ʷֵ */
    PD->prev_Error = error;
    
    /* 8. ����޷����� */
    output = (output > PD->outPut_max) ? PD->outPut_max : 
            ((output < PD->outPut_min) ? PD->outPut_min : output);
    
    return output;
}
