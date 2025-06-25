#ifndef __PID_H
#define __PID_H

/**
  * @brief PIDƽ����ƽṹ��
  * @note  ������������ز���
  */
typedef struct {
    float Kp;                   ///< ����ϵ��
    float Ki;                   ///< ����ϵ����������
    float Kd;                   ///< ΢��ϵ��
    float outPut_max;           ///< �������
    float outPut_min;           ///< �������
    float prev_Error;           ///< ��һ�ε����ֵ
    float integral;             ///< �������ۼ�ֵ��������
    float filtered_derivative;  ///< �˲����΢����
    float filtered_error;       ///< �˲�������ֵ
} PID_BalanceLoop;

/**
  * @brief ��ʼ��PID������
  * @param PD PID�ṹ��ָ��
  * @param Kp ����ϵ��
  * @param Ki ����ϵ����������
  * @param Kd ΢��ϵ��
  * @param outPut_max �������
  * @param outPut_min �������
  */
void init_PID_BalanceLoop(PID_BalanceLoop *PD, float Kp, float Ki, float Kd, 
                         float outPut_max, float outPut_min);

/**
  * @brief PID���������㺯�������������
  * @param PD PID�ṹ��ָ��
  * @param set_point �趨ֵ
  * @param measure_value ����ֵ
  * @param dt ʱ�䲽�����룩
  * @param alpha_d ΢�����˲�ϵ��(0~1)
  * @param alpha_p �������˲�ϵ��(0~1)
  * @return ���������ֵ
  */
float PID_Calculate(PID_BalanceLoop *PD, float set_point, float measure_value, 
                   float dt, float alpha_d, float alpha_p);

#endif
