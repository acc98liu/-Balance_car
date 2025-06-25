#include "PID.h"

/**
  * @brief  初始化PID控制器参数
  */
void init_PID_BalanceLoop(PID_BalanceLoop *PD, float Kp, float Ki, float Kd, 
                         float outPut_max, float outPut_min)
{
    PD->Kp = Kp;
    PD->Ki = Ki;  // 初始化积分系数
    PD->Kd = Kd;
    PD->outPut_max = outPut_max;
    PD->outPut_min = outPut_min;
    PD->prev_Error = 0;
    PD->integral = 0;           // 积分项初始化为0
    PD->filtered_derivative = 0;
    PD->filtered_error = 0;
}

/**
  * @brief  PID控制器计算函数（完整PID实现）
  */
float PID_Calculate(PID_BalanceLoop *PD, float set_point, float measure_value, 
                   float dt, float alpha_d, float alpha_p)
{
    /* 1. 计算当前误差 */
    float error = set_point - measure_value;
    
    /* 2. 对误差进行均值滤波（用于Kp项） */
    PD->filtered_error = alpha_p * PD->filtered_error + (1 - alpha_p) * error;
    
    /* 3. 积分项计算与抗饱和处理（新增） */
    PD->integral += error * dt;
    // 积分限幅防止饱和
    if(PD->integral > PD->outPut_max/PD->Ki) 
        PD->integral = PD->outPut_max/PD->Ki;
    else if(PD->integral < PD->outPut_min/PD->Ki) 
        PD->integral = PD->outPut_min/PD->Ki;
    
    /* 4. 计算原始微分项 */
    float derivative = (error - PD->prev_Error) / dt;
    
    /* 5. 对微分项进行低通滤波 */
    PD->filtered_derivative = alpha_d * PD->filtered_derivative + (1 - alpha_d) * derivative;
    
    /* 6. 计算PID输出（比例+积分+微分） */
    float output = PD->Kp * PD->filtered_error    // 比例项
                 + PD->Ki * PD->integral         // 积分项（新增）
                 + PD->Kd * PD->filtered_derivative; // 微分项
    
    /* 7. 更新误差历史值 */
    PD->prev_Error = error;
    
    /* 8. 输出限幅处理 */
    output = (output > PD->outPut_max) ? PD->outPut_max : 
            ((output < PD->outPut_min) ? PD->outPut_min : output);
    
    return output;
}
