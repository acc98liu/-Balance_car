#ifndef __PID_H
#define __PID_H

/**
  * @brief PID平衡控制结构体
  * @note  新增积分项相关参数
  */
typedef struct {
    float Kp;                   ///< 比例系数
    float Ki;                   ///< 积分系数（新增）
    float Kd;                   ///< 微分系数
    float outPut_max;           ///< 输出上限
    float outPut_min;           ///< 输出下限
    float prev_Error;           ///< 上一次的误差值
    float integral;             ///< 积分项累加值（新增）
    float filtered_derivative;  ///< 滤波后的微分项
    float filtered_error;       ///< 滤波后的误差值
} PID_BalanceLoop;

/**
  * @brief 初始化PID控制器
  * @param PD PID结构体指针
  * @param Kp 比例系数
  * @param Ki 积分系数（新增）
  * @param Kd 微分系数
  * @param outPut_max 输出上限
  * @param outPut_min 输出下限
  */
void init_PID_BalanceLoop(PID_BalanceLoop *PD, float Kp, float Ki, float Kd, 
                         float outPut_max, float outPut_min);

/**
  * @brief PID控制器计算函数（新增积分项）
  * @param PD PID结构体指针
  * @param set_point 设定值
  * @param measure_value 测量值
  * @param dt 时间步长（秒）
  * @param alpha_d 微分项滤波系数(0~1)
  * @param alpha_p 比例项滤波系数(0~1)
  * @return 控制器输出值
  */
float PID_Calculate(PID_BalanceLoop *PD, float set_point, float measure_value, 
                   float dt, float alpha_d, float alpha_p);

#endif
