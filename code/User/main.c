#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "Motor.h"
#include "PID.h"
#include "TIM.h"

/* PID参数定义 */
#define KP1 3.4f       // 小角度范围比例项
#define KI1 0.014f      // 小角度范围积分项（新增）
#define KD1 0.007f     // 小角度范围微分项
#define ANGLE 6.0f     // 角度切换阈值
#define KP2 4.8f       // 大角度范围比例项
#define KI2 0.02f      // 大角度范围积分项（新增）
#define KD2 0.005f     // 大角度范围微分项

/* 输出限制 */
#define OUTPOT_MAX 100.0f
#define OUTPUT_MIN -100.0f
#define SetPoint 0.0f

/* 滤波系数 */
#define FILTER_ALPHA_DT 0.8f    // 时间步长滤波系数
#define KD_FILTER_ALPHA 0.8f    // 微分项滤波系数
#define KP_FILTER_ALPHA 0.3f    // 比例项误差滤波系数

/* 全局变量 */
float Pitch, Roll, Yaw;
PID_BalanceLoop PD_Balance;
float output_Balance = 0.0f;
uint32_t dt = 0;
float filtered_dt = 0;

/**
  * @brief  输出值饱和处理函数
  */
int8_t saturate_int8(float value) {
    return (value > 100) ? 100 : ((value < -100) ? -100 : (int8_t)value);
}

int main(void) {
    /* 硬件初始化（新增Ki参数） */
    init_PID_BalanceLoop(&PD_Balance, KP1, KI1, KD1, OUTPOT_MAX, OUTPUT_MIN);
    MPU6050_Init();
    MPU6050_DMP_Init();
    Motor_Init();
    TIM1_Init(71, 0xFFFF); 
    delay_ms(500);

    while (1) {
        Measure_Start();  // 开始计时
        
        /* 数据采集 */
        MPU6050_DMP_Get_Data(&Pitch, &Roll, &Yaw);

        /* 平衡控制（角度超过±0.5度时启动） */
        if (!(Pitch > -0.5 && Pitch < 0.5)) {
            /* 参数自适应切换（新增Ki项） */
            PD_Balance.Kp = (Pitch < ANGLE && Pitch > -ANGLE) ? KP1 : KP2;
            PD_Balance.Ki = (Pitch < ANGLE && Pitch > -ANGLE) ? KI1 : KI2;
            PD_Balance.Kd = (Pitch < ANGLE && Pitch > -ANGLE) ? KD1 : KD2;
            
            /* PID计算（注意函数名改为PID_Calculate） */
            output_Balance = PID_Calculate(&PD_Balance, SetPoint, Pitch, 
                                         filtered_dt, KD_FILTER_ALPHA, KP_FILTER_ALPHA);
            
            /* 电机控制 */
            PD_SetSpeed(saturate_int8(output_Balance));
        }
        
        /* 时间处理 */
        dt = Measure_End(72000000, 71);  // 获取时间差(us)
        filtered_dt = FILTER_ALPHA_DT * filtered_dt + (1 - FILTER_ALPHA_DT) * (dt / 1000000.0f);
    }
}
