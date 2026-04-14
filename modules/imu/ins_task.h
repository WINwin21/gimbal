/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @author  annotation and modification by NeoZeng
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention INS任务的初始化不要放入实时系统!应该由application拥有实例,随后在
 *            应用层调用初始化函数.
 *
 ******************************************************************************
 */
#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
#include "BMI088driver.h"
#include "QuaternionEKF.h"

#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1

typedef struct
{
    float Gyro[3];        // 角速度 (rad/s) - 机体系三轴角速度 [X, Y, Z]
    float Accel[3];       // 加速度 (m/s²) - 机体系三轴加速度 [X, Y, Z]
    // 还需要增加角速度数据
    float Roll;           // 横滚角 (rad) - 绕X轴旋转角度
    float Pitch;          // 俯仰角 (rad) - 绕Y轴旋转角度
    float Yaw;            // 偏航角 (rad) - 绕Z轴旋转角度，范围 [-π, π]
    float YawTotalAngle;  // 偏航总角度 (degree) - 多圈累积，用于云台连续旋转控制
} attitude_t; // 最终解算得到的角度,以及yaw转动的总角度(方便多圈控制)

typedef struct
{
    float q[4]; // 四元数估计值 (无量纲) - [q0, q1, q2, q3]，归一化四元数

    float MotionAccel_b[3]; // 机体坐标运动加速度 (m/s²) - 已去除重力分量
    float MotionAccel_n[3]; // 惯性系运动加速度 (m/s²) - 已去除重力分量

    float AccelLPF; // 加速度低通滤波系数 (无量纲) - 范围 [0, 1]

    // bodyframe在绝对系的向量表示 (无量纲) - 单位向量
    float xn[3]; // 机体X轴在惯性系的方向余弦
    float yn[3]; // 机体Y轴在惯性系的方向余弦
    float zn[3]; // 机体Z轴在惯性系的方向余弦

    // 加速度在机体系和XY两轴的夹角
    // float atanxz;
    // float atanyz;

    // IMU量测值
    float Gyro[3];        // 角速度 (rad/s) - 机体系三轴角速度 [X, Y, Z]
    float Accel[3];       // 加速度 (m/s²) - 机体系三轴加速度 [X, Y, Z]
    // 位姿
    float Roll;           // 横滚角 (rad) - 绕X轴旋转角度
    float Pitch;          // 俯仰角 (rad) - 绕Y轴旋转角度
    float Yaw;            // 偏航角 (rad) - 绕Z轴旋转角度，范围 [-π, π]
    float YawTotalAngle;  // 偏航总角度 (degree) - 多圈累积

    uint8_t init;
} INS_t;

/* 用于修正安装误差的参数 */
typedef struct
{
    uint8_t flag;

    float scale[3];  // 加速度标度因子 (无量纲) - 三轴缩放系数

    float Yaw;       // 安装偏航角 (rad) - IMU相对机体系的偏航偏差
    float Pitch;     // 安装俯仰角 (rad) - IMU相对机体系的俯仰偏差
    float Roll;      // 安装横滚角 (rad) - IMU相对机体系的横滚偏差
} IMU_Param_t;

/**
 * @brief 初始化惯导解算系统
 *
 */
attitude_t *INS_Init(void);

/**
 * @brief 此函数放入实时系统中,以1kHz频率运行
 *        p.s. osDelay(1);
 *
 */
void INS_Task(void);

/**
 * @brief 四元数更新函数,即实现dq/dt=0.5Ωq
 *
 * @param q  四元数
 * @param gx
 * @param gy
 * @param gz
 * @param dt 距离上次调用的时间间隔
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);

/**
 * @brief 四元数转换成欧拉角 ZYX
 *
 * @param q
 * @param Yaw
 * @param Pitch
 * @param Roll
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);

/**
 * @brief ZYX欧拉角转换为四元数
 *
 * @param Yaw
 * @param Pitch
 * @param Roll
 * @param q
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);

/**
 * @brief 机体系到惯性系的变换函数
 *
 * @param vecBF body frame
 * @param vecEF earth frame
 * @param q
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);

/**
 * @brief 惯性系转换到机体系
 *
 * @param vecEF
 * @param vecBF
 * @param q
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif
