#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include  "dmmotor.h"
#include "math.h"

// pitch重力补偿前馈力矩（Nm），由GimbalTask每周期更新
// 物理含义：τ = K_g * cos(pitch_rad)，K_g 需根据实际云台质量/质心位置调参
// 调参方法：逐渐增大直到pitch在各角度都能稳定保持，不再下垂
#define PITCH_GRAVITY_COMP_COEF 0.9f  // 初始值，根据实际调整（典型范围0.3~2.0 Nm）
#define DEGREE_2_RAD 0.01745329f
static float pitch_gravity_feedforward = 0.0f;

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor;
static DMMotorInstance *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static gimbal_mode_e last_gimbal_mode = GIMBAL_ZERO_FORCE; // 记录上一次云台模式
static BMI088Instance *bmi088; // 云台IMU
void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 16, // 8
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 200,
            },
            .speed_PID = {
                .Kp = 100,  // 50
                .Ki = 200, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 2000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE, // IMU方向与电机驱动方向相反,翻转输出修正正反馈
        },
        .motor_type = GM6020};

    //  PITCH 轴（DM4310）：使用 MIT 固件，MIT 模式控制
    //    CAN ID = 0x002，MIT 模式支持位置、速度、电流控制
    Motor_Init_Config_s dm_pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 0x002,  // DM电机使用不同的CAN ID
            .rx_id = 0x001,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.8, // 10
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5,
                .MaxOut = 2.25f,
            },
            .speed_PID = {
                .Kp = 0.500f,  // 50
                .Ki = 0.071, // 350
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2,
                .MaxOut = 2.25,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Roll,
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
            .current_feedforward_ptr = &pitch_gravity_feedforward,  // 重力补偿前馈
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
            .feedforward_flag = CURRENT_FEEDFORWARD,  // 启用重力补偿前馈
        },
        .motor_type =MOTOR_TYPE_NONE ,//dm4310用通用类型
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&dm_pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DMMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_motor);
        DMMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        // 注意：达妙电机不需要调用DMMotorOuterLoop，闭环类型在初始化时已设置
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_motor);
        DMMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        // 注意：达妙电机不需要调用DMMotorOuterLoop，闭环类型在初始化时已设置
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }

    // 计算pitch重力补偿前馈力矩：τ_ff = K_g * cos(pitch_angle_rad)
    // feedback_reverse_flag会对最终输出取反，因此前馈符号需与速度环输出方向一致
    // 若俯仰方向抖动或补偿反向，将PITCH_GRAVITY_COMP_COEF改为负值
    pitch_gravity_feedforward = PITCH_GRAVITY_COMP_COEF * cosf(gimba_IMU_data->Roll * DEGREE_2_RAD);

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}