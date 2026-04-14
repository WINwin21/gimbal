#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr)
{
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}

/**
 * @brief 获取DM电机测量数据结构指针
 * @param motor DM电机实例指针
 * @return DM_Motor_Measure_s* 测量数据结构指针
 */
DM_Motor_Measure_s* DMMotorGetMeasure(DMMotorInstance *motor)
{
    if (motor == NULL) return NULL;
    return &motor->measure;
}

/**
 * @brief 获取DM电机状态
 * @param motor DM电机实例指针
 * @return Motor_Working_Type_e 电机工作状态
 */
Motor_Working_Type_e DMMotorGetStatus(DMMotorInstance *motor)
{
    if (motor == NULL) return MOTOR_STOP;
    return motor->stop_flag;
}

/**
 * @brief 设置DM电机目标位置（用于位置控制）
 * @param motor DM电机实例指针
 * @param position 目标位置（弧度）
 */
void DMMotorSetPosition(DMMotorInstance *motor, float position)
{
    if (motor == NULL) return;
    motor->pid_ref = position;
}

/**
 * @brief 设置DM电机目标电流（用于电流控制）
 * @param motor DM电机实例指针
 * @param current 目标电流（A）
 */
void DMMotorSetCurrent(DMMotorInstance *motor, float current)
{
    if (motor == NULL) return;
    motor->pid_ref = current;
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    DMMotorCaliEncoder(motor);
    DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}


// DM电机控制任务，实现位置和速度环PID控制（已去掉电流环）
void DMMotorTask(void const *argument)
{
    float pid_ref, set, pid_measure;
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    DM_Motor_Measure_s *measure = &motor->measure;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    DMMotor_Send_s motor_send_mailbox;

    while (1)
    {
        // 获取PID设定值
        pid_ref = motor->pid_ref;

        // 如果电机停止，直接输出零力矩
        if (motor->stop_flag == MOTOR_STOP)
        {
            motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
            motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
            motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
            motor_send_mailbox.Kp = 0;
            motor_send_mailbox.Kd = 0;
        }
        else
        {
            // 计算位置环PID（如果启用且为外环）
            if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
            {
                if (setting->angle_feedback_source == OTHER_FEED)
                    pid_measure = *motor->other_angle_feedback_ptr;
                else
                    pid_measure = measure->position; // 使用电机自身位置反馈

                // 位置环PID计算
                pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
            }

            // 计算速度环PID（如果启用且外环为速度或位置）
            if ((setting->close_loop_type & SPEED_LOOP) && (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
            {
                if (setting->speed_feedback_source == OTHER_FEED)
                    pid_measure = *motor->other_speed_feedback_ptr;
                else
                    pid_measure = measure->velocity; // 使用电机自身速度反馈

                // 速度环PID计算
                pid_ref = PIDCalculate(&motor->speed_PID, pid_measure, pid_ref);
            }

            // 注意：已去掉电流环控制，直接使用速度环输出作为力矩指令

            // 力矩前馈（用于重力补偿等）
            if ((motor->motor_settings.feedforward_flag & CURRENT_FEEDFORWARD) && motor->current_feedforward_ptr != NULL)
                pid_ref += *motor->current_feedforward_ptr;

            // 应用反馈反向标志
            if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
                pid_ref *= -1;

            // 应用电机反向标志
            if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                pid_ref *= -1;

            // 限制输出范围
            LIMIT_MIN_MAX(pid_ref, DM_T_MIN, DM_T_MAX);

            // 设置发送数据
            // 对于MIT模式，position_des和velocity_des应该是期望值，而不是当前测量值
            // 这里我们使用PID计算后的结果作为期望值
            if (setting->outer_loop_type == ANGLE_LOOP) {
                // 位置控制模式：position_des为目标位置，velocity_des为PID输出的速度期望
                motor_send_mailbox.position_des = float_to_uint(motor->pid_ref, DM_P_MIN, DM_P_MAX, 16);
                motor_send_mailbox.velocity_des = float_to_uint(pid_ref, DM_V_MIN, DM_V_MAX, 12);
            } else if (setting->outer_loop_type == SPEED_LOOP) {
                // 速度控制模式：velocity_des为目标速度
                motor_send_mailbox.position_des = float_to_uint(measure->position, DM_P_MIN, DM_P_MAX, 16);
                motor_send_mailbox.velocity_des = float_to_uint(motor->pid_ref, DM_V_MIN, DM_V_MAX, 12);
            } else {
                // 电流控制模式：torque_des为目标电流
                motor_send_mailbox.position_des = float_to_uint(measure->position, DM_P_MIN, DM_P_MAX, 16);
                motor_send_mailbox.velocity_des = float_to_uint(measure->velocity, DM_V_MIN, DM_V_MAX, 12);
            }

            motor_send_mailbox.torque_des = float_to_uint(pid_ref, DM_T_MIN, DM_T_MAX, 12);
            motor_send_mailbox.Kp = 0; // DM电机MIT模式不需要Kp/Kd
            motor_send_mailbox.Kd = 0;
        }

        // 组装CAN发送数据
        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

        // 发送CAN数据
        CANTransmit(motor->motor_can_instace, 1);

        osDelay(2);
    }
}
void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}