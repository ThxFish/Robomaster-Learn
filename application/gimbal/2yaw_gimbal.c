#include "gimbal.h"
#include "sentry_def.h"
#include "dji_motor.h"
#include "xm_motor.h"
// #include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
// #include "bmi088.h"
#include "ins_task.h"
#include "bsp_dwt.h"

static attitude_t *gimbal_IMU_data;
static float GyroDeg[3];
static DJIMotorInstance *yaw_motor, *pitch_motor;
static XMMotorInstance *xm_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

// 云台初始化：负责分配电机实例、填写PID、绑定IMU反馈数据指针，并建立PUB/SUB连接。
void GimbalInit()
{
    // 获取由 INStask(惯性导航解算任务) 计算出的最新设备空间姿态指针
    gimbal_IMU_data = INS_Init();
    // YAW 轴(大疆GM6020电机) 初始化配置
    // 这里体现了高度的模块化：直接把“IMU的Yaw角度指针”和“Z轴陀螺仪角速度指针”
    // 塞给了DJI_MOTOR模块的PID。
    // 这样底层一计算位置环和速度环，自然读到的就是外界陀螺仪的真实空间姿态，而不是电机转子的相对度数。
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hfdcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 30,
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 1000,
                .MaxOut = 1000,
            },
            .speed_PID = {
                .Kp = 50,
                .Ki = 200,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            // .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
            .other_speed_feedback_ptr = &GyroDeg[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            // .angle_feedback_source = MOTOR_FEED,
            // .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};

    // PITCH 轴 (小米CyberGear电机)
    // 根据小米电机的特性，它往往支持MIT模式(力矩/位置/速度混合阻抗控制)，此处使用Kp和Kd。
    Motor_Init_Config_s xm_config = {
        .can_init_config = {
            .can_handle = &hfdcan3,
            .rx_id = 1, // 电机ID (1~127)
        },
        .controller_param_init_config = {
            .angle_PID = {.Kp = 20.0f}, // MIT模式位置Kp
            .speed_PID = {.Kd = 1.0f},  // MIT模式阻尼Kd
        },
        .controller_setting_init_config = {
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    // DWT_Delay(1); // 小米电机初始化需要1s时间
    xm_motor = XMMotorInit(&xm_config);
    XMMotorControlInit();

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心循环任务。该任务在 RTOS 中的 `sentry.c` 等文件中周期调度 */
void GimbalTask()
{
    // 非阻塞式获取中枢指令：尝试从队列拿到最新的控制命令(包括期望Pitch、Yaw角度以及工作模式)。
    // 若没拿到， `gimbal_cmd_recv` 的值将维持上一次的缓存。
    // 后续可增加超时断开保护机制
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // 对 Pitch 轴 (小米电机)的独立运动算法控制
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_IMU:
    case GIMBAL_LOCK:
        // 取出控制台下发的期望 Pitch 角度
        float target_imu_angle = gimbal_cmd_recv.pitch;
        // 计算与实际 IMU 姿态的误差
        float imu_error = target_imu_angle - gimbal_IMU_data->Pitch;

        // 算出小米电机期望达到的相对目标物理位置(需要把 IMU角度差 叠加到 电机当前的转子位置 上)
        float mit_target_angle = xm_motor->measure.position + 1.0 * imu_error * DEGREE_2_RAD;

        // 速度补偿：为了抵抗陀螺仪读出的角速度干扰，这里向该方向产生反作用虚拟速度，减小动态超调
        float mit_target_speed = xm_motor->measure.velocity - 1.0 * gimbal_IMU_data->Gyro[0];

        // 对算出的小米电机实际下发位置做硬限幅保护，防止云台磕到底盘或限位架
        if (mit_target_angle > PITCH_MAX_RAD)
        {
            mit_target_angle = PITCH_MAX_RAD;
        }
        else if (mit_target_angle < PITCH_MIN_RAD)
        {
            mit_target_angle = PITCH_MIN_RAD;
        }

        if (mit_target_speed > PITCH_MAX_VEL)
        {
            mit_target_speed = PITCH_MAX_VEL;
        }
        else if (mit_target_speed < PITCH_MIN_VEL)
        {
            mit_target_speed = PITCH_MIN_VEL;
        }

        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
        XMMotorEnable(xm_motor);
        // Pitch 轴力矩控制输出给小米驱动层
        XMMotorSetRef(xm_motor, mit_target_angle, mit_target_speed, 0);
        break;
    default:
        break;
    }

    // 对 Yaw 轴 (大疆GM6020大电机)的特化算法控制
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_IMU:
        // 在陀螺仪增稳模式下：强制将大疆电机的闭环反馈源头切为 IMU陀螺仪，实现地磁系中“钉死”一个角度。
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // 将摇杆要求的中枢控制Yaw给下去
        break;
    case GIMBAL_LOCK:
        // 在底盘跟随锁定模式下：切回电机自己的编码器读数。
        // 这时它的目标就成了“把自身的转子刻度掰回到车头正前方对齐”。也就是所谓的“车云协同解耦控制”。
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorSetRef(yaw_motor, YAW_CHASSIS_ALIGN_DEG);
        break;
    default:
        break;
    }

    // 最后，作为 Pub-Sub 中的“勤勤恳恳”节点，将自己此时此刻的真实云台方位向全网散播（尤其发给中枢和底盘任务用作正运动解算），完成逻辑闭环。
    gimbal_feedback_data.yaw_deg = yaw_motor->measure.total_angle - YAW_CHASSIS_ALIGN_DEG;
    gimbal_feedback_data.yaw_speed = yaw_motor->measure.speed_aps;

    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
    // XMMotorSetRef(xm_motor, gimbal_cmd_recv.pitch, 0.0f, 0.0f);

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    // switch (gimbal_cmd_recv.gimbal_mode)
    // {
    // // 停止
    // case GIMBAL_ZERO_FORCE:
    //     DJIMotorStop(yaw_motor);
    //     DJIMotorStop(pitch_motor);
    //     break;
    // // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    // case GIMBAL_GYRO_MODE: // 后续只保留此模式
    //     DJIMotorEnable(yaw_motor);
    //     DJIMotorEnable(pitch_motor);
    //     DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
    //     DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
    //     DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
    //     break;
    // // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    // case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
    //     DJIMotorEnable(yaw_motor);
    //     DJIMotorEnable(pitch_motor);
    //     DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
    //     DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
    //     DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
    //     break;
    // default:
    //     break;
    // }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    // gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    // gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}

void GimbalDataTask()
{
    GyroDeg[0] = gimbal_IMU_data->Gyro[0] * RAD_2_DEGREE;
    GyroDeg[1] = gimbal_IMU_data->Gyro[1] * RAD_2_DEGREE;
    GyroDeg[2] = gimbal_IMU_data->Gyro[2] * RAD_2_DEGREE;
}