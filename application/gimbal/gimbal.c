#include "gimbal.h"
#include "dji_motor.h"
#include "robot_def.h"
#include "message_center.h"
#include "general_def.h"
#include <math.h>

/*
 * 正式控制链（两轴均为 GM6020 电流模式）：
 * 命令角度(°) -> 驱动按 motor_reverse_flag 取负 -> 位置 PID
 * -> 目标角速度(rad/s) -> 速度 PID -> 电流原始指令。
 * Pitch 在速度 PID 后叠加重力前馈；最终总指令由驱动限制在 ±16384。
 * IMU 安装转换及 Yaw/Pitch 角速度低通在 modules/imu/ins_task.c 中完成。
 * 本文件只连接目标、反馈和前馈；实际 PID 运算和 CAN 打包在 dji_motor.c。
 * 主控只开启位置环和速度环，GM6020 内部自行执行电流控制。
 */
static attitude_t *gimba_IMU_data;                                                              // 指向 INS 的实时姿态；角度为°，Gyro 为 rad/s
static DJIMotorInstance *yaw_motor, *pitch_motor;                                               // 水平轴、俯仰轴电机实例

static Publisher_t *gimbal_pub;                                                                 // 发布给命令层的反馈通道：gimbal_feed
static Subscriber_t *gimbal_sub;                                                                // 接收命令层目标的通道：gimbal_cmd
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;                                                       // robot_cmd.c 发来的模式与两轴目标角度
static Gimbal_Upload_Data_s gimbal_feedback_data;                                               // 发回 robot_cmd.c 的 IMU 和 Yaw 编码器信息

// Pitch 重力补偿：保留已调好的固定电流偏置，不是随角度变化的重力模型。
// 单位为 GM6020 电流原始指令；符号与最终 CAN 指令一致。
// volatile 便于调试器在线改值；重启后仍恢复源码初值，负方向为当前机构的抬升方向。
static volatile float pitch_gravity_ff_target = -4000.0f;                                       // 希望施加的固定补偿
static float pitch_gravity_ff = 0.0f;                                                           // 实际供驱动读取的前馈；无力时为0
static const float pitch_gravity_ff_limit = 4000.0f;                                            // 只限制前馈自身幅值，不是PID限幅

// 向其他模块提供同一份实时 IMU 数据，不在此复制或重新解算姿态。
attitude_t *get_IMU_data(void)
{
    return gimba_IMU_data;
}

/* RobotInit() 调用一次：先取得 IMU，再创建电机及命令/反馈通道。 */
void GimbalInit(void)
{
    gimba_IMU_data = INS_Init();                                                                // 后面取成员地址前，必须先取得有效指针

    /* 两轴的公共参数含义：
     * 位置环：角度误差(°) -> 目标角速度(rad/s)；速度环：速度误差(rad/s) -> 电流指令。
     * P 是当前误差的比例修正；I 累积持续误差；D 用于误差变化的修正（当前均为0）。
     * 积分采用 Ki*Err*dt；IntegralLimit 限 Iout，MaxOut 限 P+I+D 的总输出。
     * Improve 保留原配置。当前 controller.c 仅调用了其中的积分限幅/抗饱和处理；
     * 微分先行调用仍被注释，不能仅凭 PID_Derivative_On_Measurement 标志认定已启用。
     */

    // Yaw：CAN2 / ID4；位置反馈为累计角度，Z轴速度反馈使用30Hz一阶低通。
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,                                                               // 云台两轴都挂 CAN2
            .tx_id = 4,                                                                         // 电机物理ID；驱动换算为控制帧0x1FE的字节6/7，反馈0x208
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.18f,                                                                    // 每1°角度误差产生0.18rad/s的比例速度目标
                .Ki = 0,                                                                        // 位置环不累积积分
                .Kd = 0,                                                                        // 位置环不使用微分
                .DeadBand = 0,                                                                  // 保留原值；公共PID的死区判断当前未启用
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 200,                                                           // Iout限幅，单位rad/s；当前Ki=0，不新增积分
                .MaxOut = 6.2831853f,                                                           // rad/s，±60 rpm
            },
            .speed_PID = {
                .Kp = 8000.0f,                                                                  // 角速度误差的比例修正
                .Ki = 140000.0f,                                                                // 速度误差经dt累积，修正持续的速度偏差
                .Kd = 0,                                                                        // 速度环不使用微分
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000,                                                         // Iout限制为±10000个电流指令单位
                .MaxOut = GM6020_CURRENT_MAX_CMD,                                               // PID总输出±16384，对应给定±3A
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,                         // IMU累计角度(°)，跨圈连续
            .other_speed_feedback_ptr = INS_GetYawGyroFeedback(),                               // 30Hz低通后的Z轴角速度(rad/s)，保留Gyro[2]方向
        },
        .controller_setting_init_config = {
            .gm6020_control_mode = GM6020_CURRENT_CONTROL,                                      // 使用GM6020电流控制帧
            .angle_feedback_source = OTHER_FEED,                                                // 读取上面的外部IMU指针，而非电机编码器
            .speed_feedback_source = OTHER_FEED,
            .feedforward_flag = FEEDFORWARD_NONE,                                               // Yaw没有启用额外前馈
            .outer_loop_type = ANGLE_LOOP,                                                      // SetRef传入角度目标，先算位置环
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,                                         // 位置环输出接到速度环输入
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,                                      // 驱动在PID前对命令目标取负
        },
        .motor_type = GM6020,
    };

    // Pitch：CAN2 / ID2；速度反馈使用 20 Hz 一阶低通，原始 INS.Gyro 不变。
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2,                                                                         // 电机物理ID；控制帧0x1FE的字节2/3，反馈0x206（字节从0计数）
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.35f,                                                                    // 每1°角度误差产生0.35rad/s的比例速度目标
                .Ki = 0.5f,                                                                     // 对持续角度偏差累积修正，输出单位仍是rad/s
                .Kd = 0,                                                                        // 位置环不使用微分
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 0.5f,                                                          // 位置积分输出：rad/s
                .MaxOut = 6.0f,                                                                 // 目标角速度：rad/s
            },
            .speed_PID = {
                .Kp = 3500,                                                                     // 对比目标速度和滤波后的实际速度，给出比例电流修正
                .Ki = 90000,                                                                    // 累积速度误差，配合固定前馈补足负载变化
                .Kd = 0,                                                                        // 速度环不使用微分
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000,                                                         // 速度积分输出±10000个电流指令单位
                .MaxOut = GM6020_CURRENT_MAX_CMD,                                               // 此处限PID输出，驱动还会限叠加前馈后的总值
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,                                 // IMU俯仰角(°)，当前抬头为负
            // 返回滤波结果的地址；INS任务每次采样更新，当前参数为20Hz，不在这里重复滤波。
            .other_speed_feedback_ptr = INS_GetPitchGyroFeedback(),                             // rad/s，保留Gyro[1]的方向
            .current_feedforward_ptr = &pitch_gravity_ff,                                       // 指向实际前馈，驱动每次控制时读取
        },
        .controller_setting_init_config = {
            .gm6020_control_mode = GM6020_CURRENT_CONTROL,
            .angle_feedback_source = OTHER_FEED,                                                // 位置环使用IMU Pitch
            .speed_feedback_source = OTHER_FEED,                                                // 速度环使用上面的低通反馈
            .feedforward_flag = CURRENT_FEEDFORWARD,                                            // 在速度PID之后加固定电流偏置
            .outer_loop_type = ANGLE_LOOP,                                                      // 从位置环开始计算
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,                                         // 正式启用位置、速度串级
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,                                      // 命令取负，前馈不随命令再次反向
        },
        .motor_type = GM6020,
    };

    yaw_motor = DJIMotorInit(&yaw_config);                                                      // 驱动复制配置、初始化各PID，并注册CAN反馈回调；局部配置退出函数后不再使用。
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));                      // 这里注册通道，真正收发在GimbalTask中进行，名称要和robot_cmd.c一致。
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* enabled=1：建立补偿；enabled=0：清除补偿，不负责设置电机停止标志。 */
static void PitchGravityFeedforwardUpdate(uint8_t enabled)
{
    float target = pitch_gravity_ff_target;                                                     // 本次使用的目标快照，避免在线改值时重复读取
    if (!enabled || !isfinite(target) ||
        !isfinite(gimba_IMU_data->Pitch) || !isfinite(gimba_IMU_data->Gyro[1]))
    {
        pitch_gravity_ff = 0.0f;                                                                // isfinite排除NaN和无穷值；这里仅关闭前馈，不替代整车急停或IMU异常处理。
        return;
    }

    LIMIT_MIN_MAX(target, -pitch_gravity_ff_limit, pitch_gravity_ff_limit);                     // 限前馈自身幅值
    pitch_gravity_ff = target;                                                                  // 使能后直接建立固定偏置，退出立即清零，沿用调参完成时的行为。
}

/* GYRO/FREE模式共用：使能电机，选择IMU反馈，将两轴角度目标交给电机驱动。 */
static void GimbalApplyReferences(void)
{
    DJIMotorEnable(yaw_motor);
    DJIMotorEnable(pitch_motor);
    DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);                                      // 这里只选反馈来源，不重新初始化PID，也不改变已调好的串级配置。
    DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
    DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
    DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);

    // SetRef只存目标，不在这里执行PID。驱动再将命令取负得到angle_PID.Ref。
    DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);                                             // 例如Pitch命令+10° -> PID目标-10° -> 实际抬头；不要在这里重复取负。
    DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
}

/* RobotTask周期调用：接收命令 -> 按模式设置前馈和电机 -> 发布当前反馈。 */
void GimbalTask(void)
{
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);                                                // 获取robot_cmd.c发布的最新命令

    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_ZERO_FORCE:
        PitchGravityFeedforwardUpdate(0);                                                       // 先清前馈，再标记停止；电机任务检查停止标志后复位PID并发送零电流。
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        break;

    // 两种工作模式当前都使用 IMU 反馈。
    case GIMBAL_GYRO_MODE:
    case GIMBAL_FREE_MODE:
        PitchGravityFeedforwardUpdate(1);                                                       // 先准备实际前馈，再使能和更新目标；不在这里直接发送CAN。
        GimbalApplyReferences();
        break;

    default:
        PitchGravityFeedforwardUpdate(0);                                                       // 沿用原有未知模式处理：清除 Pitch 前馈，电机使能/目标保持原状态。
        break;
    }

    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;                                     // 复制IMU，供命令层使用
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;  // 编码器单圈角度用于底盘与云台相对角度计算；Yaw位置PID仍使用IMU累计角度。
    PubPushMessage(gimbal_pub, &gimbal_feedback_data);                                          // 无力模式也回传状态，不停止反馈
}
