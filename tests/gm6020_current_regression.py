"""Run production DJI grouping/control functions as native C with CAN/PID stubs.

Usage: python tests/gm6020_current_regression.py --cc path/to/gcc
Checks protocol routing/packing/limits/stop, production Pitch direct feedforward
and the independent Yaw/Pitch gyro low-pass response and feedback routing,
not electrical behavior, gravity balance or RTOS preemption.
"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "modules/motor/DJImotor/dji_motor.c").read_text(encoding="utf-8")
header = (root / "modules/motor/DJImotor/dji_motor.h").read_text(encoding="utf-8")
definitions = (root / "modules/motor/motor_def.h").read_text(encoding="utf-8")
gimbal_source = (root / "application/gimbal/gimbal.c").read_text(encoding="utf-8")
robot_def = (root / "application/robot_def.h").read_text(encoding="utf-8")
ins_source = (root / "modules/imu/ins_task.c").read_text(encoding="utf-8")
pitch_start = gimbal_source.index("Motor_Init_Config_s pitch_config = {")
pitch_config = gimbal_source[pitch_start:gimbal_source.index("\n    };", pitch_start) + len("\n    };")]
yaw_start = gimbal_source.index("Motor_Init_Config_s yaw_config = {")
yaw_config = gimbal_source[yaw_start:gimbal_source.index("\n    };", yaw_start) + len("\n    };")]
gravity_globals = "\n".join(re.findall(
    r"^static (?:volatile |const )?float pitch_gravity_ff(?:_target|_limit)?\s*=[^;]+;",
    gimbal_source, re.M))
filter_globals = ins_source[ins_source.index("// Pitch速度反馈低通"):
                            ins_source.index("static void IMU_Param_Correction")]


def function(name, text=source):
    m = re.search(r"(?:static\s+)?(?:void\s+|float\s*\*\s*)" + name + r"\([^)]*\)\s*\{", text)
    assert m, name
    depth = 1
    for end in range(m.end(), len(text)):
        depth += (text[end] == "{") - (text[end] == "}")
        if depth == 0:
            return text[m.start():end + 1]
    raise AssertionError(name)


def typedef(name):
    end = robot_def.index("} " + name + ";") + len("} " + name + ";")
    return robot_def[robot_def.rfind("typedef", 0, end):end]


preamble = r"""
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
typedef struct { int bus; } CAN_HandleTypeDef;
static CAN_HandleTypeDef hcan1 = {1}, hcan2 = {2};
typedef struct { unsigned StdId, IDE, RTR, DLC; } TestCANHeader;
typedef struct CANInstance {
    CAN_HandleTypeDef *can_handle;
    TestCANHeader txconf;
    uint8_t tx_buff[8];
    unsigned tx_id, rx_id;
} CANInstance;
typedef struct { CAN_HandleTypeDef *can_handle; unsigned tx_id, rx_id; } CAN_Init_Config_s;
typedef struct { float Ref, Measure, Output, Iout; } PIDInstance;
typedef struct { float Kp, Ki, Kd, IntegralLimit, MaxOut, DeadBand; unsigned Improve; } PID_Init_Config_s;
#define PID_Integral_Limit 1
#define PID_Derivative_On_Measurement 8
typedef struct { int unused; } DaemonInstance;
#define CAN_ID_STD 0
#define CAN_RTR_DATA 0
static void test_log_error(const char *format, ...) { (void)format; abort(); }
#define LOGERROR(...) test_log_error(__VA_ARGS__)
"""
config_defs = definitions[definitions.index("#define LIMIT_MIN_MAX"):definitions.index("#endif")]
motor_types = header[header.index("typedef struct"):header.index("/**\n * @brief 调用")]
macros = "\n".join(re.findall(r"^#define (?:DJI_MOTOR_CNT|GM6020_CURRENT_MAX_CMD)\s+[^\n]+", header, re.M))
globals_code = source[source.index("#define DJI_SENDER_GROUP_COUNT"):source.index("static void MotorSenderGrouping")]
stubs = r"""
static CANInstance sent[16];
static unsigned sent_count;
static int pid_calls;
struct TestAttitude { float Pitch, Gyro[3], YawTotalAngle; };
typedef struct TestAttitude attitude_t;
static struct TestAttitude imu;
static struct TestAttitude *gimba_IMU_data = &imu;
static void CANTransmit(CANInstance *can, int timeout) {
    (void)timeout;
    assert(sent_count < 16);
    sent[sent_count++] = *can;
}
static void PIDReset(PIDInstance *pid) { memset(pid, 0, sizeof(*pid)); }
static float PIDCalculate(PIDInstance *pid, float measure, float ref) {
    ++pid_calls;
    pid->Measure = measure;
    pid->Ref = ref;
    return pid->Output;
}
"""
task_stubs = r"""
typedef struct { int unused; } Publisher_t;
typedef struct { int unused; } Subscriber_t;
static Publisher_t *gimbal_pub;
static Subscriber_t *gimbal_sub;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv, next_cmd;
static Gimbal_Upload_Data_s gimbal_feedback_data, published;
static DJIMotorInstance yaw_instance, pitch_instance;
static DJIMotorInstance *yaw_motor = &yaw_instance, *pitch_motor = &pitch_instance;
static unsigned publications;
static void SubGetMessage(Subscriber_t *sub, Gimbal_Ctrl_Cmd_s *cmd) {
    (void)sub;
    *cmd = next_cmd;
}
static void PubPushMessage(Publisher_t *pub, const Gimbal_Upload_Data_s *data) {
    (void)pub;
    published = *data;
    ++publications;
}
"""
checks = r"""
static void check_gimbal_task(void) {
    imu = (attitude_t){.Pitch=8, .Gyro={0, .2f, -.1f}, .YawTotalAngle=725};
    yaw_instance.measure.angle_single_round = 120;
    const gimbal_mode_e modes[] = {GIMBAL_GYRO_MODE, GIMBAL_FREE_MODE};
    for (unsigned i=0; i<2; ++i) {
        next_cmd = (Gimbal_Ctrl_Cmd_s){.gimbal_mode=modes[i], .yaw=31, .pitch=-7};
        yaw_instance.stop_flag = pitch_instance.stop_flag = MOTOR_STOP;
        GimbalTask();
        assert(yaw_instance.stop_flag == MOTOR_ENALBED && pitch_instance.stop_flag == MOTOR_ENALBED);
        // Task passes command signs unchanged; direction reversal remains in the driver.
        assert(yaw_instance.motor_controller.pid_ref == 31 && pitch_instance.motor_controller.pid_ref == -7);
        assert(yaw_instance.motor_settings.angle_feedback_source == OTHER_FEED);
        assert(yaw_instance.motor_settings.speed_feedback_source == OTHER_FEED);
        assert(pitch_instance.motor_settings.angle_feedback_source == OTHER_FEED);
        assert(pitch_instance.motor_settings.speed_feedback_source == OTHER_FEED);
        assert(pitch_gravity_ff == pitch_gravity_ff_target);
        assert(published.gimbal_imu_data.Pitch == 8 && published.gimbal_imu_data.YawTotalAngle == 725);
        assert(published.yaw_motor_single_round_angle == 120);
        next_cmd.gimbal_mode = GIMBAL_ZERO_FORCE;
        GimbalTask();
        assert(yaw_instance.stop_flag == MOTOR_STOP && pitch_instance.stop_flag == MOTOR_STOP);
        assert(pitch_gravity_ff == 0);
    }
    // Mode re-entry immediately restores the fixed feedforward and latest commands.
    next_cmd = (Gimbal_Ctrl_Cmd_s){.gimbal_mode=GIMBAL_GYRO_MODE, .yaw=-12, .pitch=3};
    GimbalTask();
    assert(pitch_gravity_ff == pitch_gravity_ff_target);
    assert(yaw_instance.motor_controller.pid_ref == -12 && pitch_instance.motor_controller.pid_ref == 3);
    next_cmd.gimbal_mode = (gimbal_mode_e)99;
    GimbalTask();
    assert(pitch_gravity_ff == 0); // preserve existing unknown-mode behavior
    assert(yaw_instance.motor_controller.pid_ref == -12 && pitch_instance.motor_controller.pid_ref == 3);
    assert(publications == 6); // feedback still publishes in zero-force and unknown modes
}

static float filter_gain(float hz) {
    pitch_gyro_filter_initialized = 0;
    double input_energy = 0, output_energy = 0;
    for (unsigned i=0; i<2000; ++i) {
        float sample = sinf(6.28318530718f * hz * (float)i * .001f);
        PitchGyroFilterUpdate(sample, .001f);
        if (i >= 1000) {
            input_energy += sample * sample;
            output_energy += pitch_gyro_filtered * pitch_gyro_filtered;
        }
    }
    return (float)sqrt(output_energy / input_energy);
}
static void check_pitch_gyro_filter(float cutoff_hz, float step_min, float step_max,
                                    float high_min, float high_max) {
    pitch_gyro_filter_initialized = 0;
    PitchGyroFilterUpdate(.4f, .001f);
    assert(pitch_gyro_filtered == .4f); // first sample has no false zero-speed ramp
    PitchGyroFilterUpdate(1.0f, .001f);
    assert(pitch_gyro_filtered > step_min && pitch_gyro_filtered < step_max);
    float held = pitch_gyro_filtered;
    PitchGyroFilterUpdate(-1.0f, 0);
    PitchGyroFilterUpdate(-1.0f, -.001f);
    assert(pitch_gyro_filtered == held);
    for (unsigned i=0; i<150; ++i)
        PitchGyroFilterUpdate(-.8f, i%2 ? .0015f : .0005f);
    assert(fabsf(pitch_gyro_filtered + .8f) < 1e-5f); // DC/sign retained with timing jitter
    PitchGyroFilterUpdate(-.3f, .5f);
    assert(pitch_gyro_filtered == -.3f); // long gap cannot leave stale speed history
    PitchGyroFilterUpdate(.2f, NAN);
    assert(pitch_gyro_filtered == .2f);
    const float invalid[] = {NAN, INFINITY, -INFINITY};
    for (unsigned i=0; i<3; ++i) {
        PitchGyroFilterUpdate(invalid[i], .001f);
        assert(!isfinite(pitch_gyro_filtered) && !pitch_gyro_filter_initialized);
        PitchGyroFilterUpdate(-.2f, .001f);
        assert(pitch_gyro_filtered == -.2f && pitch_gyro_filter_initialized);
    }
    float low = filter_gain(2), cutoff = filter_gain(cutoff_hz), high = filter_gain(100);
    assert(low > .95f && low < 1.01f); // preserve slow physical motion
    assert(cutoff > .64f && cutoff < .73f);
    assert(high > high_min && high < high_max); // suppress high-frequency sensor/vibration input
    assert(INS_GetPitchGyroFeedback() == &pitch_gyro_filtered);
}
static void check_gravity_direct(void) {
    const float configured_target = pitch_gravity_ff_target;
    pitch_gravity_ff_target = -2500; // fixture independent of the current tuning value
    imu.Pitch = 28;
    PitchGravityFeedforwardUpdate(0);
    assert(pitch_gravity_ff == 0);
    PitchGravityFeedforwardUpdate(1);
    assert(pitch_gravity_ff == -2500); // first enabled update applies full offset
    PitchGravityFeedforwardUpdate(1);
    assert(pitch_gravity_ff == -2500); // subsequent calls hold the configured value
    PitchGravityFeedforwardUpdate(0);
    assert(pitch_gravity_ff == 0);
    PitchGravityFeedforwardUpdate(1);
    assert(pitch_gravity_ff == -2500); // re-entry also applies full offset immediately

    pitch_gravity_ff_target = -1e8f;
    PitchGravityFeedforwardUpdate(1);
    assert(fabsf(pitch_gravity_ff + pitch_gravity_ff_limit) < .1f);
    pitch_gravity_ff_target = 1e8f;
    PitchGravityFeedforwardUpdate(1);
    assert(fabsf(pitch_gravity_ff - pitch_gravity_ff_limit) < .1f);
    const float invalid[] = {NAN, INFINITY, -INFINITY};
    for (unsigned i=0; i<3; ++i) {
        pitch_gravity_ff_target = invalid[i];
        PitchGravityFeedforwardUpdate(1);
        assert(pitch_gravity_ff == 0);
    }
    pitch_gravity_ff_target = -2500;
    imu.Pitch = NAN;
    PitchGravityFeedforwardUpdate(1);
    assert(pitch_gravity_ff == 0);
    imu.Pitch = 28;
    imu.Gyro[1] = INFINITY;
    PitchGravityFeedforwardUpdate(1);
    assert(pitch_gravity_ff == 0);
    imu.Gyro[1] = 0;
    PitchGravityFeedforwardUpdate(1);
    assert(pitch_gravity_ff == -2500); // recover on the next valid update
    PitchGravityFeedforwardUpdate(0);
    pitch_gravity_ff_target = configured_target;
}
static void clear(void) {
    idx = 0; sent_count = 0; pid_calls = 0;
    memset(dji_motor_instance, 0, sizeof(dji_motor_instance));
    memset(sender_enable_flag, 0, sizeof(sender_enable_flag));
    for (unsigned i=0; i<DJI_SENDER_GROUP_COUNT; ++i)
        memset(sender_assignment[i].tx_buff, 0, 8);
}
static CANInstance configs[DJI_MOTOR_CNT];
static void register_motor(DJIMotorInstance *motor, unsigned id, Motor_Type_e type,
                           GM6020_Control_Mode_e mode, CAN_HandleTypeDef *bus) {
    memset(motor, 0, sizeof(*motor));
    motor->motor_type = type;
    motor->motor_settings.gm6020_control_mode = mode;
    motor->stop_flag = MOTOR_ENALBED;
    CAN_Init_Config_s config = {.can_handle=bus, .tx_id=id};
    MotorSenderGrouping(motor, &config);
    configs[idx] = (CANInstance){.can_handle=bus, .tx_id=id, .rx_id=config.rx_id};
    motor->motor_can_instance = &configs[idx];
    dji_motor_instance[idx++] = motor;
}
static int read_slot(const CANInstance *frame, unsigned slot) {
    return (int16_t)((uint16_t)frame->tx_buff[2*slot] << 8 | frame->tx_buff[2*slot+1]);
}
int main(void) {
    assert(GM6020_CURRENT_MAX_CMD == 16384);
    check_pitch_gyro_filter(20, .46f, .48f, .14f, .25f);
    float saved_pitch_feedback = pitch_gyro_filtered;
    check_yaw_gyro_filter(30, .49f, .50f, .20f, .35f);
    assert(pitch_gyro_filtered == saved_pitch_feedback); // independent axis histories
    check_gravity_direct();
    check_gimbal_task();
    DJIMotorInstance motors[4];
    // Production Yaw config must route the filtered Z-axis speed into its PID.
    imu = (attitude_t){.YawTotalAngle=725, .Gyro={0, .125f, .4f}};
    YAW_CONFIG
    assert(yaw_config.controller_param_init_config.other_speed_feedback_ptr == INS_GetYawGyroFeedback());
    clear();
    register_motor(&motors[0], yaw_config.can_init_config.tx_id, yaw_config.motor_type,
                   yaw_config.controller_setting_init_config.gm6020_control_mode,
                   yaw_config.can_init_config.can_handle);
    motors[0].motor_settings = yaw_config.controller_setting_init_config;
    Motor_Controller_s *yaw = &motors[0].motor_controller;
    yaw->other_angle_feedback_ptr = yaw_config.controller_param_init_config.other_angle_feedback_ptr;
    yaw->other_speed_feedback_ptr = yaw_config.controller_param_init_config.other_speed_feedback_ptr;
    yaw_gyro_filtered = -.2f; // deliberately differs from raw Z-axis speed
    yaw->pid_ref = -730;
    yaw->angle_PID.Output = .25f;
    yaw->speed_PID.Output = 1234;
    DJIMotorControl();
    assert(pid_calls == 2);
    assert(yaw->angle_PID.Ref == 730 && yaw->angle_PID.Measure == 725);
    assert(yaw->speed_PID.Ref == .25f && yaw->speed_PID.Measure == -.2f);
    assert(imu.Gyro[2] == .4f); // raw data remains available to attitude estimation
    assert(sent_count == 1 && sent[0].can_handle == &hcan2 && sent[0].txconf.StdId == 0x1fe);
    assert(read_slot(&sent[0], 3) == 1234 && read_slot(&sent[0], 1) == 0);
    // All supported GM6020 IDs, both buses and both modes.
    for (unsigned bus=0; bus<2; ++bus) {
        for (unsigned mode=0; mode<2; ++mode) {
            for (unsigned id=1; id<=7; ++id) {
                clear();
                register_motor(&motors[0], id, GM6020, mode, bus ? &hcan2 : &hcan1);
                unsigned expected = mode ? (id<=4 ? 0x1fe : 0x2fe)
                                         : (id<=4 ? 0x1ff : 0x2ff);
                assert(motors[0].motor_can_instance->rx_id == 0x204+id);
                assert(motors[0].message_num == (id-1)%4);
                motors[0].motor_controller.pid_ref = 0x1234;
                DJIMotorControl();
                assert(sent_count == 1 && sent[0].txconf.StdId == expected);
                assert(sent[0].can_handle == (bus ? &hcan2 : &hcan1));
                assert(sent[0].txconf.IDE == CAN_ID_STD && sent[0].txconf.RTR == CAN_RTR_DATA);
                assert(sent[0].txconf.DLC == 8);
                for (unsigned slot=0; slot<4; ++slot)
                    assert(read_slot(&sent[0], slot) == (slot==(id-1)%4 ? 0x1234 : 0));
            }
        }
    }
    // Actual CAN2 layout: Yaw ID4/Pitch ID2 current, friction ID7 M3508, loader ID1.
    clear();
    register_motor(&motors[0], 4, GM6020, GM6020_CURRENT_CONTROL, &hcan2);
    register_motor(&motors[1], 2, GM6020, GM6020_CURRENT_CONTROL, &hcan2);
    register_motor(&motors[2], 7, M3508, GM6020_VOLTAGE_CONTROL, &hcan2);
    register_motor(&motors[3], 1, M2006, GM6020_VOLTAGE_CONTROL, &hcan2);
    motors[0].motor_controller.pid_ref = 16384;
    motors[1].motor_controller.pid_ref = -16384;
    motors[2].motor_controller.pid_ref = 1234;
    motors[3].motor_controller.pid_ref = -5678;
    DJIMotorControl();
    assert(sent_count == 3);
    assert(sent[0].txconf.StdId == 0x1ff && read_slot(&sent[0], 2) == 1234);
    assert(sent[1].txconf.StdId == 0x200 && read_slot(&sent[1], 0) == -5678);
    assert(sent[2].txconf.StdId == 0x1fe);
    const uint8_t expected[8] = {0,0,0xc0,0,0,0,0x40,0};
    assert(memcmp(sent[2].tx_buff, expected, 8) == 0);

    // Limits before integer conversion, including extreme values and nonfinite inputs.
    const float values[] = {30000, -30000, 1e10f, -1e10f, NAN, INFINITY, -INFINITY, 100.9f};
    const int limits[] = {16384, -16384, 16384, -16384, 0, 0, 0, 100};
    for (unsigned n=0; n<sizeof(values)/sizeof(values[0]); ++n) {
        motors[0].motor_controller.pid_ref = values[n];
        sent_count = 0;
        DJIMotorControl();
        assert(read_slot(&sent[2], 3) == limits[n]);
        assert(read_slot(&sent[2], 1) == -16384);
        assert(read_slot(&sent[0], 2) == 1234);
    }
    // Stop only zeros its slot, retains the other axis and still transmits the current frame.
    motors[0].stop_flag = MOTOR_STOP;
    motors[0].motor_controller.speed_PID.Iout = 500;
    sent_count = 0;
    DJIMotorControl();
    assert(read_slot(&sent[2], 3) == 0 && read_slot(&sent[2], 1) == -16384);
    assert(motors[0].motor_controller.speed_PID.Iout == 0);

    // Voltage and M3508 paths retain their existing command scale (no current-mode clamp).
    for (unsigned type=0; type<2; ++type) {
        clear();
        register_motor(&motors[0], 1, type ? M3508 : GM6020, GM6020_VOLTAGE_CONTROL, &hcan1);
        motors[0].motor_controller.pid_ref = 20000;
        DJIMotorControl();
        assert(read_slot(&sent[0], 0) == 20000);
    }
    // Final speed PID output, with existing direction convention, is sent as a current command.
    clear();
    register_motor(&motors[0], 4, GM6020, GM6020_CURRENT_CONTROL, &hcan2);
    motors[0].motor_settings.outer_loop_type = SPEED_LOOP;
    motors[0].motor_settings.close_loop_type = SPEED_LOOP;
    motors[0].motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motors[0].motor_controller.pid_ref = -2;
    motors[0].motor_controller.speed_PID.Output = 25000;
    DJIMotorControl();
    assert(pid_calls == 1 && motors[0].motor_controller.speed_PID.Ref == 2);
    assert(sent_count == 1 && sent[0].txconf.StdId == 0x1fe && read_slot(&sent[0], 3) == 16384);
    // Instantiate the actual production Pitch config, then verify both PID stages,
    // feedback sources, command sign and the final CAN slot through production control.
    imu = (struct TestAttitude){.Pitch=5, .Gyro={0, .125f, 0}};
    PITCH_CONFIG
    assert(pitch_config.controller_setting_init_config.outer_loop_type == ANGLE_LOOP);
    assert(pitch_config.controller_setting_init_config.close_loop_type == (ANGLE_LOOP | SPEED_LOOP));
    assert(isfinite(pitch_config.controller_param_init_config.angle_PID.MaxOut));
    assert(pitch_config.controller_param_init_config.angle_PID.MaxOut > 0);
    assert(pitch_config.controller_param_init_config.angle_PID.IntegralLimit >= 0);
    assert(pitch_config.controller_param_init_config.angle_PID.IntegralLimit <=
           pitch_config.controller_param_init_config.angle_PID.MaxOut);
    assert(pitch_config.controller_param_init_config.speed_PID.MaxOut == GM6020_CURRENT_MAX_CMD);
    // Permit PI tuning while requiring bounded integral correction.
    assert(isfinite(pitch_config.controller_param_init_config.speed_PID.Ki));
    assert(pitch_config.controller_param_init_config.speed_PID.Ki >= 0);
    assert(pitch_config.controller_param_init_config.speed_PID.Improve & PID_Integral_Limit);
    assert(pitch_config.controller_param_init_config.speed_PID.IntegralLimit > 0);
    assert(pitch_config.controller_param_init_config.speed_PID.IntegralLimit <= GM6020_CURRENT_MAX_CMD);
    assert(pitch_config.controller_setting_init_config.feedforward_flag == CURRENT_FEEDFORWARD);
    assert(pitch_config.controller_param_init_config.current_feedforward_ptr == &pitch_gravity_ff);
    assert(pitch_config.controller_param_init_config.other_speed_feedback_ptr == &pitch_gyro_filtered);
    clear();
    register_motor(&motors[0], pitch_config.can_init_config.tx_id, pitch_config.motor_type,
                   pitch_config.controller_setting_init_config.gm6020_control_mode,
                   pitch_config.can_init_config.can_handle);
    motors[0].motor_settings = pitch_config.controller_setting_init_config;
    Motor_Controller_s *pitch = &motors[0].motor_controller;
    pitch->other_angle_feedback_ptr = pitch_config.controller_param_init_config.other_angle_feedback_ptr;
    pitch->other_speed_feedback_ptr = pitch_config.controller_param_init_config.other_speed_feedback_ptr;
    pitch->current_feedforward_ptr = pitch_config.controller_param_init_config.current_feedforward_ptr;
    pitch_gyro_filtered = -.2f; // deliberately differs from raw imu.Gyro[1] = +.125
    pitch_gravity_ff = -2500;
    pitch->pid_ref = -10;
    pitch->angle_PID.Output = .25f;
    pitch->speed_PID.Output = 1234;
    DJIMotorControl();
    assert(pid_calls == 2);
    assert(pitch->angle_PID.Ref == 10 && pitch->angle_PID.Measure == 5);
    assert(pitch->speed_PID.Ref == .25f && pitch->speed_PID.Measure == -.2f);
    assert(imu.Gyro[1] == .125f); // filtering must not replace raw IMU measurements
    assert(sent_count == 1 && sent[0].txconf.StdId == 0x1fe);
    assert(read_slot(&sent[0], 1) == -1266 && read_slot(&sent[0], 3) == 0);
    // Feedforward adds after PID without changing speed Ref; the final sum is limited.
    pitch->speed_PID.Output = -16000;
    sent_count = 0;
    DJIMotorControl();
    assert(read_slot(&sent[0], 1) == -16384);
    assert(pitch->speed_PID.Output == -16000 && pitch->speed_PID.Ref == .25f);
    pitch->speed_PID.Output = 20000;
    sent_count = 0;
    DJIMotorControl();
    assert(read_slot(&sent[0], 1) == 16384);
    sent_count = 0;
    motors[0].stop_flag = MOTOR_STOP;
    DJIMotorControl();
    assert(pid_calls == 6 && read_slot(&sent[0], 1) == 0);
    puts("GM6020 native C checks passed: protocol/limits/stop; Yaw/Pitch gyro LPF and feedback routing; Pitch cascade/feedforward; gimbal modes, command signs, stop/re-entry and feedback publication.");
    return 0;
}
"""

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--cc", default="gcc")
args = parser.parse_args()
code = preamble + config_defs + macros + "\n" + motor_types + globals_code + stubs
code += filter_globals + function("PitchGyroFilterUpdate", ins_source)
code += function("INS_GetPitchGyroFeedback", ins_source)
code += function("YawGyroFilterUpdate", ins_source)
code += function("INS_GetYawGyroFeedback", ins_source)
code += gravity_globals + function("PitchGravityFeedforwardUpdate", gimbal_source)
code += "\n".join(typedef(name) for name in
                  ("gimbal_mode_e", "Gimbal_Ctrl_Cmd_s", "Gimbal_Upload_Data_s"))
code += task_stubs
code += "\n".join(function(name) for name in
                  ("DJIMotorEnable", "DJIMotorStop", "DJIMotorSetRef", "DJIMotorChangeFeed"))
code += function("GimbalApplyReferences", gimbal_source) + function("GimbalTask", gimbal_source)
# Run the same response, frequency attenuation and invalid-sample checks on both axes.
yaw_filter_checks = checks[checks.index("static float filter_gain("):checks.index("static void check_gravity_direct(")]
yaw_filter_checks = yaw_filter_checks.replace("Pitch", "Yaw").replace("pitch", "yaw").replace("filter_gain", "yaw_filter_gain")
code += function("MotorSenderGrouping") + function("DJIMotorControl") + yaw_filter_checks
code += checks.replace("PITCH_CONFIG", pitch_config).replace("YAW_CONFIG", yaw_config)
with tempfile.TemporaryDirectory(prefix="gm6020-current-") as temp:
    path = Path(temp)
    (path / "test.c").write_text(code, encoding="utf-8")
    subprocess.run([args.cc, "-std=c11", "-Wall", "-Wextra", "-Werror", str(path / "test.c"),
                    "-o", str(path / "test.exe"), "-lm"], check=True)
    subprocess.run([str(path / "test.exe")], check=True)
