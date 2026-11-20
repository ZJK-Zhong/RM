"""Execute production PowerControl and CAN packing with stub PID/transport.

Checks numerical power limiting and commands, not physical motor/power hardware.
Usage: python tests/chassis_power_regression.py --cc path/to/gcc
"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "modules/motor/power_control.c").read_text(encoding="utf-8")
header = (ROOT / "modules/motor/DJImotor/dji_motor.h").read_text(encoding="utf-8")
definitions = (ROOT / "modules/motor/motor_def.h").read_text(encoding="utf-8")


def function(name):
    match = re.search(r"(?:static\s+)?(?:void|float)\s+" + name + r"\([^)]*\)\s*\{", source)
    assert match, name
    depth = 1
    for end in range(match.end(), len(source)):
        depth += (source[end] == "{") - (source[end] == "}")
        if depth == 0:
            return source[match.start():end + 1]
    raise AssertionError(name)


preamble = r"""
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
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
typedef struct { float Kp, Ki, Kd, IntegralLimit, MaxOut; unsigned Improve; } PID_Init_Config_s;
typedef struct { int unused; } DaemonInstance;
#define CAN_ID_STD 0
#define CAN_RTR_DATA 0
#define DJI_MOTOR_CNT 12
static void test_error(const char *format, ...) { (void)format; abort(); }
#define LOGERROR(...) test_error(__VA_ARGS__)
typedef struct { bool enable_dcdc; unsigned referee_power_limit, referee_energy_buffer; } SuperCap_Control_t;
typedef struct { float chassis_power_limit_w; } SuperCap_Feedback_t;
typedef struct { SuperCap_Control_t control; } SuperCapInstance;
"""

stubs = r"""
static CANInstance sent;
static unsigned sent_count, pid_calls;
static SuperCapInstance *GetSuperCapInstance(void) { return NULL; }
static void SuperCapSend(SuperCapInstance *cap, SuperCap_Control_t *cmd) { (void)cap; (void)cmd; }
static SuperCap_Feedback_t SuperCapGet(SuperCapInstance *cap) {
    (void)cap; return (SuperCap_Feedback_t){0};
}
static void CANTransmit(CANInstance *can, int timeout) {
    (void)timeout; sent = *can; ++sent_count;
}
static void PIDReset(PIDInstance *pid) { memset(pid, 0, sizeof(*pid)); }
static float PIDCalculate(PIDInstance *pid, float measure, float ref) {
    ++pid_calls; pid->Measure = measure; pid->Ref = ref; return pid->Output;
}
"""

checks = r"""
static DJIMotorInstance motors[4];
static CANInstance configs[4];
static void setup(float limit) {
    idx = 0; sent_count = pid_calls = 0;
    memset(motors, 0, sizeof(motors));
    memset(sender_enable_flag, 0, sizeof(sender_enable_flag));
    memset(initial_torque, 0, sizeof(initial_torque));
    memset(initial_give_power, 0, sizeof(initial_give_power));
    memset(power_control_out, 0, sizeof(power_control_out));
    for (unsigned i=0; i<6; ++i) memset(sender_assignment[i].tx_buff, 0, 8);
    const unsigned ids[] = {2, 3, 4, 1}; // actual LF/RF/LB/RB layout
    for (unsigned i=0; i<4; ++i) {
        DJIMotorInstance *motor = &motors[i];
        motor->motor_type = M3508;
        motor->stop_flag = MOTOR_ENALBED;
        motor->motor_settings.outer_loop_type = SPEED_LOOP;
        motor->motor_settings.close_loop_type = SPEED_LOOP;
        CAN_Init_Config_s cfg = {.can_handle=&hcan1, .tx_id=ids[i]};
        MotorSenderGrouping(motor, &cfg);
        configs[i] = (CANInstance){.can_handle=&hcan1, .rx_id=cfg.rx_id};
        motor->motor_can_instance = &configs[i];
        dji_motor_instance[idx++] = motor;
    }
    SetPowerLimit(limit);
}
static int command(unsigned i) {
    unsigned slot = motors[i].message_num;
    return (int16_t)((uint16_t)sent.tx_buff[2*slot] << 8 | sent.tx_buff[2*slot+1]);
}
static void run(void) {
    sent_count = 0;
    PowerControl();
    assert(sent_count == 1 && sent.txconf.StdId == 0x200 && sent.can_handle == &hcan1);
    for (unsigned i=0; i<4; ++i) {
        assert(isfinite(power_control_out[i]));
        assert(abs(command(i)) <= 16384);
    }
}
int main(int argc, char **argv) {
    (void)chassis_power;
    setup(45);
    motors[0].measure.speed_aps = 2000 * 6; // coasting at 2000 rpm
    for (unsigned i=1; i<4; ++i) motors[i].motor_controller.speed_PID.Output = 15000;
    run();
    printf("zero PID demand at 2000 rpm under 45 W limit: CAN command=%d\n", command(0));
    fflush(stdout);
    if (argc > 1 && strcmp(argv[1], "--observe") == 0) return 0;
    assert(command(0) == 0); // power limiting cannot invent a nonzero command

    // Limiting may reduce a request, but cannot reverse or amplify it, even while braking.
    const float speeds[] = {-10000, -3000, -1000, 0, 1000, 3000, 10000};
    const float requests[] = {-15000, -1000, 0, 1000, 15000};
    const float limits[] = {1, 10, 45, 100, 10000};
    for (unsigned s=0; s<7; ++s) for (unsigned r=0; r<5; ++r)
    for (unsigned l=0; l<5; ++l) {
        setup(limits[l]);
        for (unsigned i=0; i<4; ++i) {
            motors[i].measure.speed_aps = speeds[s] * 6;
            motors[i].motor_controller.speed_PID.Output = requests[r];
        }
        run();
        for (unsigned i=0; i<4; ++i) {
            assert(fabsf(power_control_out[i]) <= fabsf(requests[r]) + .01f);
            assert(power_control_out[i] * requests[r] >= 0);
            assert(abs(command(i)) <= fabsf(requests[r]));
        }
    }
    // Model and inverse agree; in this feasible motoring case, actual modeled sum is 45 W.
    setup(45);
    for (unsigned i=0; i<4; ++i) {
        motors[i].measure.speed_aps = 3000 * 6;
        motors[i].motor_controller.speed_PID.Output = 15000;
    }
    run();
    float total = 0;
    for (unsigned i=0; i<4; ++i) {
        float current = power_control_out[i], rpm = 3000;
        total += 1.23e-7f*current*current + 1.453e-7f*rpm*rpm +
                 1.99688994e-6f*rpm*current + 4.081f;
    }
    assert(fabsf(total - 45) < .02f);

    // Nonlimited commands are unchanged; reversal is applied exactly once in both branches.
    for (unsigned limited=0; limited<2; ++limited) {
        setup(limited ? 45 : 10000);
        for (unsigned i=0; i<4; ++i) motors[i].motor_controller.speed_PID.Output = 15000;
        motors[0].motor_settings.feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE;
        run();
        assert(command(0) == -command(1));
        if (!limited) assert(command(1) == 15000);
        motors[0].stop_flag = MOTOR_STOP;
        motors[0].motor_controller.speed_PID.Iout = 100;
        run();
        assert(command(0) == 0 && power_control_out[0] == 0);
        assert(motors[0].motor_controller.speed_PID.Iout == 0);
        motors[0].stop_flag = MOTOR_ENALBED;
        motors[0].motor_controller.speed_PID.Output = 1000;
        run();
        assert(command(0) <= 0 && command(0) >= -1000);
    }
    // Invalid/oversize values cannot reach a float-to-int conversion unguarded.
    const float bad[] = {NAN, INFINITY, -INFINITY, 1e10f, -1e10f};
    for (unsigned i=0; i<5; ++i) {
        setup(10000);
        motors[0].motor_controller.speed_PID.Output = bad[i];
        run();
        assert(command(0) == (i<3 ? 0 : (i==3 ? 16384 : -16384)));
    }
    setup(45);
    motors[0].measure.speed_aps = NAN;
    motors[0].motor_controller.speed_PID.Output = 500;
    run();
    assert(command(0) == 0);
    puts("PASS production chassis power/CAN checks (175 speed/request/limit combinations plus edge cases)");
    return 0;
}
"""

parser = argparse.ArgumentParser()
parser.add_argument("--cc", required=True)
parser.add_argument("--observe", action="store_true", help="print the original fault without asserting the fix")
args = parser.parse_args()
config_defs = definitions[definitions.index("#define LIMIT_MIN_MAX"):definitions.index("#endif")]
motor_types = header[header.index("typedef struct"):header.index("/**\n * @brief 调用")]
globals_code = source[source.index("static SuperCapInstance"):source.index("/**\n * @brief 设置底盘")]
if "static float ChassisClampCurrent(" in globals_code:
    globals_code = globals_code[:globals_code.index("static float ChassisClampCurrent(")]
helpers = ""
if "static float ChassisClampCurrent(" in source:
    helpers = function("ChassisClampCurrent")
    helpers = "#define CHASSIS_CURRENT_MAX_CMD 16384.0f\n" + helpers
code = preamble + config_defs + motor_types + globals_code + stubs + helpers
code += function("SetPowerLimit") + function("MotorSenderGrouping") + function("PowerControl") + checks
with tempfile.TemporaryDirectory(prefix="chassis-power-") as temp:
    path = Path(temp)
    (path / "test.c").write_text(code, encoding="utf-8")
    subprocess.run([args.cc, "-std=c11", "-Wall", "-Wextra", "-Werror", str(path / "test.c"),
                    "-o", str(path / "test.exe"), "-lm"], check=True)
    subprocess.run([str(path / "test.exe")] + (["--observe"] if args.observe else []), check=True)
