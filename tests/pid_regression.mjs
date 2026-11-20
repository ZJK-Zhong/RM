// Run: node tests/pid_regression.mjs
// Evaluate the production scalar C function bodies with syntax-only translation.
// This checks control logic using JS numbers; it does not replace a C build or
// hardware testing, and does not model float32 rounding or task preemption.
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const root = new URL('../', import.meta.url);
const read = path => readFileSync(new URL(path, root), 'utf8');
const uncomment = text => text.replace(/\/\*[\s\S]*?\*\/|\/\/[^\r\n]*/g, '');
const source = uncomment(read('modules/algorithm/controller.c'));

function bodyOf(text, name) {
  const match = text.match(new RegExp(`(?:static\\s+)?(?:void|float)\\s+${name}\\([^)]*\\)\\s*\\{`));
  assert.ok(match, `Missing C function ${name}`);
  const start = match.index + match[0].length;
  let depth = 1;
  for (let end = start; end < text.length; ++end) {
    if (text[end] === '{') ++depth;
    if (text[end] === '}' && --depth === 0) return text.slice(start, end);
  }
  throw new Error(`Unclosed C function ${name}`);
}

function translate(body) {
  return body
    .replace(/DWT_GetDeltaT\(&pid->DWT_CNT\)/g, 'DWT_GetDeltaT(pid)')
    .replace(/&(motor_controller->\w+)/g, '$1')
    .replace(/->/g, '.')
    .replace(/\b(?:static\s+)?float\s+/g, 'let ')
    .replace(/\b(\d+(?:\.\d+)?(?:e[+-]?\d+)?)f\b/gi, '$1')
    .replace(/\bfabsf\(/g, 'Math.abs(');
}

let timeMs = 0;
function DWT_GetDeltaT(pid) {
  const dt = (timeMs - pid.DWT_CNT) / 1000;
  pid.DWT_CNT = timeMs;
  return dt;
}
const functions = ['f_Integral_Limit', 'f_Output_Limit', 'PIDReset', 'PIDCalculate'];
const compiled = functions.map(name => `function ${name}(pid, measure, ref) {${translate(bodyOf(source, name))}}`).join('\n');
const api = new Function('DWT_GetDeltaT', 'PID_Integral_Limit', 'PID_ERROR_NONE',
  `${compiled}\nreturn {PIDCalculate, PIDReset, f_Integral_Limit};`)(DWT_GetDeltaT, 1, 0);

function pid(overrides = {}) {
  const p = { Kp: 0, Ki: 1, Kd: 0, MaxOut: 100, IntegralLimit: 50,
    Improve: 1, ERRORHandler: {}, ...overrides };
  api.PIDReset(p);
  return p;
}
function tick(p, measure, ref, seconds = 1) {
  timeMs += seconds * 1000;
  return api.PIDCalculate(p, measure, ref);
}
function near(actual, expected) {
  assert.ok(Math.abs(actual - expected) < 1e-6, `${actual} != ${expected}`);
}
let cases = 0;
function test(name, run) {
  run(); ++cases; console.log(`PASS ${name}`);
}
function integralCase(values) {
  const p = pid(values);
  Object.assign(p, values);
  api.f_Integral_Limit(p);
  p.Iout += p.ITerm;
  return p;
}

test('rectangular integration and per-instance state', () => {
  const a = pid({ Ki: 2 });
  tick(a, 0, 3); // first sample establishes history
  tick(a, 0, 3, .5); tick(a, 0, 3, .5);
  const b = pid({ Ki: 2 });
  tick(b, 0, 3); tick(b, 0, 3, .5);
  near(a.Iout, 6); near(b.Iout, 3);
});
test('reject positive and negative windup starting from zero integral', () => {
  for (const sign of [-1, 1]) {
    const p = integralCase({ Pout: sign * 20, Iout: 0, ITerm: sign,
      Dout: 0, MaxOut: 10, IntegralLimit: 100, Err: sign });
    near(p.Iout, 0); near(p.ITerm, 0);
  }
});
test('evaluate candidate output including this sample integral', () => {
  for (const sign of [-1, 1]) {
    const p = integralCase({ Pout: sign * 9, Iout: 0, ITerm: sign * 2,
      Dout: 0, MaxOut: 10, IntegralLimit: 100, Err: sign });
    near(p.Iout, 0);
  }
});
test('rejected increment cannot clamp old integral to a stale candidate', () => {
  for (const sign of [-1, 1]) {
    const p = integralCase({ Pout: sign * 100, Iout: sign * 9, ITerm: sign * 2,
      Dout: 0, MaxOut: 10, IntegralLimit: 10, Err: sign });
    near(p.Iout, sign * 9); near(p.ITerm, 0);
  }
});
test('allow unwinding even while output remains saturated', () => {
  for (const sign of [-1, 1]) {
    const p = integralCase({ Pout: sign * 20, Iout: sign * 5, ITerm: -sign * 2,
      Dout: 0, MaxOut: 10, IntegralLimit: 100 });
    near(p.Iout, sign * 3);
  }
});
test('integral clamps accept the remaining headroom', () => {
  for (const sign of [-1, 1]) {
    const p = integralCase({ Pout: 0, Iout: sign * 9, ITerm: sign * 2,
      Dout: 0, MaxOut: 100, IntegralLimit: 10 });
    near(p.Iout, sign * 10); near(p.ITerm, sign);
  }
});
test('lowering the integral limit bounds existing history', () => {
  const p = integralCase({ Pout: 0, Iout: 20, ITerm: 0, Dout: 0,
    MaxOut: 100, IntegralLimit: 5 });
  near(p.Iout, 5);
});
test('a zero integral limit disables accumulated integral', () => {
  const p = integralCase({ Pout: 0, Iout: 4, ITerm: 10, Dout: 0,
    MaxOut: 100, IntegralLimit: 0 });
  near(p.Iout, 0); near(p.ITerm, 0);
});
test('reset clears history, retains tuning, and avoids restart I/D kicks', () => {
  const p = pid({ Kp: 2, Ki: 3, Kd: 4, MaxOut: 100, IntegralLimit: 20 });
  tick(p, 0, 1); tick(p, 0, 1); assert.ok(p.Iout > 0);
  api.PIDReset(p);
  near(p.Iout, 0); near(p.ITerm, 0); near(p.Last_ITerm, 0); near(p.Output, 0);
  near(p.Kp, 2); near(p.Ki, 3); near(p.Kd, 4); near(p.IntegralLimit, 20);
  near(tick(p, 0, 2, 30), 4); near(p.Iout, 0); near(p.Dout, 0);
  tick(p, 0, 2, .5); near(p.Iout, 3);
});
test('output remains bounded in both directions', () => {
  const p = pid({ Kp: 20, MaxOut: 10 });
  near(tick(p, 0, 1), 10);
  for (let n = 0; n < 100; ++n) near(tick(p, 0, 1), 10);
  near(p.Iout, 0); near(tick(p, 0, -1), -10); near(p.Iout, 0);
});

// Execute the actual early-stop blocks as well as the scalar PID functions.
function stopGuard(path, name) {
  const body = bodyOf(uncomment(read(path)), name);
  const guard = body.indexOf('if (motor->stop_flag == MOTOR_STOP)');
  assert.ok(guard >= 0 && guard < body.indexOf('PIDCalculate('), 'Stop must precede PID evaluation');
  let depth = 0, end = body.indexOf('{', guard);
  const start = end;
  do {
    if (body[end] === '{') ++depth;
    if (body[end] === '}') --depth;
    ++end;
  } while (depth);
  let block = body.slice(guard, end)
    .replace(/memset\(sender_assignment\[motor->sender_group\]\.tx_buff \+ 2 \* motor->message_num, 0, sizeof\(uint16_t\)\);/,
      'sender_assignment[motor.sender_group].tx_buff.fill(0, 2 * motor.message_num, 2 * motor.message_num + 2);')
    .replace(/continue;/g, 'return true;');
  assert.ok(start >= 0);
  return new Function('motor', 'motor_controller', 'sender_assignment', 'initial_torque',
    'initial_give_power', 'power_control_out', 'i', 'PIDReset', 'MOTOR_STOP', translate(block));
}
for (const [name, path, fn] of [
  ['ordinary motor', 'modules/motor/DJImotor/dji_motor.c', 'DJIMotorControl'],
  ['chassis motor', 'modules/motor/power_control.c', 'PowerControl'],
]) {
  test(`${name} stop clears all loops and skips PID`, () => {
    const guard = stopGuard(path, fn);
    const controller = { angle_PID: pid(), speed_PID: pid(), current_PID: pid() };
    for (const p of Object.values(controller)) { p.Iout = 12; p.Output = 20; }
    const frames = [{ tx_buff: new Uint8Array(8).fill(255) }];
    const torque = [12], power = [30], output = [20];
    const motor = { stop_flag: 0, sender_group: 0, message_num: 1 };
    for (let n = 0; n < 20; ++n) {
      timeMs += 1000;
      assert.equal(guard(motor, controller, frames, torque, power, output, 0, api.PIDReset, 0), true);
      for (const p of Object.values(controller)) { near(p.Iout, 0); near(p.Output, 0); }
    }
    if (fn === 'DJIMotorControl') {
      assert.deepEqual([...frames[0].tx_buff], [255, 255, 0, 0, 255, 255, 255, 255]);
    } else {
      near(torque[0], 0); near(power[0], 0); near(output[0], 0);
      assert.match(uncomment(read(path)), /motor->stop_flag == MOTOR_STOP \|\| initial_give_power\[i\] < 0/);
    }
    motor.stop_flag = 1;
    assert.equal(guard(motor, controller, frames, torque, power, output, 0, api.PIDReset, 0), undefined);
    near(tick(controller.speed_PID, 0, 5, 30), 0); // default Kp=0, no restart I
    tick(controller.speed_PID, 0, 5, .5); near(controller.speed_PID.Iout, 2.5);
  });
}
test('trapezoid integration is absent from active controller and motor configuration', () => {
  assert.doesNotMatch(source, /Trapezoid|Trapezoid_Intergral/);
  for (const path of ['application/chassis/chassis.c', 'application/gimbal/gimbal.c'])
    assert.doesNotMatch(uncomment(read(path)), /PID_Trapezoid_Intergral/);
});
console.log(`${cases} regression checks passed (scalar source evaluation; not a C binary).`);
