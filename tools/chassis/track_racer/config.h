#pragma once
// A17 v1.0. Fill measured values; see README. No guessed calibration unlocks RUN.
namespace cfg {
constexpr bool COMMISSIONED = false;
constexpr bool DIRECTION_CHECKED = true; // 2026-10-23: user JOG and separate forward hand-turn checks.
constexpr bool MOUNTING_CHECKED = false;
constexpr bool SWEPT_PATH_CHECKED = false; // Real R400/zigzag test with whole body.
// 2026-10-23: user confirmed both JOG directions push the vehicle forward.
constexpr int MOTOR_SIGN[2] = {+1, -1};
// 2026-10-23 correction: left JOG-forward 5 turns +4083, then 1 turn +825;
// right forward raw counts decrease. Earlier left -485 observation superseded.
constexpr int ENCODER_SIGN[2] = {+1, -1}; // Applied to path/velocity, not logged raw ticks.
constexpr int SERVO_LEFT_SIGN = -1;       // Smaller pulse = left? Verify.
constexpr int CENTER_US = 1500;
constexpr int LEFT_US = 1200;
constexpr int RIGHT_US = 1800;
// Zero means unknown, never silently replaced by an old design dimension.
constexpr float WIDTH_MM = 0;          // Entire car, including tires/protection.
constexpr float WHEELBASE_MM = 0;
constexpr float REAR_TRACK_MM = 0;
// User-reported rolling distance: each wheel 3 turns = 650 mm (2026-10-23).
constexpr float WHEEL_MM[2] = {650.0f/(3.0f*3.14159265359f), 650.0f/(3.0f*3.14159265359f)};
// Initial hand-turn estimate: left 9066/11 turns, right 8257/10 turns.
// Five-turn repeat spread: left ~1.82%, right ~0.315%; refine before speed acceptance.
constexpr float CPR4[2] = {824.2f, 825.7f}; // QUADRATURE counts / output turn; initial estimate.
constexpr float FRONT_INSET_MM = 0;    // Window behind foremost swept envelope.
constexpr float SIDE_INSET_MM[2] = {0, 0}; // Side windows inside widest envelope.
constexpr float LEFT_ANGLE_DEG = 0;    // Equivalent bicycle angle at LEFT_US.
constexpr float RIGHT_ANGLE_DEG = 0;
constexpr float MIN_TURN_RADIUS_MM = 0; // Worst measured rear-axle-centre radius.
constexpr float BRAKE_MPS2 = 0;       // Conservative lower bound, brake-to-stop.
constexpr float REACTION_S = 0;      // Measured latency incl servo/brake response.
constexpr float LATERAL_MPS2 = 0;    // Verified acceleration limit, not assumed mu*g.
constexpr float ERROR_MM[3] = {0, 0, 0}; // Front/left/right worst positive error bound.
constexpr float BIAS_MM[3] = {0, 0, 0};  // measured - true; corrected = raw - bias.
constexpr float CERTIFIED_MAX_MM[3] = {0, 0, 0};
constexpr float MAX_WHEEL_MPS = 0;    // Verified speed ceiling for either wheel.
constexpr float FF_DUTY_PER_MPS[2] = {0, 0}; // Loaded feedforward slope, duty/(m/s).
constexpr float FF_STATIC[2] = {0, 0};        // Loaded breakaway compensation.
// Initial engineering settings. These are NOT test results or acceptance limits.
constexpr float INITIAL_MAX_MPS = 0.20f;
constexpr float ACCEL_MPS2 = 0.20f;
constexpr int MAX_DUTY = 100; // 39.2%; increase only after loaded verification.
constexpr float SPEED_KP = 90, SPEED_KI = 55;
constexpr float WALL_KP_RAD_PER_MM = 0.0025f;
constexpr float WALL_KD_RAD_S_PER_MM = 0.00045f;
constexpr float MIN_CLEAR_MM = 35, FRONT_RESERVE_MM = 70;
constexpr float SIDE_SLOW_MM = 85;
constexpr unsigned FRONT_AGE_MS = 150, SIDE_AGE_MS = 220;
constexpr unsigned MEASURE_TIMEOUT_MS = 70;
constexpr unsigned COMMAND_LEASE_MS = 80;
constexpr unsigned BRAKE_HOLD_MS = 2000; // Must exceed maxWheelSpeed/brakeDecel.
constexpr unsigned MAX_RUN_MS = 120000;
constexpr float FINISH_DISTANCE_MM = 0; // 0: disabled; no finish recognition claim.
constexpr bool FOLLOW_RIGHT = true; // Outer right wall, travelling upward at start.
}
