// config.h -- every pin, rate, limit and default gain for flight_computer.
//
// Pins MUST match docs/pinout.md. Change both in the same commit.
// Values tagged [EST] are placeholders until measured; [DES] are design
// choices; [SPEC] come from a datasheet (see docs/components.md).

#pragma once
#include <Arduino.h>

// =====================================================================
// PINS (Teensy 4.0 -- 3.3 V logic, NOT 5 V tolerant)
// =====================================================================
constexpr int PIN_ESC     = 2;    // PWM throttle to the ESC signal wire
constexpr int PIN_IMU_INT = 3;    // GY-521 INT (reserved, unused for now)
constexpr int PIN_LED     = 13;   // onboard LED: shows the state

#define SERVO_BUS_SERIAL Serial1  // pins 0 (RX1) / 1 (TX1), through the level shifter to the BusLinker
constexpr uint32_t SERVO_BUS_BAUD = 115200;   // [SPEC] HTD-30H default
constexpr uint8_t  SERVO_ID       = 1;        // [SPEC] factory default ID

// =====================================================================
// LOOP RATES
// =====================================================================
constexpr uint32_t LOOP_HZ       = 400;   // [DES] control loop, same as tvc_params.m loop_rate
constexpr uint32_t SERVO_CMD_HZ  = 100;   // [DES] move packets (~0.9 ms each on the bus)
constexpr uint32_t SERVO_READ_HZ = 20;    // [DES] position/voltage readback (each read blocks ~2 ms)
constexpr uint32_t TELEM_HZ      = 50;    // [DES] CSV rows to the Pi

// =====================================================================
// GIMBAL / SERVO (HTD-30H: 0..1000 counts = 0..240 deg)
// =====================================================================
constexpr float GIMBAL_MAX_DEG      = 14.0;          // [DES] command clamp, = delta_cmd_max in tvc_params.m
constexpr float SERVO_COUNTS_PER_DEG = 1000.0 / 240.0; // [SPEC]
constexpr int   SERVO_CENTER        = 500;           // [SPEC] mid-travel; fine-tune with the "trim" command
constexpr float SERVO_DIR           = 1.0;
// +1 or -1. Flip if a POSITIVE gimbal command tilts the fan the wrong way
// (see the direction check in firmware/flight_computer/README.md). Wrong
// sign = positive feedback = the rocket slams into its stops.
constexpr float GIMBAL_RATIO        = 1.0;           // servo deg per fan deg (1.0 = horn drives gimbal directly)
constexpr float SERVO_VMIN = 9.0, SERVO_VMAX = 13.0; // [SPEC] HTD-30H is rated 9-12.6 V

// =====================================================================
// ESC / EDF
// =====================================================================
constexpr int   ESC_MIN_US   = 1000;  // [VERIFY] stop, after throttle-range calibration
constexpr int   ESC_MAX_US   = 2000;  // [VERIFY] full throttle
constexpr float THROTTLE_MAX = 0.25;
// [DES] HARD CAP. The EDF can pull ~50 A at full throttle; the EDF branch
// fuse is 15 A. Raise this only in small steps while watching a watt meter,
// and log each step in docs/icd/edf_esc.md.

// =====================================================================
// IMU (GY-521 / MPU-6050)
// =====================================================================
constexpr uint8_t MPU_ADDR      = 0x68;   // AD0 low/open
constexpr bool    IMU_USE_Y_AXIS = false; // true if the gimbal plane lines up with the IMU's Y axis
constexpr float   IMU_SIGN      = 1.0;    // make "nose tips toward +x" read as POSITIVE theta
constexpr float   COMP_TAU      = 1.0;    // [DES] complementary filter time constant [s], = cf_tau in tvc_params.m
constexpr float   D_LPF_HZ      = 40.0;   // [DES] low-pass on the gyro rate used by the D term, = d_lpf_hz

// =====================================================================
// SAFETY
// =====================================================================
constexpr float    THETA_TRIP_DEG  = 25.0;  // [DES] past this -> FAULT (hard stops are at 20 deg in the sim)
constexpr uint8_t  SERVO_MAX_MISSES = 10;   // consecutive failed servo reads in ARMED/RUN -> FAULT
constexpr uint32_t HOST_TIMEOUT_MS = 0;
// If > 0: in RUN, no line from the Pi for this long -> SAFE. 0 = disabled
// (so the Arduino Serial Monitor works). Set to ~1000 once tvc_logger.py
// sends "hb" automatically.

// =====================================================================
// DEFAULT GAINS -- gimbal angle [rad] per body angle [rad]
// =====================================================================
// From tvc_design_gains.m with the CURRENT [EST] parameters in
// tvc_params.m (wn = 12 rad/s, zeta = 0.8, throttle 0.75 of a 6.5 N EDF).
// Re-run the sim with measured mass, CG, inertia and thrust, then update.
// The law is the same as tvc_simulate.m:
//   delta = Kp*(ref - theta) - Kd*theta_rate + Ki*integral(ref - theta)
constexpr float KP_DEFAULT = 5.2;
constexpr float KI_DEFAULT = 0.0;
constexpr float KD_DEFAULT = 0.62;
