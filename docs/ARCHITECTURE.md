# Architecture

This project is a simulation-only UAV control reference. It separates command validation, safety policy, control loops, motor mixing, and simulation so each part can be studied independently.

## Control Data Flow

```text
ControlCommand + SensorSample
        |
        v
FlightController::update
        |
        |-- hard safety checks
        |-- arming checks
        |-- command failsafe policy
        |-- command limiting
        |-- mode-specific setpoint generation
        |-- PID control
        v
QuadXMixer::mix
        |
        v
Motor outputs in [0.0, 1.0]
```

## `SafetyConfig`

Collects conservative limits for voltage, tilt, altitude, geofence radius, and command age. Keeping these values together makes it easier to review the safety boundary before changing behavior.

## `FlightMode`

The reference controller supports:

- `Manual`: bounded attitude and throttle-style reference.
- `AltitudeHold`: altitude PID with bounded attitude commands.
- `PositionHold`: local north/east position hold using simple outer-loop PIDs.
- `ReturnToLaunch`: targets the saved home position, then switches to landing.
- `Land`: descends gradually and disarms near the ground.

## `FlightController`

The main control loop. Each `update` call:

1. Runs hard safety checks such as emergency stop, low voltage, attitude limit, altitude limit, and hard geofence breach.
2. Evaluates conservative arming requirements.
3. Converts stale commands, geofence boundaries, and altitude limit events into safer modes.
4. Clamps user commands before they reach the PID controllers.
5. Generates mode-specific roll, pitch, yaw-rate, and altitude setpoints.
6. Mixes outputs for a quadcopter X frame.

## `EventLog`

Stores the most recent safety and mode-transition events. It is intentionally small and deterministic so it can be adapted to embedded systems later.

## `QuadXMixer`

Motor mixer for a quadcopter X-frame layout. Output order:

```text
0: front-left
1: front-right
2: rear-right
3: rear-left
```

Different airframes should add or replace the mixer, for example plus-frame quads, hexacopters, octocopters, or fixed-wing control surfaces.

## `DroneSimulator`

A minimal simulation model used only to observe closed-loop behavior. It does not include real aerodynamics, propeller models, IMU noise, sensor delay, wind disturbance, GPS faults, magnetometer errors, or motor saturation dynamics.

## Program Entry Points

- The default mode runs `run_demo` and prints CSV-style state output.
- Passing `--self-test` runs `run_self_test`.
