# Architecture

This project is a simulation-only UAV control reference. It separates preflight checks, mission validation, command validation, safety policy, battery failsafes, obstacle avoidance, control loops, telemetry, motor mixing, and simulation so each part can be studied independently.

## Control Data Flow

```text
ControlCommand + SensorSample
        |
        v
PreflightChecker / MissionPlanner
        |
        v
FlightController::update
        |
        |-- hard safety checks
        |-- arming checks
        |-- one-key return and command failsafe policy
        |-- battery warning, return, landing, and cutoff policy
        |-- command limiting
        |-- mode-specific setpoint generation
        |-- obstacle avoidance overlay
        |-- PID control
        v
QuadXMixer::mix
        |
        v
Motor outputs in [0.0, 1.0]
        |
        v
TelemetryEncoder snapshot
```

## `SafetyConfig`

Collects conservative limits for voltage, tilt, geofence radius, obstacle distance, and command age. The legal altitude limit is intentionally locked as `SafetyConfig::locked_legal_altitude_limit_m` and is not a normal runtime setting.

## `AdvancedSafety`

`AdvancedSafety.h` and `AdvancedSafety.cpp` contain higher-level safety-support modules:

- `PreflightChecker`: produces a go/no-go arming report.
- `MissionPlanner`: rejects missions outside geofence or altitude limits.
- `SensorHealthMonitor`: summarizes navigation, obstacle, and battery health.
- `TelemetryEncoder`: emits compact safety telemetry frames.

## `FlightMode`

The reference controller supports:

- `Manual`: bounded attitude and throttle-style reference.
- `AltitudeHold`: altitude PID with bounded attitude commands.
- `PositionHold`: local north/east position hold using simple outer-loop PIDs.
- `ReturnToLaunch`: targets the saved home position, then switches to landing.
- `Land`: descends gradually and disarms near the ground.

## One-Key Return-To-Launch

`ControlCommand::one_key_return_to_launch` forces return-to-launch when a valid home position exists. If position is unavailable, the controller chooses landing instead of pretending that a safe return path exists.

## Battery Failsafes

The controller uses staged battery handling:

- Warning voltage: keep flying but report that the operator should return soon.
- Return voltage: force return-to-launch if position is valid.
- Land voltage: force landing because returning may no longer be safe.
- Emergency cutoff voltage: stop output because the simulated vehicle is below the reference control boundary.

## Obstacle Avoidance

`ObstacleDistances` provides simple front, back, left, right, up, and down readings. `apply_obstacle_avoidance` overlays small attitude and altitude changes on top of the active mode. This is only a reactive simulation example; it is not a replacement for tested perception, mapping, or path planning.

## `FlightController`

The main control loop. Each `update` call:

1. Runs hard safety checks such as emergency stop, low voltage, attitude limit, altitude limit, and hard geofence breach.
2. Evaluates conservative arming requirements.
3. Converts one-key return, stale commands, battery thresholds, geofence boundaries, and altitude limit events into safer modes.
4. Clamps user commands before they reach the PID controllers.
5. Generates mode-specific roll, pitch, yaw-rate, and altitude setpoints.
6. Applies simple obstacle avoidance.
7. Mixes outputs for a quadcopter X frame.

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
