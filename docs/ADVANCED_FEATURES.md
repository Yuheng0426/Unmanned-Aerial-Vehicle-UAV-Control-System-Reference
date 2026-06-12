# Advanced Safety Features

This project includes additional simulation-only support modules in `AdvancedSafety.h` and `AdvancedSafety.cpp`. These modules are intended to help readers study how a safer UAV software stack can be organized before any hardware work begins.

## Preflight Checker

`PreflightChecker` produces a go/no-go report before arming. It checks:

- Arm request state
- Emergency stop state
- Command-link freshness
- Battery voltage
- Level attitude
- Position availability for position-dependent modes
- Whether the vehicle appears to already be airborne

The checker is conservative by design. A failed preflight check should block arming in any real adaptation.

## Mission Planner

`MissionPlanner` validates local waypoint missions before turning them into position-hold commands. It rejects:

- Empty missions
- Waypoints above `SafetyConfig::locked_legal_altitude_limit_m`
- Waypoints outside the configured geofence radius

This is not autonomous navigation software. It is a safety-bounded mission sandbox for simulation.

## Sensor Health Monitor

`SensorHealthMonitor` summarizes:

- Navigation health
- Obstacle-distance health
- Battery health

Health states are `ok`, `warning`, or `critical`. A real system should treat warning and critical states as operator-visible safety events.

## Telemetry Encoder

`TelemetryEncoder` creates CSV-style snapshots with:

- Flight mode
- Local position
- Altitude
- Battery voltage
- Distance from home
- Battery health
- Navigation health
- Last safety message

The demo prints these frames as comment-style telemetry lines.

## Safety Boundary

These features do not make the project flight-ready. They do not replace certified sensors, tested estimators, redundant failsafes, legal review, or independent emergency stop hardware.

Do not use these modules to weaken altitude limits, geofences, emergency stop behavior, battery failsafes, or legal compliance checks.
