# Hardware Adaptation Guide

This project is currently a simulation reference. Validate each layer independently before adapting it to real hardware.

## Parts That Must Be Replaced

- `DroneSimulator::sensors`: replace with estimator outputs from real IMU, barometer, battery-voltage, GPS, optical-flow, or other sensors.
- `DroneSimulator::apply_motors`: replace with a carefully tested PWM, DShot, CAN, or other ESC output layer.
- `ControlCommand` source: replace with radio-controller, ground-station, or companion-computer input that has authentication and timeout handling.
- `ObstacleDistances`: replace with validated range sensors, perception, or a tested obstacle-detection stack.
- `PreflightChecker`: keep or strengthen preflight gates before any hardware arming path.
- `MissionPlanner`: keep mission validation ahead of any autonomous waypoint command path.
- `TelemetryEncoder`: route safety telemetry to logs and operator-visible displays.
- Fixed `dt_s`: replace with elapsed time from a monotonic clock.
- Local position model: replace with a validated estimator and coordinate frame.

## Required Safety Work Before Real Flight

- Independent emergency stop that does not depend on the main control loop.
- Propeller-off motor direction and failsafe testing.
- Radio link-loss testing.
- Battery sag and brownout testing.
- Battery warning, forced-return, forced-landing, and emergency-cutoff testing.
- Obstacle sensor failure testing.
- Watchdog timer and stuck-loop recovery.
- Sensor calibration and bias estimation.
- Attitude estimation, such as a complementary filter or extended Kalman filter.
- Logging suitable for reviewing unsafe behavior.
- Ground-station parameter save and restore with validation.
- Geofence and altitude limits that match local regulations.
- A lower altitude limit than the locked reference limit if local law requires it.

## Real-Hardware Test Order

1. Build and run the self-test.
2. Validate in simulation.
3. Validate on a bench with motor output disconnected.
4. Validate command timeout, emergency stop, and low-voltage behavior.
5. Validate one-key return-to-launch and forced battery return in simulation.
6. Validate obstacle sensor behavior without motors connected.
7. Validate preflight rejection cases and mission rejection cases.
8. Verify motor direction with propellers removed.
9. Run restrained low-power tests.
10. Attempt low-altitude hover only in an open, legal, controlled area.

## Risk Notice

Real UAVs involve fast rotating parts, lithium batteries, high current, electromagnetic interference, unpredictable environments, and legal compliance. Hardware adaptation should never skip safety checks, propeller-off testing, emergency-stop design, or local airspace review.

Do not raise, bypass, disable, or weaken the locked altitude limit. If local law requires a lower limit, use the lower limit.
