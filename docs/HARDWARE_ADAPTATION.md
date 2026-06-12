# Hardware Adaptation Guide

This project is currently a simulation reference. Validate each layer independently before adapting it to real hardware.

## Parts That Must Be Replaced

- `DroneSimulator::sensors`: replace with real IMU, barometer, battery-voltage, and other sensor readings.
- `DroneSimulator::apply_motors`: replace with real PWM, DShot, CAN, or other ESC output.
- `ControlCommand` source: replace with radio-controller, ground-station, or companion-computer input.
- Fixed `dt_s`: replace with elapsed time from a monotonic clock.

## Recommended Additions

- Sensor calibration and bias estimation.
- Attitude estimation, such as a complementary filter or extended Kalman filter.
- Radio link-loss failsafe.
- Black-box logging.
- Independent hardware emergency stop.
- Propeller-off test mode.
- Ground-station parameter save and restore.

## Real-Hardware Test Order

1. Build and run the self-test.
2. Validate in simulation.
3. Validate on a bench with motor output disconnected.
4. Verify motor direction with propellers removed.
5. Run restrained low-power tests.
6. Attempt low-altitude hover only in an open, legal, controlled area.

## Risk Notice

Real UAVs involve fast rotating parts, lithium batteries, high current, electromagnetic interference, and legal compliance. Hardware adaptation should never skip safety checks, propeller-off testing, or emergency-stop design.
