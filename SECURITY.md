# Security and Safety Policy

This project is for educational and research reference. It does not provide complete firmware that can be deployed directly to a real UAV.

## Reporting Issues

If you find a defect that could cause loss of control, bypass safety limits, arm incorrectly, ignore emergency stop, ignore geofence limits, ignore the locked altitude limit, ignore forced battery return, ignore obstacle avoidance, or produce unsafe motor outputs, please open a GitHub issue with:

- Reproduction steps
- Expected behavior
- Actual behavior
- Compiler, operating system, and run command
- Whether the issue affects real-hardware adaptation

## Safety Principles

- Keep limits conservative by default.
- Prefer return-to-launch or landing for soft failsafes.
- Prefer shutting down output when hard safety checks fail.
- Treat stale commands, invalid position estimates, low voltage, and geofence breaches as safety events.
- Treat altitude-limit bypasses as safety defects.
- Treat battery thresholds and one-key return behavior as safety-critical logic.
- Treat preflight gates, mission validation, and telemetry visibility as safety-critical support logic.
- Real-hardware adaptation must pass simulation, bench tests, propeller-off tests, and controlled low-power tests.
- Contributions that bypass laws, regulations, geofencing, or safety protections are not accepted.

## Responsible Scope

This repository must not be used to support unsafe flights, flights near people, illegal airspace operations, weaponization, harmful payloads, or attempts to evade safety systems.
