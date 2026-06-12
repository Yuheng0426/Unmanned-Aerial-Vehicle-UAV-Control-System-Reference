# Security and Safety Policy

This project is for educational and research reference. It does not provide complete firmware that can be deployed directly to a real UAV.

## Reporting Issues

If you find a defect that could cause loss of control, bypass safety limits, arm incorrectly, or produce unsafe motor outputs, please open a GitHub issue with:

- Reproduction steps
- Expected behavior
- Actual behavior
- Compiler, operating system, and run command
- Whether the issue affects real-hardware adaptation

## Safety Principles

- Keep limits conservative by default.
- Prefer shutting down output when safety checks fail.
- Real-hardware adaptation must pass simulation, bench tests, propeller-off tests, and controlled low-power tests.
- Contributions that bypass laws, regulations, geofencing, or safety protections are not accepted.
