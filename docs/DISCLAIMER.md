# Disclaimer

This repository is an educational software reference. It is not a certified autopilot, not production flight firmware, not an airworthiness approval, and not legal permission to fly any UAV.

## Operator Responsibility

Anyone who builds, modifies, tests, or flies a drone using ideas from this repository is responsible for:

- Understanding the code and its limits.
- Complying with all local laws, aviation rules, registration requirements, altitude limits, and airspace restrictions.
- Keeping the locked altitude limit intact and using lower limits where local rules require them.
- Treating preflight checks, sensor-health warnings, battery failsafes, and mission validation as minimum safety gates rather than optional suggestions.
- Testing in simulation before hardware tests.
- Testing with propellers removed before any powered propeller test.
- Using safe locations away from people, vehicles, buildings, roads, airports, restricted areas, and power lines.
- Designing independent emergency stop and power-cut mechanisms.
- Accepting responsibility for property damage, injury, flyaway events, legal violations, or other consequences.

## Not Flight-Ready

This project does not include many systems required by a real aircraft, including:

- Certified sensor drivers
- Robust attitude and position estimation
- Redundant failsafe channels
- Hardware watchdogs
- Motor and ESC validation
- Radio-link validation
- GPS fault handling
- Magnetometer calibration
- Flight logging suitable for incident review
- Regulatory compliance validation
- Certified autonomous obstacle avoidance or path planning

## Safe Use

Use this project to study control-system structure, safety-state design, and simulator behavior. Do not connect it directly to motors or flight hardware without a complete independent engineering review.

If a drone made from ideas in this repository flies away, enters unsafe airspace, exceeds legal limits, damages property, or injures anyone, responsibility remains with the builder and operator.
