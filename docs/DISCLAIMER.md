# Disclaimer

This repository is an educational software reference. It is not a certified autopilot, not production flight firmware, not an airworthiness approval, and not legal permission to fly any UAV.

## Operator Responsibility

Anyone who builds, modifies, tests, or flies a drone using ideas from this repository is responsible for:

- Understanding the code and its limits.
- Complying with all local laws, aviation rules, registration requirements, and airspace restrictions.
- Testing in simulation before hardware tests.
- Testing with propellers removed before any powered propeller test.
- Using safe locations away from people, animals, vehicles, buildings, roads, airports, restricted areas, and power lines.
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

## Safe Use

Use this project to study control-system structure, safety-state design, and simulator behavior. Do not connect it directly to motors or flight hardware without a complete independent engineering review.
