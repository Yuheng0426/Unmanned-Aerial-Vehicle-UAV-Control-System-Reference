# Architecture

This project splits the flight-control reference into small modules that are easy to inspect.

## `PidController`

Converts target-vs-actual error into a bounded control output. The current implementation includes proportional, integral, and derivative terms, and clamps both the integral term and final output.

## `FlightController`

The main control loop. Each `update` call:

1. Checks whether battery voltage and attitude allow flight.
2. Decides whether the vehicle may arm.
3. Runs PID controllers for roll, pitch, yaw rate, and altitude.
4. Sends the control outputs to the mixer to produce four motor commands.

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

A minimal simulation model used only to observe closed-loop behavior. It does not include real aerodynamics, propeller models, IMU noise, sensor delay, or wind disturbance.

## Program Entry Points

- The default mode runs `run_demo` and prints CSV-style state output.
- Passing `--self-test` runs `run_self_test`.
