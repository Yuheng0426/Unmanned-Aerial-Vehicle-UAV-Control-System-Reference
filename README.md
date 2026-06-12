# UAVcontrol_system

## Read This First: Safety and Legal Notice

This repository is a **simulation-only educational reference**. It is not flight-ready firmware, not an aircraft certification package, not a product, and not permission to fly a UAV. Any builder or operator is responsible for local aviation laws, registration, airspace restrictions, safe test locations, independent emergency stop design, and all consequences of the aircraft's behavior.

The reference controller includes a locked legal altitude limit of `120.0 m` as a conservative software boundary. Do not raise, bypass, disable, or weaken this limit. Some locations require lower limits, and local law always takes priority. If anyone changes the altitude limit or uses the project in violation of law or safety practice, they accept full responsibility for the outcome.

Read [docs/DISCLAIMER.md](docs/DISCLAIMER.md) and [docs/ALTITUDE_LIMIT_POLICY.md](docs/ALTITUDE_LIMIT_POLICY.md) before studying or adapting the code.

A C++20 educational UAV flight-control reference project for drone DIY learners, robotics students, and developers who want to study the structure of a safety-focused control loop. The current version is a **simulation-only teaching system** with PID stabilization, one-key return-to-launch, obstacle avoidance, flight modes, geofence logic, battery failsafes, command-link failsafe handling, event logging, and conservative command limiting.

> Safety disclaimer: this repository is not flight-ready firmware, not an aircraft certification package, and not permission to fly a UAV. It does not directly drive ESCs, motors, GPS modules, radio receivers, or real sensors. Anyone adapting this reference to hardware is responsible for simulation, bench testing, propeller-off testing, legal compliance, local airspace rules, safe operating areas, and all consequences of the aircraft's behavior.

## Features

- C++20 and CMake, with a compact codebase intended for study and modification.
- Quad X-frame motor mixer with motor outputs clamped to `0.0` through `1.0`.
- PID controllers for roll, pitch, yaw rate, altitude, and local-position hold.
- Flight modes: manual, altitude hold, position hold, return-to-launch, and land.
- One-key return-to-launch command for immediate return behavior when position is valid.
- Reactive obstacle avoidance from front, back, left, right, up, and down distance readings.
- Battery monitoring with low-battery warning, critical-battery forced return, very-low-battery forced landing, and emergency cutoff.
- Safety systems: conservative arming checks, emergency stop, command-link timeout handling, geofence return, hard geofence disarm, and locked altitude limiting.
- Built-in event log for mode transitions and safety actions.
- Built-in `--self-test` mode for quick logic checks.
- No third-party runtime libraries.

## Project Layout

```text
.
|-- UAVcontrol_system/
|   |-- CMakeLists.txt
|   |-- UAVcontrol_system.cpp
|   `-- UAVcontrol_system.h
|-- docs/
|   |-- ARCHITECTURE.md
|   |-- ALTITUDE_LIMIT_POLICY.md
|   |-- DEPENDENCIES_AND_PLUGINS.md
|   |-- DISCLAIMER.md
|   `-- HARDWARE_ADAPTATION.md
|-- .github/workflows/cmake.yml
|-- .gitignore
|-- CMakeLists.txt
|-- CMakePresets.json
|-- CONTRIBUTING.md
|-- LICENSE
`-- SECURITY.md
```

## Build

Requirements:

- CMake 3.16 or newer
- A C++20-capable compiler such as MSVC, Clang, or GCC
- Optional: Ninja

Generic build commands:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Visual Studio / MSVC users can also use the included preset:

```powershell
cmake --preset x64-debug
cmake --build out/build/x64-debug
ctest --test-dir out/build/x64-debug --output-on-failure
```

## Run

Run the simulation demo:

```bash
./build/UAVcontrol_system/UAVcontrol_system
```

Run the self-test:

```bash
./build/UAVcontrol_system/UAVcontrol_system --self-test
```

On Windows, the executable is usually located at:

```text
build\UAVcontrol_system\UAVcontrol_system.exe
```

## How to Study the Code

- Start with `FlightController::update` to see the full control cycle.
- Read `SafetyConfig` to understand the conservative safety boundaries.
- Read `apply_failsafe_policy` for one-key return, command timeout, battery failsafes, geofence, and altitude behavior.
- Read `apply_obstacle_avoidance` to see the simple reactive avoidance layer.
- Read `QuadXMixer` if you want to adapt the project to another airframe layout.
- Read [docs/DISCLAIMER.md](docs/DISCLAIMER.md) and [docs/HARDWARE_ADAPTATION.md](docs/HARDWARE_ADAPTATION.md) before thinking about real hardware.

## Safety Boundary

This repository provides an educational control-structure example only. A real UAV needs sensor calibration, attitude estimation, link-loss handling, geofencing, logging, hardware redundancy, electromagnetic-interference testing, regulatory compliance checks, and extensive flight validation.

Do not use this project directly for crewed aircraft, flights near people, unsafe payloads, restricted areas, or any operation that is not legal in your location. If a drone built from ideas in this repository flies away, damages property, injures anyone, violates airspace rules, or behaves unpredictably, that responsibility remains with the builder and operator.

## Plugins and Third-Party Dependencies

No plugins were downloaded while preparing this update, and no third-party code libraries were added. Tool links and dependency notes are documented in [docs/DEPENDENCIES_AND_PLUGINS.md](docs/DEPENDENCIES_AND_PLUGINS.md).

## License

This project is released under the MIT License. See [LICENSE](LICENSE).
