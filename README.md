# UAVcontrol_system

A C++20 educational flight-control reference project for UAV and drone DIY learners. The current version is a **teaching simulator**: it demonstrates common control-system building blocks such as sensor sampling, PID control, quadcopter X-frame motor mixing, safe arming, low-voltage protection, and failsafe disarming.

> Important: this project is not flight-ready firmware. It does not directly drive ESCs, motors, GPS modules, radio receivers, or real sensors. Validate every change in simulation, on a bench, and in a controlled environment before adapting it to hardware.

## Features

- C++20 and CMake, with a small codebase that is easy to read and modify.
- Example quadcopter X-frame mixer with motor outputs clamped to `0.0` through `1.0`.
- PID controller examples for roll, pitch, yaw rate, and altitude.
- Safety examples for pre-arm attitude checks, low-voltage arming rejection, and in-flight failsafe disarming.
- Built-in `--self-test` mode for quick logic checks.
- No third-party runtime libraries.

## Project Layout

```text
.
├── UAVcontrol_system/
│   ├── CMakeLists.txt
│   ├── UAVcontrol_system.cpp
│   └── UAVcontrol_system.h
├── docs/
│   ├── ARCHITECTURE.md
│   ├── DEPENDENCIES_AND_PLUGINS.md
│   └── HARDWARE_ADAPTATION.md
├── .github/workflows/cmake.yml
├── .gitignore
├── CMakeLists.txt
├── CMakePresets.json
├── CONTRIBUTING.md
├── LICENSE
└── SECURITY.md
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

- Start with `FlightController::update` to see how the control loop is organized.
- Read `PidController` to understand the bounded PID implementation.
- Read `QuadXMixer` if you want to adapt the project to another airframe layout.
- Read [docs/HARDWARE_ADAPTATION.md](docs/HARDWARE_ADAPTATION.md) before connecting the design to real hardware.

## Safety Boundary

This repository provides an educational control-structure example only. A real UAV needs sensor calibration, attitude estimation, link-loss handling, geofencing, logging, hardware redundancy, electromagnetic-interference testing, regulatory compliance checks, and extensive flight validation.

Do not use this project directly for crewed aircraft, flights near people, unsafe payloads, restricted areas, or any operation that is not legal in your location.

## Plugins and Third-Party Dependencies

No plugins were downloaded while preparing this repository, and no third-party code libraries were added. Tool links and dependency notes are documented in [docs/DEPENDENCIES_AND_PLUGINS.md](docs/DEPENDENCIES_AND_PLUGINS.md).

## License

This project is released under the MIT License. See [LICENSE](LICENSE).
