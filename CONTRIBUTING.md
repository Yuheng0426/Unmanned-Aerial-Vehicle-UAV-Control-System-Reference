# Contributing

Contributions are welcome, especially documentation, tests, simulation improvements, and mixer examples for different airframes.

## Development Flow

1. Create a branch.
2. Change code or documentation.
3. Run the self-test.
4. Open a pull request that explains the motivation, test results, and safety impact.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

## Code Style

- Add short English comments above important classes, functions, and safety logic.
- Do not describe unvalidated real-hardware control code as flight-ready.
- Do not raise, bypass, disable, or weaken `SafetyConfig::locked_legal_altitude_limit_m`.
- When adding an external library, update `docs/DEPENDENCIES_AND_PLUGINS.md` with its download link, purpose, and license.

## Out of Scope

- Features that bypass laws, geofencing, regulatory limits, or safety protections.
- Changes that weaken the locked altitude limit or hide the legal disclaimer.
- Payload, evasion, or tracking-avoidance functionality intended for harmful use.
- Real-flight control changes without clear risk documentation.
