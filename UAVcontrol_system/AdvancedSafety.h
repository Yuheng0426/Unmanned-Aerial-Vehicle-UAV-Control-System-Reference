#pragma once

#include "UAVcontrol_system.h"

#include <cstddef>
#include <string>
#include <vector>

namespace uav {

// Health state used by preflight checks and telemetry summaries.
enum class HealthState {
	Ok,
	Warning,
	Critical,
};

// Mission waypoint in local coordinates. Altitude is clamped by the locked altitude policy.
struct MissionWaypoint {
	LocalPosition position_m{};
	double altitude_m{};
	int hold_time_ms{};
};

// Mission validation result that explains why a mission is accepted or rejected.
struct MissionValidation {
	bool accepted{};
	std::vector<std::string> messages{};
};

// Compact telemetry frame suitable for logs, dashboards, or simulator output.
struct TelemetrySnapshot {
	FlightMode mode{FlightMode::AltitudeHold};
	LocalPosition position_m{};
	double altitude_m{};
	double battery_voltage_v{};
	double distance_to_home_m{};
	HealthState battery_health{HealthState::Ok};
	HealthState navigation_health{HealthState::Ok};
	std::string safety_message{};
};

// Preflight report with a final go/no-go decision and human-readable findings.
struct PreflightReport {
	bool ready_to_arm{};
	std::vector<std::string> findings{};
};

// Evaluates sensor, battery, link, geofence, and altitude conditions before arming.
class PreflightChecker {
public:
	explicit PreflightChecker(SafetyConfig safety = {});

	PreflightReport evaluate(const SensorSample& sample, const ControlCommand& command) const;

private:
	SafetyConfig safety_;
};

// Monitors estimator and obstacle-sensor health for a conservative safety summary.
class SensorHealthMonitor {
public:
	explicit SensorHealthMonitor(SafetyConfig safety = {});

	HealthState navigation_health(const SensorSample& sample) const;
	HealthState obstacle_health(const SensorSample& sample) const;
	HealthState battery_health(const SensorSample& sample) const;

private:
	SafetyConfig safety_;
};

// Creates bounded waypoint commands for simulation-only mission demonstrations.
class MissionPlanner {
public:
	explicit MissionPlanner(SafetyConfig safety = {});

	MissionValidation validate(const std::vector<MissionWaypoint>& mission, const LocalPosition& home_m) const;
	ControlCommand command_for_waypoint(const MissionWaypoint& waypoint, bool arm_requested) const;

private:
	SafetyConfig safety_;
};

// Builds concise safety telemetry without exposing hardware-control behavior.
class TelemetryEncoder {
public:
	explicit TelemetryEncoder(SafetyConfig safety = {});

	TelemetrySnapshot snapshot(
		const SensorSample& sample,
		const FlightController& controller,
		const LocalPosition& home_m) const;
	std::string to_csv_header() const;
	std::string to_csv(const TelemetrySnapshot& snapshot) const;

private:
	SensorHealthMonitor health_;
};

std::string to_string(HealthState state);

}  // namespace uav
