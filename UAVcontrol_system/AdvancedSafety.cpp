#include "AdvancedSafety.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace {

double clamp_local(double value, double low, double high)
{
	return std::max(low, std::min(value, high));
}

double distance_between_local(const uav::LocalPosition& a, const uav::LocalPosition& b)
{
	const double north = a.north_m - b.north_m;
	const double east = a.east_m - b.east_m;
	return std::sqrt(north * north + east * east);
}

bool range_is_invalid(double value_m)
{
	return value_m < 0.0 || value_m > 1000.0;
}

}  // namespace

namespace uav {

std::string to_string(HealthState state)
{
	switch (state) {
	case HealthState::Ok:
		return "ok";
	case HealthState::Warning:
		return "warning";
	case HealthState::Critical:
		return "critical";
	}

	return "unknown";
}

PreflightChecker::PreflightChecker(SafetyConfig safety)
	: safety_(safety)
{
}

// Build a conservative go/no-go report before allowing a simulated arm request.
PreflightReport PreflightChecker::evaluate(const SensorSample& sample, const ControlCommand& command) const
{
	PreflightReport report;
	report.ready_to_arm = true;

	if (!command.arm_requested) {
		report.ready_to_arm = false;
		report.findings.push_back("arm request is not active");
	}

	if (command.emergency_stop) {
		report.ready_to_arm = false;
		report.findings.push_back("emergency stop is active");
	}

	if (command.command_age_ms > safety_.max_command_age_ms) {
		report.ready_to_arm = false;
		report.findings.push_back("command link is stale");
	}

	if (sample.battery_voltage_v < safety_.min_arm_voltage_v) {
		report.ready_to_arm = false;
		report.findings.push_back("battery voltage is below arming threshold");
	} else if (sample.battery_voltage_v < safety_.battery_warning_voltage_v) {
		report.findings.push_back("battery is low; do not begin a new flight");
	}

	if (std::abs(sample.attitude_deg.x) > safety_.max_arm_tilt_deg ||
		std::abs(sample.attitude_deg.y) > safety_.max_arm_tilt_deg) {
		report.ready_to_arm = false;
		report.findings.push_back("airframe is not level enough to arm");
	}

	if (!sample.position_valid &&
		(command.requested_mode == FlightMode::PositionHold || command.requested_mode == FlightMode::ReturnToLaunch)) {
		report.ready_to_arm = false;
		report.findings.push_back("position estimate is required for requested mode");
	}

	if (sample.altitude_m > 1.0) {
		report.ready_to_arm = false;
		report.findings.push_back("vehicle appears airborne during preflight");
	}

	if (report.findings.empty()) {
		report.findings.push_back("preflight checks passed for simulation");
	}

	return report;
}

SensorHealthMonitor::SensorHealthMonitor(SafetyConfig safety)
	: safety_(safety)
{
}

HealthState SensorHealthMonitor::navigation_health(const SensorSample& sample) const
{
	if (!sample.position_valid) {
		return HealthState::Critical;
	}

	if (std::abs(sample.attitude_deg.x) > safety_.max_flight_tilt_deg * 0.8 ||
		std::abs(sample.attitude_deg.y) > safety_.max_flight_tilt_deg * 0.8) {
		return HealthState::Warning;
	}

	return HealthState::Ok;
}

HealthState SensorHealthMonitor::obstacle_health(const SensorSample& sample) const
{
	if (range_is_invalid(sample.obstacle_m.front_m) || range_is_invalid(sample.obstacle_m.back_m) ||
		range_is_invalid(sample.obstacle_m.left_m) || range_is_invalid(sample.obstacle_m.right_m) ||
		range_is_invalid(sample.obstacle_m.up_m) || range_is_invalid(sample.obstacle_m.down_m)) {
		return HealthState::Critical;
	}

	if (sample.obstacle_m.front_m < safety_.obstacle_warning_distance_m ||
		sample.obstacle_m.back_m < safety_.obstacle_warning_distance_m ||
		sample.obstacle_m.left_m < safety_.obstacle_warning_distance_m ||
		sample.obstacle_m.right_m < safety_.obstacle_warning_distance_m ||
		sample.obstacle_m.up_m < safety_.obstacle_stop_distance_m ||
		sample.obstacle_m.down_m < safety_.obstacle_stop_distance_m) {
		return HealthState::Warning;
	}

	return HealthState::Ok;
}

HealthState SensorHealthMonitor::battery_health(const SensorSample& sample) const
{
	if (sample.battery_voltage_v < safety_.battery_land_voltage_v) {
		return HealthState::Critical;
	}

	if (sample.battery_voltage_v < safety_.battery_warning_voltage_v) {
		return HealthState::Warning;
	}

	return HealthState::Ok;
}

MissionPlanner::MissionPlanner(SafetyConfig safety)
	: safety_(safety)
{
}

// Validate that every waypoint stays inside the geofence and locked altitude boundary.
MissionValidation MissionPlanner::validate(const std::vector<MissionWaypoint>& mission, const LocalPosition& home_m) const
{
	MissionValidation result;
	result.accepted = true;

	if (mission.empty()) {
		result.accepted = false;
		result.messages.push_back("mission has no waypoints");
		return result;
	}

	for (std::size_t i = 0; i < mission.size(); ++i) {
		const MissionWaypoint& waypoint = mission[i];
		if (waypoint.altitude_m < 0.0 || waypoint.altitude_m > SafetyConfig::locked_legal_altitude_limit_m) {
			result.accepted = false;
			result.messages.push_back("waypoint " + std::to_string(i) + " violates locked altitude limit");
		}

		if (distance_between_local(waypoint.position_m, home_m) > safety_.geofence_radius_m) {
			result.accepted = false;
			result.messages.push_back("waypoint " + std::to_string(i) + " is outside the geofence");
		}
	}

	if (result.messages.empty()) {
		result.messages.push_back("mission accepted for simulation");
	}

	return result;
}

ControlCommand MissionPlanner::command_for_waypoint(const MissionWaypoint& waypoint, bool arm_requested) const
{
	ControlCommand command;
	command.requested_mode = FlightMode::PositionHold;
	command.target_position_m = waypoint.position_m;
	command.target_altitude_m = clamp_local(waypoint.altitude_m, 0.0, SafetyConfig::locked_legal_altitude_limit_m);
	command.arm_requested = arm_requested;
	command.command_age_ms = 20;
	return command;
}

TelemetryEncoder::TelemetryEncoder(SafetyConfig safety)
	: health_(safety)
{
}

TelemetrySnapshot TelemetryEncoder::snapshot(
	const SensorSample& sample,
	const FlightController& controller,
	const LocalPosition& home_m) const
{
	TelemetrySnapshot frame;
	frame.mode = controller.mode();
	frame.position_m = sample.position_m;
	frame.altitude_m = sample.altitude_m;
	frame.battery_voltage_v = sample.battery_voltage_v;
	frame.distance_to_home_m = distance_between_local(sample.position_m, home_m);
	frame.battery_health = health_.battery_health(sample);
	frame.navigation_health = health_.navigation_health(sample);
	frame.safety_message = controller.last_safety_message();
	return frame;
}

std::string TelemetryEncoder::to_csv_header() const
{
	return "mode,north_m,east_m,altitude_m,battery_v,distance_home_m,battery_health,navigation_health,safety_message";
}

std::string TelemetryEncoder::to_csv(const TelemetrySnapshot& snapshot) const
{
	std::ostringstream out;
	out << std::fixed << std::setprecision(2)
		<< to_string(snapshot.mode) << ','
		<< snapshot.position_m.north_m << ','
		<< snapshot.position_m.east_m << ','
		<< snapshot.altitude_m << ','
		<< snapshot.battery_voltage_v << ','
		<< snapshot.distance_to_home_m << ','
		<< to_string(snapshot.battery_health) << ','
		<< to_string(snapshot.navigation_health) << ','
		<< snapshot.safety_message;
	return out.str();
}

}  // namespace uav
