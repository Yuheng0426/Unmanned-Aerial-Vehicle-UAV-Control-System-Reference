#include "UAVcontrol_system.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string_view>

namespace {

// Clamp values so control outputs and simulated state stay bounded.
double clamp(double value, double low, double high)
{
	return std::max(low, std::min(value, high));
}

// Compare floating-point values with a small tolerance for lightweight self-tests.
bool near(double actual, double expected, double tolerance)
{
	return std::abs(actual - expected) <= tolerance;
}

// Compute planar distance in meters.
double distance_between(const uav::LocalPosition& a, const uav::LocalPosition& b)
{
	const double north = a.north_m - b.north_m;
	const double east = a.east_m - b.east_m;
	return std::sqrt(north * north + east * east);
}

// Convert a bounded horizontal acceleration request into a small attitude command.
double acceleration_to_attitude_deg(double acceleration_m_s2)
{
	return clamp(acceleration_m_s2 * 3.0, -8.0, 8.0);
}

}  // namespace

namespace uav {

std::string to_string(FlightMode mode)
{
	switch (mode) {
	case FlightMode::Manual:
		return "manual";
	case FlightMode::AltitudeHold:
		return "altitude_hold";
	case FlightMode::PositionHold:
		return "position_hold";
	case FlightMode::ReturnToLaunch:
		return "return_to_launch";
	case FlightMode::Land:
		return "land";
	}

	return "unknown";
}

PidController::PidController(double kp, double ki, double kd, double output_limit)
	: kp_(kp), ki_(ki), kd_(kd), output_limit_(output_limit)
{
}

// Compute a bounded PID output and clamp the integral term to reduce windup.
double PidController::update(double target, double actual, double dt_s)
{
	if (dt_s <= 0.0) {
		return 0.0;
	}

	const double error = target - actual;
	integral_ = clamp(integral_ + error * dt_s, -output_limit_, output_limit_);

	const double derivative = has_previous_error_ ? (error - previous_error_) / dt_s : 0.0;
	previous_error_ = error;
	has_previous_error_ = true;

	const double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
	return clamp(output, -output_limit_, output_limit_);
}

// Clear PID state, usually when arming, disarming, or changing modes.
void PidController::reset()
{
	integral_ = 0.0;
	previous_error_ = 0.0;
	has_previous_error_ = false;
}

EventLog::EventLog(std::size_t capacity)
	: capacity_(capacity)
{
}

// Keep the most recent safety and mode events without growing memory forever.
void EventLog::push(const std::string& message)
{
	if (entries_.size() == capacity_) {
		entries_.erase(entries_.begin());
	}

	entries_.push_back(message);
}

const std::vector<std::string>& EventLog::entries() const
{
	return entries_;
}

// Quad X mixer. Output order: front-left, front-right, rear-right, rear-left.
QuadXMixer::MotorOutputs QuadXMixer::mix(double throttle, double roll, double pitch, double yaw) const
{
	MotorOutputs motors{
		throttle + pitch + roll - yaw,
		throttle + pitch - roll + yaw,
		throttle - pitch - roll - yaw,
		throttle - pitch + roll + yaw,
	};

	for (double& motor : motors) {
		motor = clamp(motor, 0.0, 1.0);
	}

	return motors;
}

FlightController::FlightController(SafetyConfig safety)
	: safety_(safety),
	  roll_pid_(0.055, 0.010, 0.018, 0.25),
	  pitch_pid_(0.055, 0.010, 0.018, 0.25),
	  yaw_rate_pid_(0.018, 0.002, 0.000, 0.18),
	  altitude_pid_(0.220, 0.040, 0.120, 0.35),
	  north_pid_(0.080, 0.000, 0.030, 1.20),
	  east_pid_(0.080, 0.000, 0.030, 1.20)
{
}

// One control-cycle step: validate the command, apply failsafes, compute mode setpoints, and mix motors.
QuadXMixer::MotorOutputs FlightController::update(const SensorSample& sample, const ControlCommand& command, double dt_s)
{
	if (!safety_allows_flight(sample, command)) {
		return {};
	}

	ControlCommand safe_command = apply_failsafe_policy(sample, command);
	safe_command = constrain_command(sample, safe_command);

	if (!armed_ && should_arm(sample, safe_command)) {
		roll_pid_.reset();
		pitch_pid_.reset();
		yaw_rate_pid_.reset();
		altitude_pid_.reset();
		north_pid_.reset();
		east_pid_.reset();
		armed_ = true;
		home_set_ = sample.position_valid;
		home_position_m_ = home_or_current_position(sample);
		transition_to(safe_command.requested_mode, "armed");
		last_safety_message_ = "armed";
	}

	if (!armed_) {
		last_safety_message_ = "waiting for safe arming";
		return {};
	}

	transition_to(safe_command.requested_mode, "commanded");

	double roll_target_deg = safe_command.target_roll_deg;
	double pitch_target_deg = safe_command.target_pitch_deg;
	double altitude_target_m = safe_command.target_altitude_m;

	if (active_mode_ == FlightMode::PositionHold || active_mode_ == FlightMode::ReturnToLaunch) {
		const LocalPosition target_position =
			active_mode_ == FlightMode::ReturnToLaunch ? home_or_current_position(sample) : safe_command.target_position_m;
		const double north_accel = north_pid_.update(target_position.north_m, sample.position_m.north_m, dt_s);
		const double east_accel = east_pid_.update(target_position.east_m, sample.position_m.east_m, dt_s);
		pitch_target_deg = acceleration_to_attitude_deg(north_accel);
		roll_target_deg = -acceleration_to_attitude_deg(east_accel);

		if (active_mode_ == FlightMode::ReturnToLaunch && distance_from_home(sample) < 1.5) {
			safe_command.requested_mode = FlightMode::Land;
			transition_to(FlightMode::Land, "home reached");
		}
	}

	if (active_mode_ == FlightMode::Land) {
		altitude_target_m = std::max(0.0, sample.altitude_m - 0.25);
		if (sample.altitude_m < 0.08 && std::abs(sample.vertical_speed_m_s) < 0.20) {
			disarm("landed");
			return {};
		}
	}

	const double roll = roll_pid_.update(roll_target_deg, sample.attitude_deg.x, dt_s);
	const double pitch = pitch_pid_.update(pitch_target_deg, sample.attitude_deg.y, dt_s);
	const double yaw = yaw_rate_pid_.update(safe_command.target_yaw_rate_deg_s, sample.angular_rate_deg_s.z, dt_s);
	const double altitude = altitude_pid_.update(altitude_target_m, sample.altitude_m, dt_s);

	const double hover_throttle = 0.50;
	const double throttle = active_mode_ == FlightMode::Manual
		? clamp(hover_throttle + safe_command.target_altitude_m * 0.10, 0.0, 0.75)
		: clamp(hover_throttle + altitude, 0.0, 0.85);

	return mixer_.mix(throttle, roll, pitch, yaw);
}

// Disarming immediately stops output and resets every controller.
void FlightController::disarm(const std::string& reason)
{
	if (armed_) {
		event_log_.push("disarmed: " + reason);
	}

	armed_ = false;
	roll_pid_.reset();
	pitch_pid_.reset();
	yaw_rate_pid_.reset();
	altitude_pid_.reset();
	north_pid_.reset();
	east_pid_.reset();
	last_safety_message_ = "disarmed: " + reason;
}

bool FlightController::armed() const
{
	return armed_;
}

FlightMode FlightController::mode() const
{
	return active_mode_;
}

std::string FlightController::last_safety_message() const
{
	return last_safety_message_;
}

const std::vector<std::string>& FlightController::events() const
{
	return event_log_.entries();
}

// Arming is intentionally conservative: level attitude, healthy battery, fresh command, and valid position.
bool FlightController::should_arm(const SensorSample& sample, const ControlCommand& command)
{
	if (!command.arm_requested) {
		return false;
	}

	if (command.emergency_stop) {
		last_safety_message_ = "arming rejected: emergency stop is active";
		return false;
	}

	if (command.command_age_ms > safety_.max_command_age_ms) {
		last_safety_message_ = "arming rejected: command link is stale";
		return false;
	}

	if (std::abs(sample.attitude_deg.x) > safety_.max_arm_tilt_deg ||
		std::abs(sample.attitude_deg.y) > safety_.max_arm_tilt_deg) {
		last_safety_message_ = "arming rejected: excessive tilt";
		return false;
	}

	if (sample.battery_voltage_v < safety_.min_arm_voltage_v) {
		last_safety_message_ = "arming rejected: battery voltage too low";
		return false;
	}

	if (!sample.position_valid &&
		(command.requested_mode == FlightMode::PositionHold || command.requested_mode == FlightMode::ReturnToLaunch)) {
		last_safety_message_ = "arming rejected: position estimate required";
		return false;
	}

	return true;
}

// Hard safety checks disarm the controller when continuing output would be unsafe.
bool FlightController::safety_allows_flight(const SensorSample& sample, const ControlCommand& command)
{
	if (command.emergency_stop) {
		disarm("emergency stop");
		return false;
	}

	if (sample.battery_voltage_v < safety_.min_flight_voltage_v) {
		disarm("battery voltage too low");
		return false;
	}

	if (std::abs(sample.attitude_deg.x) > safety_.max_flight_tilt_deg ||
		std::abs(sample.attitude_deg.y) > safety_.max_flight_tilt_deg) {
		disarm("attitude limit exceeded");
		return false;
	}

	if (sample.altitude_m > safety_.max_altitude_m + 2.0) {
		disarm("altitude limit exceeded");
		return false;
	}

	if (home_set_ && distance_from_home(sample) > safety_.geofence_radius_m + 5.0) {
		disarm("geofence breached");
		return false;
	}

	return true;
}

// Soft failsafes prefer return-to-launch or landing before a hard shutdown is needed.
ControlCommand FlightController::apply_failsafe_policy(const SensorSample& sample, const ControlCommand& command)
{
	ControlCommand safe_command = command;

	if (safe_command.command_age_ms > safety_.max_command_age_ms) {
		if (sample.position_valid && home_set_) {
			safe_command.requested_mode = FlightMode::ReturnToLaunch;
			last_safety_message_ = "failsafe: stale command, returning to launch";
		} else {
			safe_command.requested_mode = FlightMode::Land;
			last_safety_message_ = "failsafe: stale command, landing";
		}
	}

	if (home_set_ && distance_from_home(sample) > safety_.geofence_radius_m) {
		safe_command.requested_mode = FlightMode::ReturnToLaunch;
		last_safety_message_ = "failsafe: geofence return";
	}

	if (sample.altitude_m > safety_.max_altitude_m) {
		safe_command.requested_mode = FlightMode::Land;
		last_safety_message_ = "failsafe: altitude limit landing";
	}

	return safe_command;
}

// Clamp user commands to conservative limits before they reach the controller.
ControlCommand FlightController::constrain_command(const SensorSample& sample, const ControlCommand& command)
{
	ControlCommand safe_command = command;
	safe_command.target_roll_deg = clamp(safe_command.target_roll_deg, -12.0, 12.0);
	safe_command.target_pitch_deg = clamp(safe_command.target_pitch_deg, -12.0, 12.0);
	safe_command.target_yaw_rate_deg_s = clamp(safe_command.target_yaw_rate_deg_s, -90.0, 90.0);
	safe_command.target_altitude_m = clamp(safe_command.target_altitude_m, 0.0, safety_.max_altitude_m);

	if (home_set_) {
		const double distance = distance_between(safe_command.target_position_m, home_position_m_);
		if (distance > safety_.geofence_radius_m) {
			const double scale = safety_.geofence_radius_m / distance;
			safe_command.target_position_m.north_m =
				home_position_m_.north_m + (safe_command.target_position_m.north_m - home_position_m_.north_m) * scale;
			safe_command.target_position_m.east_m =
				home_position_m_.east_m + (safe_command.target_position_m.east_m - home_position_m_.east_m) * scale;
			last_safety_message_ = "target constrained to geofence";
		}
	} else if (safe_command.requested_mode == FlightMode::PositionHold) {
		safe_command.target_position_m = sample.position_m;
	}

	return safe_command;
}

LocalPosition FlightController::home_or_current_position(const SensorSample& sample)
{
	return home_set_ ? home_position_m_ : sample.position_m;
}

double FlightController::distance_from_home(const SensorSample& sample) const
{
	return home_set_ ? distance_between(sample.position_m, home_position_m_) : 0.0;
}

void FlightController::transition_to(FlightMode next_mode, const std::string& reason)
{
	if (active_mode_ == next_mode) {
		return;
	}

	active_mode_ = next_mode;
	event_log_.push("mode: " + to_string(active_mode_) + " (" + reason + ")");
}

DroneSimulator::DroneSimulator(VehicleState initial_state)
	: state_(initial_state)
{
}

// The simulator returns current state directly; real flight code should use estimator outputs here.
SensorSample DroneSimulator::sensors() const
{
	return SensorSample{
		state_.attitude_deg,
		state_.angular_rate_deg_s,
		state_.position_m,
		state_.velocity_m_s,
		state_.altitude_m,
		state_.vertical_speed_m_s,
		state_.battery_voltage_v,
		true,
	};
}

// Advance a simplified state from four motor outputs; this is educational and not a real dynamics model.
void DroneSimulator::apply_motors(const QuadXMixer::MotorOutputs& motors, double dt_s)
{
	const double average = std::accumulate(motors.begin(), motors.end(), 0.0) / motors.size();
	const double roll_torque = (motors[0] + motors[3]) - (motors[1] + motors[2]);
	const double pitch_torque = (motors[0] + motors[1]) - (motors[2] + motors[3]);
	const double yaw_torque = (-motors[0] + motors[1] - motors[2] + motors[3]);

	state_.angular_rate_deg_s.x = clamp(state_.angular_rate_deg_s.x + roll_torque * 80.0 * dt_s, -120.0, 120.0);
	state_.angular_rate_deg_s.y = clamp(state_.angular_rate_deg_s.y + pitch_torque * 80.0 * dt_s, -120.0, 120.0);
	state_.angular_rate_deg_s.z = clamp(state_.angular_rate_deg_s.z + yaw_torque * 60.0 * dt_s, -180.0, 180.0);

	state_.attitude_deg.x = clamp(state_.attitude_deg.x + state_.angular_rate_deg_s.x * dt_s, -80.0, 80.0);
	state_.attitude_deg.y = clamp(state_.attitude_deg.y + state_.angular_rate_deg_s.y * dt_s, -80.0, 80.0);
	state_.attitude_deg.z += state_.angular_rate_deg_s.z * dt_s;

	const double north_acceleration = std::sin(state_.attitude_deg.y * 3.14159265358979323846 / 180.0) * 2.0;
	const double east_acceleration = -std::sin(state_.attitude_deg.x * 3.14159265358979323846 / 180.0) * 2.0;
	state_.velocity_m_s.north_m = clamp((state_.velocity_m_s.north_m + north_acceleration * dt_s) * 0.995, -6.0, 6.0);
	state_.velocity_m_s.east_m = clamp((state_.velocity_m_s.east_m + east_acceleration * dt_s) * 0.995, -6.0, 6.0);
	state_.position_m.north_m += state_.velocity_m_s.north_m * dt_s;
	state_.position_m.east_m += state_.velocity_m_s.east_m * dt_s;

	const double vertical_acceleration = (average - 0.50) * 7.5;
	state_.vertical_speed_m_s = clamp(state_.vertical_speed_m_s + vertical_acceleration * dt_s, -4.0, 4.0);
	state_.altitude_m = std::max(0.0, state_.altitude_m + state_.vertical_speed_m_s * dt_s);

	state_.angular_rate_deg_s.x *= 0.96;
	state_.angular_rate_deg_s.y *= 0.96;
	state_.angular_rate_deg_s.z *= 0.98;
	state_.vertical_speed_m_s *= 0.99;
	state_.battery_voltage_v = std::max(9.8, state_.battery_voltage_v - average * 0.002 * dt_s);
}

const VehicleState& DroneSimulator::state() const
{
	return state_;
}

// Run a safety-focused demo with position hold, stale-command return, and automatic landing.
int run_demo()
{
	FlightController controller;
	DroneSimulator simulator;

	ControlCommand command;
	command.requested_mode = FlightMode::PositionHold;
	command.target_altitude_m = 1.8;
	command.target_position_m = {5.0, 0.0};
	command.arm_requested = true;

	constexpr double dt_s = 0.02;
	constexpr int steps = static_cast<int>(14.0 / dt_s);

	std::cout << "time_s,armed,mode,north_m,east_m,altitude_m,roll_deg,pitch_deg,battery_v,m1,m2,m3,m4,status\n";

	for (int i = 0; i <= steps; ++i) {
		const double time_s = i * dt_s;
		command.command_age_ms = time_s > 7.0 ? 900 : 20;
		if (time_s > 11.0) {
			command.requested_mode = FlightMode::Land;
			command.command_age_ms = 20;
		}

		const auto motors = controller.update(simulator.sensors(), command, dt_s);
		simulator.apply_motors(motors, dt_s);

		if (i % 25 == 0) {
			const auto& state = simulator.state();
			std::cout << std::fixed << std::setprecision(2)
					  << time_s << ','
					  << (controller.armed() ? "yes" : "no") << ','
					  << to_string(controller.mode()) << ','
					  << state.position_m.north_m << ','
					  << state.position_m.east_m << ','
					  << state.altitude_m << ','
					  << state.attitude_deg.x << ','
					  << state.attitude_deg.y << ','
					  << state.battery_voltage_v << ','
					  << motors[0] << ','
					  << motors[1] << ','
					  << motors[2] << ','
					  << motors[3] << ','
					  << controller.last_safety_message() << '\n';
		}
	}

	return 0;
}

// Self-test coverage: mixer clamping, arming checks, emergency stop, stale link, and geofence failsafe.
int run_self_test()
{
	QuadXMixer mixer;
	const auto motors = mixer.mix(0.5, 0.8, 0.8, 0.8);
	for (double motor : motors) {
		if (motor < 0.0 || motor > 1.0) {
			std::cerr << "FAIL: motor output outside [0, 1]\n";
			return 1;
		}
	}

	FlightController controller;
	ControlCommand command;
	command.arm_requested = true;
	command.requested_mode = FlightMode::PositionHold;
	command.target_altitude_m = 1.5;

	SensorSample low_battery{};
	low_battery.battery_voltage_v = 10.0;
	controller.update(low_battery, command, 0.02);
	if (controller.armed()) {
		std::cerr << "FAIL: controller armed on low battery\n";
		return 1;
	}

	SensorSample normal{};
	normal.battery_voltage_v = 12.2;
	controller.update(normal, command, 0.02);
	if (!controller.armed() || controller.mode() != FlightMode::PositionHold) {
		std::cerr << "FAIL: controller did not arm into position hold\n";
		return 1;
	}

	ControlCommand stale = command;
	stale.command_age_ms = 900;
	controller.update(normal, stale, 0.02);
	if (controller.mode() != FlightMode::ReturnToLaunch) {
		std::cerr << "FAIL: stale command did not trigger return-to-launch\n";
		return 1;
	}

	ControlCommand stop = command;
	stop.emergency_stop = true;
	controller.update(normal, stop, 0.02);
	if (controller.armed()) {
		std::cerr << "FAIL: emergency stop did not disarm\n";
		return 1;
	}

	SafetyConfig tight_geofence;
	tight_geofence.geofence_radius_m = 5.0;
	FlightController geofence_controller(tight_geofence);
	geofence_controller.update(normal, command, 0.02);
	normal.position_m.north_m = 6.0;
	geofence_controller.update(normal, command, 0.02);
	if (geofence_controller.mode() != FlightMode::ReturnToLaunch) {
		std::cerr << "FAIL: geofence boundary did not trigger return-to-launch\n";
		return 1;
	}

	normal.position_m.north_m = 11.0;
	geofence_controller.update(normal, command, 0.02);
	if (geofence_controller.armed()) {
		std::cerr << "FAIL: hard geofence breach did not disarm\n";
		return 1;
	}

	const auto hover = mixer.mix(0.5, 0.0, 0.0, 0.0);
	if (!near(hover[0], 0.5, 0.001) || !near(hover[1], 0.5, 0.001) ||
		!near(hover[2], 0.5, 0.001) || !near(hover[3], 0.5, 0.001)) {
		std::cerr << "FAIL: hover mix is not balanced\n";
		return 1;
	}

	std::cout << "All self-tests passed.\n";
	return 0;
}

}  // namespace uav

// Program entry point: run the demo by default, or run self-tests with --self-test.
int main(int argc, char* argv[])
{
	if (argc > 1 && std::string_view(argv[1]) == "--self-test") {
		return uav::run_self_test();
	}

	return uav::run_demo();
}
