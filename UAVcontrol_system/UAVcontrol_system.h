#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace uav {

// Three-axis vector used for attitude, angular rates, acceleration, and similar data.
struct Vec3 {
	double x{};
	double y{};
	double z{};
};

// Local tangent-plane position in meters relative to an arbitrary launch origin.
struct LocalPosition {
	double north_m{};
	double east_m{};
};

// High-level flight modes used by the reference controller.
enum class FlightMode {
	Manual,
	AltitudeHold,
	PositionHold,
	ReturnToLaunch,
	Land,
};

// Vehicle state used by the simulator. Attitude is measured in degrees and altitude in meters.
struct VehicleState {
	Vec3 attitude_deg{};
	Vec3 angular_rate_deg_s{};
	LocalPosition position_m{};
	LocalPosition velocity_m_s{};
	double altitude_m{};
	double vertical_speed_m_s{};
	double battery_voltage_v{12.6};
};

// Command target from a radio controller or companion computer.
struct ControlCommand {
	FlightMode requested_mode{FlightMode::AltitudeHold};
	double target_roll_deg{};
	double target_pitch_deg{};
	double target_yaw_rate_deg_s{};
	double target_altitude_m{1.0};
	LocalPosition target_position_m{};
	bool arm_requested{};
	bool emergency_stop{};
	int command_age_ms{};
};

// Sensor sample. Real hardware should provide these values from sensors and estimator outputs.
struct SensorSample {
	Vec3 attitude_deg{};
	Vec3 angular_rate_deg_s{};
	LocalPosition position_m{};
	LocalPosition velocity_m_s{};
	double altitude_m{};
	double vertical_speed_m_s{};
	double battery_voltage_v{};
	bool position_valid{true};
};

// Safety limits collected in one place so users can inspect and tune conservative defaults.
struct SafetyConfig {
	double min_arm_voltage_v{10.8};
	double min_flight_voltage_v{10.4};
	double max_arm_tilt_deg{10.0};
	double max_flight_tilt_deg{55.0};
	double max_altitude_m{25.0};
	double geofence_radius_m{80.0};
	int max_command_age_ms{500};
};

// Single PID controller that converts tracking error into a bounded control output.
class PidController {
public:
	PidController(double kp, double ki, double kd, double output_limit);

	double update(double target, double actual, double dt_s);
	void reset();

private:
	double kp_{};
	double ki_{};
	double kd_{};
	double output_limit_{};
	double integral_{};
	double previous_error_{};
	bool has_previous_error_{};
};

// Fixed-size event log for safety and mode-transition messages.
class EventLog {
public:
	explicit EventLog(std::size_t capacity = 12);

	void push(const std::string& message);
	const std::vector<std::string>& entries() const;

private:
	std::size_t capacity_{};
	std::vector<std::string> entries_;
};

// Motor mixer for a quadcopter with an X frame layout.
class QuadXMixer {
public:
	using MotorOutputs = std::array<double, 4>;

	MotorOutputs mix(double throttle, double roll, double pitch, double yaw) const;
};

// Flight-control loop: validates commands, applies failsafes, computes setpoints, and mixes motors.
class FlightController {
public:
	explicit FlightController(SafetyConfig safety = {});

	QuadXMixer::MotorOutputs update(const SensorSample& sample, const ControlCommand& command, double dt_s);
	void disarm(const std::string& reason = "manual disarm");
	bool armed() const;
	FlightMode mode() const;
	std::string last_safety_message() const;
	const std::vector<std::string>& events() const;

private:
	bool should_arm(const SensorSample& sample, const ControlCommand& command);
	bool safety_allows_flight(const SensorSample& sample, const ControlCommand& command);
	ControlCommand apply_failsafe_policy(const SensorSample& sample, const ControlCommand& command);
	ControlCommand constrain_command(const SensorSample& sample, const ControlCommand& command);
	LocalPosition home_or_current_position(const SensorSample& sample);
	double distance_from_home(const SensorSample& sample) const;
	void transition_to(FlightMode next_mode, const std::string& reason);

	SafetyConfig safety_;
	PidController roll_pid_;
	PidController pitch_pid_;
	PidController yaw_rate_pid_;
	PidController altitude_pid_;
	PidController north_pid_;
	PidController east_pid_;
	QuadXMixer mixer_;
	EventLog event_log_;
	bool armed_{};
	bool home_set_{};
	LocalPosition home_position_m_{};
	FlightMode active_mode_{FlightMode::AltitudeHold};
	std::string last_safety_message_{"disarmed"};
};

// Minimal physics model used to demonstrate the control loop without a real drone.
class DroneSimulator {
public:
	explicit DroneSimulator(VehicleState initial_state = {});

	SensorSample sensors() const;
	void apply_motors(const QuadXMixer::MotorOutputs& motors, double dt_s);
	const VehicleState& state() const;

private:
	VehicleState state_;
};

std::string to_string(FlightMode mode);

// Demo entry point: runs a fixed-duration simulation and prints key state values.
int run_demo();

// Self-test entry point: verifies mixer, failsafe, geofence, and arming behavior.
int run_self_test();

}  // namespace uav
