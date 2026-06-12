#pragma once

#include <array>
#include <iostream>
#include <string>

namespace uav {

// Three-axis vector used for attitude, angular rates, acceleration, and similar data.
struct Vec3 {
	double x{};
	double y{};
	double z{};
};

// Vehicle state. Attitude is measured in degrees and altitude is measured in meters.
struct VehicleState {
	Vec3 attitude_deg{};
	Vec3 angular_rate_deg_s{};
	double altitude_m{};
	double vertical_speed_m_s{};
	double battery_voltage_v{12.6};
};

// Command target from a radio controller or companion computer.
struct ControlCommand {
	double target_roll_deg{};
	double target_pitch_deg{};
	double target_yaw_rate_deg_s{};
	double target_altitude_m{1.0};
	bool arm_requested{};
};

// Sensor sample. Real hardware should provide these values from an IMU, barometer, and battery monitor.
struct SensorSample {
	Vec3 attitude_deg{};
	Vec3 angular_rate_deg_s{};
	double altitude_m{};
	double vertical_speed_m_s{};
	double battery_voltage_v{};
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

// Motor mixer for a quadcopter with an X frame layout.
class QuadXMixer {
public:
	using MotorOutputs = std::array<double, 4>;

	MotorOutputs mix(double throttle, double roll, double pitch, double yaw) const;
};

// Flight-control loop: reads sensors, computes PID outputs, mixes motors, and enforces safety limits.
class FlightController {
public:
	FlightController();

	QuadXMixer::MotorOutputs update(const SensorSample& sample, const ControlCommand& command, double dt_s);
	void disarm();
	bool armed() const;
	std::string last_safety_message() const;

private:
	bool should_arm(const SensorSample& sample, const ControlCommand& command);
	bool safety_allows_flight(const SensorSample& sample);

	PidController roll_pid_;
	PidController pitch_pid_;
	PidController yaw_rate_pid_;
	PidController altitude_pid_;
	QuadXMixer mixer_;
	bool armed_{};
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

// Demo entry point: runs a fixed-duration simulation and prints key state values.
int run_demo();

// Self-test entry point: verifies mixer and safety behavior.
int run_self_test();

}  // namespace uav
