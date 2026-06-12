#include "UAVcontrol_system.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
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

}  // namespace

namespace uav {

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

FlightController::FlightController()
	: roll_pid_(0.055, 0.010, 0.018, 0.25),
	  pitch_pid_(0.055, 0.010, 0.018, 0.25),
	  yaw_rate_pid_(0.018, 0.002, 0.000, 0.18),
	  altitude_pid_(0.220, 0.040, 0.120, 0.35)
{
}

// One control-cycle step: safety checks, arming logic, PID updates, and motor mixing.
QuadXMixer::MotorOutputs FlightController::update(const SensorSample& sample, const ControlCommand& command, double dt_s)
{
	if (!safety_allows_flight(sample)) {
		disarm();
		return {};
	}

	if (!armed_ && should_arm(sample, command)) {
		roll_pid_.reset();
		pitch_pid_.reset();
		yaw_rate_pid_.reset();
		altitude_pid_.reset();
		armed_ = true;
		last_safety_message_ = "armed";
	}

	if (!armed_) {
		last_safety_message_ = "waiting for safe arming";
		return {};
	}

	const double roll = roll_pid_.update(command.target_roll_deg, sample.attitude_deg.x, dt_s);
	const double pitch = pitch_pid_.update(command.target_pitch_deg, sample.attitude_deg.y, dt_s);
	const double yaw = yaw_rate_pid_.update(command.target_yaw_rate_deg_s, sample.angular_rate_deg_s.z, dt_s);
	const double altitude = altitude_pid_.update(command.target_altitude_m, sample.altitude_m, dt_s);

	const double hover_throttle = 0.50;
	const double throttle = clamp(hover_throttle + altitude, 0.0, 0.85);
	return mixer_.mix(throttle, roll, pitch, yaw);
}

// Disarming immediately stops output and resets every controller.
void FlightController::disarm()
{
	armed_ = false;
	roll_pid_.reset();
	pitch_pid_.reset();
	yaw_rate_pid_.reset();
	altitude_pid_.reset();
}

bool FlightController::armed() const
{
	return armed_;
}

std::string FlightController::last_safety_message() const
{
	return last_safety_message_;
}

// Arming is intentionally conservative: level attitude, healthy battery, and an explicit arming request.
bool FlightController::should_arm(const SensorSample& sample, const ControlCommand& command)
{
	if (!command.arm_requested) {
		return false;
	}

	if (std::abs(sample.attitude_deg.x) > 10.0 || std::abs(sample.attitude_deg.y) > 10.0) {
		last_safety_message_ = "arming rejected: excessive tilt";
		return false;
	}

	if (sample.battery_voltage_v < 10.8) {
		last_safety_message_ = "arming rejected: battery voltage too low";
		return false;
	}

	return true;
}

// In-flight safety limits disarm the controller on low voltage or excessive attitude.
bool FlightController::safety_allows_flight(const SensorSample& sample)
{
	if (sample.battery_voltage_v < 10.4) {
		last_safety_message_ = "failsafe disarm: battery voltage too low";
		return false;
	}

	if (std::abs(sample.attitude_deg.x) > 55.0 || std::abs(sample.attitude_deg.y) > 55.0) {
		last_safety_message_ = "failsafe disarm: attitude limit exceeded";
		return false;
	}

	return true;
}

DroneSimulator::DroneSimulator(VehicleState initial_state)
	: state_(initial_state)
{
}

// The simulator returns current state directly; real flight code should fuse IMU, barometer, and other sensors here.
SensorSample DroneSimulator::sensors() const
{
	return SensorSample{
		state_.attitude_deg,
		state_.angular_rate_deg_s,
		state_.altitude_m,
		state_.vertical_speed_m_s,
		state_.battery_voltage_v,
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

// Run an eight-second command demo that shows the controller stabilizing to a target altitude.
int run_demo()
{
	FlightController controller;
	DroneSimulator simulator;

	ControlCommand command;
	command.target_altitude_m = 1.5;
	command.arm_requested = true;

	constexpr double dt_s = 0.02;
	constexpr int steps = static_cast<int>(8.0 / dt_s);

	std::cout << "time_s,armed,altitude_m,roll_deg,pitch_deg,battery_v,m1,m2,m3,m4,status\n";

	for (int i = 0; i <= steps; ++i) {
		const double time_s = i * dt_s;
		if (time_s > 3.0) {
			command.target_roll_deg = 3.0;
		}

		const auto motors = controller.update(simulator.sensors(), command, dt_s);
		simulator.apply_motors(motors, dt_s);

		if (i % 25 == 0) {
			const auto& state = simulator.state();
			std::cout << std::fixed << std::setprecision(2)
					  << time_s << ','
					  << (controller.armed() ? "yes" : "no") << ','
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

// Self-test coverage: mixer clamping, low-voltage arming rejection, normal arming, and tilt failsafe.
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
	if (!controller.armed()) {
		std::cerr << "FAIL: controller did not arm on safe state\n";
		return 1;
	}

	normal.attitude_deg.x = 60.0;
	controller.update(normal, command, 0.02);
	if (controller.armed()) {
		std::cerr << "FAIL: controller stayed armed after excessive tilt\n";
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
