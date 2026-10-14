#include "sixsim/hal/imu_6dof.hpp"
#include "sixsim/sim/scenario.hpp"
#include "hal/sitl/sensors.hpp"

#include <utility>

namespace sixsim::hal {

class Sitl_imu_6dof : public imu_6dof, public SitlSensor {
 public:
  struct config_type {
    Vector3 position_body_m{};
    Quaternion q_body2imu{};
    Vector3 accel_bias{};
    Vector3 gyro_bias{};
    Matrix3 accel_sfcc{Matrix3::identity()};  // Scale factor and cross coupling matrix.
    Matrix3 gyro_sfcc{Matrix3::identity()};  // Scale factor and cross coupling matrix.
  };

  explicit Sitl_imu_6dof(std::string name)
      : SitlSensor(std::move(name)) {}

  config_type& config() { return config_; }
  const config_type& config() const { return config_; }

  void update(const sim::SensorTruthInputs& inputs) override {

    // Accelerometer measures acceleration - gravity resolved in accel frame.
    Vector3 gravity_ned_mps2 = inputs.environment.gravity->evaluate(
        inputs.time, inputs.vehicle_state.position_ned_m
    ).acceleration_ned_mps2;
    Quaternion q_ned2body = conjugate(inputs.vehicle_state.q_body2ned);
    Vector3 gravity_body_mps2 = rotate(q_ned2body, gravity_ned_mps2);
    
    // Specific force at the center of mass
    Vector3 cm_specific_force_body =
        inputs.vehicle_derivative.velocity_body_derivative_mps2 +
        cross(inputs.vehicle_state.omega_body_rps,
              inputs.vehicle_state.velocity_body_mps) -
        gravity_body_mps2;

    // Specific force at the IMU location
    Vector3 imu_specific_force_body =
        cm_specific_force_body +
        cross(inputs.vehicle_derivative.omega_body_derivative_rps2,
              config_.position_body_m) +
        cross(inputs.vehicle_state.omega_body_rps,
              cross(inputs.vehicle_state.omega_body_rps,
                    config_.position_body_m));
                    
    // Resolve specific force in IMU axes, then apply scale factor errors,
    // cross coupling errors, and bias.
    Vector3 a_accel_imu_mps2_ideal =
        rotate(config_.q_body2imu, imu_specific_force_body);
    Vector3 a_accel_imu_mps2 =
        config_.accel_sfcc * a_accel_imu_mps2_ideal + config_.accel_bias;



    // Resolve angular rate in IMU axes, then apply scale factor errors,
    // cross coupling errors, and bias.
    Vector3 omega_imu_rps =
        rotate(config_.q_body2imu, inputs.vehicle_state.omega_body_rps);
    Vector3 w_gyro_imu_rps =
        config_.gyro_sfcc * omega_imu_rps + config_.gyro_bias;

    sample_.a_accel_imu_mps2 = a_accel_imu_mps2;
    sample_.w_gyro_imu_rps = w_gyro_imu_rps;
    sample_.measurement_time_s = inputs.time.simtime_s;
    sample_.valid = true;
  }

  IMU6DOFSample read() override {
    return sample_;
  }

 private:
  config_type config_{};
  IMU6DOFSample sample_{};
};

}  // namespace sixsim::hal
