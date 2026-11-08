#ifndef ROBOTARM_HARDWARE__DM_SYSTEM_HPP_
#define ROBOTARM_HARDWARE__DM_SYSTEM_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace robotarm_hardware
{

class DmSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DmSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    std::string name;
    int can_id{0};
    int mst_id{0};
    int dir{1};
    double offset{0.0};
    double kp{0.0};
    double kd{0.0};
    double pmax{12.5};
    double vmax{30.0};
    double tmax{10.0};
    double pm_to_xout{0.0};
    double state_pos{0.0};
    double state_vel{0.0};
    double state_eff{0.0};
    double cmd_pos{0.0};
  };

  bool open_can(const std::string & iface);
  void close_can();
  bool read_reg(uint8_t mid, uint8_t rid, uint32_t & raw, int timeout_ms = 50);
  static float as_float(uint32_t v);
  bool load_calibration(const std::string & path);

  // ---- 重力补偿（Pinocchio 关节空间 g(q)，作为 MIT 帧的 t_ff 下发）----
  bool load_gravity();
  bool send_raw(int can_id, const uint8_t data[8]);
  void enable_motors();
  void disable_motors();
  double gravity_torque(size_t i) const;
  // ±180° 软限位：超出后叠加一个回正弹簧力矩，让手推到这里"推不动"
  double limit_torque(size_t i) const;

  std::string can_interface_{"can0"};
  std::string calibration_file_;
  int can_fd_{-1};
  std::vector<Joint> joints_;
  std::unordered_map<int, size_t> mst_index_;
  uint64_t rx_frames_{0};
  uint64_t rx_miss_{0};

  // 重力模型：从 robotarm 包的 robotarm.urdf 建，参数读 config/gravity_comp.yaml
  bool gravity_ok_{false};
  std::unique_ptr<pinocchio::Model> pin_model_;
  std::unique_ptr<pinocchio::Data> pin_data_;
  Eigen::VectorXd pin_q_;
  std::vector<int> q_idx_;   // 每个关节在 q 向量里的下标（nq）
  std::vector<int> v_idx_;   // 每个关节在 g 向量里的下标（nv）
  std::vector<double> grav_gain_;      // q >= 0 用的系数
  std::vector<double> grav_gain_neg_;  // q < 0 用的系数（没配就沿用 grav_gain_）
  std::vector<double> grav_phase_;     // 重力模型的角度修正（rad），补偿质心/零点偏差
  double grav_scale_{1.0};
  double grav_limit_{6.0};
  double grav_ramp_s_{4.0};
  std::chrono::steady_clock::time_point grav_t0_{};
  bool grav_ramp_started_{false};
  std::chrono::steady_clock::time_point limit_warn_t0_{};
};

}  // namespace robotarm_hardware

#endif  // ROBOTARM_HARDWARE__DM_SYSTEM_HPP_
