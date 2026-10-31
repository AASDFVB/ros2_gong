#ifndef ROBOTARM_HARDWARE__DM_SYSTEM_HPP_
#define ROBOTARM_HARDWARE__DM_SYSTEM_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

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

  std::string can_interface_{"can0"};
  std::string calibration_file_;
  int can_fd_{-1};
  std::vector<Joint> joints_;
  std::unordered_map<int, size_t> mst_index_;
  uint64_t rx_frames_{0};
  uint64_t rx_miss_{0};
};

}  // namespace robotarm_hardware

#endif  // ROBOTARM_HARDWARE__DM_SYSTEM_HPP_
