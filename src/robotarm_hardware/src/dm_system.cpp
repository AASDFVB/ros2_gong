#include "robotarm_hardware/dm_system.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

static constexpr uint8_t REG_XOUT = 0x51;
static constexpr uint8_t REG_P_M = 0x50;
static constexpr uint8_t REG_MST = 0x07;
static constexpr uint8_t REG_PMAX = 0x15;
static constexpr uint8_t REG_VMAX = 0x16;
static constexpr uint8_t REG_TMAX = 0x17;

namespace robotarm_hardware
{

static double u2f(uint32_t v, double lo, double hi, int bits)
{
  return v * (hi - lo) / static_cast<double>((1u << bits) - 1) + lo;
}

static uint16_t f2u(double x, double lo, double hi, int bits)
{
  if (x < lo) {x = lo;}
  if (x > hi) {x = hi;}
  return static_cast<uint16_t>(
    (x - lo) * static_cast<double>((1u << bits) - 1) / (hi - lo));
}

// 打一帧 MIT 指令。2a 阶段用 kp=kd=t=0，只为了换来反馈帧，不产生任何力矩。
static void pack_mit(
  uint8_t out[8], double p, double v, double kp, double kd, double t,
  double pmax, double vmax, double tmax)
{
  const uint16_t pi = f2u(p, -pmax, pmax, 16);
  const uint16_t vi = f2u(v, -vmax, vmax, 12);
  const uint16_t kpi = f2u(kp, 0.0, 500.0, 12);
  const uint16_t kdi = f2u(kd, 0.0, 5.0, 12);
  const uint16_t ti = f2u(t, -tmax, tmax, 12);
  out[0] = static_cast<uint8_t>(pi >> 8);
  out[1] = static_cast<uint8_t>(pi & 0xFF);
  out[2] = static_cast<uint8_t>(vi >> 4);
  out[3] = static_cast<uint8_t>(((vi & 0xF) << 4) | (kpi >> 8));
  out[4] = static_cast<uint8_t>(kpi & 0xFF);
  out[5] = static_cast<uint8_t>(kdi >> 4);
  out[6] = static_cast<uint8_t>(((kdi & 0xF) << 4) | (ti >> 8));
  out[7] = static_cast<uint8_t>(ti & 0xFF);
}

float DmSystemHardware::as_float(uint32_t v)
{
  float f;
  std::memcpy(&f, &v, sizeof(f));
  return f;
}

// ---------------------------------------------------------------- lifecycle
hardware_interface::CallbackReturn DmSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto get = [&](const std::string & k, const std::string & def) {
      auto it = info.hardware_parameters.find(k);
      return it == info.hardware_parameters.end() ? def : it->second;
    };
  can_interface_ = get("can_interface", "can0");
  calibration_file_ = get("calibration_file", "");

  joints_.clear();
  for (const auto & ji : info.joints) {
    Joint j;
    j.name = ji.name;
    auto pit = ji.parameters.find("can_id");
    if (pit != ji.parameters.end()) {
      j.can_id = std::stoi(pit->second);
    }
    pit = ji.parameters.find("kp");
    if (pit != ji.parameters.end()) {
      j.kp = std::stod(pit->second);
    }
    pit = ji.parameters.find("kd");
    if (pit != ji.parameters.end()) {
      j.kd = std::stod(pit->second);
    }
    joints_.push_back(j);
  }
  if (joints_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("DmSystemHardware"), "没有配置任何 joint");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("DmSystemHardware"), "on_init: %zu 轴, can=%s",
    joints_.size(), can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool DmSystemHardware::load_calibration(const std::string & path)
{
  try {
    YAML::Node root = YAML::LoadFile(path);
    YAML::Node offs = root["offsets"];
    YAML::Node dirs = root["directions"];
    for (auto & j : joints_) {
      if (offs && offs[j.name]) {
        j.offset = offs[j.name].as<double>();
      }
      if (dirs && dirs[j.name]) {
        j.dir = dirs[j.name].as<int>();
      }
      RCLCPP_INFO(
        rclcpp::get_logger("DmSystemHardware"), "  %-3s dir=%+d offset=%+.6f",
        j.name.c_str(), j.dir, j.offset);
    }
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("DmSystemHardware"), "读标定文件 %s 失败: %s",
      path.c_str(), e.what());
    return false;
  }
}

// ---------------------------------------------------------------- 重力补偿
bool DmSystemHardware::send_raw(int can_id, const uint8_t data[8])
{
  struct can_frame f;
  std::memset(&f, 0, sizeof(f));
  f.can_id = static_cast<canid_t>(can_id);
  f.can_dlc = 8;
  std::memcpy(f.data, data, 8);
  return ::write(can_fd_, &f, sizeof(f)) == static_cast<ssize_t>(sizeof(f));
}

void DmSystemHardware::enable_motors()
{
  static const uint8_t k_clear[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
  static const uint8_t k_enable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  for (auto & j : joints_) {
    send_raw(j.can_id, k_clear);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  for (auto & j : joints_) {
    send_raw(j.can_id, k_enable);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  // 清掉使能过程产生的回包，别让 read() 把它们当普通反馈
  struct can_frame junk;
  while (::read(can_fd_, &junk, sizeof(junk)) == sizeof(junk)) {}
}

void DmSystemHardware::disable_motors()
{
  static const uint8_t k_disable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  // 先补几帧零力矩，再失能，避免带力矩直接断电造成抖动
  for (int k = 0; k < 5; ++k) {
    for (auto & j : joints_) {
      uint8_t d[8];
      const double p_des = j.dir * (j.state_pos - j.offset);
      pack_mit(d, p_des, 0.0, 0.0, 0.0, 0.0, j.pmax, j.vmax, j.tmax);
      send_raw(j.can_id, d);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  for (auto & j : joints_) {
    send_raw(j.can_id, k_disable);
  }
}

bool DmSystemHardware::load_gravity()
{
  const auto log = rclcpp::get_logger("DmSystemHardware");
  std::string share;
  try {
    share = ament_index_cpp::get_package_share_directory("robotarm");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(log, "定位 robotarm 包失败，重力补偿关闭: %s", e.what());
    return false;
  }
  const std::string urdf_path = share + "/urdf/robotarm.urdf";
  const std::string yaml_path = share + "/config/gravity_comp.yaml";

  try {
    pin_model_ = std::make_unique<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf_path, *pin_model_);
    pin_data_ = std::make_unique<pinocchio::Data>(*pin_model_);
    pin_q_ = Eigen::VectorXd::Zero(pin_model_->nq);

    q_idx_.assign(joints_.size(), -1);
    v_idx_.assign(joints_.size(), -1);
    for (size_t i = 0; i < joints_.size(); ++i) {
      const std::string & name = joints_[i].name;
      if (!pin_model_->existJointName(name)) {
        RCLCPP_ERROR(log, "URDF 里找不到关节 %s，重力补偿关闭", name.c_str());
        return false;
      }
      const auto id = pin_model_->getJointId(name);
      q_idx_[i] = pin_model_->joints[id].idx_q();
      v_idx_[i] = pin_model_->joints[id].idx_v();
    }

    grav_gain_.assign(joints_.size(), 1.0);
    const YAML::Node node = YAML::LoadFile(yaml_path);
    if (node["common"]) {
      grav_scale_ = node["common"]["scale"].as<double>(grav_scale_);
      grav_limit_ = node["common"]["limit"].as<double>(grav_limit_);
      grav_ramp_s_ = node["common"]["ramp"].as<double>(grav_ramp_s_);
    }
    if (node["gains"]) {
      for (size_t i = 0; i < joints_.size(); ++i) {
        const YAML::Node g = node["gains"][joints_[i].name];
        if (g) {
          grav_gain_[i] = g.as<double>();
        }
      }
    }
    // 负角度侧（q < 0）单独一套系数；没配的轴沿用 grav_gain_
    grav_gain_neg_ = grav_gain_;
    if (node["gains_neg"]) {
      for (size_t i = 0; i < joints_.size(); ++i) {
        const YAML::Node g = node["gains_neg"][joints_[i].name];
        if (g) {
          grav_gain_neg_[i] = g.as<double>();
        }
      }
    }

    // 逐轴角度修正（度 → rad）：补偿质心/零点偏差，等价于用 q+phase 评估重力模型
    grav_phase_.assign(joints_.size(), 0.0);
    if (node["phase_deg"]) {
      for (size_t i = 0; i < joints_.size(); ++i) {
        const YAML::Node ph = node["phase_deg"][joints_[i].name];
        if (ph) {
          grav_phase_[i] = ph.as<double>() * M_PI / 180.0;
        }
      }
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(log, "建重力模型失败，重力补偿关闭: %s", e.what());
    return false;
  }

  RCLCPP_INFO(log, "重力补偿: nq=%d scale=%.2f limit=%.1fN·m ramp=%.1fs",
    pin_model_->nq, grav_scale_, grav_limit_, grav_ramp_s_);
  return true;
}

double DmSystemHardware::limit_torque(size_t i) const
{
  // 与 URDF 声明一致：|q| > π 就开始顶回去。刚度选得比较温和，
  // 目的是给出"到这里了"的手感，而不是把手臂弹开。
  static constexpr double kLimitRad = 3.141592653589793;
  static constexpr double kDeadband = 0.02;   // rad ≈1.1°，标定贴着限位时不误触发
  static constexpr double kStiffness = 8.0;   // N·m/rad
  static constexpr double kMaxNm = 3.0;       // 软限位最多顶 3 N·m

  const double q = joints_[i].state_pos;
  const double over = std::fabs(q) - kLimitRad - kDeadband;
  if (over <= 0.0) {
    return 0.0;
  }
  const double push = std::min(over * kStiffness, kMaxNm);
  // q 超过正限位就要往负方向推，反之亦然；最后映射到电机侧
  const double tau_joint = (q > 0.0) ? -push : push;
  return joints_[i].dir * tau_joint;
}

double DmSystemHardware::gravity_torque(size_t i) const
{
  double ramp = 1.0;
  if (grav_ramp_started_ && grav_ramp_s_ > 1e-6) {
    const double el = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - grav_t0_).count();
    ramp = std::clamp(el / grav_ramp_s_, 0.0, 1.0);
  }
  // q >= 0 用 grav_gain_，q < 0 用 grav_gain_neg_；零点附近线性过渡，避免力矩跳变
  static constexpr double kBlendRad = 0.10;   // ±5.7° 过渡带
  const double q = joints_[i].state_pos;
  const double w = std::clamp((q + kBlendRad) / (2.0 * kBlendRad), 0.0, 1.0);
  const double gain = grav_gain_neg_[i] * (1.0 - w) + grav_gain_[i] * w;

  const double tau =
    joints_[i].dir * pin_data_->g[v_idx_[i]] * gain * grav_scale_ * ramp;
  return std::clamp(tau, -grav_limit_, grav_limit_);
}

hardware_interface::CallbackReturn DmSystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  // 标定文件：优先用参数，否则从 robotarm 包的 share 目录找
  std::string path = calibration_file_;
  if (path.empty()) {
    try {
      path = ament_index_cpp::get_package_share_directory("robotarm") +
        "/config/joint_offsets.yaml";
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("DmSystemHardware"), "定位 robotarm 包失败: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("DmSystemHardware"), "标定文件: %s", path.c_str());
  if (!load_calibration(path)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 重力模型失败不致命：退化成原来的零力矩行为，只是没有重力补偿
  gravity_ok_ = load_gravity();
  if (!gravity_ok_) {
    RCLCPP_WARN(rclcpp::get_logger("DmSystemHardware"),
      "重力补偿不可用，本次运行只做反馈读取（不出力）");
  }

  if (!open_can(can_interface_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 逐台读电机参数
  for (auto & j : joints_) {
    uint32_t raw = 0;
    if (!read_reg(j.can_id, REG_MST, raw)) {
      RCLCPP_ERROR(rclcpp::get_logger("DmSystemHardware"),
        "motor %d 无应答（检查 ID / 供电 / 接线）", j.can_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    j.mst_id = static_cast<int>(raw) & 0xFF;

    if (read_reg(j.can_id, REG_PMAX, raw)) {j.pmax = as_float(raw);}
    if (read_reg(j.can_id, REG_VMAX, raw)) {j.vmax = as_float(raw);}
    if (read_reg(j.can_id, REG_TMAX, raw)) {j.tmax = as_float(raw);}

    uint32_t r_pm = 0, r_xo = 0;
    bool ok_pm = read_reg(j.can_id, REG_P_M, r_pm);
    bool ok_xo = read_reg(j.can_id, REG_XOUT, r_xo);
    if (ok_pm && ok_xo) {
      j.pm_to_xout = as_float(r_xo) - as_float(r_pm);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("DmSystemHardware"),
        "%s 读 p_m/xout 失败，pm_to_xout 置 0", j.name.c_str());
    }
    const double enc = ok_xo ? as_float(r_xo) : 0.0;
    j.state_pos = j.dir * enc + j.offset;
    j.cmd_pos = j.state_pos;

    RCLCPP_INFO(rclcpp::get_logger("DmSystemHardware"),
      "  %-3s can=%d mst=0x%02X P=%.3f V=%.2f T=%.1f pm_to_xout=%+.4f q=%+.4f",
      j.name.c_str(), j.can_id, j.mst_id, j.pmax, j.vmax, j.tmax,
      j.pm_to_xout, j.state_pos);
  }

  mst_index_.clear();
  for (size_t i = 0; i < joints_.size(); ++i) {
    mst_index_[joints_[i].mst_id] = i;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  // 激活瞬间把指令对齐到当前实测，避免一上电就跳
  for (auto & j : joints_) {
    j.cmd_pos = j.state_pos;
  }
  // 清掉配置阶段残留的寄存器回包，避免被 read() 当成反馈帧
  struct can_frame junk;
  while (::read(can_fd_, &junk, sizeof(junk)) == sizeof(junk)) {}
  // 使能电机，并用 4 秒把重力补偿渐入，避免一上电猛地一撑
  enable_motors();
  grav_t0_ = std::chrono::steady_clock::now();
  grav_ramp_started_ = true;

  RCLCPP_INFO(rclcpp::get_logger("DmSystemHardware"),
    "已激活：重力补偿%s（渐入 %.1fs，单轴限幅 %.1fN·m）",
    gravity_ok_ ? "开" : "关", grav_ramp_s_, grav_limit_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  grav_ramp_started_ = false;
  if (can_fd_ >= 0) {
    disable_motors();
  }
  RCLCPP_INFO(rclcpp::get_logger("DmSystemHardware"), "已停用（已失能）");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------- interfaces
std::vector<hardware_interface::StateInterface> DmSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.reserve(joints_.size() * 3);
  for (auto & j : joints_) {
    si.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.state_pos);
    si.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.state_vel);
    // effort = 电机反馈的实测力矩（已乘 dir），用来对拍重力模型/调 gain
    si.emplace_back(j.name, hardware_interface::HW_IF_EFFORT, &j.state_eff);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface> DmSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.reserve(joints_.size());
  for (auto & j : joints_) {
    ci.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.cmd_pos);
  }
  return ci;
}

// ---------------------------------------------------------------- CAN
bool DmSystemHardware::open_can(const std::string & iface)
{
  can_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("DmSystemHardware"), "socket() 失败: %s", std::strerror(errno));
    return false;
  }
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("DmSystemHardware"), "找不到接口 %s", iface.c_str());
    close_can();
    return false;
  }
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(can_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("DmSystemHardware"), "bind() 失败: %s", std::strerror(errno));
    close_can();
    return false;
  }
  int flags = ::fcntl(can_fd_, F_GETFL, 0);
  ::fcntl(can_fd_, F_SETFL, flags | O_NONBLOCK);
  RCLCPP_INFO(rclcpp::get_logger("DmSystemHardware"), "CAN %s 已打开（非阻塞）", iface.c_str());
  return true;
}

void DmSystemHardware::close_can()
{
  if (can_fd_ >= 0) {
    ::close(can_fd_);
    can_fd_ = -1;
  }
}

bool DmSystemHardware::read_reg(uint8_t mid, uint8_t rid, uint32_t & raw, int timeout_ms)
{
  const uint8_t q[8] = {mid, 0x00, 0x33, rid, 0, 0, 0, 0};
  struct can_frame f;
  std::memset(&f, 0, sizeof(f));
  f.can_id = 0x7FF;
  f.can_dlc = 8;
  std::memcpy(f.data, q, 8);

  // 先清掉积压
  struct can_frame junk;
  while (::read(can_fd_, &junk, sizeof(junk)) == sizeof(junk)) {}

  for (int rep = 0; rep < 2; ++rep) {
    ::write(can_fd_, &f, sizeof(f));
    ::usleep(2000);
  }
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (::read(can_fd_, &f, sizeof(f)) == sizeof(f)) {
      const uint32_t id = f.can_id & 0x7FF;
      if (id != 0x7FF && f.data[2] == 0x33 && f.data[3] == rid &&
        (f.data[0] & 0x0F) == (mid & 0x0F))
      {
        raw = static_cast<uint32_t>(f.data[4]) |
          (static_cast<uint32_t>(f.data[5]) << 8) |
          (static_cast<uint32_t>(f.data[6]) << 16) |
          (static_cast<uint32_t>(f.data[7]) << 24);
        return true;
      }
    } else {
      ::usleep(300);
    }
  }
  return false;
}

// ---------------------------------------------------------------- cycle
hardware_interface::return_type DmSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  struct can_frame f;
  while (::read(can_fd_, &f, sizeof(f)) == sizeof(f)) {
    const int id = static_cast<int>(f.can_id & 0x7FF);
    auto it = mst_index_.find(id);
    if (it == mst_index_.end()) {
      continue;
    }
    auto & j = joints_[it->second];
    // D[0]=ERR<<4|ID, D[1:2]=POS, D[3:4]=VEL, D[4:5]=TORQUE
    const uint32_t p_i = (static_cast<uint32_t>(f.data[1]) << 8) | f.data[2];
    const uint32_t v_i = (static_cast<uint32_t>(f.data[3]) << 4) | (f.data[4] >> 4);
    const uint32_t t_i = (static_cast<uint32_t>(f.data[4] & 0x0F) << 8) | f.data[5];
    const double p_m = u2f(p_i, -j.pmax, j.pmax, 16);
    const double enc = p_m + j.pm_to_xout;      // 修回输出轴
    j.state_pos = j.dir * enc + j.offset;
    j.state_vel = j.dir * u2f(v_i, -j.vmax, j.vmax, 12);
    j.state_eff = j.dir * u2f(t_i, -j.tmax, j.tmax, 12);
    ++rx_frames_;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DmSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // 先把当前关节角填进 Pinocchio 的 q，一次算完 g(q)
  if (gravity_ok_) {
    for (size_t i = 0; i < joints_.size(); ++i) {
      // 加 phase：模型用的角度可以偏离实测角，用来补偿质心/零点偏差
      pin_q_[q_idx_[i]] = joints_[i].state_pos + grav_phase_[i];
    }
    pinocchio::computeGeneralizedGravity(*pin_model_, *pin_data_, pin_q_);
  }

  // 有轴超 ±180° 就节流告警（1 Hz），方便知道是撞限位了
  {
    bool over = false;
    for (auto & j : joints_) {
      if (std::fabs(j.state_pos) > 3.141592653589793 + 0.02) {
        over = true;
        break;
      }
    }
    const auto now = std::chrono::steady_clock::now();
    if (over && std::chrono::duration<double>(now - limit_warn_t0_).count() > 1.0) {
      limit_warn_t0_ = now;
      RCLCPP_WARN(rclcpp::get_logger("DmSystemHardware"),
        "有轴超出 ±180° 软限位，正在施加回正力矩");
    }
  }

  // MIT 帧：kp/kd 保持 0（推哪停哪），t_ff 放重力补偿力矩。
  for (size_t i = 0; i < joints_.size(); ++i) {
    auto & j = joints_[i];
    uint8_t d[8];
    const double p_des = j.dir * (j.state_pos - j.offset);  // 回到电机侧
    const double t_ff = (gravity_ok_ ? gravity_torque(i) : 0.0) + limit_torque(i);
    pack_mit(d, p_des, 0.0, 0.0, 0.0, t_ff, j.pmax, j.vmax, j.tmax);
    struct can_frame f;
    std::memset(&f, 0, sizeof(f));
    f.can_id = static_cast<canid_t>(j.can_id);
    f.can_dlc = 8;
    std::memcpy(f.data, d, 8);
    ::write(can_fd_, &f, sizeof(f));   // 非阻塞
  }
  return hardware_interface::return_type::OK;
}

}  // namespace robotarm_hardware

PLUGINLIB_EXPORT_CLASS(
  robotarm_hardware::DmSystemHardware, hardware_interface::SystemInterface)
