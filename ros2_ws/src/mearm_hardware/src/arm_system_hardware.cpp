// arm_system_hardware.cpp
// ========================
// Topic-based ros2_control SystemInterface plugin implementation.
//
// Bridges the ros2_control read/write cycle to the STM32 over a direct
// serial link — no micro-ROS, no XRCE agent. A separate host-side node
// (pca9685_multiple's servo_serial_bridge) subscribes /arm_joint_commands
// and writes plain "J<n> <rad>\n" lines to the STM32's USB-serial port:
//   Publishes:  /arm_joint_commands  (std_msgs/Float64MultiArray)
//   Subscribes: /stm32/joint_states  (sensor_msgs/JointState)
//
// See arm_system_hardware.hpp for the full architecture description.

#include "mearm_hardware/arm_system_hardware.hpp"

#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mearm_hardware
{

// ── on_init ─────────────────────────────────────────────────────────────────
// Called once when the controller_manager first loads this plugin.
// Reads joint definitions from the URDF <ros2_control> tag and allocates
// the state/command buffers.
hardware_interface::CallbackReturn ArmSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // Delegate to the base class to parse HardwareInfo from URDF
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Allocate buffers sized to the number of joints declared in the URDF
  // ros2_control block (should be 4: joint_0, joint_1, joint_2, joint_3)
  const std::size_t n = params.hardware_info.joints.size();
  hw_positions_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  hw_commands_.assign(n, 0.0);
  received_positions_.assign(n, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"),
    "Initialised %zu joints. Listening for STM32 on /stm32/joint_states.", n);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_configure ────────────────────────────────────────────────────────────
// Transitions from UNCONFIGURED → INACTIVE.
// Creates the private ROS 2 node, publisher, subscriber, and starts the
// executor spin thread.
hardware_interface::CallbackReturn ArmSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"), "Configuring ...");

  // Create a private node for pub/sub — isolated from controller_manager's
  // own node so our topics don't interfere with the CM's internal comms.
  hw_node_ = rclcpp::Node::make_shared("arm_hw_bridge");

  // Publisher: sends commanded joint positions over the ROS 2 graph.
  // servo_serial_bridge subscribes /arm_joint_commands and forwards each
  // value to the STM32 as a plain serial line, which drives the MG90S servos.
  cmd_pub_ = hw_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/arm_joint_commands", rclcpp::QoS(10));

  // Subscriber: receives position feedback from the STM32.
  // We use /stm32/joint_states (NOT /joint_states) to avoid a circular
  // dependency with joint_state_broadcaster, which publishes the canonical
  // /joint_states topic from our state interface buffers.
  state_sub_ = hw_node_->create_subscription<sensor_msgs::msg::JointState>(
    "/stm32/joint_states",
    rclcpp::QoS(10),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      joint_state_callback(msg);
    });

  // Use a SingleThreadedExecutor so we can cancel() it cleanly during
  // on_cleanup().  We do NOT call rclcpp::shutdown() — that would destroy
  // the global context shared with the controller_manager.
  hw_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  hw_executor_->add_node(hw_node_);

  // Spin the executor in a dedicated thread so callbacks are processed
  // asynchronously from the controller_manager update loop.
  stop_spin_ = false;
  spin_thread_ = std::thread([this]() {
    hw_executor_->spin();
  });

  RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"),
    "Publishing /arm_joint_commands, subscribing /stm32/joint_states");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_activate ─────────────────────────────────────────────────────────────
// Transitions from INACTIVE → ACTIVE.
// After this, the controller_manager will call read()/write() every cycle.
hardware_interface::CallbackReturn ArmSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"),
    "Activated — sending commands to STM32.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate ───────────────────────────────────────────────────────────
// Transitions from ACTIVE → INACTIVE.
// The controller_manager stops calling read()/write() after this.
hardware_interface::CallbackReturn ArmSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"), "Deactivating ...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_cleanup ──────────────────────────────────────────────────────────────
// Transitions from INACTIVE → UNCONFIGURED.
// Cancels the executor, joins the spin thread, and releases all ROS objects.
hardware_interface::CallbackReturn ArmSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Cancel the executor (NOT rclcpp::shutdown — that destroys the global
  // context shared with the controller_manager and causes a crash!)
  if (hw_executor_) {
    hw_executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }

  // Release all ROS objects in reverse order of creation
  state_sub_.reset();
  cmd_pub_.reset();
  hw_executor_.reset();
  hw_node_.reset();

  RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"), "Cleaned up.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── export_state_interfaces ─────────────────────────────────────────────────
// Register the state interfaces (position, velocity) that controllers and
// the joint_state_broadcaster will read from.  Each interface holds a
// pointer into hw_positions_[] or hw_velocities_[] — the control loop
// updates these in read().
std::vector<hardware_interface::StateInterface>
ArmSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    // Only export interfaces that were DECLARED in the URDF <ros2_control>
    // block.  This avoids confusing the resource_manager with undeclared
    // interfaces.
    for (const auto & si : info_.joints[i].state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        state_interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
      } else if (si.name == hardware_interface::HW_IF_VELOCITY) {
        state_interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
      }
    }
  }
  return state_interfaces;
}

// ── export_command_interfaces ───────────────────────────────────────────────
// Register the command interfaces (position) that the arm_controller will
// write to.  Each interface holds a pointer into hw_commands_[] — the
// control loop reads these in write().
std::vector<hardware_interface::CommandInterface>
ArmSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    // Only export command interfaces that were DECLARED in the URDF
    // ros2_control tag.  Exporting an undeclared interface confuses
    // the resource_manager in Humble.
    for (const auto & ci : info_.joints[i].command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) {
        command_interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
      }
    }
  }
  return command_interfaces;
}

// ── read ────────────────────────────────────────────────────────────────────
// Called every update cycle by the controller_manager.
// Copies the latest STM32 feedback into hw_positions_[] and hw_velocities_[].
// If no feedback has arrived yet (STM32 not connected), echo the commanded
// positions as a safe fallback.
hardware_interface::return_type ArmSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // For open-loop servos (no encoders), the best practice is to echo the
  // commanded positions directly as the state.  This ensures that the
  // joint_trajectory_controller sees perfectly smooth feedback that
  // exactly matches its own trajectory, eliminating "hiccups" or "recoil"
  // effects caused by micro-ROS transport latency.

  std::lock_guard<std::mutex> lock(state_mutex_);

  for (std::size_t i = 0; i < hw_positions_.size(); ++i) {
    // If we have received feedback from the STM32 (at least once), we use it
    // only if the command is still at its default zero (startup case).
    // Otherwise, we mirror the last command to ensure a perfectly smooth trajectory.
    if (state_received_ && hw_commands_[i] == 0.0 && received_positions_[i] != 0.0) {
      hw_positions_[i] = received_positions_[i];
    } else {
      hw_positions_[i] = hw_commands_[i];
    }
    hw_velocities_[i] = 0.0;
  }

  return hardware_interface::return_type::OK;
}

// ── write ───────────────────────────────────────────────────────────────────
// Called every update cycle by the controller_manager.
// Packs the 4 commanded positions into a Float64MultiArray and publishes
// to /arm_joint_commands for the STM32 to consume.
hardware_interface::return_type ArmSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data.resize(hw_commands_.size());
  for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
    msg.data[i] = hw_commands_[i];
  }
  cmd_pub_->publish(msg);
  return hardware_interface::return_type::OK;
}

// ── joint_state_callback ────────────────────────────────────────────────────
// Processes incoming JointState messages from /stm32/joint_states.
// Maps incoming joint names to our internal order (robust to any reordering
// in the message) by matching against info_.joints[].name from the URDF.
void ArmSystemHardware::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    for (std::size_t j = 0; j < msg->name.size(); ++j) {
      if (msg->name[j] == info_.joints[i].name) {
        if (j < msg->position.size()) {
          received_positions_[i] = msg->position[j];
        }
        break;  // Found the match for joint i, move to next joint
      }
    }
  }
  state_received_ = true;  // At least one valid message has arrived
}

}  // namespace mearm_hardware

// ── pluginlib export ─────────────────────────────────────────────────────────
// Register this class as a pluginlib plugin so ros2_control can load it at
// runtime based on the <plugin> tag in the URDF ros2_control block.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mearm_hardware::ArmSystemHardware,
  hardware_interface::SystemInterface)
