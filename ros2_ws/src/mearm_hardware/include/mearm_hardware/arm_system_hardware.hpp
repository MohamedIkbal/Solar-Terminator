// arm_system_hardware.hpp
// ========================
// ros2_control SystemInterface plugin header for the 4-DOF MeArm acrylic arm.
//
// Architecture (direct ROS 2 topic bridge, no micro-ROS):
// ──────────────────────────────────────
//   This plugin runs inside the controller_manager process on the host.
//   It publishes plain ROS 2 topics that a separate host-side node
//   (pca9685_multiple's servo_serial_bridge) forwards to the STM32 over a
//   plain serial line — the STM32 itself runs no ROS/DDS client at all.
//
//     Publishes:  /arm_joint_commands   (std_msgs/Float64MultiArray)
//                 4 position values in radians, one per joint.
//
//     Subscribes: /stm32/joint_states   (sensor_msgs/JointState)
//                 Optional position feedback, if some bridge node ever
//                 publishes it back from the STM32.
//                 NOTE: This is /stm32/joint_states (NOT /joint_states)
//                 to avoid a circular dependency with joint_state_broadcaster.
//                 The current direct-serial bridge is command-only (no
//                 feedback channel), so read() below falls back to echoing
//                 the last commanded position — this is expected open-loop
//                 behaviour for servos with no encoders.
//
// Mock hardware (use_mock_hardware:=true):
// ──────────────────────────────────────
//   When mock hardware is selected via the xacro argument, the system uses
//   mock_components/GenericSystem instead of this plugin.  This plugin is
//   only loaded when use_mock_hardware:=false (real hardware mode).
//
// Threading model:
// ──────────────────────────────────────
//   controller_manager doesn't expose a node handle to hardware plugins
//   in ROS 2 Humble.  This plugin creates its own private rclcpp::Node
//   ("arm_hw_bridge"), adds it to a SingleThreadedExecutor, and spins
//   that executor in a dedicated std::thread.  The spin thread is started
//   during on_configure() and joined during on_cleanup().

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace mearm_hardware
{

/// @brief ros2_control SystemInterface for the 4-DOF MeArm arm.
///
/// Bridges the ros2_control read/write cycle to the STM32 over a direct
/// serial link (via pca9685_multiple's servo_serial_bridge node)
/// by publishing joint commands and subscribing to joint state feedback
/// over standard ROS 2 topics.
class ArmSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmSystemHardware)

  // ── Lifecycle callbacks ────────────────────────────────────────────────────
  // Called by the controller_manager as the hardware transitions through its
  // lifecycle: unconfigured → inactive → active → (and back on shutdown).

  /// Initialise joint buffers from the URDF HardwareInfo.
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  /// Create the private ROS 2 node, publisher, subscriber, and spin thread.
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Mark the hardware as ready to send commands.
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Pause command sending (hardware remains configured).
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Tear down the private node, publisher, subscriber, and spin thread.
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── Interface export ───────────────────────────────────────────────────────
  // Called once after on_init() to register the joint state/command handles
  // that controllers will read from and write to.

  /// Export position and velocity state interfaces for each joint.
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /// Export position command interfaces for each joint.
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── Control loop ──────────────────────────────────────────────────────────
  // Called every update cycle (100 Hz by default, set in controllers.yaml).

  /// Copy the latest STM32 feedback into the state interface buffers.
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Publish the current command interface values to /arm_joint_commands.
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Joint state/command buffers ────────────────────────────────────────────
  // These vectors are sized to info_.joints.size() during on_init().
  // The state/command interfaces hold pointers into these buffers.

  std::vector<double> hw_positions_;    ///< Current joint positions (rad)
  std::vector<double> hw_velocities_;   ///< Current joint velocities (rad/s)
  std::vector<double> hw_commands_;     ///< Commanded joint positions (rad)

  // ── Internal ROS 2 node for pub/sub ────────────────────────────────────────
  // controller_manager doesn't expose a node for hardware plugins in Humble,
  // so we create a private node, executor, and spin it in a separate thread.

  rclcpp::Node::SharedPtr hw_node_;                                        ///< Private node
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> hw_executor_; ///< Executor for hw_node_
  std::thread spin_thread_;                                                ///< Thread running hw_executor_->spin()
  std::atomic<bool> stop_spin_{false};                                     ///< (unused, kept for future graceful stop)

  // ── Publisher: /arm_joint_commands ─────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;

  // ── Subscriber: /stm32/joint_states ───────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;

  // ── Thread-safe state buffer ──────────────────────────────────────────────
  // Written by joint_state_callback() on the spin thread, read by read() on
  // the controller_manager update thread.

  mutable std::mutex state_mutex_;           ///< Protects received_positions_ and state_received_
  std::vector<double> received_positions_;   ///< Latest positions from STM32
  bool state_received_{false};               ///< True once at least one message arrived

  // ── Callback ──────────────────────────────────────────────────────────────

  /// Process an incoming JointState message from the STM32.
  /// Maps joint names to the internal buffer order (robust to re-ordering).
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}  // namespace mearm_hardware
