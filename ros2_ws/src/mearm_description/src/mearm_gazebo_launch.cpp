// C++ replacement for launch/gazebo.launch.py
//
// ROS2 launch files are conventionally Python; there is no native C++ launch
// system, so this program reproduces the Python launch file's behaviour by
// forking and exec'ing the same processes in the same order, with the same
// startup delays (TimerAction periods) the original used.
//
// Usage:
//   ros2 run mearm_description mearm_gazebo_launch

#include <array>
#include <chrono>
#include <csignal>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <sys/wait.h>
#include <unistd.h>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace
{
std::vector<pid_t> g_children;

void forward_signal(int sig)
{
  for (pid_t pid : g_children) {
    if (pid > 0) {
      kill(pid, sig);
    }
  }
}

// Run a shell command and capture stdout (used for `xacro <file>`).
std::string capture_output(const std::string & cmd)
{
  std::array<char, 4096> buffer{};
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen failed for: " + cmd);
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

// Fork+exec a process with an argv list; optionally add/override environment
// variables (mirrors ExecuteProcess's additional_env).
pid_t spawn(
  const std::vector<std::string> & args,
  const std::vector<std::pair<std::string, std::string>> & extra_env = {})
{
  pid_t pid = fork();
  if (pid < 0) {
    std::perror("fork");
    return -1;
  }
  if (pid == 0) {
    for (const auto & kv : extra_env) {
      setenv(kv.first.c_str(), kv.second.c_str(), 1);
    }
    std::vector<char *> argv;
    argv.reserve(args.size() + 1);
    for (const auto & a : args) {
      argv.push_back(const_cast<char *>(a.c_str()));
    }
    argv.push_back(nullptr);
    execvp(argv[0], argv.data());
    std::perror("execvp");
    _exit(127);
  }
  return pid;
}

// Write robot_description and use_sim_time to a temporary YAML params file
// for robot_state_publisher, avoiding passing large URDF strings on the
// command line.
std::string write_robot_description_params_file(
  const std::string & robot_description,
  const std::string & use_sim_time)
{
  std::string path_template = "/tmp/mearm_gazebo_rsp_params.XXXXXX";
  std::vector<char> buf(path_template.begin(), path_template.end());
  buf.push_back('\0');
  int fd = mkstemp(buf.data());
  if (fd == -1) {
    throw std::runtime_error("mkstemp failed for robot_state_publisher params file");
  }
  std::string path(buf.data());

  FILE * f = fdopen(fd, "w");
  if (!f) {
    throw std::runtime_error("fdopen failed for robot_state_publisher params file");
  }

  std::ostringstream yaml;
  yaml << "robot_state_publisher:\n";
  yaml << "  ros__parameters:\n";
  yaml << "    use_sim_time: " << use_sim_time << "\n";
  yaml << "    robot_description: |\n";

  std::istringstream lines(robot_description);
  std::string line;
  while (std::getline(lines, line)) {
    yaml << "      " << line << "\n";
  }

  const std::string yaml_str = yaml.str();
  std::fwrite(yaml_str.data(), 1, yaml_str.size(), f);
  std::fclose(f);

  return path;
}
}  // namespace

int main(int, char **)
{
  const std::string pkg_share =
    ament_index_cpp::get_package_share_directory("mearm_description");
  const std::string xacro_path = pkg_share + "/urdf/mearm.urdf.xacro";
  const std::string world_path = pkg_share + "/worlds/mearm_world.sdf";
  const std::string controllers_yaml = pkg_share + "/config/mearm_controllers.yaml";

  // Replicates the resource/physics-engine path logic from gazebo.launch.py.
  const char * ros_distro_env = std::getenv("ROS_DISTRO");
  const std::string ros_prefix =
    ros_distro_env ? ("/opt/ros/" + std::string(ros_distro_env)) : "/opt/ros/lyrical";

  // pkg_share looks like <prefix>/share/mearm_description; the resource path
  // needs to point at <prefix>/share so model://mearm_description/... resolves.
  const std::string gz_resource_path = pkg_share.substr(0, pkg_share.find_last_of('/'));
  const std::string gz_physics_engine_path =
    ros_prefix + "/opt/gz_physics_vendor/lib/gz-physics-9/engine-plugins";

  std::string merged_resource_path = gz_resource_path;
  if (const char * existing = std::getenv("GZ_SIM_RESOURCE_PATH")) {
    if (existing[0] != '\0') {
      merged_resource_path = gz_resource_path + ":" + existing;
    }
  }

  // 1) Start Gazebo (gz sim) with our world.
  pid_t gz_pid = spawn(
    {"gz", "sim", "-r", world_path},
    {
      {"GZ_SIM_RESOURCE_PATH", merged_resource_path},
      {"GZ_SIM_PHYSICS_ENGINE_PATH", gz_physics_engine_path},
    });
  g_children.push_back(gz_pid);
  std::cout << "[mearm_gazebo_launch] gz sim started (pid " << gz_pid << ")" << std::endl;

  std::signal(SIGINT, forward_signal);
  std::signal(SIGTERM, forward_signal);

  // 2) robot_state_publisher publishes /robot_description and TF.
  std::string robot_description;
  try {
    robot_description = capture_output("xacro " + xacro_path);
  } catch (const std::exception & e) {
    std::cerr << "[mearm_gazebo_launch] failed to run xacro: " << e.what() << std::endl;
    return 1;
  }

  std::string rsp_params_file;
  try {
    rsp_params_file = write_robot_description_params_file(robot_description, "true");
  } catch (const std::exception & e) {
    std::cerr << "[mearm_gazebo_launch] failed to write rsp params file: " << e.what()
              << std::endl;
    return 1;
  }

  pid_t rsp_pid = spawn(
    {"ros2", "run", "robot_state_publisher", "robot_state_publisher",
      "--ros-args", "--params-file", rsp_params_file});
  g_children.push_back(rsp_pid);
  std::cout << "[mearm_gazebo_launch] robot_state_publisher started (pid " << rsp_pid << ")"
            << std::endl;

  // 3) Bridge /clock from Gazebo to ROS so use_sim_time works.
  pid_t clock_bridge_pid = spawn(
    {"ros2", "run", "ros_gz_bridge", "parameter_bridge",
      "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"});
  g_children.push_back(clock_bridge_pid);
  std::cout << "[mearm_gazebo_launch] clock bridge started (pid " << clock_bridge_pid << ")"
            << std::endl;

  // 4) Spawn the robot into Gazebo once it has started (TimerAction period=5.0).
  std::this_thread::sleep_for(std::chrono::seconds(5));
  pid_t spawn_pid = spawn(
    {"ros2", "run", "ros_gz_sim", "create",
      "-topic", "/robot_description", "-name", "mearm", "-z", "0.0"});
  g_children.push_back(spawn_pid);
  std::cout << "[mearm_gazebo_launch] robot spawn requested (pid " << spawn_pid << ")"
            << std::endl;

  // 5) Spawn controllers once controller_manager (hosted by gz_ros2_control) is up.
  std::this_thread::sleep_for(std::chrono::seconds(3));  // total 8.0s from start
  pid_t jsb_pid = spawn(
    {"ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster"});
  g_children.push_back(jsb_pid);
  std::cout << "[mearm_gazebo_launch] joint_state_broadcaster spawner started (pid "
            << jsb_pid << ")" << std::endl;

  std::this_thread::sleep_for(std::chrono::seconds(1));  // total 9.0s from start
  pid_t arm_pid = spawn(
    {"ros2", "run", "controller_manager", "spawner", "arm_controller",
      "--param-file", controllers_yaml});
  g_children.push_back(arm_pid);
  std::cout << "[mearm_gazebo_launch] arm_controller spawner started (pid " << arm_pid << ")"
            << std::endl;

  std::this_thread::sleep_for(std::chrono::seconds(1));  // total 10.0s from start
  pid_t gripper_pid = spawn(
    {"ros2", "run", "controller_manager", "spawner", "gripper_controller",
      "--param-file", controllers_yaml});
  g_children.push_back(gripper_pid);
  std::cout << "[mearm_gazebo_launch] gripper_controller spawner started (pid "
            << gripper_pid << ")" << std::endl;

  // Wait on the long-lived processes (gz sim, robot_state_publisher, clock bridge).
  int status = 0;
  waitpid(gz_pid, &status, 0);
  forward_signal(SIGTERM);
  for (pid_t pid : g_children) {
    if (pid > 0 && pid != gz_pid) {
      waitpid(pid, nullptr, WNOHANG);
    }
  }
  return WIFEXITED(status) ? WEXITSTATUS(status) : 1;
}
