// C++ replacement for launch/display.launch.py
//
// RViz-only visualization of the MeArm (NO Gazebo, NO ros2_control).
// Starts, in order: robot_state_publisher, joint_state_publisher_gui, and
// (after a short delay, matching the original TimerAction) rviz2.
//
// Usage:
//   ros2 run mearm_description mearm_display_launch [use_sim_time:=false] [rviz_config:=/path]

#include <array>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>
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

pid_t spawn(const std::vector<std::string> & args)
{
  pid_t pid = fork();
  if (pid < 0) {
    std::perror("fork");
    return -1;
  }
  if (pid == 0) {
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

bool parse_kv(const std::string & arg, const std::string & key, std::string & out)
{
  const std::string prefix = key + ":=";
  if (arg.compare(0, prefix.size(), prefix) == 0) {
    out = arg.substr(prefix.size());
    return true;
  }
  return false;
}

std::string write_robot_description_params_file(
  const std::string & robot_description, const std::string & use_sim_time)
{
  std::string path_template = "/tmp/mearm_display_rsp_params.XXXXXX";
  std::vector<char> buf(path_template.begin(), path_template.end());
  buf.push_back('\0');

  int fd = mkstemp(buf.data());
  if (fd == -1) {
    throw std::runtime_error("mkstemp failed for robot_state_publisher params file");
  }

  FILE * f = fdopen(fd, "w");
  if (!f) {
    throw std::runtime_error("fdopen failed for robot_state_publisher params file");
  }

  std::ostringstream oss;
  oss << "robot_state_publisher:\n";
  oss << "  ros__parameters:\n";
  oss << "    use_sim_time: " << use_sim_time << "\n";
  oss << "    robot_description: |\n";

  std::istringstream iss(robot_description);
  std::string line;
  while (std::getline(iss, line)) {
    oss << "      " << line << "\n";
  }

  const std::string content = oss.str();
  fwrite(content.data(), 1, content.size(), f);
  fclose(f);

  return std::string(buf.data());
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string use_sim_time = "false";
  std::string rviz_config;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    std::string value;
    if (parse_kv(arg, "use_sim_time", value)) {
      use_sim_time = value;
    } else if (parse_kv(arg, "rviz_config", value)) {
      rviz_config = value;
    }
  }

  const std::string pkg_share =
    ament_index_cpp::get_package_share_directory("mearm_description");
  const std::string xacro_path = pkg_share + "/urdf/mearm.urdf.xacro";
  if (rviz_config.empty()) {
    rviz_config = pkg_share + "/rviz/mearm_display.rviz";
  }

  std::string robot_description;
  try {
    robot_description = capture_output("xacro " + xacro_path);
  } catch (const std::exception & e) {
    std::cerr << "[mearm_display_launch] failed to run xacro: " << e.what() << std::endl;
    return 1;
  }

  std::signal(SIGINT, forward_signal);
  std::signal(SIGTERM, forward_signal);

  std::string rsp_params_file;
  try {
    rsp_params_file = write_robot_description_params_file(robot_description, use_sim_time);
  } catch (const std::exception & e) {
    std::cerr << "[mearm_display_launch] failed to write params file: " << e.what() << std::endl;
    return 1;
  }
  // 1) robot_state_publisher
  pid_t rsp_pid = spawn(
    {"ros2", "run", "robot_state_publisher", "robot_state_publisher",
      "--ros-args", "--params-file", rsp_params_file});
  g_children.push_back(rsp_pid);
  std::cout << "[mearm_display_launch] robot_state_publisher started (pid " << rsp_pid << ")"
            << std::endl;

  // 2) joint_state_publisher_gui
  pid_t jsp_pid = spawn(
    {"ros2", "run", "joint_state_publisher_gui", "joint_state_publisher_gui",
      "--ros-args", "-p", "use_sim_time:=" + use_sim_time});
  g_children.push_back(jsp_pid);
  std::cout << "[mearm_display_launch] joint_state_publisher_gui started (pid "
            << jsp_pid << ")" << std::endl;

  // 3) rviz2, staggered 3s after the Qt GUI starts up (matches TimerAction period=3.0).
  std::this_thread::sleep_for(std::chrono::seconds(3));
  pid_t rviz_pid = spawn(
    {"ros2", "run", "rviz2", "rviz2",
      "-d", rviz_config,
      "--ros-args", "-p", "use_sim_time:=" + use_sim_time});
  g_children.push_back(rviz_pid);
  std::cout << "[mearm_display_launch] rviz2 started (pid " << rviz_pid << ")" << std::endl;

  int status = 0;
  waitpid(rviz_pid, &status, 0);
  forward_signal(SIGTERM);
  return WIFEXITED(status) ? WEXITSTATUS(status) : 1;
}
