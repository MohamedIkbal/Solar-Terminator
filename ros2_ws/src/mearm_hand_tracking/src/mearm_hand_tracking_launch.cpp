// C++ replacement for launch/mearm_hand_tracking.launch.py
//
// ROS2 launch files are conventionally Python; there is no native C++ launch
// system. This program reproduces the same behaviour as the Python launch
// file by starting the node directly via `ros2 run`, passing the same
// parameters file and camera_index argument.
//
// Usage (mirrors the launch arguments):
//   ros2 run mearm_hand_tracking mearm_hand_tracking_launch \
//       camera_index:=0 \
//       params_file:=/path/to/mearm_hand_tracking_params.yaml

#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace
{
pid_t g_child_pid = -1;

void forward_signal(int sig)
{
  if (g_child_pid > 0) {
    kill(g_child_pid, sig);
  }
}

// Parse a `key:=value` launch-style argument.
bool parse_kv(const std::string & arg, const std::string & key, std::string & out)
{
  const std::string prefix = key + ":=";
  if (arg.compare(0, prefix.size(), prefix) == 0) {
    out = arg.substr(prefix.size());
    return true;
  }
  return false;
}
}  // namespace

int main(int argc, char ** argv)
{
  // index 0 is the Surface's IR/Windows-Hello sensor on this machine (no usable
  // color image); the real RGB "Integrated Camera" is index 4
  std::string camera_index = "4";
  std::string params_file;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    std::string value;
    if (parse_kv(arg, "camera_index", value)) {
      camera_index = value;
    } else if (parse_kv(arg, "params_file", value)) {
      params_file = value;
    }
  }

  if (params_file.empty()) {
    try {
      const std::string pkg_share =
        ament_index_cpp::get_package_share_directory("mearm_hand_tracking");
      params_file = pkg_share + "/config/mearm_hand_tracking_params.yaml";
    } catch (const std::exception & e) {
      std::cerr << "mearm_hand_tracking_launch: could not resolve package share dir: "
                << e.what() << std::endl;
      return 1;
    }
  }

  std::signal(SIGINT, forward_signal);
  std::signal(SIGTERM, forward_signal);

  pid_t pid = fork();
  if (pid < 0) {
    std::perror("fork");
    return 1;
  }

  if (pid == 0) {
    // Child: exec the node exactly as the launch file would.
    execlp(
      "ros2", "ros2", "run", "mearm_hand_tracking", "mearm_hand_tracking_node",
      "--ros-args",
      "--params-file", params_file.c_str(),
      "-p", ("camera_index:=" + camera_index).c_str(),
      static_cast<char *>(nullptr));
    std::perror("execlp");
    _exit(127);
  }

  g_child_pid = pid;
  std::cout << "[mearm_hand_tracking_launch] started mearm_hand_tracking_node (pid "
            << pid << ") with params_file=" << params_file
            << " camera_index=" << camera_index << std::endl;

  int status = 0;
  waitpid(pid, &status, 0);
  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  return 1;
}
