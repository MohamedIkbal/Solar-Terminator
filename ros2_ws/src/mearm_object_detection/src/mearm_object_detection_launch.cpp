// C++ replacement for launch/mearm_object_detection.launch.py
//
// ROS2 launch files are conventionally Python; there is no native C++ launch
// system. This program reproduces the same behaviour as the Python launch
// file by starting the detection node directly via `ros2 run`, with the
// same default paths and the ability to override each one, matching the
// launch arguments camera_index / params_file / model_path / labels_path /
// unknown_image_dir.
//
// Usage:
//   ros2 run mearm_object_detection mearm_object_detection_launch \
//       camera_index:=0 \
//       params_file:=/path/to/params.yaml \
//       model_path:=/path/to/space_object_classifier.onnx \
//       labels_path:=/path/to/labels.txt \
//       unknown_image_dir:=/path/to/unknown

#include <cstdlib>
#include <csignal>
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

bool parse_kv(const std::string & arg, const std::string & key, std::string & out)
{
  const std::string prefix = key + ":=";
  if (arg.compare(0, prefix.size(), prefix) == 0) {
    out = arg.substr(prefix.size());
    return true;
  }
  return false;
}

std::string home_dir()
{
  const char * home = std::getenv("HOME");
  return home ? std::string(home) : std::string("");
}
}  // namespace

int main(int argc, char ** argv)
{
  // index 0 is the Surface's IR/Windows-Hello sensor on this machine (no usable
  // color image); the real RGB "Integrated Camera" is index 4
  std::string camera_index = "4";
  std::string params_file;
  std::string model_path;
  std::string labels_path;
  std::string unknown_image_dir =
    home_dir() + "/ros2_ws/src/mearm_object_detection/training_images/unknown";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    std::string value;
    if (parse_kv(arg, "camera_index", value)) {
      camera_index = value;
    } else if (parse_kv(arg, "params_file", value)) {
      params_file = value;
    } else if (parse_kv(arg, "model_path", value)) {
      model_path = value;
    } else if (parse_kv(arg, "labels_path", value)) {
      labels_path = value;
    } else if (parse_kv(arg, "unknown_image_dir", value)) {
      unknown_image_dir = value;
    }
  }

  std::string pkg_share;
  try {
    pkg_share = ament_index_cpp::get_package_share_directory("mearm_object_detection");
  } catch (const std::exception & e) {
    std::cerr << "mearm_object_detection_launch: could not resolve package share dir: "
              << e.what() << std::endl;
    return 1;
  }

  if (params_file.empty()) {
    params_file = pkg_share + "/config/mearm_object_detection_params.yaml";
  }
  if (model_path.empty()) {
    model_path = pkg_share + "/models/space_object_classifier.onnx";
  }
  if (labels_path.empty()) {
    labels_path = pkg_share + "/models/labels.txt";
  }

  std::signal(SIGINT, forward_signal);
  std::signal(SIGTERM, forward_signal);

  pid_t pid = fork();
  if (pid < 0) {
    std::perror("fork");
    return 1;
  }

  if (pid == 0) {
    execlp(
      "ros2", "ros2", "run", "mearm_object_detection", "mearm_object_detection_node",
      "--ros-args",
      "--params-file", params_file.c_str(),
      "-p", ("camera_index:=" + camera_index).c_str(),
      "-p", ("model_path:=" + model_path).c_str(),
      "-p", ("labels_path:=" + labels_path).c_str(),
      "-p", ("unknown_image_dir:=" + unknown_image_dir).c_str(),
      static_cast<char *>(nullptr));
    std::perror("execlp");
    _exit(127);
  }

  g_child_pid = pid;
  std::cout << "[mearm_object_detection_launch] started mearm_object_detection_node (pid "
            << pid << ")\n"
            << "  params_file=" << params_file << "\n"
            << "  model_path=" << model_path << "\n"
            << "  labels_path=" << labels_path << "\n"
            << "  unknown_image_dir=" << unknown_image_dir << "\n"
            << "  camera_index=" << camera_index << std::endl;

  int status = 0;
  waitpid(pid, &status, 0);
  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  return 1;
}
