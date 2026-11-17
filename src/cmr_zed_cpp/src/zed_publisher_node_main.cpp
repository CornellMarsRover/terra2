// Standalone `zed_publisher_node` executable.
#include "cmr_zed_cpp/standalone_main.hpp"
#include "cmr_zed_cpp/zed_camera_publisher.hpp"

int main(int argc, char ** argv)
{
  return cmr_zed_cpp::run_standalone<cmr_zed_cpp::ZEDCameraPublisher>(
    argc, argv, "zed_publisher_node");
}
