// Standalone `threaded` executable (threaded.py spins MultiThreadedExecutor(num_threads=4)).
#include "cmr_zed_cpp/standalone_main.hpp"
#include "cmr_zed_cpp/zed_threaded.hpp"

int main(int argc, char ** argv)
{
  return cmr_zed_cpp::run_standalone<cmr_zed_cpp::ZedThreaded,
           rclcpp::executors::MultiThreadedExecutor>(argc, argv, "threaded", 4);
}
