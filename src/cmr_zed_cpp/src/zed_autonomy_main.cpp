// Standalone `zed_autonomy` executable.
#include "cmr_zed_cpp/standalone_main.hpp"
#include "cmr_zed_cpp/zed_autonomy.hpp"

int main(int argc, char ** argv)
{
  return cmr_zed_cpp::run_standalone<cmr_zed_cpp::ZedAutonomy>(argc, argv, "zed_autonomy");
}
