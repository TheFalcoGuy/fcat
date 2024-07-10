#include <sched.h>
#include <sys/types.h>
#include <unistd.h>
#include "fcat/fcat_services.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(),  // Default options
      2                           // Number of threads
  );

  auto node = std::make_shared<FcatServices>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
