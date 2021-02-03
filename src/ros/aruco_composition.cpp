#include <memory>

#include "aruco/Aruco.hpp"
#include "aruco/aruco_tf_producer.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Initialize any global resources needed by the middleware and the client library.
  // This will also parse command line arguments one day (as of Beta 1 they are not used).
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Create an executor that will be responsible for execution of callbacks for a set of nodes.
  // With this version, all callbacks will be called from within this thread (the main one).
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto aruco_detector = std::make_shared<aruco::Aruco>(options);
  exec.add_node(aruco_detector);

  auto tf_pub = std::make_shared<aruco::ArucoTFProducer>(options);
  exec.add_node(tf_pub);

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::Rate rate(30);
  while(rclcpp::ok())
  {
    exec.spin_some();
    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}

