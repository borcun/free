// Include the ROS C++ APIs
#include <ros/ros.h>

// Standard C++ entry point
int main(int argc, char** argv) 
{
  // Announce this program to the ROS master as a "node" called "hello_ros"
  ros::init(argc, argv, "hello_ros");
  // Start the node resource managers (communication, time, etc)
  ros::start();
  // Broadcast a simple log message
  ROS_INFO_STREAM("Hello, ROS!\n");
  // Process ROS callbacks until receiving a SIGINT (ctrl-c)
	ros::spin();
  // Stop the node's resources
  ros::shutdown();
  // Exit tranquilly
  return 0;
}
