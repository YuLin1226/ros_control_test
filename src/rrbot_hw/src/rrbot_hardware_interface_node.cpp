#include <rrbot_hw/rrbot_hardware_interface.h>
// #include "../include/rrbot_hw/rrbot_hardware_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrbot_hardware_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(1);
  spinner.start();

  rrbot_hardware_interface::RRBOTHardwareInterface rrbot1(nh);

  // ros::spin();
  ros::waitForShutdown();

  return 0;
}
