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
  ros::Rate loop_rate(100);
  rrbot_hardware_interface::RRBOTHardwareInterface rrbot(nh);
  ros::Time last_time = ros::Time::now();

  while (ros::ok())
  {
    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time = current_time - last_time;
    last_time = current_time;

    rrbot.read();

    rrbot.controller_manager_->update(current_time, elapsed_time);

    rrbot.write();

    loop_rate.sleep();
  }
  // ros::waitForShutdown();

  return 0;
}
