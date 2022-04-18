#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sstream>
#include <motor_modbus/motor_driver.h>

// void chatterCallback(const std_msgs::String::ConstPtr& msg)
// {
//     ROS_INFO("I heard: [%s]", msg->data.c_str());
// }

int main(int argc, char **argv)
{
    // ROS 宣告
    ros::init(argc, argv, "motor_current_detection_node");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float64>("motor_current", 1000);
    ros::Rate loop_rate(10);

    // Modbus 宣告
    std::shared_ptr<Motor::MotorDriver> p_motor = std::make_shared<Motor::MotorDriver>(argc > 1 ? argv[1] : "/dev/ttyUSB0", 115200);
    p_motor->open();

    // // CMD: Motor Move.
    // p_motor -> JG(300, false);
    // sleep(1);

    // TEST: Multi_CMD_Lite
    uint8_t num_ = 2;
    std::vector<uint8_t> motor_id_({0x01, 0x02});
    std::vector<uint16_t> cmd_rpm_({100, 100});
    std::vector<uint16_t> echo_({0x7F, 0x7F});

    p_motor->Multi_JG_Lite(num_, motor_id_, cmd_rpm_, echo_);
    // p_motor -> JG_Lite(300, false);
    sleep(1);


    // CMD: Read Motor Current
    while (ros::ok())
    {
        try
        {
            std_msgs::Float64 current_data;
            current_data.data = p_motor->get_Current();
            pub.publish(current_data);
            ros::spinOnce();
            loop_rate.sleep();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    

    // ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    // ros::spin();

    return 0;
}