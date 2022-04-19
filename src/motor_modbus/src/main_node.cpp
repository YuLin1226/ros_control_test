#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sstream>
#include <motor_modbus/motor_driver.h>


std::shared_ptr<Motor::MotorDriver> p_motor = std::make_shared<Motor::MotorDriver>("/dev/ttyUSB0", 115200);

void subCallback(const std_msgs::Float64 msg)
{
    uint8_t num_ = 2;
    std::vector<uint8_t> motor_id_({0x01, 0x02});
    std::vector<int16_t> cmd_rpm_;
    if(msg.data > 0){
        cmd_rpm_ = {100, 100};
    }
    else if(msg.data < 0){
        cmd_rpm_ = {-100, -100};
    }
    else{
        cmd_rpm_ = {0, 0};
    }
    p_motor->Multi_JG_Lite(num_, motor_id_, cmd_rpm_, true);
}

int main(int argc, char **argv)
{
    // ROS 宣告
    ros::init(argc, argv, "motor_current_detection_node");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float64>("motor_current", 1000);
    ros::Subscriber sub = n.subscribe("cmd_key", 1000, subCallback);
    ros::Rate loop_rate(10);

    // Modbus 宣告
    
    p_motor->open();

    /*  ===============
        CMD: Motor Move
        =============== */
    // p_motor -> JG(300, false);
    // sleep(1);

    /*  ====================
        TEST: Multi_CMD_Lite
        ==================== */
    // uint8_t num_ = 2;
    // std::vector<uint8_t> motor_id_({0x01, 0x02});
    // std::vector<uint16_t> cmd_rpm_({100, 100});

    // p_motor->Multi_JG_Lite(num_, motor_id_, cmd_rpm_, true);
    // // p_motor -> JG_Lite(300, false);
    // sleep(1);

    /*  ===============
        TEST: Multi_CMD
        =============== */
    
    // uint8_t num_ = 2;
    // std::vector<uint8_t> motor_id_({0x01, 0x02});
    // // std::vector<int16_t> cmd_rpm_({100, 100});

    // std::vector<int16_t> i_({0, 0});
    // std::vector<uint16_t> s_({0, 0});

    // // p_motor->Multi_JG(num_, motor_id_, cmd_rpm_, true);
    // p_motor->Multi_CS(num_, motor_id_, i_, s_, true);

    // sleep(1);
    // std::vector<int16_t> index_({10, 10});
    // std::vector<uint16_t> step_({5000, 5000});

    // // p_motor->Multi_JG(num_, motor_id_, cmd_rpm_, true);
    // // p_motor->Multi_CMA(num_, motor_id_, index_, step_, true);
    // p_motor -> JG_Lite(300, false);
    // sleep(1);

    


    // CMD: Read Motor Current
    // while (ros::ok())
    // {
    //     try
    //     {
    //         std_msgs::Float64 current_data;
    //         current_data.data = p_motor->get_Current();
    //         pub.publish(current_data);
    //         ros::spinOnce();
    //         loop_rate.sleep();
    //     }
    //     catch(const std::exception& e)
    //     {
    //         std::cerr << e.what() << '\n';
    //     }
    // }

    

    
    ros::spin();
    return 0;
}