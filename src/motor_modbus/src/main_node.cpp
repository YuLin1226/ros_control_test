#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <motor_modbus/motor_driver.h>


std::shared_ptr<Motor::MotorDriver> p_motor = std::make_shared<Motor::MotorDriver>("/dev/ttyUSB0", 115200);

void subCallback(const geometry_msgs::Twist msg)
{
    uint8_t num_ = 2;
    std::vector<uint8_t> drive_motor_id_({0x01, 0x02});
    std::vector<uint8_t> steer_motor_id_({0x03, 0x04});
    std::vector<int16_t> cmd_rpm_;
    std::vector<int16_t> index_;
    std::vector<uint16_t> step_;
    if (msg.linear.y < 0){
        index_ = {5, 5};
        step_ = {0, 0};
    }
    else if (msg.linear.y > 0){
        index_ = {-5, -5};
        step_ = {0, 0};
    }
    else{
        index_ = {0,0};
        step_ = {0, 0};
    }
    p_motor->Multi_CMR(num_, steer_motor_id_, index_, step_, false);
    usleep(50000);
    
    if(msg.linear.x > 0){
        cmd_rpm_ = {-100, -100};
    }
    else if(msg.linear.x < 0){
        cmd_rpm_ = {100, 100};
    }
    else{
        cmd_rpm_ = {0, 0};
    }
    p_motor->Multi_JG_Lite(num_, drive_motor_id_, cmd_rpm_, false);
    usleep(50000);
}

void init_encoder(){
    
    /* 尋home待測試 */
    std::vector<uint8_t> steer_motor_id_({0x03, 0x04});
    for(auto i=0; i<steer_motor_id_.size(); i++){
        p_motor->find_Steering_Home(steer_motor_id_[i]);
    }

    /* 未完成尋home用這邊 */
    // uint8_t num_ = 2;
    // std::vector<uint8_t> steer_motor_id_({0x03, 0x04});
    // std::vector<int16_t> index_({0,0});
    // std::vector<uint16_t> step_({0,0});
    // p_motor->Multi_CS(num_, steer_motor_id_, index_, step_, false);
    // usleep(10000);

}

int main(int argc, char **argv)
{
    // ROS 宣告
    ros::init(argc, argv, "motor_current_detection_node");
    ros::NodeHandle n;
    ros::Publisher pub_front = n.advertise<std_msgs::Float64>("front_motor_current", 1000);
    ros::Publisher pub_rear  = n.advertise<std_msgs::Float64>("rear_motor_current", 1000);
    ros::Subscriber sub = n.subscribe("cmd_key", 1000, subCallback);
    ros::Rate loop_rate(10);

    // Modbus 宣告
    p_motor->open();
    init_encoder();
    

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

    

    /*  ===================
    CMD: Read Motor Current
        =================== */
    while (ros::ok())
    {
        try
        {
            // std_msgs::Float64 front_current_data;
            // front_current_data.data = p_motor->get_Current(0x01);
            // pub_front.publish(front_current_data);
            // usleep(50000);

            std_msgs::Float64 rear_current_data;
            rear_current_data.data = p_motor->get_Current(0x02);
            pub_rear.publish(rear_current_data);
            usleep(50000);

            ros::spinOnce();
            loop_rate.sleep();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    

    
    ros::spin();
    return 0;
}