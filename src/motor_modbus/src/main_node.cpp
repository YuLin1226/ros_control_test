#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <motor_modbus/motor_driver.h>


std::shared_ptr<Motor::MotorDriver> p_motor = std::make_shared<Motor::MotorDriver>("/dev/ttyUSB0", 115200);

void subCallback(const geometry_msgs::Twist msg)
{
    uint8_t num_ = 2;
    const int steer_pos = 5;
    const int drive_vel = 200;
    std::vector<uint8_t> drive_motor_id_({0x01, 0x02});
    std::vector<uint8_t> steer_motor_id_({0x03, 0x04});
    std::vector<int16_t> cmd_rpm_;
    std::vector<int16_t> index_;
    std::vector<uint16_t> step_;
    if (msg.linear.y < 0){
        index_ = {steer_pos, steer_pos};
        step_ = {0, 0};
    }
    else if (msg.linear.y > 0){
        index_ = {-steer_pos, -steer_pos};
        step_ = {0, 0};
    }
    else{
        index_ = {0,0};
        step_ = {0, 0};
    }
    p_motor->Multi_CMR(num_, steer_motor_id_, index_, step_, false);
    usleep(50000);
    
    if(msg.linear.x > 0){
        cmd_rpm_ = {-drive_vel, -drive_vel};
    }
    else if(msg.linear.x < 0){
        cmd_rpm_ = {drive_vel, drive_vel};
    }
    else{
        cmd_rpm_ = {0, 0};
    }
    p_motor->Multi_JG_Lite(num_, drive_motor_id_, cmd_rpm_, false);
    usleep(50000);
}

void init_encoder(bool is_find_home){
    
    if(is_find_home){
        /* 使用尋home */
        std::vector<uint8_t> steer_motor_id_({0x03, 0x04});
        for(auto i=0; i<steer_motor_id_.size(); i++){
            p_motor->find_Steering_Home(steer_motor_id_[i]);
            std::cout << "Steer Motor ID." << (uint16_t)steer_motor_id_[i] << " found home.\n";
        }
        std::cout << "Steering Motor Find Home Finished.\n";
    }
    else{
        /* 不使用尋home */
        uint8_t num_ = 2;
        std::vector<uint8_t> steer_motor_id_({0x03, 0x04});
        std::vector<int16_t> index_({0,0});
        std::vector<uint16_t> step_({0,0});
        p_motor->Multi_CS(num_, steer_motor_id_, index_, step_, false);
        usleep(10000);
    }


}

int main(int argc, char **argv)
{
    // ROS 宣告
    ros::init(argc, argv, "motor_current_detection_node");
    ros::NodeHandle n;
    ros::Publisher pub_front_drive = n.advertise<std_msgs::Float64>("front_drive_current", 1000);
    ros::Publisher pub_rear_drive  = n.advertise<std_msgs::Float64>("rear_drive_current", 1000);
    ros::Publisher pub_front_steer = n.advertise<std_msgs::Float64>("front_steer_current", 1000);
    ros::Publisher pub_rear_steer  = n.advertise<std_msgs::Float64>("rear_steer_current", 1000);
    ros::Subscriber sub = n.subscribe("cmd_key", 1000, subCallback);
    ros::Rate loop_rate(10);

    // Modbus 宣告
    p_motor->open();
    init_encoder(true);
    

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
            std_msgs::Float64 front_drive_current_data;
            front_drive_current_data.data = p_motor->get_Current(0x01);
            pub_front_drive.publish(front_drive_current_data);
            usleep(10000);

            std_msgs::Float64 rear_drive_current_data;
            rear_drive_current_data.data = p_motor->get_Current(0x02);
            pub_rear_drive.publish(rear_drive_current_data);
            usleep(10000);

            std_msgs::Float64 front_steer_current_data;
            front_steer_current_data.data = p_motor->get_Current(0x03);
            pub_front_steer.publish(front_steer_current_data);
            usleep(10000);

            std_msgs::Float64 rear_steer_current_data;
            rear_steer_current_data.data = p_motor->get_Current(0x04);
            pub_rear_steer.publish(rear_steer_current_data);
            usleep(10000);

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