#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <motor_modbus/motor_driver.h>


std::shared_ptr<Motor::MotorDriver> p_motor = std::make_shared<Motor::MotorDriver>("/dev/ttyUSB0", 115200);


double v_limit(double v, double v_up_bound, double v_low_bound){
    if(v > v_up_bound) v = v_up_bound;

    if(v < v_low_bound) v = v_low_bound;

    return v;
}


void subCallback_kinematic(const geometry_msgs::Twist cmd)
{
    double D = 1.35;
    double radius = 0.1;
    int gear_retio = 24;
    int steer_ratio = 350;

    double v_x = cmd.linear.x;
    double v_y = cmd.linear.y;
    double w   = cmd.angular.z;

    double v1x = v_x;
    double v1y = v_y + w*D/2;
    double v2x = v_x;
    double v2y = v_y - w*D/2;

    double v_1 = std::sqrt(v1x*v1x+v1y*v1y);
    double v_2 = std::sqrt(v2x*v2x+v2y*v2y);
    double theta_1 = std::atan2(v1y, v1x);
    double theta_2 = std::atan2(v2y, v2x);

    double v_upper_bound = 500.0;
    double v_lower_bound = 0.0;

    v_1 = v_limit(v_1, v_upper_bound, v_lower_bound);
    v_2 = v_limit(v_2, v_upper_bound, v_lower_bound);


    if(theta_1 > M_PI/2){
        theta_1 -= M_PI;
        v_1 *= -1;
    }
    else if(theta_1 < -M_PI/2){
        theta_1 += M_PI;
        v_1 *= -1;
    }

    if(theta_2 > M_PI/2){
        theta_2 -= M_PI;
        v_2 *= -1;
    }
    else if(theta_2 < -M_PI/2){
        theta_2 += M_PI;
        v_2 *= -1;
    }


    uint8_t num_ = 2;
    int16_t front_rpm = v_1*gear_retio/radius;
    int16_t rear_rpm  = v_2*gear_retio/radius;

    
    double front_steer_pos = theta_1*steer_ratio/(2.0*M_PI);
    double rear_steer_pos  = theta_2*steer_ratio/(2.0*M_PI);
    
    std::cout << "VEL: (" << front_rpm << " , " << rear_rpm << ")\n";
    std::cout << "POS: (" << front_steer_pos << " , " << rear_steer_pos << ")\n";
    
    int16_t  front_index, rear_index;
    uint16_t front_step,  rear_step;
    if(front_steer_pos < 0){
        front_index = front_steer_pos-1;
        front_step  = (front_steer_pos - front_index)*10000;
    }
    else{
        front_index = front_steer_pos;
        front_step  = (front_steer_pos - front_index)*10000;
    }

    if(rear_steer_pos < 0){
        rear_index = rear_steer_pos-1;
        rear_step  = (rear_steer_pos - rear_index)*10000;
    }
    else{
        rear_index = rear_steer_pos;
        rear_step  = (rear_steer_pos - rear_index)*10000;
    }
    std::vector<uint8_t> drive_motor_id_({0x02, 0x01});
    std::vector<uint8_t> steer_motor_id_({0x03, 0x04});
    std::vector<int16_t> cmd_rpm_({front_rpm, rear_rpm});
    std::vector<int16_t> index_({front_index, rear_index});
    std::vector<uint16_t> step_({front_step, rear_step});

    p_motor->Multi_CMA(num_, steer_motor_id_, index_, step_, false);
    usleep(10000);
    p_motor->Multi_JG_Lite(num_, drive_motor_id_, cmd_rpm_, false);
    usleep(10000);
}

void subCallback(const geometry_msgs::Twist msg)
{
    uint8_t num_ = 2;
    const int steer_idx = 5;
    const int steer_stp = 0;
    const int drive_vel = 500;
    std::vector<uint8_t> drive_motor_id_({0x02, 0x01});
    std::vector<uint8_t> steer_motor_id_({0x03, 0x04});
    std::vector<int16_t> cmd_rpm_;
    std::vector<int16_t> index_;
    std::vector<uint16_t> step_;
    if (msg.linear.y < 0){
        index_ = {steer_idx, steer_idx};
        step_ = {steer_stp, steer_stp};
    }
    else if (msg.linear.y > 0){
        index_ = {-steer_idx, -steer_idx};
        step_ = {steer_stp, steer_stp};
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
    ros::Subscriber sub;
    ros::Publisher pub_front_drive;
    ros::Publisher pub_rear_drive;
    ros::Publisher pub_front_steer;
    ros::Publisher pub_rear_steer;

    bool isFindHome;
    bool isPubCurrent;
    bool isUseModbus;
    int subMode;
    n.param<bool>("/dsr/isFindHome"   , isFindHome, true);
    n.param<bool>("/dsr/isPubCurrent" , isPubCurrent, true);
    n.param<bool>("/dsr/isUseModbus" , isUseModbus, true);
    n.param<int>("/dsr/subMode", subMode, 1);



    if(isPubCurrent)
    {
        pub_front_drive = n.advertise<std_msgs::Float64>("front_drive_current", 1000);
        pub_rear_drive  = n.advertise<std_msgs::Float64>("rear_drive_current", 1000);
        pub_front_steer = n.advertise<std_msgs::Float64>("front_steer_current", 1000);
        pub_rear_steer  = n.advertise<std_msgs::Float64>("rear_steer_current", 1000);
    }
    
    if(isUseModbus){
        if(subMode == 1){
            sub = n.subscribe("cmd_key", 1000, subCallback);
        }
        else if(subMode == 2){
            sub = n.subscribe("cmd_vel", 1000, subCallback_kinematic);
        }
    }
    
    ros::Rate loop_rate(10);
    

    // Modbus 宣告
    if(isUseModbus){
        p_motor->open();
        init_encoder(isFindHome);
    }
    

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
            if(isPubCurrent && isUseModbus){
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
            }

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