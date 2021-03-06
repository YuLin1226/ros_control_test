#include <sstream>
// #include <rrbot_hw/rrbot_hardware_interface.h>
#include <rrbot_hw/rrbot_hardware_interface.h>
// #include <joint_limits_interface/joint_limits_interface.h>
// #include <joint_limits_interface/joint_limits.h>
// #include <joint_limits_interface/joint_limits_urdf.h>
// #include <joint_limits_interface/joint_limits_rosparam.h>
// #include <tr1cpp/tr1.h>
// #include <tr1cpp/joint.h>
#include <cmath>

using namespace hardware_interface;
// using joint_limits_interface::JointLimits;
// using joint_limits_interface::SoftJointLimits;
// using joint_limits_interface::PositionJointSoftLimitsHandle;
// using joint_limits_interface::PositionJointSoftLimitsInterface;


namespace rrbot_hardware_interface
{

    RRBOTHardwareInterface::RRBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh), is_echo(true)
	{
		init();
		std::cout << "\n\ninit RRBOT HW...\n\n";
		controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
		nh_.param("/rrbot/hardware_interface/loop_hz", loop_hz_, 1.0);
		ROS_DEBUG_STREAM_NAMED("constructor","Using loop freqency of " << loop_hz_ << " hz");
		ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
		non_realtime_loop_ = nh_.createTimer(update_freq, &RRBOTHardwareInterface::update, this);
		ROS_INFO_NAMED("hardware_interface", "Loaded generic_hardware_interface.");
		zero_encoder();
	}
    
    RRBOTHardwareInterface::~RRBOTHardwareInterface()
	{
	}


	void RRBOTHardwareInterface::init()
	{
		serial_setup("/dev/ttyUSB0", 115200);
        
	   	hardware_interface::JointStateHandle state_handle_a("joint1", &joints_.position, &joints_.velocity, &joints_.effort);
   		jnt_state_interface.registerHandle(state_handle_a);
		registerInterface(&jnt_state_interface);

        hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("joint1"), &joints_.position_command);
        jnt_pos_interface.registerHandle(pos_handle_a);
        registerInterface(&jnt_pos_interface);

	}

	void RRBOTHardwareInterface::serial_setup(std::string port_name, int baud_rate)
	{
		multi_drive_ = std::make_shared<Motor::MotorDriver>(port_name, baud_rate);
		multi_drive_->open();
	}
	
	void RRBOTHardwareInterface::zero_encoder(){
		multi_drive_->ISTOP(false);
		usleep(10000);
		// while(multi_drive_->get_Encoder() != 0){
		// 	multi_drive_->CS(0, 0, false);
		// 	usleep(10000);
		// }
		joints_.init_position =	multi_drive_->get_Encoder();
		usleep(10000);
	}

    void RRBOTHardwareInterface::update(const ros::TimerEvent& e)
	{
		std::cout <<"--------UPDATE Start--------\n";
		elapsed_time_ = ros::Duration(e.current_real - e.last_real);
		read();
		controller_manager_->update(ros::Time::now(), elapsed_time_);
		write();
		std::cout <<"--------UPDATE End--------\n";
	}


	void RRBOTHardwareInterface::read()
	{
		joints_.position = multi_drive_->get_Encoder() - joints_.init_position;	
		std::cout <<"position: " << joints_.position << std::endl;
		usleep(20000);
	}

	void RRBOTHardwareInterface::write()
	{
		// uint16_t cmd_rpm = joints_.velocity_command;
		// multi_drive_->JG(cmd_rpm, false);

		// ???????????? CMR?????????????????????pid?????????
		double cmd_pos	= (joints_.position_command)/(2.0*M_PI);
		int16_t _index;
		uint16_t _step;
		if(cmd_pos < 0){
			_index = cmd_pos-1;
			_step  = (cmd_pos - _index)*10000;
		}
		else{
			_index = cmd_pos;
			_step  = (cmd_pos - _index)*10000;
		}
		
		std::cout << std::dec <<"command: " << joints_.position_command << std::endl;
		std::cout << std::dec <<"command_index: " << _index << "\ncommand_step: " << _step << std::endl;

		multi_drive_->CMR(_index, _step, false);
		usleep(20000);

		// // ???????????? CMA????????????????????????pid??????p=1, i=d=0
		// double cmd_pos	= (joints_.position_command)/(2.0*M_PI);
		// int16_t _index;
		// uint16_t _step;
		// if(cmd_pos < 0){
		// 	_index = cmd_pos-1;
		// 	_step  = (cmd_pos - _index)*10000;
		// }
		// else{
		// 	_index = cmd_pos;
		// 	_step  = (cmd_pos - _index)*10000;
		// }
		
		// std::cout << std::dec <<"command: " << joints_.position_command << std::endl;
		// std::cout << std::hex <<"command_index: " << _index << "\ncommand_step: " << _step << std::endl;

		// multi_drive_->CMA(_index, _step, false);
		// usleep(20000);
	}

} // namespace









// namespace tr1_hardware_interface
// {
// 	TR1HardwareInterface::TR1HardwareInterface(ros::NodeHandle& nh) \
// 		: nh_(nh)
// 	{
// 		init();
// 		controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

// 		nh_.param("/tr1/hardware_interface/loop_hz", loop_hz_, 0.1);
// 		ROS_DEBUG_STREAM_NAMED("constructor","Using loop freqency of " << loop_hz_ << " hz");
// 		ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
// 		non_realtime_loop_ = nh_.createTimer(update_freq, &TR1HardwareInterface::update, this);

// 		ROS_INFO_NAMED("hardware_interface", "Loaded generic_hardware_interface.");
// 	}

// 	TR1HardwareInterface::~TR1HardwareInterface()
// 	{
// 	}

// 	void TR1HardwareInterface::init()
// 	{
// 		//joint_mode_ = 3; // ONLY EFFORT FOR NOW
// 		// Get joint names
// 		nh_.getParam("/tr1/hardware_interface/joints", joint_names_);
// 		if (joint_names_.size() == 0)
// 		{
// 		  ROS_FATAL_STREAM_NAMED("init","No joints found on parameter server for controller. Did you load the proper yaml file?");
// 		}
// 		num_joints_ = joint_names_.size();

// 		// Resize vectors
// 		joint_position_.resize(num_joints_);
// 		joint_velocity_.resize(num_joints_);
// 		joint_effort_.resize(num_joints_);
// 		joint_position_command_.resize(num_joints_);
// 		joint_velocity_command_.resize(num_joints_);
// 		joint_effort_command_.resize(num_joints_);


// 		// Initialize controller
// 		for (int i = 0; i < num_joints_; ++i)
// 		{
// 			tr1cpp::Joint joint = tr1.getJoint(joint_names_[i]);

// 		  ROS_DEBUG_STREAM_NAMED("constructor","Loading joint name: " << joint.name);

// 			nh_.getParam("/tr1/joint_offsets/" + joint.name, joint.angleOffset);
// 			nh_.getParam("/tr1/joint_read_ratio/" + joint.name, joint.readRatio);
// 			tr1.setJoint(joint);

// 		  // Create joint state interface
// 			JointStateHandle jointStateHandle(joint.name, &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
// 		  joint_state_interface_.registerHandle(jointStateHandle);

// 		  // Create position joint interface
// 			JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
// 			JointLimits limits;
//  	   	SoftJointLimits softLimits;
// 			if (getJointLimits(joint.name, nh_, limits) == false) {
// 				ROS_ERROR_STREAM("Cannot set joint limits for " << joint.name);
// 			} else {
// 				PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
// 				positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
// 			}
// 		  position_joint_interface_.registerHandle(jointPositionHandle);

// 		  // Create velocity joint interface
// 			//JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
// 		  //effort_joint_interface_.registerHandle(jointVelocityHandle);

// 		  // Create effort joint interface
// 			JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
// 		  effort_joint_interface_.registerHandle(jointEffortHandle);

// 		}

// 		registerInterface(&joint_state_interface_);
// 		registerInterface(&position_joint_interface_);
// 		//registerInterface(&velocity_joint_interface_);
// 		registerInterface(&effort_joint_interface_);
// 		registerInterface(&positionJointSoftLimitsInterface);
// 	}

// 	void TR1HardwareInterface::update(const ros::TimerEvent& e)
// 	{
// 		_logInfo = "\n";
// 		_logInfo += "Joint Position Command:\n";
// 		for (int i = 0; i < num_joints_; i++)
// 		{
// 			std::ostringstream jointPositionStr;
// 			jointPositionStr << joint_position_command_[i];
// 			_logInfo += "  " + joint_names_[i] + ": " + jointPositionStr.str() + "\n";
// 		}

// 		elapsed_time_ = ros::Duration(e.current_real - e.last_real);

// 		read();
// 		controller_manager_->update(ros::Time::now(), elapsed_time_);
// 		write(elapsed_time_);

// 		//ROS_INFO_STREAM(_logInfo);
// 	}

// 	void TR1HardwareInterface::read()
// 	{
// 		_logInfo += "Joint State:\n";
// 		for (int i = 0; i < num_joints_; i++)
// 		{
// 			tr1cpp::Joint joint = tr1.getJoint(joint_names_[i]);

// 			if (joint.getActuatorType() == ACTUATOR_TYPE_MOTOR)
// 			{
// 				joint_position_[i] = joint.readAngle();

// 				std::ostringstream jointPositionStr;
// 				jointPositionStr << joint_position_[i];
// 				_logInfo += "  " + joint.name + ": " + jointPositionStr.str() + "\n";
// 			}

// 			tr1.setJoint(joint);
// 		}
// 	}

// 	void TR1HardwareInterface::write(ros::Duration elapsed_time)
// 	{
// 		positionJointSoftLimitsInterface.enforceLimits(elapsed_time);

// 		_logInfo += "Joint Effort Command:\n";
// 		for (int i = 0; i < num_joints_; i++)
// 		{
// 			tr1cpp::Joint joint = tr1.getJoint(joint_names_[i]);
// 			//if (joint_effort_command_[i] > 1) joint_effort_command_[i] = 1;
// 			//if (joint_effort_command_[i] < -1) joint_effort_command_[i] = -1;

// 			double effort = joint_effort_command_[i];
// 			uint8_t duration = 15;

// 			if (joint.getActuatorType() == 1) { // servo
// 				double previousEffort = joint.getPreviousEffort();
// 				effort += previousEffort;
// 			}
			
// 			joint.actuate(effort, duration);

// 			std::ostringstream jointEffortStr;
// 			jointEffortStr << joint_effort_command_[i];
// 			_logInfo += "  " + joint.name + ": " + jointEffortStr.str() + "\n";
// 		}
// 	}
// }

