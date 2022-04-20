#ifndef ROS_CONTROL__RRBOT_HARDWARE_INTERFACE_H
#define ROS_CONTROL__RRBOT_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
// #include <joint_limits_interface/joint_limits_interface.h>
// #include <joint_limits_interface/joint_limits.h>
// #include <joint_limits_interface/joint_limits_urdf.h>
// #include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <rrbot_hw/rrbot_hardware.h>
#include <rrbot_hw/motor_driver.h>

using namespace hardware_interface;
// using joint_limits_interface::JointLimits;
// using joint_limits_interface::SoftJointLimits;
// using joint_limits_interface::PositionJointSoftLimitsHandle;
// using joint_limits_interface::PositionJointSoftLimitsInterface;


namespace rrbot_hardware_interface
{
	static const double POSITION_STEP_FACTOR = 10;
	static const double VELOCITY_STEP_FACTOR = 10;

	class RRBOTHardwareInterface: public rrbot_hardware_interface::RRBOTHardware
	{
		public:
			RRBOTHardwareInterface(ros::NodeHandle& nh);
			~RRBOTHardwareInterface();
			void init();
			void update(const ros::TimerEvent& e);
			void read();
			void write();

			bool is_echo;
			// controller_manager 原本放在protected
			boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

		protected:
			ros::NodeHandle nh_;
			ros::Timer non_realtime_loop_;
			ros::Duration control_period_;
			ros::Duration elapsed_time_;
			PositionJointInterface positionJointInterface;
			// PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
			double loop_hz_;
			
			double p_error_, v_error_, e_error_;
			std::string _logInfo;
			

        private:
			void serial_setup(std::string port_name, int baud_rate);
			std::shared_ptr<Motor::MotorDriver> multi_drive_;

            hardware_interface::JointStateInterface jnt_state_interface;
            // hardware_interface::PositionJointInterface jnt_pos_interface;
			hardware_interface::EffortJointInterface jnt_pos_interface;
			struct Joint
			{
				double position;
				double velocity;
				double effort;
				double velocity_command;
				double position_command;
				double effort_command;

				Joint() : position(0), velocity(0), effort(0), velocity_command(0), position_command(0), effort_command(0)
				{
				}
			} joints_;

	};
} // namespace

#endif
