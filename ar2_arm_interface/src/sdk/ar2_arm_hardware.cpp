/******************************************************************
arm hardware interface of ar2 under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- ar2 hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "ar2_arm_interface/ar2_arm_hardware.h"

namespace whi_arm_hardware_interface
{
	using namespace hardware_interface;

    Ar2HardwareInterface::Ar2HardwareInterface(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : ArmHardware(NodeHandle)
    {
        init();
    }

    void Ar2HardwareInterface::init()
    {
        // joints
        node_handle_->getParam("/ar2_arm/hardware_interface/joints", joint_names_);
        if (joint_names_.size() == 0)
        {
            // especially for rosrun mode
            std::string sum;
            for (int i = 0; i < 6; ++i)
            {
                joint_names_.push_back("joint_" + std::to_string(i + 1));
                sum += joint_names_.back() + "\n";
            }
            sum.pop_back();
            ROS_WARN((std::string("No joints found on parameter server for controller. Name them with:\n") + sum).c_str());
        }

        // resize vectors
        num_joints_ = joint_names_.size();
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);

        // initialize controller
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            // create joint state interface
            JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
            joint_state_interface_.registerHandle(jointStateHandle);

            // create velocity joint interface
            JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
            position_joint_interface_.registerHandle(jointPositionHandle);
        }

        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);

        // controller
        node_handle_->param("/ar2_arm/hardware_interface/loop_hz", loop_hz_, 10.0);
        controller_manager_ = std::make_unique<controller_manager::ControllerManager>(this, *node_handle_);
        ros::Duration updateFreq = ros::Duration(1.0 / loop_hz_);
        non_realtime_loop_ = std::make_unique<ros::Timer>(node_handle_->createTimer(updateFreq, std::bind(&Ar2HardwareInterface::update, this, std::placeholders::_1)));
    }

    void Ar2HardwareInterface::update(const ros::TimerEvent& Event)
    {
        elapsed_time_ = ros::Duration(Event.current_real - Event.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
    }

    void Ar2HardwareInterface::read()
    {

    }

    void Ar2HardwareInterface::write(ros::Duration ElapsedTime)
    {

    }
}
