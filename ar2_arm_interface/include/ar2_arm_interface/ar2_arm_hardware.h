/******************************************************************
arm hardware interface of ar2 under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- ar2 hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-06-16: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_arm_hardware_base.h"

namespace whi_arm_hardware_interface
{
    class Ar2HardwareInterface : public ArmHardware
    {
    public:
        Ar2HardwareInterface(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~Ar2HardwareInterface() = default;

    protected:
        void init();
        void update(const ros::TimerEvent& Event);
        void read();
        void write(ros::Duration ElapsedTime);
    };
}
