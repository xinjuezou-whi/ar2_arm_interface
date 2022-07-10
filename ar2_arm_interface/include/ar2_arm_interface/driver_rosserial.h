/******************************************************************
dc-motor driver instance for rosserial module

Features:
- dc-motor operation logic for rosserial hardware
- xxx

Prerequisites:
- sudo apt install ros-<ros distro>-rosserial
- sudo apt install ros-<ros distro>-rosserial-arduino

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-07-10: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "driver_base.h"

class DriverRosserial : public DriverBase
{
public:
	DriverRosserial() = delete;
	DriverRosserial(const std::string& JointName, std::shared_ptr<ros::NodeHandle> NodeHandle, std::string Topic);
	~DriverRosserial() override;

public:
	// override
	double readAngle() override;
	void actuate(double Command) override;
	void actuate(std::string Command) override;
	std::shared_ptr<RotaryEncoderBase> getEncoder() override { return encoder_; };
	void cal_angularVel2PwmDuty() override;

public:
	// specific
	void setMotor(int ForwardDir);
	void setEncoder(unsigned int Resolution);

protected:
	void resetEnc();

protected:
	std::unique_ptr<ros::Publisher> pub_{ nullptr };
	int forward_dir_{ 1 };
	double angular_value_;
	std::shared_ptr<RotaryEncoderBase> encoder_{ nullptr };
	long pre_encoder_{ 0 };
};
