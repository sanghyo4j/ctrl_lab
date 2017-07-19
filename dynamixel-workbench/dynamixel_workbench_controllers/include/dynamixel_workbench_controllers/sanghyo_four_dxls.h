/* Authors: Taehoon Lim (Darby) */
/* Editor: Sanghyo Jeong */

#ifndef DYNAMIXEL_WORKBENCH_MULTI_PORT_H
#define DYNAMIXEL_WORKBENCH_MULTI_PORT_H

#include <ros/ros.h>

#include <dynamixel_workbench_toolbox/dynamixel_driver.h>

#include <dynamixel_workbench_msgs/DynamixelState.h>
#include <dynamixel_workbench_msgs/JointCommand.h>

namespace sanghyo_four_dxls
{
#define MOTOR 0
// #define PAN   0 // original
// #define TILT  1 // original

#define MOTOR1  0
#define MOTOR2  1
#define MOTOR3  2
#define MOTOR4  3

typedef struct
{
  std::vector<uint8_t>  torque;
  std::vector<uint32_t> pos;
}WriteValue;

class SanghyoFourDxls
{
 private:
	// ROS NodeHandle
	ros::NodeHandle node_handle_;
	ros::NodeHandle node_handle_priv_;

	// ROS Parameters

	// ROS Topic Publisher
	// original 
	/*
	ros::Publisher pan_state_pub_; 
	ros::Publisher tilt_state_pub_;
	*/

	ros::Publisher motor1_state_pub_;
	ros::Publisher motor2_state_pub_;
	ros::Publisher motor3_state_pub_;
	ros::Publisher motor4_state_pub_;

	// ROS Topic Subscriber

	// ROS Service Server
	ros::ServiceServer joint_command_server;
	// ROS Service Client

	// ROS Topic Publisher

	// ROS Service Server

	// Dynamixel Workbench Parameters
	std::vector<dynamixel_driver::DynamixelInfo* > dynamixel_info_;
	// original
/*
	dynamixel_driver::DynamixelDriver* pan_driver_;
	dynamixel_driver::DynamixelDriver* tilt_driver_;
*/
    dynamixel_driver::DynamixelDriver* motor1_driver_;
    dynamixel_driver::DynamixelDriver* motor2_driver_;
    dynamixel_driver::DynamixelDriver* motor3_driver_;
    dynamixel_driver::DynamixelDriver* motor4_driver_;

	WriteValue *writeValue_;


	//original
	/*
	std::map<std::string, int32_t> pan_data_;
	std::map<std::string, int32_t> tilt_data_;
	*/
	std::map<std::string, int32_t> motor1_data_;
	std::map<std::string, int32_t> motor2_data_;
	std::map<std::string, int32_t> motor3_data_;
	std::map<std::string, int32_t> motor4_data_;

 public:
  SanghyoFourDxls();
  ~SanghyoFourDxls();
  bool controlLoop(void);

 private:
  bool loadDynamixel();
  bool checkLoadDynamixel();
  bool initDynamixelStatePublisher();
  bool initDynamixelInfoServer();

  bool setTorque(bool onoff);
//  bool setPosition(uint32_t pan_pos, uint32_t tilt_pos); // original
    bool setPosition(uint32_t motor1_pos, uint32_t motor2_pos, uint32_t motor3_pos, uint32_t motor4_pos);

  bool readValue(uint8_t motor, std::string addr_name);
  bool readDynamixelState(uint8_t motor);
  bool dynamixelStatePublish(uint8_t motor);

  uint32_t convertRadian2Value(uint8_t motor, float radian);

  bool jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                               dynamixel_workbench_msgs::JointCommand::Response &res);
};
}

#endif //DYNAMIXEL_WORKBENCH_MULTI_PORT_H
