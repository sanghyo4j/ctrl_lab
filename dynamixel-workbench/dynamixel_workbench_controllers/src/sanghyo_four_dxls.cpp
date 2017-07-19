/* Authors: Taehoon Lim (Darby) */
/* Editor: Sanghyo Jeong */

#include "dynamixel_workbench_controllers/sanghyo_four_dxls.h"

using namespace sanghyo_four_dxls;

SanghyoFourDxls::SanghyoFourDxls()
	:node_handle_(""),
	node_handle_priv_("~")
{
	if (loadDynamixel()){
		checkLoadDynamixel();
		ROS_INFO("Load dynamixel...");
	} else {
		ROS_ERROR("Can't Load Dynamixel, Please check Parameter");
	}
	writeValue_ = new WriteValue;
	ROS_INFO("Writing values...");

	setTorque(true);
	ROS_INFO("Setting Torques...");

	initDynamixelStatePublisher();
	ROS_INFO("Initialise Dynamixel State Publisher...");

	initDynamixelInfoServer();
	ROS_INFO("Initialise Dynamixel Info Server...");
}

SanghyoFourDxls::~SanghyoFourDxls()
{
  setTorque(false);

  ros::shutdown();
}

bool SanghyoFourDxls::loadDynamixel()
{
  bool ret = false;

// original source code
/*
  dynamixel_driver::DynamixelInfo *pan_info = new dynamixel_driver::DynamixelInfo;

  pan_info->lode_info.device_name      = node_handle_.param<std::string>("pan/device_name", "/dev/ttyUSB0");
  pan_info->lode_info.baud_rate        = node_handle_.param<int>("pan/baud_rate", 57600);
  pan_info->lode_info.protocol_version = node_handle_.param<float>("pan/protocol_version", 2.0);
  pan_info->model_id                   = node_handle_.param<int>("pan/id", 1);

  dynamixel_info_.push_back(pan_info);

  dynamixel_driver::DynamixelInfo *tilt_info = new dynamixel_driver::DynamixelInfo;

  tilt_info->lode_info.device_name      = node_handle_.param<std::string>("tilt/device_name", "/dev/ttyUSB1");
  tilt_info->lode_info.baud_rate        = node_handle_.param<int>("tilt/baud_rate", 57600);
  tilt_info->lode_info.protocol_version = node_handle_.param<float>("tilt/protocol_version", 1.0);
  tilt_info->model_id                   = node_handle_.param<int>("tilt/id", 2);

  dynamixel_info_.push_back(tilt_info);

  */
    // the parameters of first motor.
    dynamixel_driver::DynamixelInfo *motor1_info = new dynamixel_driver::DynamixelInfo;

    motor1_info->lode_info.device_name      = node_handle_.param<std::string>("motor1/device_name", "/dev/ttyUSB0");
    motor1_info->lode_info.baud_rate        = node_handle_.param<int>("motor1/baud_rate", 57600);
    motor1_info->lode_info.protocol_version = node_handle_.param<float>("motor1/protocol_version", 2.0);
    motor1_info->model_id                   = node_handle_.param<int>("motor1/id", 1);

    dynamixel_info_.push_back(motor1_info);

    // the parameters of second motor.
    dynamixel_driver::DynamixelInfo *motor2_info = new dynamixel_driver::DynamixelInfo;

    motor2_info->lode_info.device_name      = node_handle_.param<std::string>("motor2/device_name", "/dev/ttyUSB0");
    motor2_info->lode_info.baud_rate        = node_handle_.param<int>("motor2/baud_rate", 57600);
    motor2_info->lode_info.protocol_version = node_handle_.param<float>("motor2/protocol_version", 1.0);
    motor2_info->model_id                   = node_handle_.param<int>("motor2/id", 2);

    dynamixel_info_.push_back(motor2_info);

    // the parameters of third motor.
    dynamixel_driver::DynamixelInfo *motor3_info = new dynamixel_driver::DynamixelInfo;

    motor3_info->lode_info.device_name      = node_handle_.param<std::string>("motor3/device_name", "/dev/ttyUSB0");
    motor3_info->lode_info.baud_rate        = node_handle_.param<int>("motor3/baud_rate", 57600);
    motor3_info->lode_info.protocol_version = node_handle_.param<float>("motor3/protocol_version", 1.0);
    motor3_info->model_id                   = node_handle_.param<int>("motor3/id", 3);

    dynamixel_info_.push_back(motor3_info);

    // the parameters of fourth motor.
    dynamixel_driver::DynamixelInfo *motor4_info = new dynamixel_driver::DynamixelInfo;

    motor4_info->lode_info.device_name      = node_handle_.param<std::string>("motor4/device_name", "/dev/ttyUSB0");
    motor4_info->lode_info.baud_rate        = node_handle_.param<int>("motor4/baud_rate", 57600);
    motor4_info->lode_info.protocol_version = node_handle_.param<float>("motor4/protocol_version", 1.0);
    motor4_info->model_id                   = node_handle_.param<int>("motor4/id", 4);

    dynamixel_info_.push_back(motor4_info);

    
    // original source codes
/*
  pan_driver_  = new dynamixel_driver::DynamixelDriver(dynamixel_info_[PAN]->lode_info.device_name,
                                                       dynamixel_info_[PAN]->lode_info.baud_rate,
                                                       dynamixel_info_[PAN]->lode_info.protocol_version);

  tilt_driver_ = new dynamixel_driver::DynamixelDriver(dynamixel_info_[TILT]->lode_info.device_name,
                                                       dynamixel_info_[TILT]->lode_info.baud_rate,
                                                       dynamixel_info_[TILT]->lode_info.protocol_version);
*/
    motor1_driver_  = new dynamixel_driver::DynamixelDriver(dynamixel_info_[MOTOR1]->lode_info.device_name,
                                                   dynamixel_info_[MOTOR1]->lode_info.baud_rate,
                                                   dynamixel_info_[MOTOR1]->lode_info.protocol_version);

    motor2_driver_ = new dynamixel_driver::DynamixelDriver(dynamixel_info_[MOTOR2]->lode_info.device_name,
                                                   dynamixel_info_[MOTOR2]->lode_info.baud_rate,
                                                   dynamixel_info_[MOTOR2]->lode_info.protocol_version);

    motor3_driver_  = new dynamixel_driver::DynamixelDriver(dynamixel_info_[MOTOR3]->lode_info.device_name,
                                                   dynamixel_info_[MOTOR3]->lode_info.baud_rate,
                                                   dynamixel_info_[MOTOR3]->lode_info.protocol_version);

    motor4_driver_ = new dynamixel_driver::DynamixelDriver(dynamixel_info_[MOTOR4]->lode_info.device_name,
                                                   dynamixel_info_[MOTOR4]->lode_info.baud_rate,
                                                   dynamixel_info_[MOTOR4]->lode_info.protocol_version);
    

    // original
    /*
  ret = pan_driver_ ->ping(dynamixel_info_[PAN]->model_id);
  ret = tilt_driver_->ping(dynamixel_info_[TILT]->model_id);
*/
    
  ret = motor1_driver_->ping(dynamixel_info_[MOTOR1]->model_id);
  ret = motor2_driver_->ping(dynamixel_info_[MOTOR2]->model_id);
  ret = motor3_driver_->ping(dynamixel_info_[MOTOR3]->model_id);
  ret = motor4_driver_->ping(dynamixel_info_[MOTOR4]->model_id);
    
  if (ret)
  {
      //    original
      /*
    dynamixel_info_[PAN] ->model_name  = pan_driver_->dynamixel_->model_name_.c_str();
    dynamixel_info_[TILT]->model_name  = tilt_driver_->dynamixel_->model_name_.c_str();
    */
      
    dynamixel_info_[MOTOR1]->model_name  = motor1_driver_->dynamixel_->model_name_.c_str();
    dynamixel_info_[MOTOR2]->model_name  = motor2_driver_->dynamixel_->model_name_.c_str();
    dynamixel_info_[MOTOR3]->model_name  = motor3_driver_->dynamixel_->model_name_.c_str();
    dynamixel_info_[MOTOR4]->model_name  = motor4_driver_->dynamixel_->model_name_.c_str();
      
  }

 return ret;
}

bool SanghyoFourDxls::setTorque(bool onoff)
{
  writeValue_->torque.clear();
  writeValue_->torque.push_back(onoff);
  writeValue_->torque.push_back(onoff);

    // original
    /*
  if (!pan_driver_->writeRegister("torque_enable", writeValue_->torque.at(PAN)))
  {
    ROS_ERROR("Write Pan Torque Failed!");
    return false;
  }

  if (!tilt_driver_->writeRegister("torque_enable", writeValue_->torque.at(TILT)))
  {
    ROS_ERROR("Write Tilt Torque Failed!");
    return false;
  }
  */
    
    if (!motor1_driver_->writeRegister("torque_enable", writeValue_->torque.at(MOTOR1))) {
        ROS_ERROR("Write Motor 1 Torque Failed!");
        return false;
    }

    if (!motor2_driver_->writeRegister("torque_enable", writeValue_->torque.at(MOTOR2))) {
        ROS_ERROR("Write Motor 2 Torque Failed!");
        return false;  
    }  

    if (!motor3_driver_->writeRegister("torque_enable", writeValue_->torque.at(MOTOR3))) {
        ROS_ERROR("Write Motor 3 Torque Failed!");
        return false;
    }

    if (!motor4_driver_->writeRegister("torque_enable", writeValue_->torque.at(MOTOR4))) {
        ROS_ERROR("Write Motor 4 Torque Failed!");
        return false;
    }
  return true;
}

// original
// bool SanghyoFourDxls::setPosition(uint32_t pan_pos, uint32_t tilt_pos)
bool SanghyoFourDxls::setPosition(uint32_t motor1_pos, uint32_t motor2_pos, uint32_t motor3_pos, uint32_t motor4_pos) {
    writeValue_->pos.clear();
    
    // original 
    /*
    writeValue_->pos.push_back(pan_pos);
    writeValue_->pos.push_back(tilt_pos);
    */
    
    writeValue_->pos.push_back(motor1_pos);
    writeValue_->pos.push_back(motor2_pos);
    writeValue_->pos.push_back(motor3_pos);
    writeValue_->pos.push_back(motor4_pos);
    
    
    // original
    /*
    if (!pan_driver_->writeRegister("goal_position", writeValue_->pos.at(PAN)))
    {
    ROS_ERROR("Write Pan Position Failed!");
    return false;
    }

    if (!tilt_driver_->writeRegister("goal_position", writeValue_->pos.at(TILT)))
    {
    ROS_ERROR("Write Tilt Position Failed!");
    return false;
    }
    */
    
    if (!motor1_driver_->writeRegister("goal_position", writeValue_->pos.at(MOTOR1)))   {
        ROS_ERROR("Write Motor 1 Position Failed!");
        return false;
    }

    if (!motor2_driver_->writeRegister("goal_position", writeValue_->pos.at(MOTOR2)))  {
        ROS_ERROR("Write Motor 2 Position Failed!");
        return false;
    }    
    
    if (!motor3_driver_->writeRegister("goal_position", writeValue_->pos.at(MOTOR3)))  {
        ROS_ERROR("Write Motor 3 Position Failed!");
        return false;
    }

    if (!motor4_driver_->writeRegister("goal_position", writeValue_->pos.at(MOTOR4))) {
        ROS_ERROR("Write Motor 4 Position Failed!");
        return false;
    }
    return true;
}

bool SanghyoFourDxls::checkLoadDynamixel()
{
    ROS_INFO("-------------------------------------------------------------------");
    ROS_INFO("       dynamixel_workbench controller; 4-axis position control     ");
    ROS_INFO("             for control laboratory by Sanghyo Jeong               ");
    ROS_INFO("-------------------------------------------------------------------");
    ROS_INFO("MOTOR 1 INFO");
    ROS_INFO("Device Name    : %s", dynamixel_info_[MOTOR1]->lode_info.device_name.c_str());
    ROS_INFO("ID             : %d", dynamixel_info_[MOTOR1]->model_id);
    ROS_INFO("MODEL          : %s", dynamixel_info_[MOTOR1]->model_name.c_str());
    ROS_INFO(" ");
    ROS_INFO("MOTOR 2 INFO");
    ROS_INFO("Device Name    : %s", dynamixel_info_[MOTOR2]->lode_info.device_name.c_str());
    ROS_INFO("ID             : %d", dynamixel_info_[MOTOR2]->model_id);
    ROS_INFO("MODEL          : %s", dynamixel_info_[MOTOR2]->model_name.c_str());
    ROS_INFO(" ");
    ROS_INFO("MOTOR 3 INFO");
    ROS_INFO("Device Name    : %s", dynamixel_info_[MOTOR3]->lode_info.device_name.c_str());
    ROS_INFO("ID             : %d", dynamixel_info_[MOTOR3]->model_id);
    ROS_INFO("MODEL          : %s", dynamixel_info_[MOTOR3]->model_name.c_str());
    ROS_INFO(" ");
    ROS_INFO("MOTOR 4 INFO");
    ROS_INFO("Device Name    : %s", dynamixel_info_[MOTOR4]->lode_info.device_name.c_str());
    ROS_INFO("ID             : %d", dynamixel_info_[MOTOR4]->model_id);
    ROS_INFO("MODEL          : %s", dynamixel_info_[MOTOR4]->model_name.c_str());
//    ROS_INFO(" ");
//    ROS_INFO("Profile Velocity     : %d", profile_velocity_);
//    ROS_INFO("Profile Acceleration : %d", profile_acceleration_); // I will add these paramater later.
    ROS_INFO("-------------------------------------------------------------------");

}

bool SanghyoFourDxls::initDynamixelStatePublisher()
{
     // original 
    /*
  pan_state_pub_  = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelState>("/multi_port/pan_state", 10);
  tilt_state_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelState>("/multi_port/tilt_state", 10);
  */
    
  motor1_state_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelState>("/sanghyo_four_dxls/motor1_state", 10);
  motor2_state_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelState>("/sanghyo_four_dxls/motor2_state", 10);
  motor3_state_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelState>("/sanghyo_four_dxls/motor3_state", 10);
  motor4_state_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelState>("/sanghyo_four_dxls/motor4_state", 10);
}

bool SanghyoFourDxls::initDynamixelInfoServer()
{
  joint_command_server = node_handle_.advertiseService("/joint_command", &SanghyoFourDxls::jointCommandMsgCallback, this);
}

bool SanghyoFourDxls::readValue(uint8_t motor, std::string addr_name)
{
    int32_t read_value;

    // original
    /*
    if (motor == PAN)
    {
        pan_driver_->readRegister(addr_name, &read_value);
        pan_data_[addr_name] = read_value;
    }
    else if (motor == TILT)
    {
        tilt_driver_->readRegister(addr_name, &read_value);
        tilt_data_[addr_name] = read_value;
    }
  
    */  
    if (motor == MOTOR1) {
        motor1_driver_->readRegister(addr_name, &read_value);
        motor1_data_[addr_name] = read_value;
    } 
    else if (motor == MOTOR2) {
        motor2_driver_->readRegister(addr_name, &read_value);
        motor2_data_[addr_name] = read_value;
    } 
    else if (motor == MOTOR3) {
        motor3_driver_->readRegister(addr_name, &read_value);
        motor3_data_[addr_name] = read_value;
    } 
    else if (motor == MOTOR4) {
        motor4_driver_->readRegister(addr_name, &read_value);
        motor4_data_[addr_name] = read_value;
    }
}

bool SanghyoFourDxls::readDynamixelState(uint8_t motor)
{
    dynamixel_driver::DynamixelDriver* dynamixel_driver;

    // original 
    /*
    if (motor == PAN)
    dynamixel_driver = pan_driver_;
    else if (motor == TILT)
    dynamixel_driver = tilt_driver_;
    */
    
    if (motor == MOTOR1){
        dynamixel_driver = motor1_driver_;
    }
    else if (motor == MOTOR2){
        dynamixel_driver = motor2_driver_;
    }
    else if (motor == MOTOR3){
        dynamixel_driver = motor3_driver_;
    }
    else if (motor == MOTOR4){
        dynamixel_driver = motor4_driver_;
    }   
    
    readValue(motor, "torque_enable");

    readValue(motor, "present_position");

    readValue(motor, "goal_position");
    readValue(motor, "moving");

    if (dynamixel_driver->getProtocolVersion() == 2.0)    {
        if (dynamixel_driver->dynamixel_->model_name_.find("XM") != std::string::npos)        {
            readValue(motor, "goal_current");
            readValue(motor, "present_current");
        }

        readValue(motor, "goal_velocity");
        readValue(motor, "present_velocity");
    }   
    else {
        readValue(motor, "moving_speed");
        readValue(motor, "present_speed");
    }
}

bool SanghyoFourDxls::dynamixelStatePublish(uint8_t motor)
{
    // original
    /*
  readDynamixelState(PAN);
  readDynamixelState(TILT);
  */
    
  readDynamixelState(MOTOR1);
  readDynamixelState(MOTOR2);
  readDynamixelState(MOTOR3);
  readDynamixelState(MOTOR4);

  dynamixel_driver::DynamixelDriver*       dynamixel_driver;
  dynamixel_workbench_msgs::DynamixelState dynamixel_state;

  std::map<std::string, int32_t> read_data;

// original 
    /*
  if (motor == PAN)
  {
    dynamixel_driver = pan_driver_;
    read_data        = pan_data_;
  }
  else if (motor == TILT)
  {
    dynamixel_driver = tilt_driver_;
    read_data        = tilt_data_;
  }

*/
    
    if (motor == MOTOR1)  {
        dynamixel_driver = motor1_driver_;
        read_data        = motor1_data_;
    }
    else if (motor == MOTOR2)  {
        dynamixel_driver = motor2_driver_;
        read_data        = motor2_data_;
    }
    else if (motor == MOTOR3)  {
        dynamixel_driver = motor3_driver_;
        read_data        = motor3_data_;
    }
    else if (motor == MOTOR4)    {
        dynamixel_driver = motor4_driver_;
        read_data        = motor4_data_;
    }
    
    
    dynamixel_state.model_name          = dynamixel_driver->dynamixel_->model_name_;
    dynamixel_state.id                  = dynamixel_driver->dynamixel_->id_;
    dynamixel_state.torque_enable       = read_data["torque_enable"];
    dynamixel_state.present_position    = read_data["present_position"];
    dynamixel_state.goal_position       = read_data["goal_position"];
    dynamixel_state.moving              = read_data["moving"];

    if (dynamixel_driver->getProtocolVersion() == 2.0)    {
        if (dynamixel_driver->dynamixel_->model_name_.find("XM") != std::string::npos)    {
            dynamixel_state.goal_current    = read_data["goal_current"];
            dynamixel_state.present_current = read_data["present_current"];
        }   

        dynamixel_state.goal_velocity    = read_data["goal_velocity"];
        dynamixel_state.present_velocity = read_data["present_velocity"];
    }
    else    {
        dynamixel_state.goal_velocity    = read_data["moving_speed"];
        dynamixel_state.present_velocity = read_data["present_speed"];
    }

    //original 
    /*
    if (motor == PAN)
    {
    pan_state_pub_.publish(dynamixel_state);
    }
    else if (motor == TILT)
    {
    tilt_state_pub_.publish(dynamixel_state);
    }
    */
    if (motor == MOTOR1)    {
        motor1_state_pub_.publish(dynamixel_state);
    }
    else if (motor == MOTOR2)    {
        motor2_state_pub_.publish(dynamixel_state);
    }
    else if (motor == MOTOR3)    {
        motor3_state_pub_.publish(dynamixel_state);
    }
    else if (motor == MOTOR4)    {
        motor4_state_pub_.publish(dynamixel_state);
    }
}

uint32_t SanghyoFourDxls::convertRadian2Value(uint8_t motor, float radian)
{
	uint32_t value = 0;
	dynamixel_driver::DynamixelDriver* dynamixel_driver;

	// original 
	/*
	if (motor == PAN)
		dynamixel_driver = pan_driver_;
	else if (motor == TILT)
		dynamixel_driver = tilt_driver_;
	*/

	if (motor == MOTOR1) {
		dynamixel_driver = motor1_driver_;
	}
	else if (motor == MOTOR2) {
		dynamixel_driver = motor2_driver_;
	}
	else if (motor == MOTOR3) {
		dynamixel_driver = motor3_driver_;
	}
	else if (motor == MOTOR4) {
		dynamixel_driver = motor4_driver_;
	}


	if (radian > 0)
	{
	if (dynamixel_driver->dynamixel_->value_of_max_radian_position_ <= dynamixel_driver->dynamixel_->value_of_0_radian_position_)
		return dynamixel_driver->dynamixel_->value_of_max_radian_position_;

	value = (radian * (dynamixel_driver->dynamixel_->value_of_max_radian_position_ - dynamixel_driver->dynamixel_->value_of_0_radian_position_) / dynamixel_driver->dynamixel_->max_radian_)
				+ dynamixel_driver->dynamixel_->value_of_0_radian_position_;
	}
	else if (radian < 0)
	{
	if (dynamixel_driver->dynamixel_->value_of_min_radian_position_ >= dynamixel_driver->dynamixel_->value_of_0_radian_position_)
		return dynamixel_driver->dynamixel_->value_of_min_radian_position_;

	value = (radian * (dynamixel_driver->dynamixel_->value_of_min_radian_position_ - dynamixel_driver->dynamixel_->value_of_0_radian_position_) / dynamixel_driver->dynamixel_->min_radian_)
				+ dynamixel_driver->dynamixel_->value_of_0_radian_position_;
	}
	else
		value = dynamixel_driver->dynamixel_->value_of_0_radian_position_;

	//  if (value > multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_)
	//    return multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_;
	//  else if (value < multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_)
	//    return multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_;

	return value;
}

bool SanghyoFourDxls::controlLoop()
{
	//original
	/*
	dynamixelStatePublish(PAN);
	dynamixelStatePublish(TILT);
	*/
	dynamixelStatePublish(MOTOR1);
	dynamixelStatePublish(MOTOR2);
	dynamixelStatePublish(MOTOR3);
	dynamixelStatePublish(MOTOR4);
}

bool SanghyoFourDxls::jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                                              dynamixel_workbench_msgs::JointCommand::Response &res)
{
	//original
	/*
  uint32_t pan_pos = 0;
  uint32_t tilt_pos = 0;
  */
	uint32_t motor1_pos = 0;
	uint32_t motor2_pos = 0;
	uint32_t motor3_pos = 0;
	uint32_t motor4_pos = 0;

	if (req.unit == "rad")	{
		// original
		/*
		pan_pos = convertRadian2Value(PAN, req.pan_pos);
		tilt_pos = convertRadian2Value(TILT, req.tilt_pos);
		*/
		motor1_pos = convertRadian2Value(MOTOR1, req.motor1_pos);
		motor2_pos = convertRadian2Value(MOTOR2, req.motor2_pos);
		motor3_pos = convertRadian2Value(MOTOR3, req.motor3_pos);
		motor4_pos = convertRadian2Value(MOTOR4, req.motor4_pos);
	}
	else if (req.unit == "raw")	{
		// original
		/*
		pan_pos = req.pan_pos;
		tilt_pos = req.tilt_pos;
		*/
		motor1_pos = req.motor1_pos;
		motor2_pos = req.motor2_pos;
		motor3_pos = req.motor3_pos;
		motor4_pos = req.motor4_pos;
	}
	else {
		// original
		/*
		pan_pos = req.pan_pos;
		tilt_pos = req.tilt_pos;
		*/
		motor1_pos = req.motor1_pos;
		motor2_pos = req.motor2_pos;
		motor3_pos = req.motor3_pos;
		motor4_pos = req.motor4_pos;
	}

	// original 
	//setPosition(pan_pos, tilt_pos);
	setPosition(motor1_pos, motor2_pos, motor3_pos, motor4_pos);
	
	// original
	/*
	res.pan_pos = pan_pos;
	res.tilt_pos = tilt_pos;
	*/
	res.motor1_pos = motor1_pos;
	res.motor2_pos = motor2_pos;
	res.motor3_pos = motor3_pos;
	res.motor4_pos = motor4_pos;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "sanghyo_four_dxls");
  SanghyoFourDxls multi;

  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    multi.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

