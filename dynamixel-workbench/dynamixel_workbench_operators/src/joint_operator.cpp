#include <ros/ros.h>
#include <dynamixel_workbench_msgs/JointCommand.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_operator");

  dynamixel_workbench_msgs::JointCommand joint_command;
  ros::NodeHandle node_handle;

  ros::ServiceClient joint_command_client =
                node_handle.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");

  if (argc != 6)
  {
    ROS_ERROR("rosrun dynamixel_workbench_operator joint_operator [mode] [motor1_pos] [motor2_pos] [motor3_pos] [motor4_pos]");
    return 1;
  }

  joint_command.request.unit = argv[1];
  joint_command.request.pan_pos = atof(argv[2]);
  joint_command.request.tilt_pos = atof(argv[3]);
  joint_command.request.motor3_pos = atof(argv[4]);
  joint_command.request.motor4_pos = atof(argv[5]);

  if (joint_command_client.call(joint_command))  {
    ROS_INFO("[pan_pos: %.2f (value)] [tilt_pos: %.2f (value)] [motor3_pos: %.2f (value)] [motor4_pos: %.2f (value)]", 
		joint_command.response.pan_pos, joint_command.response.tilt_pos, joint_command.response.motor3_pos, joint_command.response.motor4_pos);
  }
  else
  {
    ROS_ERROR("Failed to call service /joint_command");
  }

  return 0;
}
