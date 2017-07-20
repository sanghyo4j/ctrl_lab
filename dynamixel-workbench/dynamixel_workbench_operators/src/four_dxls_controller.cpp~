#include <ros/ros.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <termios.h>

int kfd = 0;
struct termios cooked, raw;

int main(int argc, char **argv) {
	const float motor1_default = 3700.0;
	const float motor2_default = 2050.0;
	const float motor3_default = 2050.0;
	const float motor4_default = 2500.0;

	float motor1_current = motor1_default;
	float motor2_current = motor2_default;
	float motor3_current = motor3_default;
	float motor4_current = motor4_default;

	ros::init(argc, argv, "four_dxls_controller"); // "joint_operator"

	dynamixel_workbench_msgs::JointCommand joint_command;
	ros::NodeHandle node_handle;

	ros::ServiceClient joint_command_client =
	  node_handle.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");

	while(ros::ok()) {
		char c;
		tcgetattr(kfd, &cooked);
		memcpy(&raw, &cooked, sizeof(struct termios));
		 raw.c_lflag &=~ (ICANON | ECHO);
		 // Setting a new line, then end of file                         
		raw.c_cc[VEOL] = 1;
		raw.c_cc[VEOF] = 2;
		tcsetattr(kfd, TCSANOW, &raw);

		puts("Reading from keyboard");
		puts("use a, s, d, f (move up of each joint)");
		puts("use z, x, c, v (move down of each joint)");
		puts("use q, w, e, r (reset position of each joint)");

		if(read(kfd, &c, 1) < 0) {
			puts("error");
			//perror("read():");
			//exit(-1);
		}

		switch(c){
			case 'a':
				motor1_current += 10;
				break;
			case 's':
				motor2_current += 10;
				break;
			case 'd':
				motor3_current += 10;
				break;
			case 'f':
				motor4_current += 10;
				break;
			case 'z':
				motor1_current -= 10;
				break;
			case 'x':
				motor2_current -= 10;
				break;
			case 'c':
				motor3_current -= 10;
				break;
			case 'v':
				motor4_current -= 10;
				break;
		}

		joint_command.request.unit = "raw"; // argv[1];
		joint_command.request.pan_pos = motor1_current;
		joint_command.request.tilt_pos = motor2_current;
		joint_command.request.motor3_pos = motor3_current;
		joint_command.request.motor4_pos = motor4_current;

		if (joint_command_client.call(joint_command))  {
		    ROS_DEBUG("value: 0x%02X\n", c);
		} else {
			ROS_ERROR("Failed to call service /joint_command");
			break;
		}

	}
	
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
	return 0;	
}
