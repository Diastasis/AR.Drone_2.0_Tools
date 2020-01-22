/*
* ardrone_joystick:
*
* This software provides a connection between a PS3-Joystick and the ardrone_brown - drone-driver
* It receives the joystick state from the joy-node and publishes the corresponding commands to the driver
*
* Original Author: Nikolas Engelhard
* Original Maintained by: Adrian P.
* Modified by: Diastasis 
*/


#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"

const int PUBLISH_FREQ {50};

using namespace std;

struct TeleopArDrone
{

	ros::Subscriber joy_sub;
	ros::Publisher pub_takeoff, pub_land, pub_toggle_state, pub_vel;
	ros::NodeHandle nh_;
	ros::ServiceClient srv_cl_cam;
	std_srvs::Empty srv_empty;
	geometry_msgs::Twist twist;

	bool got_first_joy_msg;
	bool is_flying;
	bool toggle_pressed_in_last_msg;
	bool cam_toggle_pressed_in_last_msg;

	void joyCb(const sensor_msgs::JoyConstPtr joy_msg)
	{
		// Check if the Joystick is compatible
		if (!got_first_joy_msg)
		{
			ROS_INFO("Found joystick with %zu buttons and %zu axes", joy_msg->buttons.size(), joy_msg->axes.size());
			if (joy_msg->buttons.size() < 11 || joy_msg->axes.size() < 8)
			{
				ROS_FATAL("This joystick does not look like a PS3-Joystick");
			}
			got_first_joy_msg = true;
		}

		// Button Definition
		bool A_Button = joy_msg->buttons.at(0);
		bool B_Button = joy_msg->buttons.at(1);
		bool X_Button = joy_msg->buttons.at(2);
		bool Y_Button = joy_msg->buttons.at(3);
		bool L1_Button = joy_msg->buttons.at(4);
		bool R1_Button = joy_msg->buttons.at(5);
		bool Select_Button = joy_msg->buttons.at(6);
		bool Start_Button = joy_msg->buttons.at(7);
		bool Home_Button = joy_msg->buttons.at(8);
		bool LJoy_Button = joy_msg->buttons.at(9);
		bool RJoy_Button = joy_msg->buttons.at(10);

		// Axes Definition
		double LJoy_X_Axis = joy_msg->axes[0];
		double LJoy_Y_Axis = joy_msg->axes[1];
		double L2_Axis = joy_msg->axes[2];
		double RJoy_X_Axis = joy_msg->axes[3];
		double RJoy_Y_Axis = joy_msg->axes[4];
		double R2_Axis = joy_msg->axes[5];
		double Arrows_X_Axis = joy_msg->axes[6];
		double Arrows_Y_Axis = joy_msg->axes[7];

		// Scale the velocity (Joystick response)
		float scale = 5;

		// Velocity definition
		twist.linear.y  = scale*RJoy_X_Axis; // joy_msg->axes[0]; // LJoy - Left/Right (Roll)
		twist.linear.x  = scale*RJoy_Y_Axis; //joy_msg->axes[1]; // LJoy - Up/Down  (Pitch)
		twist.angular.z = scale*LJoy_X_Axis; //joy_msg->axes[3]; // RJoy - Left/Right (Yaw)
		twist.linear.z  = scale*LJoy_Y_Axis; //joy_msg->axes[4]; // RJoy - Up/Down  (Elevation)

		// Toggle Start Button: Taking-off / Landing
		if (is_flying && Start_Button)
		{
			ROS_INFO("[Start Button] pressed, Landing!");
			pub_land.publish(std_msgs::Empty());
			is_flying = false;
			sleep(1);
		}
		 else if (!is_flying && Start_Button)
		{
			ROS_INFO("[Start Button] pressed, Taking off!");
			pub_takeoff.publish(std_msgs::Empty());
			pub_takeoff.publish(std_msgs::Empty());
			pub_takeoff.publish(std_msgs::Empty());
			is_flying = true;
			sleep(1); 
		}

		
		
		if (!cam_toggle_pressed_in_last_msg && Select_Button)
		{
			ROS_INFO("Changing Camera");
			if (!srv_cl_cam.call(srv_empty))
			{
				ROS_INFO("Failed to toggle Camera");
			}
		}
		cam_toggle_pressed_in_last_msg = Select_Button;
	}


	TeleopArDrone()
	{
		is_flying = false;
		got_first_joy_msg = false;
		toggle_pressed_in_last_msg = cam_toggle_pressed_in_last_msg = false;

		twist.linear.x = twist.linear.y = twist.linear.z = 0;
		twist.angular.x = twist.angular.y = twist.angular.z = 0;

		joy_sub = nh_.subscribe("/joy", 1,&TeleopArDrone::joyCb, this);
		
		pub_takeoff       = nh_.advertise<std_msgs::Empty>("/drone/takeoff",1);
		pub_land          = nh_.advertise<std_msgs::Empty>("/drone/land",1);
		pub_toggle_state  = nh_.advertise<std_msgs::Empty>("/drone/reset",1);
		pub_vel           = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
		srv_cl_cam        = nh_.serviceClient<std_srvs::Empty>("/drone/togglecam",1);
	}

	// Publish the twist message on the /cmd_vel topic
	void send_cmd_vel()
	{
		pub_vel.publish(twist);
	}
};

// Main
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ardrone_teleop");
	TeleopArDrone teleop;
	ros::Rate pub_rate(PUBLISH_FREQ);

	ROS_INFO("Started ArDrone joystick-Teleop");
	ROS_INFO("Press [Start] to take-off");
	ROS_INFO("Toggle [Start] for landing");

	while (teleop.nh_.ok())
	{
		ros::spinOnce();
		teleop.send_cmd_vel();
		pub_rate.sleep();
	}

	return 0;
}