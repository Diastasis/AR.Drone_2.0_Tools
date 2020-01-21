/*
* ardrone_joystick:
*
* This software provides a connection between a PS3-Joystick and the ardrone_brown - drone-driver
*
* It receives the joystick state from the joy-node and publishes the corresponding commands to the driver
*
* Author: Nikolas Engelhard
* Maintained by: Adrian P.
*
*/


#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"

// Set show_key_number to true to show the button number when it's pressed
bool show_kew_number {true};
const int PUBLISH_FREQ {50};

using namespace std;

struct TeleopArDrone
{
	ros::Subscriber joy_sub;
	ros::Publisher pub_takeoff, pub_land, pub_toggle_state, pub_vel;

	bool got_first_joy_msg;

	bool is_flying;
	bool toggle_pressed_in_last_msg;
	bool cam_toggle_pressed_in_last_msg;
	std_srvs::Empty srv_empty;

	ros::NodeHandle nh_;
	geometry_msgs::Twist twist;
	ros::ServiceClient srv_cl_cam;


	void joyCb(const sensor_msgs::JoyConstPtr joy_msg)
	{
		if (!got_first_joy_msg)
		{
			ROS_INFO("Found joystick with %zu buttons and %zu axes", joy_msg->buttons.size(), joy_msg->axes.size());
			if (joy_msg->buttons.size() < 11 || joy_msg->axes.size() < 8)
			{
				ROS_FATAL("This joystick does not look like a XBOX-Joystick");
			}

			got_first_joy_msg = true;
		}

		// mapping from joystick to velocity
		float scale = 2.5;

		twist.linear.y  = scale*joy_msg->axes[0]; // left right
		twist.linear.x  = scale*joy_msg->axes[1]; // forward, backward

		twist.angular.z = scale*joy_msg->axes[3]; // yaw
		twist.linear.z  = scale*joy_msg->axes[4]; // up down


		// button 10 (L1): dead man switch
		bool dead_man_pressed = joy_msg->buttons.at(4);
		
		// button 11 (R1): switch emergeny state 
		bool emergency_toggle_pressed = joy_msg->buttons.at(8);
		
		// button 0 (select): switch camera mode
		bool cam_toggle_pressed = joy_msg->buttons.at(6);
		
		//=======================
		// NEW - Show key number
		//=======================
		if (show_kew_number)
		{
			// button 0 ? (): NEW
			bool zero_pressed = joy_msg->buttons.at(0);
			// button 1 ? (): NEW
			bool one_pressed = joy_msg->buttons.at(1);
			// button 2 ? (): NEW
			bool two_pressed = joy_msg->buttons.at(2);
			// button 3 ? (): NEW
			bool three_pressed = joy_msg->buttons.at(3);
			// button 4 ? (): NEW
			bool four_pressed = joy_msg->buttons.at(4);
			// button 5 ? (): NEW
			bool five_pressed = joy_msg->buttons.at(5);
			// button 6 ? (): NEW
			bool six_pressed = joy_msg->buttons.at(6);
			// button 7 ? (): NEW
			bool seven_pressed = joy_msg->buttons.at(7);
			// button 8 ? (): NEW
			bool eight_pressed = joy_msg->buttons.at(8);
			// button 9 ? (): NEW
			bool nine_pressed = joy_msg->buttons.at(9);
			// button 10 ? (): NEW
			bool ten_pressed = joy_msg->buttons.at(10);

			if (zero_pressed)
			{
				ROS_INFO("zero_pressed");
				zero_pressed = false;
			}
			
			if (one_pressed)
			{
				ROS_INFO("one_pressed");
				one_pressed = false;
			}

			if (two_pressed)
			{
				ROS_INFO("two_pressed");
				two_pressed = false;
			}

			if (three_pressed)
			{
				ROS_INFO("three_pressed");
				three_pressed = false;
			}

			if (four_pressed)
			{
				ROS_INFO("four_pressed");
				four_pressed = false;
			}

			if (five_pressed)
			{
				ROS_INFO("five_pressed");
				five_pressed = false;
			}

			if (six_pressed)
			{
				ROS_INFO("six_pressed");
				six_pressed = false;
			}

			if (seven_pressed)
			{
				ROS_INFO("seven_pressed");
				seven_pressed = false;
			}

			if (eight_pressed)
			{
				ROS_INFO("eight_pressed");
				eight_pressed = false;
			}

			if (nine_pressed)
			{
				ROS_INFO("nine_pressed");
				nine_pressed = false;
			}

			if (ten_pressed)
			{
				ROS_INFO("ten_pressed");
				ten_pressed = false;
			}
		} //=============================================================
		
		if (!is_flying && dead_man_pressed)
		{
			ROS_INFO("L1 was pressed, Taking off!");
			pub_takeoff.publish(std_msgs::Empty());
			pub_takeoff.publish(std_msgs::Empty()); // NEW
			pub_takeoff.publish(std_msgs::Empty());	// NEW
			is_flying = true;
		}

		if (is_flying && !dead_man_pressed)
		{
			ROS_INFO("L1 was released, landing");
			pub_land.publish(std_msgs::Empty());
			is_flying = false;
		}
		
		// toggle only once!
		if (!toggle_pressed_in_last_msg && emergency_toggle_pressed)
		{
			ROS_INFO("Changing emergency status");
			pub_toggle_state.publish(std_msgs::Empty());
		}
		
		toggle_pressed_in_last_msg = emergency_toggle_pressed;
		
		
		if (!cam_toggle_pressed_in_last_msg && cam_toggle_pressed)
		{
			ROS_INFO("Changing Camera");
			if (!srv_cl_cam.call(srv_empty))
			{
				ROS_INFO("Failed to toggle Camera");
			}
		}
		cam_toggle_pressed_in_last_msg = cam_toggle_pressed;
	}


	TeleopArDrone()
	{
		twist.linear.x = twist.linear.y = twist.linear.z = 0;
		twist.angular.x = twist.angular.y = twist.angular.z = 0;
		
		is_flying = false;
		got_first_joy_msg = false;
		
		joy_sub = nh_.subscribe("/joy", 1,&TeleopArDrone::joyCb, this);
		toggle_pressed_in_last_msg = cam_toggle_pressed_in_last_msg = false;
		
		pub_takeoff       = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
		pub_land          = nh_.advertise<std_msgs::Empty>("/ardrone/land",1);
		pub_toggle_state  = nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);
		pub_vel           = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
		srv_cl_cam        = nh_.serviceClient<std_srvs::Empty>("/ardrone/togglecam",1);
	}

	void send_cmd_vel()
	{
		pub_vel.publish(twist);
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ardrone_teleop");
	
	ROS_INFO("Started ArDrone joystick-Teleop");
	ROS_INFO("Press L1 to toggle emergency-state");
	ROS_INFO("Press and hold L2 for takeoff");
	ROS_INFO("Press 'select' to choose camera");
	
	TeleopArDrone teleop;
	ros::Rate pub_rate(PUBLISH_FREQ);

	while (teleop.nh_.ok())
	{
		ros::spinOnce();
		teleop.send_cmd_vel();
		pub_rate.sleep();
	}

	return 0;
}
