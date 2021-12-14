/**
Author: Saurav Agarwal
Email: sagarw10@uncc.edu
This package implements actions using a joystick
Emergency Stop: Once triggered the estop remains true unless the reset buttons are pressed
Pause:					Pause till the button is pressed
Start:					Once triggered the start remains true unless the reset buttons are pressed
Stop:						Button toggles stop
**/

#ifndef JOY_ACTIONS_HPP_
#define JOY_ACTIONS_HPP_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

class JoyActions {
	private:

		/* Default button configuration */
		int button_estop_ = 0;
		int button_pause_ = 1;
		int button_start_ = 2;
		int button_stop_  = 3;

		/* Set the default values to false */
		/* They only get set to true if the corresponding button is pressed */
		std_msgs::Bool estop_;
		std_msgs::Bool pause_;
		std_msgs::Bool start_;
		std_msgs::Bool stop_;

		/* Stores previous stop */
		bool previous_stop_ = false;

		/* Name for joy topic */
		std::string joy_topic_name_ = "joy";

		/* Topic names for publishing actions */
		std::string estop_topic_name_ = "joy_actions/estop";
		std::string pause_topic_name_ = "joy_actions/pause";
		std::string start_topic_name_ = "joy_actions/start";
		std::string stop_topic_name_  = "joy_actions/stop";

		ros::NodeHandle nh_;
		/* Create a publisher for each action */
		ros::Publisher estop_pub_, pause_pub_, start_pub_, stop_pub_;

		/* Create a subscriber to messages from joystick */
		ros::Subscriber joy_sub_;

		void Reset() {
			estop_.data = false;
			pause_.data = false;
			start_.data = false;
			stop_.data  = false;
		}

		void JoyCallBack(const sensor_msgs::Joy::ConstPtr &joy_msg) {
			/* If estop is activated, it remains true until the reset buttons are pressed or node is restarted */
			if(estop_.data == false) {
				estop_.data = joy_msg->buttons[button_estop_];
			}

			/* Pause as long as the button is pressed */
			pause_.data = joy_msg->buttons[button_pause_];

			/* Toggle stop button */
			if(previous_stop_ == 0 and joy_msg->buttons[button_stop_] == 1) {
				stop_.data = !stop_.data;
			}

			/* If start is activated, it remains true until the reset buttons are pressed or node is restarted */
			if(start_.data == false) {
				start_.data = joy_msg->buttons[button_start_];
			}

			/* Reset button combination: start + pause */
			if(joy_msg->buttons[button_start_] == 1 and joy_msg->buttons[button_pause_] == 1) {
				Reset();
			}

			previous_stop_ = joy_msg->buttons[button_stop_];
			PublishActions();
		}

		void PublishActions() {
			estop_pub_.publish(estop_);
			pause_pub_.publish(pause_);
			start_pub_.publish(start_);
			stop_pub_.publish(stop_);
		}

	public:
		JoyActions(ros::NodeHandle &nh):nh_(nh) {

			Reset();

			/* Check if there is a parameter with a mapping of the joystick buttons. If not use the default values */
			nh_.param("joy_estop", button_estop_, button_estop_);
			nh_.param("joy_pause", button_pause_, button_pause_);
			nh_.param("joy_start", button_start_, button_start_);
			nh_.param("joy_stop",  button_stop_,  button_stop_);

			/* Check parameter for the topic name of joy */
			nh_.param("joy_topic_name", joy_topic_name_,  joy_topic_name_);

			/* Check parameter for the topic name of action publishers */
			nh_.param("joy_actions_estop", estop_topic_name_,  estop_topic_name_);
			nh_.param("joy_actions_pause", pause_topic_name_,  pause_topic_name_);
			nh_.param("joy_actions_start", start_topic_name_,  start_topic_name_);
			nh_.param("joy_actions_stop",  stop_topic_name_,   stop_topic_name_);

			/* Buffer size is set to 1 as we always want the latest data */
			estop_pub_ = nh_.advertise<std_msgs::Bool>(estop_topic_name_, 1);
			pause_pub_ = nh_.advertise<std_msgs::Bool>(pause_topic_name_, 1);
			start_pub_ = nh_.advertise<std_msgs::Bool>(start_topic_name_, 1);
			stop_pub_  = nh_.advertise<std_msgs::Bool>(stop_topic_name_,  1);

			/* Buffer size is set to 5; we want to retain some of the old data but not a lot */
			joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(joy_topic_name_, 5, &JoyActions::JoyCallBack, this);
		}

		void PublishActions(const ros::TimerEvent&) {
			PublishActions();
		}

};

#endif /* JOY_ACTIONS_HPP_ */
