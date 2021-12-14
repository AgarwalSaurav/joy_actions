/**
Author: Saurav Agarwal
Email: sagarw10@uncc.edu
This package implements actions using a joystick
Emergency Stop: Once triggered the estop remains true unless the reset buttons are pressed
Pause:					Pause till the button is pressed
Start:					Once triggered the start remains true unless the reset buttons are pressed
Stop:						Button toggles stop
**/

#include <joy_actions/joy_actions.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "joy_actions");
	ros::NodeHandle nh;
	JoyActions joy_actions(nh);
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), &JoyActions::PublishActions, &joy_actions);

	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();
}
