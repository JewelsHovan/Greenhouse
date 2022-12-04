#include "highlevel_controller/highlevel_controller.hpp"


// main node to intialize highlevel controller class 
int main(int argc, char **argv) {

    // initialize ros and node handle
	ros::init(argc, argv, "highlevel_controller_node");
	ros::NodeHandle nh;
	ros::Rate loopRate(1000);
	
	HighLevelController aHighLevelController(&nh);
	ros::Duration(2).sleep(); 
    // go to target from config file parameters
	aHighLevelController.goToTarget();
	
	while (ros::ok()) {
		ros::spinOnce();
        // set rate and loop
		loopRate.sleep();
        // await further instructions
	}
}	
