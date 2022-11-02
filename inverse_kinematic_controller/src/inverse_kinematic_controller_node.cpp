#include <ros/ros.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "inverse_controller");
    // handle
    ros::NodeHandle nh;
    InverseController IC = InverseController(&nh);
    spin();
    return 0;
}