#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <geometry_msgs/Twist.h>

/**
 * This node will read input from the keyboard 
 * and publish a twist for the robot (husky)
 *
 * Three keys:
 * I: forward Vx = 1
 * U: left Vx = 1 and angular_velocity_z = 1
 * O: right Vx = 1 and angular_velocity_z = -1
 * Else: twist = 0
 **/

// global variable twist to publish from callback from keyboard
geometry_msgs::Twist twist_msg;

void keyboard_callback(std_msgs::String key_input){
    // print out the output of the messsage from teleop/cmd
    //ROS_INFO("Key input: [%s]", key_input.data);

    // key determines global twist
    switch (key_input.data[0])
    {
    case 'i':
        // forward linear velocity on x to 1
        twist_msg.linear.x = 1;
        break;
    case 'u':
        // turn left 
        //linear velocity on x-axis to 1 and angular velocitry on z axis to 1
        twist_msg.linear.x = 1;
        twist_msg.angular.z = 1;
        break;
    case 'o':
        twist_msg.linear.x = 1;
        twist_msg.angular.z = -1;
        // turn right
        // linear velocity on x to 1 and angular velocity on z to -1
        break;
    default:
        // twist is a zero vector
        geometry_msgs::Twist new_twist;
        twist_msg = new_twist;
        break;
    }

}


int main(int argc, char **argv){

    // initalization 
    ros::init(argc, argv, "keyboard_listener");

    // handle
    ros::NodeHandle n;

    // subscribe from keyboard reader -> teleop/cmd topic
    ros::Subscriber sub = n.subscribe("teleop/cmd", 10, keyboard_callback);
    // publisher to publish to topic to control velocity of huksy
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);

    // 
    ros::Rate rate(2);

    // publish twist from keyboard input
    while(ros::ok()){
        pub.publish(twist_msg);
        // sleep when no callback is received and spin once received
        rate.sleep();
        ros::spinOnce();
    }
    return 0; 
}





