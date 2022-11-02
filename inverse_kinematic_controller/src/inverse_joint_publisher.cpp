#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ros/ros.h>
#include <cubic_polynomial_planner/MoveRobot.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include "cubic_polynomial.h"


// read the joint feedback by subscribing to the topic /gen3/joint_states 

// plan the trajectory for the robot end-effector positions using cubic-polynomial
// ignore orientations 


// compute the end-effector positions and Jacobian matrix using pinocchio   :w

// compute the joint velocity using inverse kinematics 
// given position q, velocity x_dot, and jacobian matrix J -> use inverse kinematics to find q_ref

// x_ref = s_dot(t)(x^tar - x^ini)
// q_dot_cmd = J * x_dot_ref

class InverseController{

    private:
    // ros fields
    ros::Subscriber joint_sub;
    ros::Publisher joint_pub;
    ros::ServiceServer move_robot_srv;
    //start positions
    Eigen::VectorXd start_pos;
    // target position and time
    Eigen::VectorXd target_position;
    Eigen::VectorXd end_effector_pos;
    double target_t;

    // callback from subscriber
    Eigen::VectorXd joint_pos;
    Eigen::VectorXd joint_vel;

    double start_time;
    double target_time;
    ros::Duration duration;
    
    // pinocchio setup
    double dt = .0002;
    std::string urdf_file_name = "/home/julienh/Desktop/Comp514/Projects/ros_kortex/kortex_description/urdf/gen3.urdf";
    pinocchio::Model model;
    int JOINT_ID = 7;
    int dim_joints;

    Eigen::VectorXd joint_pos = Eigen::VectorXd(7);
    Eigen::VectorXd joint_vel = Eigen::VectorXd(7);
    Eigen::VectorXd start_pos = Eigen::VectorXd(7);
    Eigen::VectorXd final_pos = Eigen::VectorXd(7);


    public:
    // constructor
    InverseController(ros::NodeHandle *nh){
        // initialization

        joint_sub = nh->subscribe<sensor_msgs::JointState>("/gen3/joint_states", 10, &InverseController::callback_joint, this);
        joint_pub = nh->advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_position_controller/command", 10);
        move_robot_srv = nh->advertiseService("move_robot", &InverseController::callback_service, this);

    }
    // subscriber callback to read joint positions and velocities
    void callback_joint(sensor_msgs::JointState &joint_state){
        // save joint positions
        int i = 0;
        for(i; i<joint_state.position.size(); i++){
            joint_pos[i] = joint_state.position[i];
            joint_vel[i] = joint_state.velocity[i];
        }
    }

    bool callback_service(cubic_polynomial_planner::MoveRobot::Request &req, cubic_polynomial_planner::MoveRobot::Response &res){
        // save initial position
        //joint_sub.callTopic();
        // save target positions and time
        target_position(0) = req.x;
        target_position(1) = req.y;
        target_position(2) = req.z;
        end_effector_pos = Eigen::VectorXd::Zero(6);
        for(int i=0; i<3; i++){
            end_effector_pos(i) = target_position(i);
        }

        target_t = (double) req.t;


        // setup

        // ros start time
        ros::Time ros_start_time = ros::Time::now();
        start_time = ros_start_time.toSec();

        for(int i = 0; i < joint_pos.size(); i++){
            start_pos[i] = joint_pos[i];
        }
        // update position
        this->update_position();

        res.return_msg = "Finished Task";
        duration = ros::Time::now() - ros_start_time;
        return true;


    }


    void update_position(){

        // using pinocchio to build model 
        pinocchio::urdf::buildModel(urdf_file_name, model, false);
        pinocchio::Data data(model);
        dim_joints = model.nq; // joint configuration vector

        double t = ros::Time::now().toSec() - start_time;
        double T = target_t + ros::Time::now().toSec();
        
        while(t <= T){
            // calculate x_dot_ref using cubic_polynomial planner 
            cubic_polynomial planner = cubic_polynomial(start_pos, target_position, target_t);
            Eigen::VectorXd x_dot_ref = planner.computeVelReference(ros::Time::now().toSec() - start_time);
            Eigen::VectorXd x_ref = planner.computePosReference(ros::Time::now().toSec() - start_time);

            // forward kinematics -> current end-effector pose
            pinocchio::forwardKinematics(model, data, joint_pos, joint_vel);
            pinocchio::SE3 pose_now = data.oMi[JOINT_ID];

            // translate into a vector
            Eigen::VectorXd cur_pose = Eigen::VectorXd(6); // 6d for position and orientation
            for(int i = 0; i < 3; i++){
                cur_pose(i) = (pose_now.translation())(i);
            }

            // Jacobian in local frame 
            Eigen::MatrixXd jacobian_local = Eigen::MatrixXd::Zero(6,dim_joints) ;
	        pinocchio::computeJointJacobian(model, data, joint_pos, JOINT_ID, jacobian_local);

	        // *************** Jacobian in the world frame *******************************************
	        Eigen::MatrixXd jacobian_local_world = Eigen::MatrixXd::Zero(6,dim_joints) ;
	        pinocchio::computeAllTerms(model, data, joint_pos, joint_vel) ;
	        pinocchio::getJointJacobian(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_local_world) ;

            // Pseudo-inverse of Jacobian
    	    Eigen::MatrixXd jacobian_dot = Eigen::MatrixXd::Zero(6,dim_joints) ;
	        pinocchio::computeJointJacobiansTimeVariation(model, data, joint_pos, joint_vel );
	        pinocchio::getJointJacobianTimeVariation(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_dot) ;

            // redundancy resolution
            Eigen::MatrixXd Nq = (Eigen::MatrixXd::Identity(7,7) - jacobian_dot * jacobian_local) * joint_vel;

            // Make float array to publish
            std_msgs::Float64MultiArray ref_joint_vel_float;
            ref_joint_vel_float.data.resize(7);		
            for (int i=0; i<7; i++) {
                ref_joint_vel_float.data[i] = joint_vel(i);
            }
            
            // Publish reference joint velocity
            joint_pub.publish(ref_joint_vel_float);
            double t = ros::Time::now().toSec() - start_time;

            // update joint velocities
            joint_vel = ref_joint_vel;
    }
}
};

