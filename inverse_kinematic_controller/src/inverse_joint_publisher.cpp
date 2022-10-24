#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <cubic_polynomial_planner/MoveRobot.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include "cubic_polynomial.h"
#include <ros/ros.h>


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
    // fields
    ros::Subscriber joint_sub;
    ros::Publisher joint_pub;
    ros::ServiceServer move_robot_srv;
    //start positions
    Eigen::VectorXd start_pos;
    // target position and time
    Eigen::VectorXd target_position;
    double target_t;
    // callback from subscriber
    Eigen::VectorXd joint_pos;
    Eigen::VectorXd joint_vel;

    double start_time;
    
    // pinocchio setup
    double dt = .0002;
    std::string urdf_file_name = "/home/julienh/Desktop/Comp514/Projects/ros_kortex/kortex_description/urdf/gen3.urdf";
    pinocchio::Model model;
    int JOINT_ID = 7;
    int dim_joints;




    public:
    // constructor
    InverseController(ros::NodeHandle *nh){
        // initialization

        joint_sub = nh->subscribe<sensor_msgs::JointState>("/gen3/joint_states", 10, &InverseController::callback_joint);
        joint_pub = nh->advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_position_controller/command", 10, this);
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
        target_t = (double) req.t;


        // using pinocchio to build model 
        pinocchio::urdf::buildModel(urdf_file_name, model, false);
        pinocchio::Data data(model);
        dim_joints = model.nq;

        // current time
        ros::Time ros_start_time = ros::Time::now();
        start_time = ros::Time::now().toSec();

        // calculate forward kinematics to get the start end_effector_pos
        pinocchio::forwardKinematics(model, data, joint_pos, joint_vel);
        pinocchio::SE3 pose_now = data.oMi[JOINT_ID];        
        
        // jacobian 
        Eigen::MatrixXd jacobian_local_world = Eigen::MatrixXd::Zero(3, dim_joints);
        pinocchio::computeAllTerms(model, data, joint_pos, joint_vel);
        pinocchio::getJointJacobian(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_local_world);

        Eigen::MatrixXd jacobian_dot = Eigen::MatrixXd::Zero(3, dim_joints); 
        pinocchio::computeJointJacobiansTimeVariation(model, data, joint_pos, joint_vel);
        pinocchio::getJointJacobianTimeVariation(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_dot);

        // update position
        this->update_position();

        // integrate joint_vel to compute next positions
        Eigen::VectorXd joint_pos_next = joint_pos + joint_vel * dt;
        pinocchio::forwardKinematics(model, data, joint_pos_next, joint_vel);
        pinocchio::SE3 pose_next = data.oMi[JOINT_ID];

        res.return_msg = "Finished Task";
        return true;


    }


    void update_position(){
        // calculate x_dot_ref using cubic_polynomial planner 
        cubic_polynomial planner = cubic_polynomial(start_pos, target_position, target_t);
        planner.computeVelReference(ros::Time::now().toSec() - start_time);

        // inverse kinematics = q_dot_ref = J_psuedo_inverse * x_dot_ref



        
    }
};





int main(int argc, char** argv){
    ros::init(argc, argv, "inverse_controller");
    // handle
    ros::NodeHandle nh;
    InverseController IC = InverseController(&nh);
    return 0;
}