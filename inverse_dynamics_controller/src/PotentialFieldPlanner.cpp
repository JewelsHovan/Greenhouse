#include "inverse_dynamics_controller/PotentialFieldPlanner.hpp"


PotentialFieldPlanner::PotentialFieldPlanner(ros::NodeHandle& nh): node_handle_(nh){
    // compute task space:
    // velocity, position, acceleration
}



// methods

Eigen::Vector3d PotentialFieldPlanner::getTaskSpaceVelocity(Eigen::Vector3d K_attr, Eigen::Vector3d x_tar, Eigen::Vector3d x_feedback){
    return K_attr * (x_tar - x_feedback)
}

Eigen::Vector3d PotentialFieldPlanner::getTaskSpacePosition(Eigen::Vector3d x_feedback, Eigen::Vector3d ref_velocity, double deltaTime){
    return x_feedback + (ref_velocity * deltaTime);
}

Eigen::Vector3d PotentialFieldPlanner::getTaskSpaceAccelerations(){
    return Eigen::Vector3d::Zero(3);
}