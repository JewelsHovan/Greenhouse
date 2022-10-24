#include "TaskSpaceController.hpp"


TaskSpaceController::TaskSpaceController(ros::NodeHandle& nodeHandle) : node_handle_(nodeHandle) {
    this->getJointFeedBack;
    // compute feedback task-space positions and velocities
    std::string urdf_file_name;
    if(ros::param:hasParam("urdf_file_name")){
        ros::param:getParam("urdf_file_name", urdf_file_name);
    }
    pinocchio::Model model;
	pinocchio::urdf::buildModel(urdf_file_name, model, false);	
	pinocchio::Data data(model); 
	const int JOINT_ID = 7; // end-effector joint
	double dt = 0.002;
	int dim_joints = model.nq;


	pinocchio::forwardKinematics(model, data, joint_pos, joint_vel);
	pinocchio::SE3 pose_now = data.oMi[JOINT_ID]; // end-effector pose

}


TaskSpaceController::getJointFeedBack(){
    sensor_msgs::JointState joint_feedback = ros::Topic::waitForMessage("/gen3/joint_states");

    for(int i = 0; i<joint_state.position.size(); i++){
            joint_pos[i] = joint_state.position[i];
            joint_vel[i] = joint_state.velocity[i];
        } 
}


TaskSpaceController::computeJacobian(){

}

TaskSpaceController::JacobianDot(){

}

TaskSpaceController::computeMassMatrix(){

}