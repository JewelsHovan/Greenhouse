#pragma once
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensors_msgs/JointState.h>


class TaskSpaceController{

    public:
        TaskSpaceController(ros::NodeHandle& nodeHandle);
        // feedback functions
        void getJointFeedback();
        static computeTaskSpacePosition();
        static computeTaskSpaceVelocity();
        // Computation of J, J_psuedo_inverse, M, h
        void computeJacobian();
        void computeJacobianDot(); // pseudo inverse
        void computeMassMatrix();
        void computeH();
        // implementation of TaskSpace inverse dynamics
        static cmdAcceleration();






    private:
        // fields
        ros::NodeHandle& node_handle_;
        ros::Subscriber joint_sub_;

        Eigen::VectorXd joint_pos;
        Eigen::VectorXd joint_vel;

        Eigen::MatrixXd trajJacobian;
        Eigen::MatrixXd trajJacobianDot;
        Eigen::MatrixXd massMatrix;
        Eigen::MatrixXd hMatrix;


        // variables from yaml

}