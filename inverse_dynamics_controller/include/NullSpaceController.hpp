#include <Eigen/Dense>



class NullSpaceController{


    private:
        // fields
        Eigen::VectorXd nullSpaceTarget = Eigen::VectorXd(7);
    public:
        NullSpaceController();
        static getJointSpacePositions();
        static getJointSpaceVelocities();
        // reference joint space motion
        static computeJointSpacePosition();
        static computeJointSpaceVelocity();
        static computeJointSpaceAcceleration();

        // Implement Joint-Space Inverse Dynamics
        static cmdJointSpaceAccelerration();
        static computeJointTorque();
}