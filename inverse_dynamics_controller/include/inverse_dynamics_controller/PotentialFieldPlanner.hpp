#include <Eigen/Dense>


class PotentialFieldPlanner{
    public:
        PotentialFieldPlanner(ros::NodeHandle& nodeHandle);
        Eigen::Vector3d receiveTargetPosition();
        Eigen::Vector3d getTaskSpaceVelocity(Eigen::Vector3d K_attr, Eigen::Vector3d x_tar, Eigen::Vector3d x_feedback);
        Eigen::Vector3d getTaskSpacePosition(Eigen::Vector3d x_feedback, Eigen::Vector3d ref_velocity, double deltaTime);
        Eigen::Vector3d getTaskSpaceAccelerations();

    private:
        // fields

};