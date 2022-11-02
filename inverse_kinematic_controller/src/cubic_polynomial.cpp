#include "cubic_polynomial.h"
#include <Eigen/Dense>


cubic_polynomial::cubic_polynomial(Eigen::VectorXd aStart_pos, Eigen::VectorXd aTarget_pos, double target_time){
    // given the initial position and final position and current time t
    // compute the x_velocity 
    start_pos = aStart_pos;
    target_pos = aTarget_pos;
    T = target_time;
}

Eigen::VectorXd cubic_polynomial::computeVelReference(double time){
    double s_t = time_scale_velocity(time, T);
    return compute_velocity(start_pos, target_pos, s_t);
}

Eigen::VectorXd cubic_polynomial::computePosReference(double time){
    double s_t = time_scale_position(time, T);
    return compute_position(start_pos, target_pos, s_t);
}

double cubic_polynomial::time_scale_position(double current_t, double final_t){
    // s(t) = (3t^2/T^2 - 2t^3/T^3)
    return ( (3*pow(current_t, 2)/(pow(final_t, 2))) - (2*pow(current_t, 3) / (pow(final_t, 3))) );
}

double cubic_polynomial::time_scale_velocity(double current_t, double final_t){
    // s(t) = (6t/T^2 - 6t^2/T^3)
    return ( (3*pow(current_t, 2)/(pow(final_t, 2))) - (2*pow(current_t, 3) / (pow(final_t, 3))) );
}

Eigen::VectorXd cubic_polynomial::compute_position(double cur_pos, double target_pos, double s_t){
    return cur_pos + (s_t*(target_pos - cur_pos));
}

Eigen::VectorXd cubic_polynomial::compute_velocity(Eigen::VectorXd cur_pos, Eigen::VectorXd target_pos, double s_t){
    return (s_t*(target_pos - cur_pos));
}

