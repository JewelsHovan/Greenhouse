// cubic_polynomial.h
#ifndef cubic_polynomial_H
#define cubic_polynomial_H

class cubic_polynomial{

    // private fields
    Eigen::VectorXd start_pos;
    Eigen::VectorXd target_pos;
    double T;

    public:
    cubic_polynomial(Eigen::VectorXd start_pos, Eigen::VectorXd target_pos, double time);

    double computeVelReference(double time);

    double computePosReference(double time);

    static double time_scale_position(double current_t, double final_t);

    static double time_scale_velocity(double current_t, double final_t);

    static double compute_position(double cur_pos, double target_pos, double s_t);

    static double compute_velocity(Eigen::VectorXd cur_pos, Eigen::VectorXd target_pos, double s_t);
};


#endif