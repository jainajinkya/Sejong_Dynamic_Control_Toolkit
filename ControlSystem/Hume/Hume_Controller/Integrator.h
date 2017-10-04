#ifndef INTEGRATOR
#define INTEGRATOR

#include <utils/wrap_eigen.hpp>
#include <Configuration.h>

class Integrator{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Integrator(double dt);
    ~Integrator();

    void ForwardDynamics(Eigen::Matrix<double, NUM_Q, 1> & Q,
                         Eigen::Matrix<double, NUM_QDOT, 1> & Qdot,
                         const Eigen::Matrix<double, NUM_ACT_JOINT, 1>  & torque_command,
                         const sejong::Matrix & Jc);
    sejong::Matrix J_cm_;
    sejong::Vector F_ext_;

private:
    void _ExtrinsicIntegrateConfiguration(Eigen::Matrix<double, NUM_Q, 1> & Q,
                                          const Eigen::Matrix<double, NUM_QDOT, 1> & Qdot);

    double dt_;
};


#endif
