#ifndef STATE_ESTIMATOR
#define STATE_ESTIMATOR

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>

class StateProvider;
class ValkyrieModel;
class Valkyrie_Left_Leg;
class Valkyrie_Right_Leg;
class filter;

class StateEstimator{
public:
    StateEstimator();
    ~StateEstimator();

    void Initialization(_DEF_SENSOR_DATA_);
    void Update(_DEF_SENSOR_DATA_);

protected:
    void _AllocateLeftState();
    void _AllocateRightState();
    sejong::Vector leg_q_;
    sejong::Vector leg_qdot_;

    double initial_height_;
    sejong::Vect3 foot_pos_;
    sejong::Vect3 foot_vel_;

    StateProvider* sp_;
    ValkyrieModel* robot_model_;
    Valkyrie_Left_Leg * left_leg_;
    Valkyrie_Right_Leg * right_leg_;
    std::vector<filter*> jvel_filter_;
};

#endif
