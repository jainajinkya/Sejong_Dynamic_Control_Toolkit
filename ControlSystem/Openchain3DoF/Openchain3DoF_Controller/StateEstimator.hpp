#ifndef STATE_ESTIMATOR
#define STATE_ESTIMATOR

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>

class StateProvider;
class OC3Model;
class filter;

class StateEstimator{
public:
    StateEstimator();
    ~StateEstimator();

    void Initialization(_DEF_SENSOR_DATA_);
    void Update(_DEF_SENSOR_DATA_);

protected:
    double initial_height_;
    StateProvider* sp_;
    OC3Model* robot_model_;

    std::vector<filter*> jvel_filter_;
};

#endif
