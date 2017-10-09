#ifndef INTERFACE_OPENCHAIN_2DoF 
#define INTERFACE_OPENCHAIN_2DoF 

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include "StateEstimator.hpp"

#include <map>
#include <string>

class OC2System;

class OC2_interface{
public:
    OC2_interface();
    ~OC2_interface();

public:
  void GetCommand(_DEF_SENSOR_DATA_,
                  std::vector<double> & command);

private:
    bool _Initialization(_DEF_SENSOR_DATA_);

    OC2System* oc2_sys_;

  sejong::Vector torque_command_;
    sejong::Vector sensed_torque_;
    int count_;
    double running_time_;

    sejong::Vector initial_jpos_;
    StateEstimator state_estimator_;
};

#endif
