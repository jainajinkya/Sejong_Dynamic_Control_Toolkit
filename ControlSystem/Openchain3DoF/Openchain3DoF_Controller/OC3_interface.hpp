#ifndef INTERFACE_OPENCHAIN_3D
#define INTERFACE_OPENCHAIN_3D

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include "StateEstimator.hpp"

#include <map>
#include <string>

class OC3System;

class OC3_interface{
public:
    OC3_interface();
    ~OC3_interface();

public:
  void GetCommand(_DEF_SENSOR_DATA_,
                  std::vector<double> & command);

private:
    bool _Initialization(_DEF_SENSOR_DATA_);

    OC3System* oc3_sys_;

  sejong::Vector torque_command_;
    sejong::Vector sensed_torque_;
    int count_;
    double running_time_;

    sejong::Vector initial_jpos_;
    StateEstimator state_estimator_;
};

#endif
