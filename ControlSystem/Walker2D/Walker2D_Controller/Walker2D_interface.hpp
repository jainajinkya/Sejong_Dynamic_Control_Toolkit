#ifndef INTERFACE_WALKER_2D
#define INTERFACE_WALKER_2D

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include "StateEstimator.hpp"

#include <map>
#include <string>

class Walker2D_System;

class Walker2D_interface{
public:
    Walker2D_interface();
    ~Walker2D_interface();

public:
  void GetCommand(_DEF_SENSOR_DATA_,
                  std::vector<double> & command);

private:
    bool _Initialization(_DEF_SENSOR_DATA_);

    Walker2D_System* oc2_sys_;

  sejong::Vector torque_command_;
    sejong::Vector sensed_torque_;
    int count_;
    double running_time_;

    sejong::Vector initial_jpos_;
    StateEstimator state_estimator_;
};

#endif
