#ifndef INTERFACE_H
#define INTERFACE_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include "StateEstimator.hpp"

class Test;

class interface{
public:
  interface();
  ~interface();

public:
  void GetCommand(_DEF_SENSOR_DATA_,
                  std::vector<double> & command);
  void GetReactionForce(std::vector<sejong::Vect3> & reaction_force );

  sejong::Quaternion global_ori_;
  
private:
  bool _Initialization(_DEF_SENSOR_DATA_);
  Test* test_;

  sejong::Vector virtual_sensor_;
  sejong::Vector torque_command_;
  sejong::Vector sensed_torque_;
  int count_;
  double running_time_;

  sejong::Vector initial_jpos_;
  StateEstimator state_estimator_;
};

#endif
