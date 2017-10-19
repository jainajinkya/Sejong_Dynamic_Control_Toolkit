#ifndef INTERFACE_DRACO_P1_ROTATIONAL_JOINT
#define INTERFACE_DRACO_P1_ROTATIONAL_JOINT

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include "StateEstimator.hpp"

#include <map>
#include <string>

class DracoTest;

class Interface{
public:
  Interface();
  ~Interface();

public:
  void GetCommand(_DEF_SENSOR_DATA_,
                  std::vector<double> & command);
  void GetReactionForce(std::vector<sejong::Vect3> & reaction_force );

private:
  bool _Initialization(_DEF_SENSOR_DATA_, std::vector<double> & command);
  DracoTest* draco_test_;
  
  sejong::Vector virtual_sensor_;
  sejong::Vector torque_command_;
  sejong::Vector sensed_torque_;
  int count_;
  double running_time_;

  sejong::Vector initial_jpos_;
  StateEstimator state_estimator_;
};

#endif
