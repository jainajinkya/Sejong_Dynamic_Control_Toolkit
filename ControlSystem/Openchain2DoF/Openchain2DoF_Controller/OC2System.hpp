#ifndef OPENCHAIN_2DOF_SYSTEM
#define OPENCHAIN_2DOF_SYSTEM

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

class OC2Controller;

class OC2System {
public:
  OC2System();
  ~OC2System();
  void Initialization();
  void getTorqueInput(sejong::Vector & torque_command);

  OC2Controller* controller_;
};


#endif
