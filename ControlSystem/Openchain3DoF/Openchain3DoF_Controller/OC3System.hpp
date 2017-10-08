#ifndef OPENCHAIN_3D_SYSTEM
#define OPENCHAIN_3D_SYSTEM

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

class OC3Controller;

class OC3System {
public:
  OC3System();
  ~OC3System();
  void Initialization();
  void getTorqueInput(sejong::Vector & torque_command);

  OC3Controller* controller_;
};


#endif
