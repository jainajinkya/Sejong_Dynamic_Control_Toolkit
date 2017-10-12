#ifndef WALKER_2D_SYSTEM
#define WALKER_2D_SYSTEM

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

class Walker2D_Controller;

class Walker2D_System {
public:
  Walker2D_System();
  ~Walker2D_System();
  void Initialization();
  void getTorqueInput(sejong::Vector & torque_command);

  Walker2D_Controller* controller_;
};


#endif
