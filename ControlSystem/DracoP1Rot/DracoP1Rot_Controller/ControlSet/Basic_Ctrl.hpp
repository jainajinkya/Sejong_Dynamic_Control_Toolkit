#ifndef DRACO_P1_ROTATION_BASIC_CONTROLLER
#define DRACO_P1_ROTATION_BASIC_CONTROLLER

#include <DracoP1Rot_Controller/DracoController.hpp>

class Basic_Ctrl: public DracoController{
public:
  Basic_Ctrl();
  virtual ~Basic_Ctrl();

  virtual void ComputeTorqueCommand(sejong::Vector & gamma);
  virtual void Initialization();

protected:
  void _jpos_ctrl_hanging(sejong::Vector & gamma);
  void _time_test();

  sejong::Vector jpos_ini_;
};

#endif
