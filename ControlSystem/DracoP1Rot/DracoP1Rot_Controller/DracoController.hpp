#ifndef DRACO_P1_ROTATION_JOINT_CONTROLLER
#define DRACO_P1_ROTATION_JOINT_CONTROLLER

#include <Utils/wrap_eigen.hpp>
#include "StateProvider.hpp"

class DracoModel;

enum ContactState {
  SS
};

enum WalkingPhase{
  RS_DoubleSupport = 0,
  RightStanceSingleSupport = 1,
  LS_DoubleSupport = 2,
  LeftStanceSingleSupport = 3,
};

class DracoController{
public:
  DracoController();
  virtual ~DracoController();

  virtual void ComputeTorqueCommand(sejong::Vector & gamma) = 0;
  virtual void Initialization() = 0;

protected:
  void _PreProcessing_Command();
  void _PostProcessing_Command(sejong::Vector & gamma);
  void _DynConsistent_Inverse(const sejong::Matrix & J, sejong::Matrix & Jinv);
  ContactState contact_state_;

  StateProvider* sp_;
  DracoModel* robot_model_;
  int phase_;

  sejong::Matrix A_;
  sejong::Matrix Ainv_;
  sejong::Vector grav_;
  sejong::Vector coriolis_;

  double state_machine_time_;
  double start_time_;
};

#endif
