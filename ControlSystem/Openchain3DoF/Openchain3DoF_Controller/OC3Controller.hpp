#ifndef OPENCHAIN_3D_CONTROLLER
#define OPENCHAIN_3D_CONTROLLER

#include <Utils/wrap_eigen.hpp>
#include "StateProvider.hpp"

class OC3Model;

class OC3Controller{
public:
  OC3Controller();
  virtual ~OC3Controller();

  virtual void ComputeTorqueCommand(sejong::Vector & gamma) = 0;
  virtual void Initialization() = 0;

protected:
  void _PreProcessing_Command();
  void _PostProcessing_Command(sejong::Vector & gamma);
  void _DynConsistent_Inverse(const sejong::Matrix & J, sejong::Matrix & Jinv);

  StateProvider* sp_;
  OC3Model* robot_model_;
  int phase_;

  sejong::Matrix A_;
  sejong::Matrix Ainv_;
  sejong::Vector grav_;
  sejong::Vector coriolis_;

};

#endif
