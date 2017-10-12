#ifndef WALER_2D_CONTROLLER
#define WALER_2D_CONTROLLER

#include <Utils/wrap_eigen.hpp>
#include "StateProvider.hpp"

class Walker2D_Model;

class Walker2D_Controller{
public:
  Walker2D_Controller();
  virtual ~Walker2D_Controller();

  virtual void ComputeTorqueCommand(sejong::Vector & gamma) = 0;
  virtual void Initialization() = 0;

protected:
  void _PreProcessing_Command();
  void _PostProcessing_Command(sejong::Vector & gamma);
  void _DynConsistent_Inverse(const sejong::Matrix & J, sejong::Matrix & Jinv);

  StateProvider* sp_;
  Walker2D_Model* robot_model_;
  int phase_;

  sejong::Matrix A_;
  sejong::Matrix Ainv_;
  sejong::Vector grav_;
  sejong::Vector coriolis_;

};

#endif
