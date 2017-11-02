#ifndef VALKYRIE_CONTROLLER
#define VALKYRIE_CONTROLLER

#include <Utils/wrap_eigen.hpp>
#include "StateProvider.h"

class WBLC;
class ValkyrieModel;

enum ContactState {
  DB, RS, LS
};

class ValkyrieController{
public:
  ValkyrieController();
  virtual ~ValkyrieController();

  virtual void ComputeTorqueCommand(sejong::Vector & gamma) = 0;
  virtual void Initialization() = 0;

protected:
  void _PreProcessing_Command();
  void _PostProcessing_Command(sejong::Vector & gamma);
  void _DynConsistent_Inverse(const sejong::Matrix & J, sejong::Matrix & Jinv);

  ContactState contact_state_;

  StateProvider* sp_;
  ValkyrieModel* robot_model_;
  int phase_;

  sejong::Matrix A_;
  sejong::Matrix Ainv_;
  sejong::Vector grav_;
  sejong::Vector coriolis_;
};

#endif
