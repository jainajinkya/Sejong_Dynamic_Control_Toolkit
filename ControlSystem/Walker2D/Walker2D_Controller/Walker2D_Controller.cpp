#include "Walker2D_Controller.hpp"
#include <Walker2D_Model/Walker2D_Model.hpp>

#include <Configuration.h>
#include <iostream>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>

Walker2D_Controller::Walker2D_Controller()
{
  robot_model_ = Walker2D_Model::GetWalker2D_Model();
  sp_ = StateProvider::GetStateProvider();
}

Walker2D_Controller::~Walker2D_Controller(){
}

void Walker2D_Controller::_PreProcessing_Command(){
  robot_model_->getMassInertia(A_);
  robot_model_->getInverseMassInertia(Ainv_);
  robot_model_->getGravity(grav_);
  robot_model_->getCoriolis(coriolis_);
}

void Walker2D_Controller::_DynConsistent_Inverse(const sejong::Matrix & J, sejong::Matrix & Jinv){
  Matrix Jtmp(J * Ainv_ * J.transpose());
  Matrix Jtmp_inv;
//  sejong::pseudoInverse(Jtmp, 0.00000001, Jtmp_inv, 0);
  sejong::pseudoInverse(Jtmp, 0.0001, Jtmp_inv, 0);    
  Jinv = Ainv_ * J.transpose() * Jtmp_inv;
}

void Walker2D_Controller::_PostProcessing_Command(sejong::Vector & gamma){
}
