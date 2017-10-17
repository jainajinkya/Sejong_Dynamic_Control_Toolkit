#include "OC2Controller.hpp"
#include <Openchain2DoF_Model/OC2_Model.hpp>

#include <Configuration.h>
#include <iostream>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>

OC2Controller::OC2Controller()
{
  robot_model_ = OC2Model::GetOC2Model();
  sp_ = StateProvider::GetStateProvider();
}

OC2Controller::~OC2Controller(){
}

void OC2Controller::_PreProcessing_Command(){
  robot_model_->getMassInertia(A_);
  robot_model_->getInverseMassInertia(Ainv_);
  robot_model_->getGravity(grav_);
  robot_model_->getCoriolis(coriolis_);
}

void OC2Controller::_DynConsistent_Inverse(const sejong::Matrix & J, sejong::Matrix & Jinv){
  Matrix Jtmp(J * Ainv_ * J.transpose());
  Matrix Jtmp_inv;
//  sejong::pseudoInverse(Jtmp, 0.00000001, Jtmp_inv, 0);
  sejong::pseudoInverse(Jtmp, 0.0001, Jtmp_inv, 0);  
  Jinv = Ainv_ * J.transpose() * Jtmp_inv;
}

void OC2Controller::_PostProcessing_Command(sejong::Vector & gamma){
}
