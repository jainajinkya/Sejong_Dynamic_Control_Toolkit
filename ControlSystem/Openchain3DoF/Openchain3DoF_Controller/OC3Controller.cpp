#include "OC3Controller.hpp"
#include <Openchain3DoF_Model/OC3_Model.hpp>

#include <Configuration.h>
#include <iostream>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>

OC3Controller::OC3Controller()
{
  robot_model_ = OC3Model::GetOC3Model();
  sp_ = StateProvider::GetStateProvider();
}

OC3Controller::~OC3Controller(){
}

void OC3Controller::_PreProcessing_Command(){
  robot_model_->getMassInertia(A_);
  robot_model_->getInverseMassInertia(Ainv_);
  robot_model_->getGravity(grav_);
  robot_model_->getCoriolis(coriolis_);
}

void OC3Controller::_DynConsistent_Inverse(const sejong::Matrix & J, sejong::Matrix & Jinv){
  Matrix Jtmp(J * Ainv_ * J.transpose());
  Matrix Jtmp_inv;
  sejong::pseudoInverse(Jtmp, 0.00000001, Jtmp_inv, 0);
  Jinv = Ainv_ * J.transpose() * Jtmp_inv;
}

void OC3Controller::_PostProcessing_Command(sejong::Vector & gamma){
}
