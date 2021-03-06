#include "DracoController.hpp"
#include <Draco_Model/Draco_Model.hpp>

#include <Configuration.h>
#include <iostream>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>

DracoController::DracoController():
  contact_state_(SS)
{
  robot_model_ = DracoModel::GetDracoModel();
  sp_ = StateProvider::GetStateProvider();
}

DracoController::~DracoController(){
}

void DracoController::_PreProcessing_Command(){
  robot_model_->getMassInertia(A_);
  robot_model_->getInverseMassInertia(Ainv_);
  robot_model_->getGravity(grav_);
  robot_model_->getCoriolis(coriolis_);
}

void DracoController::_DynConsistent_Inverse(const sejong::Matrix & J, sejong::Matrix & Jinv){
  Matrix Jtmp(J * Ainv_ * J.transpose());
  Matrix Jtmp_inv;
  sejong::pseudoInverse(Jtmp, 0.00000001, Jtmp_inv, 0);
  Jinv = Ainv_ * J.transpose() * Jtmp_inv;
}

void DracoController::_PostProcessing_Command(sejong::Vector & gamma){
}
