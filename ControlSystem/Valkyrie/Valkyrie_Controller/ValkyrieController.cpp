#include "ValkyrieController.h"
#include <Valkyrie_Model/Valkyrie_Model.h>

#include <Configuration.h>
#include <iostream>
#include <stdio.h>
#include <Utils/utilities.h>
#include <Utils/pseudo_inverse.hpp>
#include <ReactionForceCalculator.h>
#include <ContactWrenchCalculator.h>
#include <Utils/pseudo_inverse.hpp>

ValkyrieController::ValkyrieController():
  contact_state_(DB)
{
  robot_model_ = ValkyrieModel::GetValkyrieModel();
  sp_ = StateProvider::GetStateProvider();
}

ValkyrieController::~ValkyrieController(){
}

void ValkyrieController::_PreProcessing_Command(){
  robot_model_->getMassInertia(A_);
  robot_model_->getInverseMassInertia(Ainv_);
  robot_model_->getGravity(grav_);
  robot_model_->getCoriolis(coriolis_);
}

void ValkyrieController::_PostProcessing_Command(sejong::Vector & gamma){
}
void ValkyrieController::_DynConsistent_Inverse(const sejong::Matrix & J, sejong::Matrix & Jinv){
  Matrix Jtmp(J * Ainv_ * J.transpose());
  Matrix Jtmp_inv;
  sejong::pseudoInverse(Jtmp, 0.00000001, Jtmp_inv, 0);
  Jinv = Ainv_ * J.transpose() * Jtmp_inv;
}
