#include "Controller.hpp"
#include <Robot_Model/RobotModel.hpp>

#include <Configuration.h>
#include <iostream>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>
#include "StateProvider.hpp"

Controller::Controller()
{
  robot_model_ = RobotModel::GetRobotModel();
  sp_ = StateProvider::GetStateProvider();
}

Controller::~Controller(){
}

void Controller::_PreProcessing_Command(){
  robot_model_->getMassInertia(A_);
  robot_model_->getInverseMassInertia(Ainv_);
  robot_model_->getGravity(grav_);
  robot_model_->getCoriolis(coriolis_);
}

void Controller::_PostProcessing_Command(sejong::Vector & gamma){
}

void Controller::_DynConsistent_Inverse(const sejong::Matrix & J, sejong::Matrix & Jinv){
  Matrix Jtmp(J * Ainv_ * J.transpose());
  Matrix Jtmp_inv;
  sejong::pseudoInverse(Jtmp, 0.00000001, Jtmp_inv, 0);
  Jinv = Ainv_ * J.transpose() * Jtmp_inv;
}
