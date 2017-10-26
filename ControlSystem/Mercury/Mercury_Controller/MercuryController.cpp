#include "MercuryController.hpp"
#include <Mercury_Model/MercuryModel.hpp>
#include "StateProvider.hpp"

#include <iostream>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>

#include <Task.hpp>
#include <ContactSpec.hpp>

MercuryController::MercuryController():
  state_machine_time_(0.),
  ctrl_start_time_(0.)
{
  robot_model_ = MercuryModel::GetMercuryModel();
  sp_ = StateProvider::GetStateProvider();
  act_list_.resize(NUM_QDOT, true);
  for(int i(0); i<NUM_VIRTUAL; ++i){
    act_list_[i] = false;
  }
}

MercuryController::~MercuryController(){
}

void MercuryController::_PreProcessing_Command(){
  robot_model_->getMassInertia(A_);
  robot_model_->getInverseMassInertia(Ainv_);
  robot_model_->getGravity(grav_);
  robot_model_->getCoriolis(coriolis_);

  state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

  task_list_.clear();
  contact_list_.clear();
}

void MercuryController::_DynConsistent_Inverse(const sejong::Matrix & J, sejong::Matrix & Jinv){
  Matrix Jtmp(J * Ainv_ * J.transpose());
  Matrix Jtmp_inv;
  sejong::pseudoInverse(Jtmp, 0.00000001, Jtmp_inv, 0);
  Jinv = Ainv_ * J.transpose() * Jtmp_inv;
}

void MercuryController::_PostProcessing_Command(sejong::Vector & gamma){
  for(int i(0); i<task_list_.size(); ++i){
    task_list_[i]->UnsetTask();
  }
  for(int i(0); i<contact_list_.size(); ++i){
    contact_list_[i]->UnsetContact();
  }
}
