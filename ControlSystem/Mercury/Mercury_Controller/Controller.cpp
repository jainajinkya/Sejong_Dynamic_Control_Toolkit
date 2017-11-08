#include "Controller.hpp"
#include <Robot_Model/RobotModel.hpp>
#include <Configuration.h>
#include <iostream>
#include <Utils/pseudo_inverse.hpp>
#include "StateProvider.hpp"

#include <Task.hpp>
#include <ContactSpec.hpp>

Controller::Controller():state_machine_time_(0.),
                         ctrl_start_time_(0.)
{

  robot_model_ = RobotModel::GetRobotModel();
  sp_ = StateProvider::GetStateProvider();

  act_list_.resize(NUM_QDOT, true);
  for(int i(0); i<NUM_VIRTUAL; ++i){
    act_list_[i] = false;
  }
  printf("[Controller] Constructed\n");
}

Controller::~Controller(){
}

void Controller::_PreProcessing_Command(){
  robot_model_->getMassInertia(A_);
  robot_model_->getInverseMassInertia(Ainv_);
  robot_model_->getGravity(grav_);
  robot_model_->getCoriolis(coriolis_);

  state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

  task_list_.clear();
  contact_list_.clear();
}

void Controller::_PostProcessing_Command(sejong::Vector & gamma){
  for(int i(0); i<task_list_.size(); ++i){
    task_list_[i]->UnsetTask();
  }
  for(int i(0); i<contact_list_.size(); ++i){
    contact_list_[i]->UnsetContact();
  }
}

void Controller::_DynConsistent_Inverse(const sejong::Matrix & J, sejong::Matrix & Jinv){
  Matrix Jtmp(J * Ainv_ * J.transpose());
  Matrix Jtmp_inv;
  sejong::pseudoInverse(Jtmp, 0.00000001, Jtmp_inv, 0);
  Jinv = Ainv_ * J.transpose() * Jtmp_inv;
}
