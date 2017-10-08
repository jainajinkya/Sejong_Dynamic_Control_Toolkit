#include "StateEstimator.hpp"
#include "StateProvider.hpp"
#include <Utils/utilities.hpp>
#include <Openchain3DoF_Model/OC3_Model.hpp>
#include <Filter/filters.hpp>

StateEstimator::StateEstimator(): jvel_filter_(NUM_ACT_JOINT){
  sp_ = StateProvider::GetStateProvider();
  robot_model_ = OC3Model::GetOC3Model();

  for(int i(0); i<NUM_ACT_JOINT; ++i){
    jvel_filter_[i] = new digital_lp_filter(2*M_PI*200, SERVO_RATE);
  }
}

StateEstimator::~StateEstimator(){
}

void StateEstimator::Initialization(_DEF_SENSOR_DATA_){

  for (int i(0); i<NUM_ACT_JOINT; ++i){
    sp_->Q_[NUM_VIRTUAL + i] = jpos[i];
    jvel_filter_[i]->input(jpos[i]);
    sp_->Qdot_[NUM_VIRTUAL + i] = jvel[i];
    // sp_->Qdot_[NUM_VIRTUAL + i] = jvel_filter_[i]->output();
  }
  robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
}

void StateEstimator::Update(_DEF_SENSOR_DATA_){
  for (int i(0); i<NUM_ACT_JOINT; ++i){
    sp_->Q_[NUM_VIRTUAL + i] = jpos[i];

    jvel_filter_[i]->input(jpos[i]);
    sp_->Qdot_[NUM_VIRTUAL + i] = jvel[i];
    // sp_->Qdot_[NUM_VIRTUAL + i] = jvel_filter_[i]->output();
  }
  robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
}
