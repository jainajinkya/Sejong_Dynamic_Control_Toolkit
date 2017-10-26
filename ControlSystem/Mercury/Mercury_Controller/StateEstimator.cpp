#include "StateEstimator.hpp"
#include "StateProvider.hpp"
#include <Utils/utilities.hpp>
#include <Mercury_Model/MercuryModel.hpp>
#include <Filter/filters.hpp>

StateEstimator::StateEstimator(): jvel_filter_(NUM_ACT_JOINT){
  sp_ = StateProvider::GetStateProvider();
  robot_model_ = MercuryModel::GetMercuryModel();

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

  sejong::Vect3 foot_pos, foot_vel;
  robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_RFOOT, foot_pos);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, SJLinkID::LK_RFOOT, foot_vel);
  sp_->Q_[0] = -foot_pos[0];
  sp_->Q_[1] = -foot_pos[1];
  sp_->Qdot_[0] = -foot_vel[0];
  sp_->Qdot_[1] = -foot_vel[1];

  robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_Body, sp_->Body_pos_);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, SJLinkID::LK_Body, sp_->Body_vel_);
}

void StateEstimator::Update(_DEF_SENSOR_DATA_){

  for (int i(0); i<NUM_ACT_JOINT; ++i){
    sp_->Q_[NUM_VIRTUAL + i] = jpos[i];

    jvel_filter_[i]->input(jpos[i]);
    sp_->Qdot_[NUM_VIRTUAL + i] = jvel[i];
    // sp_->Qdot_[NUM_VIRTUAL + i] = jvel_filter_[i]->output();
  }
  // Local Frame
  sejong::Vect3 foot_pos, foot_vel;
  robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_RFOOT, foot_pos);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, SJLinkID::LK_RFOOT, foot_vel);
  sp_->Q_[0] = -foot_pos[0];
  sp_->Q_[1] = -foot_pos[1];
  sp_->Qdot_[0] = -foot_vel[0];
  sp_->Qdot_[1] = -foot_vel[1];

  robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_Body, sp_->Body_pos_);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, SJLinkID::LK_Body, sp_->Body_vel_);

}
