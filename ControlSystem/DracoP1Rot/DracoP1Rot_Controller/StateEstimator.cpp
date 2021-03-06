#include "StateEstimator.hpp"
#include "StateProvider.hpp"
#include <Utils/utilities.hpp>
#include <DracoP1Rot_Model/DracoModel.hpp>
#include <Filter/filters.hpp>

StateEstimator::StateEstimator(): jvel_filter_(NUM_ACT_JOINT){
  sp_ = StateProvider::GetStateProvider();
  robot_model_ = DracoModel::GetDracoModel();

  for(int i(0); i<NUM_ACT_JOINT; ++i){
    jvel_filter_[i] = new digital_lp_filter(2*M_PI*200, SERVO_RATE);
  }
}

StateEstimator::~StateEstimator(){
}

void StateEstimator::Initialization(_DEF_SENSOR_DATA_){
  // Local Frame
  sp_->Q_.setZero();
  sp_->Qdot_.setZero();
  // sp_->Q_[0] = body_pos[1]; // X
  // sp_->Q_[1] = body_pos[0]; // Z
  sp_->Q_[2] = body_ori;
  // sp_->Qdot_[0] = body_vel[1]; // X
  // sp_->Qdot_[1] = body_vel[0]; // Z
  sp_->Qdot_[2] = body_ang_vel;

  for (int i(0); i<NUM_ACT_JOINT; ++i){
    sp_->Q_[NUM_VIRTUAL + i] = jpos[i];
    jvel_filter_[i]->input(jpos[i]);
    sp_->Qdot_[NUM_VIRTUAL + i] = jvel[i];
    // sp_->Qdot_[NUM_VIRTUAL + i] = jvel_filter_[i]->output();
  }

  sejong::Vect3 foot_pos, foot_vel;
  robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_foot, foot_pos);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, SJLinkID::LK_foot, foot_vel);
  sp_->Q_[0] = -foot_pos[0];
  sp_->Q_[1] = -foot_pos[1];
  sp_->Qdot_[0] = -foot_vel[0];
  sp_->Qdot_[1] = -foot_vel[1];

  robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_body, sp_->Body_pos_);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, SJLinkID::LK_body, sp_->Body_vel_);
}

void StateEstimator::Update(_DEF_SENSOR_DATA_){
  // Local Frame
  sp_->Q_.setZero();
  sp_->Qdot_.setZero();
  // sp_->Q_[0] = body_pos[1]; // X
  // sp_->Q_[1] = body_pos[0]; // Z
  sp_->Q_[2] = body_ori;
  // sp_->Qdot_[0] = body_vel[1]; // X
  // sp_->Qdot_[1] = body_vel[0]; // Z
  sp_->Qdot_[2] = body_ang_vel;

  for (int i(0); i<NUM_ACT_JOINT; ++i){
    sp_->Q_[NUM_VIRTUAL + i] = jpos[i];

    jvel_filter_[i]->input(jpos[i]);
    sp_->Qdot_[NUM_VIRTUAL + i] = jvel[i];
    // sp_->Qdot_[NUM_VIRTUAL + i] = jvel_filter_[i]->output();
  }
  // robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
  // robot_model_->getPosition(sp_->Q_, SJLinkID::LK_body, sp_->Body_pos_);
  // robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, SJLinkID::LK_body, sp_->Body_vel_);

  // Local Frame
  sejong::Vect3 foot_pos, foot_vel;
  robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_foot, foot_pos);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, SJLinkID::LK_foot, foot_vel);
  sp_->Q_[0] = -foot_pos[0];
  sp_->Q_[1] = -foot_pos[1];
  sp_->Qdot_[0] = -foot_vel[0];
  sp_->Qdot_[1] = -foot_vel[1];

  robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_body, sp_->Body_pos_);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, SJLinkID::LK_body, sp_->Body_vel_);

}
