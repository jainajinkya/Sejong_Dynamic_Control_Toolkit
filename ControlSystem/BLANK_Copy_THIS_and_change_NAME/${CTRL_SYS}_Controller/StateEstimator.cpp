#include "StateEstimator.h"
#include "StateProvider.h"
#include <Utils/utilities.h>
#include <Valkyrie_Model/Valkyrie_Model.h>
#include <Valkyrie_Model/Valkyrie_Left_Leg.h>
#include <Valkyrie_Model/Valkyrie_Right_Leg.h>

#include <Filter/filters.h>

#define LOCAL_FRAME_USE 1

StateEstimator::StateEstimator():
  jvel_filter_(NUM_ACT_JOINT),
  leg_q_(NUM_VIRTUAL + 7),
  leg_qdot_(NUM_VIRTUAL + 6)
{
    sp_ = StateProvider::GetStateProvider();
    robot_model_ = ValkyrieModel::GetValkyrieModel();

    for(int i(0); i<NUM_ACT_JOINT; ++i){
      jvel_filter_[i] = new digital_lp_filter(2*M_PI*550, SERVO_RATE);
      // jvel_filter_[i] = new deriv_lp_filter(2*M_PI*150, SERVO_RATE);
      jvel_filter_[i]->clear();
    }
    left_leg_ = Valkyrie_Left_Leg::GetValkyrieLeftLeg();
    right_leg_ = Valkyrie_Right_Leg::GetValkyrieRightLeg();
}

StateEstimator::~StateEstimator(){
}

void StateEstimator::Initialization(_DEF_SENSOR_DATA_){
    for (int i(0); i<3; ++i){

#if LOCAL_FRAME_USE
      sp_->Q_[i] = 0.;
      sp_->Qdot_[i] = 0.;
#else
      sp_->Qdot_[i] = body_vel[i];
      sp_->Q_[i] = body_pos[i];
#endif
      sp_->Qdot_[i+3] = ang_vel[i];
    }

    sp_->Q_[3] = body_ori.x();
    sp_->Q_[4] = body_ori.y();
    sp_->Q_[5] = body_ori.z();
    sp_->Q_[NUM_QDOT] = body_ori.w();
    printf("[State Estimator]initialization\n");
    for (int i(0); i<NUM_ACT_JOINT; ++i){
        sp_->Q_[NUM_VIRTUAL + i] = jpos[i];
        jvel_filter_[i]->input(jvel[i]);
        sp_->Qdot_[NUM_VIRTUAL + i] = jvel[i];
        // sp_->Qdot_[NUM_VIRTUAL + i] = jvel_filter_[i]->output();
    }
    // sejong::pretty_print(sp_->Qdot_, std::cout, "Qdot");
    // sejong::pretty_print(jvel, "jvel");

#if LOCAL_FRAME_USE
    if (sp_->stance_foot_ == LK_rightCOP_Frame){
      _AllocateRightState();
      // sejong::pretty_print(leg_q_, std::cout, "leg q");
      // sejong::pretty_print(leg_qdot_, std::cout, "leg qdot");
      right_leg_->UpdateKinematics(leg_q_, leg_qdot_);
      right_leg_->getRightFootPosition(foot_pos_);
      right_leg_->getRightFootVelocity(foot_vel_);
      sp_->stance_foot_vel_  = foot_vel_;
    } else{
      _AllocateLeftState();
      left_leg_->UpdateKinematics(leg_q_, leg_qdot_);
      left_leg_->getLeftFootPosition(foot_pos_);
      left_leg_->getLeftFootVelocity(foot_vel_);
      sp_->stance_foot_vel_  = foot_vel_;
    }
    // foot_pos_[2] -= 0.04;

    for(int i(0); i<3; ++i){
      sp_->Q_[i] = -foot_pos_[i];
      // sp_->Qdot_[i] = -foot_vel_[i];
    }
#endif
    robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
}

void StateEstimator::Update(_DEF_SENSOR_DATA_){
    for (int i(0); i<3; ++i){
#if LOCAL_FRAME_USE
      sp_->Q_[i] = 0.;
      // sp_->Qdot_[i] = 0.;
      sp_->Qdot_[i] = body_vel[i];
#else
      sp_->Q_[i] = body_pos[i];
      sp_->Qdot_[i] = body_vel[i];
#endif
      sp_->Qdot_[i+3] = ang_vel[i];
    }
    sp_->Q_[3] = body_ori.x();
    sp_->Q_[4] = body_ori.y();
    sp_->Q_[5] = body_ori.z();
    sp_->Q_[NUM_QDOT] = body_ori.w();

    for (int i(0); i<NUM_ACT_JOINT; ++i){
      sp_->Q_[NUM_VIRTUAL + i] = jpos[i];
      jvel_filter_[i]->input(jvel[i]);
      sp_->Qdot_[NUM_VIRTUAL + i] = jvel[i];
      // sp_->Qdot_[NUM_VIRTUAL + i] = jvel_filter_[i]->output();
    }
    // sejong::pretty_print(sp_->Qdot_, std::cout, "Qdot");
    // sejong::pretty_print(jvel, "jvel");
#if LOCAL_FRAME_USE
    if (sp_->stance_foot_ == LK_rightCOP_Frame){
      _AllocateRightState();
      right_leg_->UpdateKinematics(leg_q_, leg_qdot_);
      right_leg_->getRightFootPosition(foot_pos_);
      right_leg_->getRightFootVelocity(foot_vel_);
    } else{
      _AllocateLeftState();
      left_leg_->UpdateKinematics(leg_q_, leg_qdot_);
      left_leg_->getLeftFootPosition(foot_pos_);
      left_leg_->getLeftFootVelocity(foot_vel_);
    }
    // foot_pos_[2] -= 0.04;

    for(int i(0); i<3; ++i){
      sp_->Q_[i] = -foot_pos_[i];
      // sp_->Qdot_[i] = -foot_vel_[i];
    }
#endif
    robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
    // sejong::pretty_print(sp_->Q_, std::cout, "config");
}


void StateEstimator::_AllocateLeftState(){
  for(int i(0); i<NUM_VIRTUAL; ++i){
    leg_q_[i] = sp_->Q_[i];
    leg_qdot_[i] = sp_->Qdot_[i];
  }
  for(int j(0); j<6; ++j){
    leg_q_[NUM_VIRTUAL + j] = sp_->Q_[leftHipYaw + j];
    leg_qdot_[NUM_VIRTUAL + j] = sp_->Qdot_[leftHipYaw + j];
  }
  // Quaternion (w)
  leg_q_[NUM_VIRTUAL + 6] = sp_->Q_[NUM_QDOT];
}

void StateEstimator::_AllocateRightState(){
  for(int i(0); i<NUM_VIRTUAL; ++i){
    leg_q_[i] = sp_->Q_[i];
    leg_qdot_[i] = sp_->Qdot_[i];
  }
  for(int j(0); j<6; ++j){
    leg_q_[NUM_VIRTUAL + j] = sp_->Q_[rightHipYaw + j];
    leg_qdot_[NUM_VIRTUAL + j] = sp_->Qdot_[rightHipYaw + j];
  }
  // Quaternion (w)
  leg_q_[NUM_VIRTUAL + 6] = sp_->Q_[NUM_QDOT];
}
