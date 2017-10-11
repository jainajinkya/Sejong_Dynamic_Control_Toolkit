#include "Basic_Ctrl.hpp"

Basic_Ctrl::Basic_Ctrl():DracoController(),
                         jpos_ini_(NUM_ACT_JOINT)
{
  printf("[Draco P1 Rot] Basic Controller \n");
}
Basic_Ctrl::~Basic_Ctrl(){}

void Basic_Ctrl::Initialization(){
  for (int i(0); i < NUM_ACT_JOINT ; ++i){
    jpos_ini_[i] = sp_->Q_[i + NUM_VIRTUAL];
  }
  start_time_ = sp_->curr_time_;
  phase_ = 10;
}

void Basic_Ctrl::ComputeTorqueCommand(sejong::Vector & gamma){
  _PreProcessing_Command();

  _jpos_ctrl_hanging(gamma);

  _PostProcessing_Command(gamma);
}

void Basic_Ctrl::_jpos_ctrl_hanging(sejong::Vector & gamma){
  double kp(10.);
  double kd(2.);
  sejong::Vector qddot(NUM_QDOT); qddot.setZero();

  jpos_des = jpos_ini_;

  for(int i(0);i<NUM_ACT_JOINT; ++i){
    qddot[i + NUM_VIRTUAL] = kp * (jpos_des[i] - sp_->Q_[i + NUM_VIRTUAL]) + kd * (-sp_->Qdot_[i + NUM_VIRTUAL]);
  }

  sejong::Vector torque = A_ * qddot + grav_;

  gamma = torque.tail(NUM_ACT_JOINT);
}
