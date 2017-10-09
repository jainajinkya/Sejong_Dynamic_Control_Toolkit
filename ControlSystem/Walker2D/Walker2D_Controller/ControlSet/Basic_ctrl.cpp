#include "Basic_ctrl.hpp"
#include <math.h>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <Walker2D_Model/Walker2D_Model.hpp>

Basic_ctrl::Basic_ctrl(): Walker2D_Controller(),
                          count_command_(0),
                          jpos_ini_(NUM_ACT_JOINT),
                          des_pos_(2),
                          act_pos_(2),
                          act_vel_(2)
{
  printf("[Basic Controller] Start\n");
}

Basic_ctrl::~Basic_ctrl(){
}

void Basic_ctrl::Initialization(){
  for (int i(0); i < NUM_ACT_JOINT ; ++i){
    jpos_ini_[i] = sp_->Q_[i + NUM_VIRTUAL];
  }
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_BODY, ee_ini_);
  start_time_ = sp_->curr_time_;
  phase_ = 10;
}


void Basic_ctrl::ComputeTorqueCommand(sejong::Vector & gamma){
  _PreProcessing_Command();
  gamma.setZero();

  _jpos_ctrl(gamma);
  // _ee_ctrl(gamma);
  ++count_command_;

  state_machine_time_ = sp_->curr_time_ - start_time_;
  _PostProcessing_Command(gamma);
}

void Basic_ctrl::_jpos_ctrl(sejong::Vector & gamma){
 
  double kp(50.);
  double kd(15.);
  sejong::Vector jpos_des = jpos_ini_;

  double amp(0.5);
  double omega(2.*M_PI * 1.);
  jpos_des[0] += amp * sin(omega * state_machine_time_);
  jpos_des[2] += amp * sin(-omega * state_machine_time_);

  sejong::Vector qddot(NUM_QDOT); qddot.setZero();
  qddot.tail(NUM_ACT_JOINT)=
    kp*(jpos_des - sp_->Q_.tail(NUM_ACT_JOINT)) +
    kd * ( - sp_->Qdot_.tail(NUM_ACT_JOINT));

  sejong::Vector torque(NUM_QDOT);

  // torque = A_ * qddot + coriolis_ + grav_;
  // torque = grav_;

  gamma = qddot.tail(NUM_ACT_JOINT);
}

void Basic_ctrl::_ee_ctrl(sejong::Vector & gamma){

  sejong::Vect3 ee_des, ee_vel_des, ee_acc;

  ee_des = ee_ini_;
  ee_vel_des.setZero();
  ee_acc.setZero();
  // Vertical Up & Down
  double amp(0.1);
  double omega(2.*M_PI * 1.);
  ee_des[0] += amp * sin(omega * state_machine_time_);
  ee_vel_des[0] = amp * omega * cos(omega * state_machine_time_);
  ee_acc[0] = -amp * omega * omega* sin(omega * state_machine_time_);

  sejong::Vect3 ee_pos;
  sejong::Vect3 ee_vel;

  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_BODY, ee_pos);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, SJLinkID::LK_BODY, ee_vel);

  // Commanded acceleration
  double kp(10.);
  double kd(1.0);
  sejong::Vector xddot(2);
  for(int i(0); i<2; ++i){ // (x, z)
    xddot[i] = ee_acc[i] + kp*(ee_des[i] - ee_pos[i]) + kd * (ee_vel_des[i] - ee_vel[i]);
  }

  sejong::Vector jpos_cmd = kp * (jpos_ini_ - sp_->Q_ ) + kd * (-sp_->Qdot_);

  // Jacobian
  sejong::Matrix Jee, Jee_inv;
  sejong::Matrix Jtmp;
  robot_model_->getFullJacobian(sp_->Q_, SJLinkID::LK_BODY, Jtmp);
  Jee = Jtmp.block(0,0, 2, NUM_QDOT);
  _DynConsistent_Inverse(Jee, Jee_inv);

  sejong::Matrix Jee_dot;
  robot_model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, SJLinkID::LK_BODY, Jtmp);
  Jee_dot = Jtmp.block(0,0, 2, NUM_QDOT);

  // Joint task
  sejong::Matrix Nee = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - Jee_inv * Jee;
  sejong::Matrix Jqee_inv;
  _DynConsistent_Inverse(Nee, Jqee_inv);

  // Torque command
  sejong::Vector qddot = Jee_inv * (xddot - Jee_dot * sp_->Qdot_);
  qddot = qddot + Jqee_inv * (jpos_cmd - qddot);
  gamma = A_ * qddot + coriolis_ + grav_;
}

