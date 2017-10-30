#include "DDP_ctrl.hpp"
#include <math.h>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <Walker2D_Model/Walker2D_Model.hpp>

DDP_ctrl::DDP_ctrl(): Walker2D_Controller(),
                          count_command_(0),
                          jpos_ini_(NUM_ACT_JOINT),
                          des_pos_(2),
                          act_pos_(2),
                          act_vel_(2)
{
  wbc_ilqr_ = new WBC_iLQR();  
  wbc_ilqr_->l_cost = std::bind( &DDP_ctrl::l_cost, this, std::placeholders::_1, std::placeholders::_2);
  wbc_ilqr_->l_cost_final = std::bind( &DDP_ctrl::l_cost_final, this, std::placeholders::_1);
  wbc_ilqr_->get_WBC_command = std::bind( &DDP_ctrl::get_WBC_command, this, std::placeholders::_1, 
                                                                        std::placeholders::_2, 
                                                                        std::placeholders::_3);

  wbc_ilqr_->compute_ilqr();
  printf("[DDP Controller] Start\n");
}

DDP_ctrl::~DDP_ctrl(){
}

void DDP_ctrl::Initialization(){
  for (int i(0); i < NUM_ACT_JOINT ; ++i){
    jpos_ini_[i] = sp_->Q_[i + NUM_VIRTUAL];
  }
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_BODY, ee_ini_);
  start_time_ = sp_->curr_time_;
  phase_ = 10;
}

double DDP_ctrl::l_cost(const sejong::Vector &x, const sejong::Vector &u){
  std::cout << " Output:" << 200.0 << std::endl;  
  return 200;
}

double DDP_ctrl::l_cost_final(const sejong::Vector &x){
  std::cout << " Output:" << 100.0 << std::endl;  
  return 100;
}

void DDP_ctrl::get_WBC_command(const sejong::Vector & x_state, 
                               const sejong::Vector & des_acc, 
                               sejong::Vector & gamma_int){
  gamma_int.setZero();
}


void DDP_ctrl::ComputeTorqueCommand(sejong::Vector & gamma){
  _PreProcessing_Command();
  gamma.setZero();
  _jpos_ctrl(gamma);

  ++count_command_;

  state_machine_time_ = sp_->curr_time_ - start_time_;
  _PostProcessing_Command(gamma);
}

void DDP_ctrl::_DDP_ctrl(sejong::Vector & gamma){
  gamma.setZero();
}

void DDP_ctrl::_jpos_ctrl(sejong::Vector & gamma){
 
  double kp(50.);
  double kd(15.);
  sejong::Vector jpos_des = jpos_ini_;

  double amp(1.0);
  double omega(2.*M_PI * 1.);
  //jpos_des[0] += amp * sin(omega * state_machine_time_);
  jpos_des[3] += amp * sin(-omega * state_machine_time_);
  jpos_des[4] += amp * sin(-omega * state_machine_time_);  

  sejong::Vector qddot(NUM_QDOT); qddot.setZero();
  qddot.tail(NUM_ACT_JOINT)=
    kp*(jpos_des - sp_->Q_.tail(NUM_ACT_JOINT)) +
    kd * ( - sp_->Qdot_.tail(NUM_ACT_JOINT));

  sejong::Vector torque(NUM_QDOT);

  // torque = A_ * qddot + coriolis_ + grav_;
  // torque = grav_;

  gamma = qddot.tail(NUM_ACT_JOINT);
}