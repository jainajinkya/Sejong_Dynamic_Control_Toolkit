#include "DDP_ctrl.hpp"
#include <math.h>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <Openchain2DoF_Model/OC2_Model.hpp>

DDP_ctrl::DDP_ctrl(): OC2Controller(),
                          count_command_(0),
                          q_temp(NUM_QDOT),
                          qdot_temp(NUM_QDOT),
                          jpos_ini_(NUM_ACT_JOINT),                          
                          des_pos_(2),
                          act_pos_(2),
                          act_vel_(2)
{
  internal_model = OC2_Sim_Model::GetOC2_Sim_Model(); // Get Model of the robot for internal simulation

  printf("[DDP Controller] Start\n");
  printf("Size of (q, qdot): (%zu, %zu)", q_temp.size(), qdot_temp.size());
}

DDP_ctrl::~DDP_ctrl(){
}

void DDP_ctrl::Initialization(){
  for (int i(0); i < NUM_ACT_JOINT ; ++i){
    jpos_ini_[i] = sp_->Q_[i + NUM_VIRTUAL];
  }
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_EE, ee_ini_);
  start_time_ = sp_->curr_time_;
  phase_ = 10;
}


void DDP_ctrl::_internal_simulate(const sejong::Vector & x_state, 
                                  const sejong::Vector & gamma, 
                                  sejong::Vector & x_next_state){
  // x_state = [q, qdot]
  // q_tmp = x_state[0] 
  // qdot = x_state[1]

  /*
  getGamma(q, qdot, u) // Whole body control based on u and current state


  internal_model->getMassInertia(A_);
  internal_model->getInverseMassInertia(Ainv_);
  internal_model->getGravity(grav_);
  internal_model->getCoriolis(coriolis_); 

  */

}


void DDP_ctrl::ComputeTorqueCommand(sejong::Vector & gamma){
  _PreProcessing_Command();
  gamma.setZero();

  //_jpos_ctrl(gamma);
  //_ee_ctrl(gamma);
  _zero_ctrl(gamma);  
  ++count_command_;

  state_machine_time_ = sp_->curr_time_ - start_time_;
  _PostProcessing_Command(gamma);
}

void DDP_ctrl::_zero_ctrl(sejong::Vector & gamma){
  gamma.setZero();
}

void DDP_ctrl::_jpos_ctrl(sejong::Vector & gamma){
 
  double kp(100.);
  double kd(10.);
  sejong::Vector qddot =
    kp*(jpos_ini_ - sp_->Q_.tail(NUM_ACT_JOINT)) +
    kd * ( - sp_->Qdot_.tail(NUM_ACT_JOINT));

  gamma = A_ * qddot + coriolis_ + grav_;
}

void DDP_ctrl::_ee_ctrl(sejong::Vector & gamma){

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

  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_EE, ee_pos);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, SJLinkID::LK_EE, ee_vel);

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
  robot_model_->getFullJacobian(sp_->Q_, SJLinkID::LK_EE, Jtmp);
  Jee = Jtmp.block(0,0, 2, NUM_QDOT);
  _DynConsistent_Inverse(Jee, Jee_inv);

  sejong::Matrix Jee_dot;
  robot_model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, SJLinkID::LK_EE, Jtmp);
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

