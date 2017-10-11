#include "DDP_ctrl.hpp"
#include <math.h>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>


DDP_ctrl::DDP_ctrl(): OC2Controller(),
                          count_command_(0),
                          jpos_ini_(NUM_ACT_JOINT),
                          N_horizon(5),                          
                          des_pos_(2),
                          act_pos_(2),
                          act_vel_(2)
{
  internal_model = OC2Model::GetOC2Model(); // Get Model of the robot for internal simulation
  x_sequence = std::vector<sejong::Vector>(N_horizon);
  u_sequence = std::vector<sejong::Vector>(N_horizon);  

  l_x  = std::vector<sejong::Vector>(N_horizon);  
  l_xx = std::vector<sejong::Matrix>(N_horizon);  
  l_u  = std::vector<sejong::Vector>(N_horizon);   
  l_uu = std::vector<sejong::Matrix>(N_horizon);   
  l_ux = std::vector<sejong::Matrix>(N_horizon);

  f_x  = std::vector<sejong::Matrix>(N_horizon);
  f_u  = std::vector<sejong::Matrix>(N_horizon);

  V_x  = std::vector<sejong::Vector>(N_horizon);
  V_xx = std::vector<sejong::Matrix>(N_horizon);

  J_cost = 0.0;
  J_cost_tail = std::vector<double>(N_horizon);

  printf("[DDP Controller] Start\n");
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


/*void DDP_ctrl::_internal_simulate(const sejong::Vector & x_state, 
                                  const sejong::Vector & gamma, 
                                  sejong::Vector & x_next_state){*/
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

//}


void DDP_ctrl::ComputeTorqueCommand(sejong::Vector & gamma){
  _PreProcessing_Command();
  gamma.setZero();

  //_jpos_ctrl(gamma);
  //_ee_ctrl(gamma);
  //_zero_ctrl(gamma);  
  _mpc_ctrl(gamma);    
  ++count_command_;

  state_machine_time_ = sp_->curr_time_ - start_time_;
  _PostProcessing_Command(gamma);
}

void DDP_ctrl::_mpc_ctrl(sejong::Vector & gamma){
  sejong::pretty_print(sp_->Q_, std::cout, "Q");
  sejong::pretty_print(sp_->Qdot_, std::cout, "Qdot");  

  // x = [q, qdot]
  sejong::Vector x_state(NUM_Q + NUM_QDOT);
  x_state.head(NUM_Q) = sp_->Q_; // Store current Q position to state
  x_state.tail(NUM_QDOT) = sp_->Qdot_; // Store current Qdot position
  sejong::pretty_print(x_state, std::cout, "x_state");  

  // Initialize x_i 
  x_sequence[0] = x_state;


  gamma.setZero();  
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

