#include "DDP_ctrl.hpp"
#include <math.h>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>


DDP_ctrl::DDP_ctrl(): OC2Controller(),
                          count_command_(0),
                          jpos_ini_(NUM_ACT_JOINT),
                          N_horizon(5),
                          DIM_WBC_TASKS(2),                          
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

  _initiailize_u_sequence();

  J_cost = 0.0;
  J_cost_tail = std::vector<double>(N_horizon);

  mpc_time_step = SERVO_RATE; //SERVO_RATE/10; //0.001;
  sim_rate = SERVO_RATE/10;

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


void DDP_ctrl::_initiailize_u_sequence(){
  // Near zero initialization
  for(size_t i = 0; i < u_sequence.size(); i++){
    sejong::Vector u_vec(DIM_WBC_TASKS); // task acceleration vector
    for (size_t j = 0; j < DIM_WBC_TASKS; j++){
      u_vec[j] = 0.1;//0.001; // this can be random
    }
    u_sequence[i] = u_vec;
  }
}

// Use U = {u1, u2, ..., uN} to find X = {x1, x2, ..., xN}
void DDP_ctrl::_initiailize_x_sequence(const sejong::Vector x_state_start){
  x_sequence[0] = x_state_start;
  _internal_simulate_sequence(u_sequence, x_sequence);
}

void DDP_ctrl::update_internal_model(const sejong::Vector & x_state){
  // Initialize States
  sejong::Vector q_int = x_state.head(NUM_Q);  
  sejong::Vector qdot_int = x_state.tail(NUM_QDOT);    

  // Update internal model
  internal_model->UpdateModel(q_int, qdot_int);
  internal_model->getMassInertia(A_int);
  internal_model->getInverseMassInertia(Ainv_int);
  internal_model->getGravity(grav_int);
  internal_model->getCoriolis(coriolis_int);  
}

void DDP_ctrl::_internal_simulate_single_step(const sejong::Vector & x_state, 
                                              const sejong::Vector & gamma_int, 
                                              sejong::Vector & x_next_state){ // x_{t+1} = f(x, gamma(u))
  // Initialize States
  sejong::Vector q_int = x_state.head(NUM_Q);  
  sejong::Vector qdot_int = x_state.tail(NUM_QDOT);    

  update_internal_model(x_state);
  // Get Joint Accelerations
  sejong::Vector qddot_int = Ainv_int*(gamma_int - coriolis_int - grav_int);

  // Perform Integration
  // Get q_{t+1} and qdot_{t+1}
  sejong::Vector qdot_int_next = qdot_int + qddot_int*sim_rate;
  sejong::Vector q_int_next = q_int + qdot_int*sim_rate ; 

  x_next_state.head(NUM_Q) = q_int_next;
  x_next_state.tail(NUM_QDOT) = qdot_int_next;  

}

// Initializes X = {x1, x2, ..., xN}
void DDP_ctrl::_internal_simulate_sequence(const std::vector<sejong::Vector> U,  std::vector<sejong::Vector> X){

  double mpc_interval = 0.0;
  for(size_t i = 1; i < N_horizon; i++){
    sejong::Vector gamma_int(NUM_ACT_JOINT);
    getWBC_command(X[i-1], U[i-1], gamma_int);

    sejong::Vector x_state_tmp = X[i-1];
    sejong::Vector x_next_state_tmp(x_state_tmp.size());

    // Simulate constant application of torque 
    while (mpc_interval <= mpc_time_step){
      //Apply the same gamma_int within the mpc time step
      _internal_simulate_single_step(x_state_tmp, gamma_int, x_next_state_tmp);
      x_state_tmp = x_next_state_tmp;
      mpc_interval += sim_rate;      
    }

    X[i] = x_next_state_tmp; // Store X[i]    
    mpc_interval = 0.0; // reset interval

    // Debug Output    
/*    std::cout << "X[" << i << "] = " << X[i][0] << " " 
                                     << X[i][1] << " "
                                     << X[i][2] << ""
                                     << X[i][3] << ""
                                     << std::endl;
*/    // sejong::pretty_print(X[i], std::cout, "X[i]");    

  }


} 

// get the torque command given desired acceleration
void DDP_ctrl::getWBC_command(const sejong::Vector & x_state, const sejong::Vector & des_acc, sejong::Vector & gamma_int){
  // Initialize States
  sejong::Vector q_int = x_state.head(NUM_Q);  
  sejong::Vector qdot_int = x_state.tail(NUM_QDOT);    

  update_internal_model(x_state);

  // Commanded Task acceleration
  sejong::Vector xddot = des_acc;

  // Commanded Joint Task
  double kp(100.);
  double kd(10.);
  sejong::Vector jpos_cmd = kp * (jpos_ini_ - q_int ) + kd * (qdot_int);

  // For Debug
  sejong::Vect3 ee_pos;
  sejong::Vect3 ee_vel;

  internal_model->getPosition(q_int, SJLinkID::LK_EE, ee_pos);
  internal_model->getVelocity(qdot_int, sp_->Qdot_, SJLinkID::LK_EE, ee_vel);
  //sejong::pretty_print(ee_pos, std::cout, "x position");
  // 

  // Jacobian
  sejong::Matrix Jee, Jee_inv;
  sejong::Matrix Jtmp;
  internal_model->getFullJacobian(q_int, SJLinkID::LK_EE, Jtmp);
  Jee = Jtmp.block(0,0, 2, NUM_QDOT);
  _DynConsistent_Inverse(Jee, Jee_inv);

  sejong::Matrix Jee_dot;
  internal_model->getFullJacobianDot(q_int, qdot_int, SJLinkID::LK_EE, Jtmp);
  Jee_dot = Jtmp.block(0,0, 2, NUM_QDOT);

  /*
  // Debug. Find rank of Jee
  sejong::pretty_print(Jee, std::cout, "Jee:");
  Eigen::JacobiSVD<sejong::Matrix> svd(Jee, Eigen::ComputeThinU | Eigen::ComputeThinV);
  std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;

  Eigen::FullPivLU<sejong::Matrix> lu(Jee);
  lu.setThreshold(1e-12);
  std::cout << "rank(Jee) = " << lu.rank() << std::endl;
  //
  */

  // Joint task
  sejong::Matrix Nee = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - Jee_inv * Jee;
  sejong::Matrix Jqee_inv;
  _DynConsistent_Inverse(Nee, Jqee_inv);

  // Torque command
  sejong::Vector qddot = Jee_inv * (xddot - Jee_dot * qdot_int);
  qddot = qddot + Jqee_inv * (jpos_cmd - qddot);
  gamma_int = A_int * qddot + coriolis_int + grav_int;  

}



void DDP_ctrl::_mpc_ctrl(sejong::Vector & gamma){
  // Get starting state   // x = [q, qdot]
  sejong::Vector x_state_start(NUM_Q + NUM_QDOT);
  x_state_start.head(NUM_Q) = sp_->Q_; // Store current Q position to state
  x_state_start.tail(NUM_QDOT) = sp_->Qdot_; // Store current Qdot position
  sejong::pretty_print(x_state_start, std::cout, "x_state_start");  

  // Initialize X = {x1, x2, ..., xN} using U = {u1, u2, ..., uN} 
  _initiailize_x_sequence(x_state_start);

  // Test WBC command given u_sequence[0]
  getWBC_command(x_sequence[0], u_sequence[0], gamma);

  // Test simulate single step
  // sejong::Vector x_next_state(NUM_Q + NUM_QDOT);
  // _internal_simulate_single_step(x_sequence[0], gamma, x_next_state);
  // sejong::pretty_print(x_next_state, std::cout, "x_next_state_pred");

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

