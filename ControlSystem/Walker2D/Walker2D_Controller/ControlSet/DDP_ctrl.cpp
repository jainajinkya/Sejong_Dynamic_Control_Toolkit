#include "DDP_ctrl.hpp"
#include <math.h>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <Walker2D_Model/Walker2D_Model.hpp>
#include "Optimizer/lcp/MobyLCP.h"
#include <chrono>

DDP_ctrl::DDP_ctrl(): Walker2D_Controller(),
                          count_command_(0),
                          jpos_ini_(NUM_ACT_JOINT),
                          des_pos_(2),
                          act_pos_(2),
                          act_vel_(2)
{

  internal_model = Walker2D_Model::GetWalker2D_Model();

  // Prepare iLQR
  ilqr_ = new iLQR();  
  ilqr_->l_cost = std::bind( &DDP_ctrl::l_cost, this, std::placeholders::_1, std::placeholders::_2);
  ilqr_->l_cost_final = std::bind( &DDP_ctrl::l_cost_final, this, std::placeholders::_1);
  ilqr_->f = std::bind( &DDP_ctrl::f, this, std::placeholders::_1, std::placeholders::_2);

  // Prepare Quadratic Cost Matrices
  // x = [x_virt, z_virt, ry_virt, lf_j1, lf_j2, rf_j1, rf_j2]
  x_des_final = sejong::Matrix::Zero(STATE_SIZE, 1);
  x_des_final[0] = 0.65; // Walk to this value from x = -0.65
  x_des_final[1] = 0.5; //  Hip at this height from the ground   
  Q_run = sejong::Matrix::Zero(STATE_SIZE, STATE_SIZE);
  Q_final = sejong::Matrix::Zero(STATE_SIZE, STATE_SIZE);

  // Running Cost on Pose of Body. ie: We want the body to be upright
  Q_run(2,2) = 1.0; 
  // Running Cost on Foot Accelerations
  N_run = sejong::Matrix::Identity(DIM_u_SIZE, DIM_u_SIZE);  

  // Final cost on final pose of the body
  Q_final.block(0, 0, NUM_VIRTUAL, NUM_VIRTUAL) = sejong::Matrix::Identity(NUM_VIRTUAL, NUM_VIRTUAL);    
  // States should have zero velocity at the end
  Q_final.block(NUM_QDOT, NUM_QDOT, NUM_QDOT, NUM_QDOT) = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT);      

  //ilqr_->compute_ilqr();
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

// ==============================================================================
// iLQR functions ---------------------------------------------------------------
// Computes the running cost
double DDP_ctrl::l_cost(const sejong::Vector &x, const sejong::Vector &u){
  sejong::Vector cost = x.transpose()*Q_run*x + u.transpose()*N_run*u; 
  return cost[0];
}

// Computes the final cost
double DDP_ctrl::l_cost_final(const sejong::Vector &x_F){
  sejong::Vector cost = x_F.transpose()*Q_final*x_F;
  return cost[0];
}

// For the defined cost, we can have analytical forms.
void DDP_ctrl::l_x_analytical(const sejong::Vector &x, const sejong::Vector &u,  sejong::Vector & l_x){
  l_x = (Q_run + Q_run.transpose())*x;
}
void DDP_ctrl::l_x_final_analytical(const sejong::Vector &x, const sejong::Vector &u,  sejong::Vector & l_x){
  l_x = (Q_final + Q_final.transpose())*x;
}
void DDP_ctrl::l_xx_analytical(const sejong::Vector &x, const sejong::Vector &u, sejong::Matrix & l_xx){
  l_xx = (Q_run + Q_run.transpose());
}
void DDP_ctrl::l_xx_final_analytical(const sejong::Vector &x, const sejong::Vector &u, sejong::Matrix & l_xx){
  l_xx = (Q_final + Q_final.transpose());
}
void DDP_ctrl::l_u_analytical(const sejong::Vector &x, const sejong::Vector &u,  sejong::Vector & l_u){
  l_u = (N_run + N_run.transpose())*u;
}
void DDP_ctrl::l_uu_analytical(const sejong::Vector &x, const sejong::Vector &u, sejong::Matrix & l_uu){
  l_uu = (N_run + N_run.transpose());  
}
void DDP_ctrl::l_ux_analytical(const sejong::Vector &x, const sejong::Vector &u, sejong::Matrix & l_ux){
  l_ux = sejong::Matrix::Zero(DIM_u_SIZE, STATE_SIZE);  
}

void DDP_ctrl::f_u_analytical(const sejong::Vector &x, const sejong::Vector &u,  sejong::Matrix & f_u){
  // Configuration doesn't change so update model only once.
  _update_internal_model(x);

  // Get B, c WBC Matrices
  sejong::Matrix B_tmp;
  sejong::Vector c;
  _get_B_c(x, B_tmp, c);

  // Extract the B components corresponding to task accelerations of the end effector and not the posture task
  sejong::Matrix B = B_tmp.block(0, 0, NUM_QDOT, DIM_u_SIZE);

  // Construct Analytical f_u
  sejong::Matrix f_u_tmp(STATE_SIZE, DIM_u_SIZE);
  sejong::Matrix U(NUM_QDOT, NUM_QDOT); // selector matrix to extract components corresponding to actuated torques
  f_u_tmp.setZero();
  U.setZero();
  U.block(NUM_VIRTUAL, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = sejong::Matrix::Identity(NUM_ACT_JOINT, NUM_ACT_JOINT);

  // Note that our f has the form f = [q_{t+1}, qdot_{t+1}]^T 
  //                                = [qdot_{t+1}, qddot_{t}]^T *dt + I [q_t, qdot_t]^T
  //                                = [qddot_{t}*dt + qdot_t, qddot_{t}]^T *dt + I [q_t, qdot_t]^T
  // Thus, f_u = [ \partial(qddot_{t}*dt^2) w.r.t. u , \partial qddot_{t}*dt w.r.t u ]^T
  //         But, qddot = A_inv (tau - b - g + external forces) 
  //         and tau = U*(A*(B*u + c) + b + g)
  //         let's assume that the external forces are also constant during this interval. 
  //         (although this is not necessarily true as ground forces can change with xddot.)
  //         so partial qddot w.r.t u is  A_inv*U*A*B
  //         thus, f_u = [ A_inv*U*A*B*dt^2, A_inv*U*A*B*dt ]^T \in \mathbb{R}^{STATE_SIZE x DIM_u_SIZE} 

  sejong::Matrix AinvUAB = Ainv_int*U*A_int*B;  // note this is partial_qddot_wrt_u

  double h = ddp_time_step;
  f_u_tmp.block(0, 0,        NUM_QDOT, DIM_u_SIZE) = AinvUAB*h*h;
  f_u_tmp.block(NUM_QDOT, 0, NUM_QDOT, DIM_u_SIZE) = AinvUAB*h; 

  f_u = f_u_tmp; 
}


// Computes x_{t+1} = f(x_t, u_t)
sejong::Vector DDP_ctrl::f(const sejong::Vector & x, const sejong::Vector & u){   
  // Compute WBC given (x,u). This also updates the model
  sejong::Vector gamma_int;
  _get_WBC_command(x, u, gamma_int);
  return f_given_tact(x, gamma_int);
}
// ---------------------------------------------------------------------
// ==============================================================================

void DDP_ctrl::_update_internal_model(const sejong::Vector & x_state){
  // Initialize States
  sejong::Vector q_int = x_state.head(NUM_QDOT);  
  sejong::Vector qdot_int = x_state.tail(NUM_QDOT);    

  // Update internal model
  internal_model->UpdateModel(q_int, qdot_int);
  internal_model->getMassInertia(A_int);
  internal_model->getInverseMassInertia(Ainv_int);
  internal_model->getGravity(grav_int);
  internal_model->getCoriolis(coriolis_int);  
}

void DDP_ctrl::_get_B_c(const sejong::Vector & x_state, sejong::Matrix & B_out, sejong::Vector & c_out){
  sejong::Vector q_int = x_state.head(NUM_QDOT);  
  sejong::Vector qdot_int = x_state.tail(NUM_QDOT);

  // Task 1 Left and Right Foot Accelerations
  // Task 2 Posture Task 
  sejong::Matrix J_lf, J_rf;  
  sejong::Matrix J_lf_dot, J_rf_dot;    
  internal_model->getFullJacobian(q_int, SJLinkID::LK_LEFT_FOOT, J_lf);
  internal_model->getFullJacobian(q_int, SJLinkID::LK_RIGHT_FOOT, J_rf);
  internal_model->getFullJacobianDot(q_int, qdot_int, SJLinkID::LK_LEFT_FOOT, J_lf_dot);
  internal_model->getFullJacobianDot(q_int, qdot_int, SJLinkID::LK_RIGHT_FOOT, J_rf_dot);

  // Stack the Jacobians
  sejong::Matrix J_feet(4, NUM_QDOT);
  J_feet.block(0,0, 2, NUM_QDOT) = J_lf.block(0, 0, 2, NUM_QDOT);
  J_feet.block(2,0, 2, NUM_QDOT) = J_rf.block(0, 0, 2, NUM_QDOT);  
  sejong::Matrix J_feet_dot(4, NUM_QDOT);
  J_feet_dot.block(0,0, 2, NUM_QDOT) = J_lf_dot.block(0, 0, 2, NUM_QDOT);
  J_feet_dot.block(2,0, 2, NUM_QDOT) = J_rf_dot.block(0, 0, 2, NUM_QDOT);  

  // Joint Position Task----------------------------------------------------------
  sejong::Matrix J_pos(NUM_ACT_JOINT, NUM_QDOT);
  sejong::Matrix J_pos_dot(NUM_ACT_JOINT, NUM_QDOT);
  J_pos.setZero();
  J_pos.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = sejong::Matrix::Identity(NUM_ACT_JOINT, NUM_ACT_JOINT);  
  J_pos_dot.setZero();

  // Set Hierarchy
  sejong::Matrix J1 = J_feet;
  sejong::Matrix J1_dot = J_feet_dot;
  sejong::Matrix J2 = J_pos;  
  sejong::Matrix J2_dot = J_pos_dot;  

  // Prepare Projections
  sejong::Matrix J1_bar;
  _DynConsistent_Inverse(J1, J1_bar);

  sejong::Matrix N1 = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - J1_bar*J1;
  sejong::Matrix J2_1 = J2*N1;
  sejong::Matrix J2_1_bar;
  _DynConsistent_Inverse(J2_1, J2_1_bar); 

  sejong::Matrix B(NUM_QDOT, 4 + NUM_ACT_JOINT);
  sejong::Vector c(NUM_QDOT);

  B.block(0, 0, NUM_QDOT, 4) = J1_bar - J2_1_bar*J2*J1_bar;
  B.block(0, 4, NUM_QDOT, NUM_ACT_JOINT) = J2_1_bar;  
  c = (-J1_bar*J1_dot - J2_1_bar*J2_dot + J2_1_bar*J2*J1_bar*J1_dot)*qdot_int;

  B_out = B;
  c_out = c;

}

void DDP_ctrl::_get_WBC_command(const sejong::Vector & x_state, 
                                const sejong::Vector & u_input, 
                                sejong::Vector & gamma_int){

  sejong::Vector q_int = x_state.head(NUM_QDOT);  
  sejong::Vector qdot_int = x_state.tail(NUM_QDOT);   

  // Task 1 Left and Right Foot Accelerations
  // Task 2 Posture Task 

  // Set Desired Accelerations
  // Feet Tasks
  sejong::Vector xddot_feet_des = u_input;

  // Joint Position Posture Task
  sejong::Vector xddot_des_pos(NUM_ACT_JOINT);
  xddot_des_pos.setZero();
  double kp(300.); // 50
  double kd(50.);  // 15
  sejong::Vector jpos_des = jpos_ini_;
  xddot_des_pos = kp*(jpos_des - q_int.tail(NUM_ACT_JOINT)) + kd * ( -qdot_int.tail(NUM_ACT_JOINT));

  // Stack xddot desired
  sejong::Vector xddot_des(4 + NUM_ACT_JOINT);
  xddot_des.head(4) = xddot_feet_des;
  xddot_des.tail(NUM_ACT_JOINT) = xddot_des_pos;  

  // Get B, c WBC Matrices
  sejong::Matrix B;
  sejong::Vector c;
  _get_B_c(x_state, B, c);

  // Calculate qddot task
  sejong::Vector qddot_des = (B*xddot_des + c);

  // Whole Body Dynamics
  _update_internal_model(x_state);
  sejong::Vector tau = A_int * qddot_des + coriolis_int + grav_int;

  // Extract Actuated Torque
  gamma_int = tau.tail(NUM_ACT_JOINT);
}


void DDP_ctrl::ComputeTorqueCommand(sejong::Vector & gamma){
  _PreProcessing_Command();
  gamma.setZero();
  //_jpos_ctrl(gamma);
  _DDP_ctrl(gamma);
  ++count_command_;

  state_machine_time_ = sp_->curr_time_ - start_time_;
  _PostProcessing_Command(gamma);
}

void DDP_ctrl::_DDP_ctrl(sejong::Vector & gamma){
  sejong::Vector x_state(STATE_SIZE);
  x_state.head(NUM_QDOT) = sp_->Q_;
  x_state.tail(NUM_QDOT) = sp_->Qdot_;

  sejong::Vector u_vec(4); 
  u_vec.setZero();

  _get_WBC_command(x_state, u_vec, gamma);
  f(x_state, u_vec);

/*  sejong::Matrix f_u;
  f_u_analytical(x_state, u_vec, f_u);*/


/*  sejong::Vector l_x, l_xF, l_u;
  sejong::Matrix l_xx, l_xxF, l_uu, l_ux;
  l_x_analytical(x_state, u_vec, l_x);
  l_x_final_analytical(x_state, u_vec, l_xF);
  l_xx_analytical(x_state, u_vec, l_xx);
  l_xx_final_analytical(x_state, u_vec, l_xxF);
  l_u_analytical(x_state, u_vec, l_u);
  l_uu_analytical(x_state, u_vec, l_uu);
  l_ux_analytical(x_state, u_vec, l_ux);*/

  //sejong::pretty_print(x_state, std::cout, "x_state");




  //_jpos_ctrl(gamma);

}

void DDP_ctrl::_jpos_ctrl(sejong::Vector & gamma){
 
  double kp(50.);
  double kd(15.);
  sejong::Vector jpos_des = jpos_ini_;

  double amp(1.0);
  double omega(2.*M_PI * 1.);
  //jpos_des[0] += amp * sin(omega * state_machine_time_);
  //jpos_des[3] += amp * sin(-omega * state_machine_time_);
  //jpos_des[4] += amp * sin(-omega * state_machine_time_);  

  sejong::Vector qddot(NUM_QDOT); qddot.setZero();
  qddot.tail(NUM_ACT_JOINT)=
    kp*(jpos_des - sp_->Q_.tail(NUM_ACT_JOINT)) +
    kd * ( - sp_->Qdot_.tail(NUM_ACT_JOINT));

  sejong::Vector torque(NUM_QDOT);

  // torque = A_ * qddot + coriolis_ + grav_;
  // torque = grav_;

  gamma = qddot.tail(NUM_ACT_JOINT);
}



// x_{t+1} = f(x, gamma);
// Provides the next state given the current state and torque inputs
sejong::Vector DDP_ctrl::f_given_tact(const sejong::Vector & x, const sejong::Vector & gamma_int){
  // ---- START TIMER 
  //std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

  sejong::Vector q_int = x.head(NUM_QDOT);  
  sejong::Vector qdot_int = x.tail(NUM_QDOT);    

  // Solve LCP problem
  // Contact Model : Time step integrator Linear Complimentary Problem
  sejong::Vect3 lf_pos;
  sejong::Matrix J_lf(3,NUM_QDOT);
  sejong::Matrix J_lfc(1,NUM_QDOT);  

  sejong::Vect3 rf_pos;  
  sejong::Matrix J_rf(3,NUM_QDOT);
  sejong::Matrix J_rfc(1,NUM_QDOT);  

  internal_model->getPosition(q_int, SJLinkID::LK_LEFT_FOOT, lf_pos);
  internal_model->getPosition(q_int, SJLinkID::LK_RIGHT_FOOT, rf_pos);  
  internal_model->getFullJacobian(q_int, SJLinkID::LK_LEFT_FOOT, J_lf);
  internal_model->getFullJacobian(q_int, SJLinkID::LK_RIGHT_FOOT, J_rf);

  J_lfc.block(0,0, 1, NUM_QDOT) = J_lf.block(1, 0, 1, NUM_QDOT); // Z
  J_rfc.block(0,0, 1, NUM_QDOT) = J_rf.block(1, 0, 1, NUM_QDOT); // Z

  sejong::Matrix J_phi(2,NUM_QDOT);  
  sejong::Vector phi(2); 

  J_phi.block(0,0, 1, NUM_QDOT) = J_lfc; // Jacobian of distance to contact point 1
  J_phi.block(1,0, 1, NUM_QDOT) = J_rfc; // Jacobian of distance to contact point 2    
  phi[0] = lf_pos[1]; // Distance to contact point 1
  phi[1] = rf_pos[1]; // Distance to contact point 2

  sejong::Vector qddot(NUM_QDOT);

  sejong::Matrix A_inv = Ainv_int;
  // With Friction Constraints ---------------------------------------------------------------------
  const int p = 2; // Number of Contacts
  const int d = 2; // Number of Friction Basis Vectors

  double mu_static = 1.0;//0.1; // Friction Coefficient
  sejong::Vector n1(2); n1[1] = 1.0; // Normal Vector for contact 1
  sejong::Vector n2(2); n2[1] = 1.0; // Normal Vector for contact 2  
  sejong::Matrix J_c1 = J_lf.block(0, 0, 2, NUM_QDOT); // Jacobian (x,z) at contact point 1
  sejong::Matrix J_c2 = J_rf.block(0, 0, 2, NUM_QDOT); // Jacobian (x,z) at contact point 2  

  // Set Projected Normal Forces Direction
  sejong::Matrix N(NUM_QDOT, p);
  N.block(0,0, NUM_QDOT, 1) = J_c1.transpose()*n1;
  N.block(0,1, NUM_QDOT, 1) = J_c2.transpose()*n2;

  // Set Basis Vectors of Friction Cone 
  sejong::Matrix D1(d, d); // Basis for contact 1 
  D1.setZero();
  D1(0,0) = 1; 
  D1(0,1) = -1;
  sejong::Matrix D2 = D1; // Basis for contact 2
  
  // Set Projected Tangential Forces Direction
  sejong::Matrix B(NUM_QDOT, p*d); 
  B.block(0,0, NUM_QDOT, d) = J_c1.transpose()*D1;
  B.block(0,2, NUM_QDOT, d) = J_c2.transpose()*D2;  

  // Unit e vecs and E binary matrix
  sejong::Vector e(d); // same size as number of friction basis direction 
  e.setOnes();
  sejong::Matrix E(p*d, d); 
  E.setZero();
  E.block(0, 0, e.size(),1) = e; 
  E.block(2, 1, e.size(),1) = e;

  // mu Matrix
  //sejong::Matrix Mu(p,p);
  //Mu.setOnes();
  sejong::Matrix Mu = sejong::Matrix::Identity(p, p);
  Mu *= mu_static;

  // Prepare the LCP problem------------------------------------------------------------
  double h = ddp_time_step; // timestep
  // Prepare Alpha Mu
  // 1st Row
  sejong::Matrix alpha_mu(p + p*d + p, p + p*d + p);
  alpha_mu.setZero();
  alpha_mu.block(0, 0, p, p) =  h*J_phi*A_inv*N;
  alpha_mu.block(0, p, p, p*d) =  h*J_phi*A_inv*B;

  // 2nd Row
  alpha_mu.block(p, 0,     p*d, p) = h*B.transpose()*A_inv*N;
  alpha_mu.block(p, p,     p*d, p*d) = h*B.transpose()*A_inv*B;
  alpha_mu.block(p, p+p*d, p*d, p) = E;

  // 3rd Row (Scaling for stabilization according to Tan, Jie. et al "Contact Handling for Articulated Rigid Bodies using LCP")
  alpha_mu.block(p+p*d, 0, p, p) = Mu * h; // Scaling by h for stabilizing contact constraints
  alpha_mu.block(p+p*d, p, p, p*d) = -E.transpose() * h;  // Scaling by h for stabilizing contact constraints

  // Prepare Beta
  sejong::Vector gamma_simulate(NUM_QDOT);
  gamma_simulate.tail(NUM_ACT_JOINT) = gamma_int;
  sejong::Vector tau_star = A_int*qdot_int + h*(gamma_simulate - coriolis_int - grav_int);
  sejong::Vector beta_mu(p + p*d + p);
  beta_mu.setZero();
  // 1st Row
  beta_mu.block(0,0, p, 1) = phi/h + J_phi*A_inv*tau_star;
  // 2nd Row
  beta_mu.block(p,0, p*d, 1) = B.transpose()*A_inv*tau_star;  

  sejong::Vector fn_fd_lambda(p + p*d + p);
  fn_fd_lambda.setZero();

  // Solve LCP Problem
  MobyLCPSolver l_mu;  
//  bool result_mu = l_mu.lcp_lemke_regularized(alpha_mu, beta_mu, &fn_fd_lambda);
  bool result_mu = l_mu.lcp_fast(alpha_mu, beta_mu, &fn_fd_lambda);  
  

  // Extract Normal and Tangential Forces
  sejong::Vector fn = fn_fd_lambda.block(0, 0, p, 1);
  sejong::Vector fd = fn_fd_lambda.block(p, 0, p*d, 1);

  sejong::Vector qddot_now, qdot_next, q_next;
  qddot_now = A_inv*(gamma_simulate - coriolis_int - grav_int + N*fn + B*fd);
  qdot_next = qddot_now * h + qdot_int;
  q_next = qdot_next * h + q_int;

  //sejong::pretty_print(q_next, std::cout, "q_next predicted"); 

  sejong::Vector x_next(NUM_QDOT + NUM_QDOT);
  x_next.head(NUM_QDOT) = q_next;
  x_next.tail(NUM_QDOT) = qdot_next;  

  // ----- END TIMER
  //std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
  //std::cout << "  f(x,u) took " << time_span1.count()*1000.0 << "ms"<<std::endl;  

  return x_next;

}

