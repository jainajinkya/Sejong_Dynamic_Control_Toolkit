#include "DDP_ctrl.hpp"
#include <math.h>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <Walker2D_Model/Walker2D_Model.hpp>
#include "Optimizer/lcp/MobyLCP.h"
#include <chrono>

#define TIME_DDP

DDP_ctrl::DDP_ctrl(): Walker2D_Controller(),
                          count_command_(0),
                          jpos_ini_(NUM_ACT_JOINT),
                          des_pos_(2),
                          act_pos_(2),
                          act_vel_(2)
{

  internal_model = Walker2D_Model::GetWalker2D_Model();

  // Prepare Selection Matrix
  Sv.resize(NUM_VIRTUAL, NUM_QDOT);
  Sa.resize(NUM_ACT_JOINT, NUM_QDOT);
  Sv.setZero();
  Sa.setZero();

  Sv.block(0,0, NUM_VIRTUAL, NUM_VIRTUAL) = sejong::Matrix::Identity(NUM_VIRTUAL, NUM_VIRTUAL);
  Sa.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = sejong::Matrix::Identity(NUM_ACT_JOINT, NUM_ACT_JOINT);

  sejong::pretty_print(Sv, std::cout, "Sv");
  sejong::pretty_print(Sa, std::cout, "Sa");

  // Virtual Joints Relaxation
  virt_relax.setOnes(NUM_VIRTUAL);
  virt_eps_relax = 1e-6;
  virt_relax *= virt_eps_relax;

  sejong::pretty_print(virt_relax, std::cout, "virt_relax");


  // Prepare Box Constraints
  tau_min.resize(NUM_ACT_JOINT); // Minimum Torque
  tau_max.resize(NUM_ACT_JOINT); // Maximum Torque
  Fr_max.resize(4); //  Maximum Reaction Force
  
  double tau_bound = 50.0;
  for(size_t i = 0; i < NUM_ACT_JOINT; i++){
    tau_max[i] = tau_bound;
    tau_min[i] = -tau_bound;
  }
  double Fr_max_bound = 100.0;
  for(size_t i = 0; i < Fr_max.size(); i++){
    Fr_max[i] = Fr_max_bound;
  }  

  _prep_QP_U_f();


  // Prepare iLQR
  ilqr_ = new iLQR();  
  ilqr_->l_cost = std::bind( &DDP_ctrl::l_cost, this, std::placeholders::_1, std::placeholders::_2);
  ilqr_->l_cost_final = std::bind( &DDP_ctrl::l_cost_final, this, std::placeholders::_1);
  ilqr_->f = std::bind( &DDP_ctrl::f, this, std::placeholders::_1, std::placeholders::_2);


  // Assign Analytical Gradients
  ilqr_->custom_l_xF = false;
  ilqr_->l_x_final_analytical = std::bind( &DDP_ctrl::l_x_final_analytical, this, std::placeholders::_1, 
                                                                                  std::placeholders::_2);
  ilqr_->custom_l_xxF = false;
  ilqr_->l_xx_final_analytical = std::bind( &DDP_ctrl::l_xx_final_analytical, this, std::placeholders::_1, 
                                                                                    std::placeholders::_2);

  ilqr_->custom_l_x = false;
  ilqr_->l_x_analytical = std::bind( &DDP_ctrl::l_x_analytical, this, std::placeholders::_1, 
                                                                      std::placeholders::_2,
                                                                      std::placeholders::_3);

  ilqr_->custom_l_xx = false;
  ilqr_->l_xx_analytical = std::bind( &DDP_ctrl::l_xx_analytical, this, std::placeholders::_1, 
                                                                      std::placeholders::_2,
                                                                      std::placeholders::_3);
  ilqr_->custom_l_u = true;
  ilqr_->l_u_analytical = std::bind( &DDP_ctrl::l_u_analytical, this, std::placeholders::_1, 
                                                                      std::placeholders::_2,
                                                                      std::placeholders::_3);
  ilqr_->custom_l_uu = true;
  ilqr_->l_uu_analytical = std::bind( &DDP_ctrl::l_uu_analytical, this, std::placeholders::_1, 
                                                                       std::placeholders::_2,
                                                                       std::placeholders::_3);
  ilqr_->custom_l_ux = true;
  ilqr_->l_ux_analytical = std::bind( &DDP_ctrl::l_ux_analytical, this, std::placeholders::_1, 
                                                                        std::placeholders::_2,
                                                                        std::placeholders::_3);  
  ilqr_->custom_f_u = true;
  ilqr_->f_u_analytical = std::bind( &DDP_ctrl::f_u_analytical, this, std::placeholders::_1, 
                                                                      std::placeholders::_2,
                                                                      std::placeholders::_3);    

  // Prepare Quadratic Cost Matrices
  // x = [x_virt, z_virt, ry_virt, lf_j1, lf_j2, rf_j1, rf_j2]
  x_des_final = sejong::Matrix::Zero(STATE_SIZE, 1);
  x_des_final[0] = 0.0;//-0.65; // Walk to this value from x = -0.65
  x_des_final[1] = 0.5; //  Hip at this height from the ground   
  x_des_final[2] = 0.0; //  Body Orientation from the ground     
  Q_run = sejong::Matrix::Zero(STATE_SIZE, STATE_SIZE);
  Q_final = sejong::Matrix::Zero(STATE_SIZE, STATE_SIZE);

  ee_des = sejong::Matrix::Zero(2, 1);

  // Running Cost on Pose of Body. ie: We want the body to be upright
  Q_run(2,2) = 1.0;
  Q_run.block(NUM_QDOT+NUM_VIRTUAL, NUM_QDOT+NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = sejong::Matrix::Identity(NUM_ACT_JOINT, NUM_ACT_JOINT);
  Q_run = Q_run * 0.00001;
  // Running Cost on Foot Accelerations
  N_run = sejong::Matrix::Identity(DIM_u_SIZE, DIM_u_SIZE)*0.00001;  //sejong::Matrix::Identity(DIM_u_SIZE, DIM_u_SIZE);  
  P_run = sejong::Matrix::Identity(2,2)*0.00001;   // EE Running State Cost  

  // Final cost on final pose of the body
  Q_final.block(0, 0, NUM_VIRTUAL, NUM_VIRTUAL) = sejong::Matrix::Identity(NUM_VIRTUAL, NUM_VIRTUAL);    
  P_final = sejong::Matrix::Identity(2,2);   // EE Running State Cost  
  // States should have zero velocity at the end
  Q_final.block(NUM_QDOT, NUM_QDOT, NUM_QDOT, NUM_QDOT) = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT);      

  Q_final *= 100;
  P_final *= 100;

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
  sejong::Vect3 rf_pos;
  internal_model->UpdateKinematics(x.head(NUM_QDOT), x.tail(NUM_QDOT));
  internal_model->getPosition(x.head(NUM_QDOT), SJLinkID::LK_RIGHT_FOOT, rf_pos);  

  sejong::Vector rf_ee(2);
  rf_ee[0] = rf_pos[0]; // X;
  rf_ee[1] = rf_pos[1]; // Z;  

  sejong::Vector cost = x.transpose()*Q_run*x + u.transpose()*N_run*u + (ee_des - rf_ee).transpose()*P_run*(ee_des - rf_ee); 
  return cost[0];
}

// Computes the final cost
double DDP_ctrl::l_cost_final(const sejong::Vector &x_F){
  sejong::Vect3 rf_pos;  
  internal_model->UpdateKinematics(x_F.head(NUM_QDOT), x_F.tail(NUM_QDOT));
  internal_model->getPosition(x_F.head(NUM_QDOT), SJLinkID::LK_RIGHT_FOOT, rf_pos);  

  sejong::Vector rf_ee(2);
  rf_ee[0] = rf_pos[0]; // X;
  rf_ee[1] = rf_pos[1]; // Z;  

  sejong::Vector cost = (x_des_final - x_F).transpose()*Q_final*(x_des_final - x_F) + (ee_des - rf_ee).transpose()*P_final*(ee_des - rf_ee);
  return cost[0];
}

// For the defined cost, we can have analytical forms.
void DDP_ctrl::l_x_analytical(const sejong::Vector &x, const sejong::Vector &u,  sejong::Vector & l_x){
  l_x = (Q_run + Q_run.transpose())*x;
}
void DDP_ctrl::l_x_final_analytical(const sejong::Vector &x,  sejong::Vector & l_xF){
  l_xF = (Q_final + Q_final.transpose())*x;
}
void DDP_ctrl::l_xx_analytical(const sejong::Vector &x, const sejong::Vector &u, sejong::Matrix & l_xx){
  l_xx = (Q_run + Q_run.transpose());
}
void DDP_ctrl::l_xx_final_analytical(const sejong::Vector &x, sejong::Matrix & l_xxF){
  l_xxF = (Q_final + Q_final.transpose());
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
  
/*  sejong::pretty_print(x, std::cout, "x");
  sejong::pretty_print(u, std::cout, "u");
  sejong::pretty_print(gamma_int, std::cout, "gamma_int");*/

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
  _update_internal_model(x_state);
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
  sejong::Matrix J_feet(DIM_u_SIZE, NUM_QDOT);
  J_feet.block(0,0, 2, NUM_QDOT) = J_lf.block(0, 0, 2, NUM_QDOT);
  J_feet.block(2,0, 2, NUM_QDOT) = J_rf.block(0, 0, 2, NUM_QDOT);  
  sejong::Matrix J_feet_dot(DIM_u_SIZE, NUM_QDOT);
  J_feet_dot.block(0,0, 2, NUM_QDOT) = J_lf_dot.block(0, 0, 2, NUM_QDOT);
  J_feet_dot.block(2,0, 2, NUM_QDOT) = J_rf_dot.block(0, 0, 2, NUM_QDOT);  

  sejong::Matrix B(NUM_QDOT, DIM_u_SIZE);
  sejong::Vector c(NUM_QDOT);

  sejong::Matrix J1 = J_feet;
  sejong::Matrix J1_dot = J_feet_dot;
  sejong::Matrix J1_bar;
  _DynConsistent_Inverse(J1, J1_bar);

  B.block(0, 0, NUM_QDOT, DIM_u_SIZE) = J1_bar;
  c = (-J1_bar*J1_dot)*qdot_int;


  /*
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
  */

  if (isnan(B(0,0))){      
    std::cout << "B became nan" << std::endl;
    sejong::pretty_print(sp_->Q_, std::cout, "sp_->Q_,");    
    sejong::pretty_print(sp_->Qdot_, std::cout, "sp_->Qdot_");  
    sejong::pretty_print(q_int, std::cout, "q_int");    
    sejong::pretty_print(qdot_int, std::cout, "qdot_int");            
    sejong::pretty_print(J1, std::cout, "J_feet");
    sejong::pretty_print(J1_bar, std::cout, "J1_bar");
/*    sejong::pretty_print(J2_1_bar, std::cout, "J2_1_bar");
    sejong::pretty_print(J2, std::cout, "J2"); */
    exit(1);
  }

  B_out = B;
  c_out = c;

}


void DDP_ctrl::_prep_QP_U_f(){
  sejong::Matrix U_f(6, 4); 
  U_f.setZero();

  sejong::Matrix U1(3,2); // Left Foot Friction Contact Matrix Constraint 
  sejong::Matrix U2(3,2); // Right Foot Friction Contact Matrix Constraint   
  U1.setZero();
  U2.setZero();

  double mu_static = 0.7; // Static Friction Assumption
                   U1(0, 1) = 1.0;
  U1(1, 0) = -1.0; U1(1, 1) = mu_static;
  U1(2, 0) = 1.0 ; U1(2, 1) = mu_static;
  U2 = U1;

  U_f.block(0,0, 3,2) = U1;
  U_f.block(3,2, 3,2) = U2;

  U_f_ = U_f;
}


void DDP_ctrl::_prep_QP_FR_sol(const sejong::Vector & x_state){
  int dim_opt = 4;  // Number of Reaction Forces: Fr = [Fx1, Fz1, Fx2, Fz2]
  int dim_eq_cstr = NUM_VIRTUAL;  // Number of Equality Constraints.
  int dim_ieq_cstr = NUM_ACT_JOINT*2 + dim_opt + U_f_.rows(); // Based on the constraints below

  /*
  Equality Constraints
   Sv (Aq_des + b + g) - Sv Jc^T Fr = 0 // Vector has NUM_VIRTUAL rows

  //Virtual Joints Relaxation
  // Sv (Aq_des + b + g) - Sv Jc^T Fr + eps >= 0 // Vector has NUM_VIRTUAL rows
  //-Sv (Aq_des + b + g) + Sv Jc^T Fr + eps >= 0 // Vector has NUM_VIRTUAL rows

  Torque Constraints
   Sa (Aq_des + b + g) - Sa Jc^T Fr - tau_min >= 0 // Vector has NUM_ACT_JOINT rows
  -Sa (Aq_des + b + g) + Sa Jc^T Fr + tau_max >= 0 // Vector has NUM_ACT_JOINT rows   

  Reaction Force Constraints
  -Fr + Fr_max >= 0 // Vector has 4 rows  = dim_opt

  Friction Contact Constraints
  U_f * Fr >= 0 // Vector has 6 rows = U_f_.rows()


  */ 


  G.resize(dim_opt, dim_opt);
  g0.resize(dim_opt);
  CE.resize(dim_opt, dim_eq_cstr);
  ce0.resize(dim_eq_cstr);
  CI.resize(dim_opt, dim_ieq_cstr);
  ci0.resize(dim_ieq_cstr);

  // Set Values to Zero
  for(int i(0); i<dim_opt; ++i){
    for(int j(0); j<dim_opt; ++j){
      G[i][j] = 0.0;
    }
    for(int j(0); j<dim_eq_cstr; ++j){
      CE[i][j] = 0.0;
    }
    for(int j(0); j<dim_ieq_cstr; ++j){
      CI[i][j] = 0.0;
    }
    g0[i] = 0.0;
  }
  for(int i(0); i<dim_ieq_cstr; ++i){
    ci0[0] = 0.0;
  }

  // Prepare state variables
  sejong::Vector q_int = x_state.head(NUM_QDOT);  
  sejong::Vector qdot_int = x_state.tail(NUM_QDOT);
  // Get Contact Jacobian
  // Task 1 Left and Right Foot Accelerations
  sejong::Matrix J_lf, J_rf;  
  internal_model->getFullJacobian(q_int, SJLinkID::LK_LEFT_FOOT, J_lf);
  internal_model->getFullJacobian(q_int, SJLinkID::LK_RIGHT_FOOT, J_rf);
  // Stack The Jacobians
  sejong::Matrix J_c(dim_opt, NUM_QDOT);
  J_c.block(0,0, 2, NUM_QDOT) = J_lf.block(0, 0, 2, NUM_QDOT);
  J_c.block(2,0, 2, NUM_QDOT) = J_rf.block(0, 0, 2, NUM_QDOT);

  // Prepare tot_tau_mtx -------------------------------------------------------
  sejong::Matrix tot_tau_mtx(NUM_QDOT, dim_opt);
  tot_tau_mtx = -J_c.transpose();

  // Prepare tot_tau_vec -------------------------------------------------------
  // Update Internal Model
  _update_internal_model(x_state);

  // Get B and c Matrices
  sejong::Matrix B;
  sejong::Vector c;
  _get_B_c(x_state, B, c);

  // Calculate qddot task
  sejong::Vector xddot_des(4); xddot_des.setZero(); // Set no desired foot acceleration
  sejong::Vector qddot_des = (B*xddot_des + c);


  // Desired Whole Body Dynamics
  sejong::Vector tot_tau_vec = A_int * qddot_des + coriolis_int + grav_int;


  sejong::Matrix Sv_tot_tau_mtx = Sv*tot_tau_mtx;
  sejong::Matrix Sa_tot_tau_mtx = Sa*tot_tau_mtx;  
  sejong::Vector Sv_tot_tau_vec = Sv*tot_tau_vec;
  sejong::Vector Sa_tot_tau_vec = Sa*tot_tau_vec;    


  // Set Constraints using Eigen
  sejong::Matrix sj_G(dim_opt, dim_opt);
  sejong::Matrix sj_CE(dim_eq_cstr, dim_opt);
  sejong::Vector sj_ce0(dim_eq_cstr);
  sejong::Matrix sj_CI(dim_ieq_cstr, dim_opt);
  sejong::Vector sj_ci0(dim_ieq_cstr);
  sj_CE.setZero();
  sj_ce0.setZero();
  sj_CI.setZero();
  sj_ci0.setZero();

  // Virtual Joints Constraint
  //Sv (Aq_des + b + g) - Sv Jc^T Fr // Vector has NUM_VIRTUAL rows
  sj_CE.block(0,0, NUM_VIRTUAL, dim_opt) = -Sv_tot_tau_mtx;
  sj_ce0.head(NUM_VIRTUAL) = Sv*tot_tau_vec;

/*
  // Virtual Joints Relaxation
  // Sv (Aq_des + b + g) - Sv Jc^T Fr + eps >= 0 // Vector has NUM_VIRTUAL rows
  sj_CI.block(0,0, NUM_VIRTUAL, dim_opt)           = -Sv_tot_tau_mtx;
  sj_ci0.head(NUM_VIRTUAL)                         = Sv_tot_tau_vec + virt_relax;

  //-Sv (Aq_des + b + g) + Sv Jc^T Fr + eps >= 0 // Vector has NUM_VIRTUAL rows
  sj_CI.block(NUM_VIRTUAL,0, NUM_VIRTUAL, dim_opt) = Sv_tot_tau_mtx;
  sj_ci0.segment(NUM_VIRTUAL, NUM_VIRTUAL)         = -Sv_tot_tau_vec + virt_relax;
*/

  // Torque Limits
  //    tau_min
  //    Sa (Aq_des + b + g) - Sa Jc^T Fr - tau_min >= 0
  sj_CI.block(0,0, NUM_ACT_JOINT, dim_opt) = -Sa_tot_tau_mtx;
  sj_ci0.segment(0, NUM_ACT_JOINT)         = Sa_tot_tau_vec - tau_min;

  //    tau_max
  //    -Sa (Aq_des + b + g) + Sa Jc^T Fr + tau_max >= 0
  sj_CI.block(NUM_ACT_JOINT ,0, NUM_ACT_JOINT, dim_opt) = Sa_tot_tau_mtx;
  sj_ci0.segment(NUM_ACT_JOINT, NUM_ACT_JOINT)          = -Sa_tot_tau_vec + tau_max;

  // Reaction Force Upper Bound
  //  -Fr + Fr_max >= 0
  sj_CI.block(NUM_ACT_JOINT*2 ,0, dim_opt, dim_opt) = -1.0*sejong::Matrix::Identity(dim_opt, dim_opt);  
  sj_ci0.segment(NUM_ACT_JOINT*2, dim_opt)          = Fr_max;

  // Friction Contact Constraints
  //  U_f * Fr >= 0 
  sj_CI.block(NUM_ACT_JOINT*2 + dim_opt, 0, U_f_.rows(), dim_opt) = U_f_;  


  // Set Cost Matrix
  sj_G = sejong::Matrix::Identity(dim_opt, dim_opt);

  // Set GoldFarb Equality Constraint
  for(int i(0); i< dim_eq_cstr; ++i){
    for(int j(0); j<dim_opt; ++j){
      CE[j][i] = sj_CE(i,j);
    }
    ce0[i] = sj_ce0[i];
  }  

  // Set GoldFarb Inequality Constraint
  for(int i(0); i< dim_ieq_cstr; ++i){
    for(int j(0); j<dim_opt; ++j){
      CI[j][i] = sj_CI(i,j);
    }
    ci0[i] = sj_ci0[i];
  }

  // Set GoldFarb G Matrix
  for(int i(0); i < dim_opt; ++i){
    G[i][i] = sj_G(i, i);
  }



  // Solve Quadratic Program
  double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z_out);  

  if(f > 1.e5){
    std::cout << "f: " << f << std::endl;
    std::cout << "x: " << z_out << std::endl;
//    std::cout << "cmd: "<<cmd<<std::endl;

/*    printf("G:\n");
    std::cout<<G<<std::endl;
    printf("g0:\n");
    std::cout<<g0<<std::endl;

    printf("CE:\n");
    std::cout<<CE<<std::endl;
    printf("ce0:\n");
    std::cout<<ce0<<std::endl;

    printf("CI:\n");
    std::cout<<CI<<std::endl;
    printf("ci0:\n");
    std::cout<<ci0<<std::endl;*/

  }


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
  
  
  sejong::Vector xddot_des = xddot_feet_des;
/*
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

*/  // Get B, c WBC Matrices
  sejong::Matrix B;
  sejong::Vector c;
  _get_B_c(x_state, B, c);

  // Calculate qddot task
  sejong::Vector qddot_des = (B*xddot_des + c);

  // Whole Body Dynamics
  sejong::Vector tau = A_int * qddot_des + coriolis_int + grav_int;

  // Extract Actuated Torque
  gamma_int = tau.tail(NUM_ACT_JOINT);
}


void DDP_ctrl::ComputeTorqueCommand(sejong::Vector & gamma){
  _PreProcessing_Command();
  gamma.setZero();

  #ifdef TIME_DDP
  // ---- START TIMER 
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();  
  #endif

  //_jpos_ctrl(gamma);
  _QP_ctrl(gamma);
  //_DDP_ctrl(gamma);

  #ifdef TIME_DDP
    // ----- END TIMER
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
    std::cout << "  Control Loop took " << time_span1.count()*1000.0 << "ms"<<std::endl;  
  #endif

  ++count_command_;

  state_machine_time_ = sp_->curr_time_ - start_time_;
  _PostProcessing_Command(gamma);
}



void DDP_ctrl::_QP_ctrl(sejong::Vector & gamma){
 
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

  sejong::Vector x_state(NUM_QDOT + NUM_QDOT);
  x_state.head(NUM_QDOT) = sp_->Q_;
  x_state.tail(NUM_QDOT) = sp_->Qdot_;
  _prep_QP_FR_sol(x_state);

  gamma = qddot.tail(NUM_ACT_JOINT);

}


void DDP_ctrl::_DDP_ctrl(sejong::Vector & gamma){
  sejong::Vector x_state(STATE_SIZE);
  x_state.head(NUM_QDOT) = sp_->Q_;
  x_state.tail(NUM_QDOT) = sp_->Qdot_;

  sejong::Vector u_vec(DIM_u_SIZE); 
  u_vec.setZero();

  ilqr_->compute_ilqr(x_state, u_vec);
  _get_WBC_command(x_state, u_vec, gamma);

  //_jpos_ctrl(gamma);


/*  sejong::Vector l_x, l_xF, l_u;
  sejong::Matrix l_xx, l_xxF, l_uu, l_ux, f_u;
  l_x_analytical(x_state, u_vec, l_x);
  l_x_final_analytical(x_state, l_xF);
  l_xx_analytical(x_state, u_vec, l_xx);
  l_xx_final_analytical(x_state, l_xxF);
  l_u_analytical(x_state, u_vec, l_u);
  l_uu_analytical(x_state, u_vec, l_uu);
  l_ux_analytical(x_state, u_vec, l_ux);
  f_u_analytical(x_state, u_vec, f_u);*/



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



/*  sejong::Quaternion ori_body;
  internal_model->getOrientation(sp_->Q_, SJLinkID::LK_BODY, ori_body);
  Eigen::Matrix3d R = ori_body.toRotationMatrix();
  //sejong::pretty_print(ori_body,std::cout,"quat");
  std::cout<<"mat R : \n"<<R<<std::endl;

  sejong::Vect3 svec; 
  svec.setZero();
  svec[0] = 1.0;
  sejong::pretty_print(svec,std::cout,"svec");
  std::cout<<"R*svec: \n"<<R*svec<<std::endl;*/

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

  double mu_static = 100.0;//0.1; // Friction Coefficient
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
  bool result_mu = l_mu.lcp_lemke_regularized(alpha_mu, beta_mu, &fn_fd_lambda);
//  bool result_mu = l_mu.lcp_fast(alpha_mu, beta_mu, &fn_fd_lambda);  
  

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

