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
  internal_model = Walker2D_Model::GetWalker2D_Model();
  ilqr_ = new iLQR();  
  ilqr_->l_cost = std::bind( &DDP_ctrl::l_cost, this, std::placeholders::_1, std::placeholders::_2);
  ilqr_->l_cost_final = std::bind( &DDP_ctrl::l_cost_final, this, std::placeholders::_1);
  ilqr_->f = std::bind( &DDP_ctrl::f, this, std::placeholders::_1, std::placeholders::_2);

  ilqr_->compute_ilqr();
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

// iLQR functions ----------------------------------------------------
sejong::Vector DDP_ctrl::f(const sejong::Vector & x, const sejong::Vector & u){
  // Compute WBC given (x,u)
  // Store gamma_out
  // Solve LCP problem
  return x;
}

double DDP_ctrl::l_cost(const sejong::Vector &x, const sejong::Vector &u){
  std::cout << " Output:" << 200.0 << std::endl;  
  return 200;
}

double DDP_ctrl::l_cost_final(const sejong::Vector &x){
  std::cout << " Output:" << 100.0 << std::endl;  
  return 100;
}
// ---------------------------------------------------------------------

void DDP_ctrl::_get_WBC_command(const sejong::Vector & x_state, 
                               const sejong::Vector & des_acc, 
                               sejong::Vector & gamma_int){

  sejong::Vector q_int = x_state.head(NUM_Q);  
  sejong::Vector qdot_int = x_state.tail(NUM_QDOT);   

  sejong::Vect3 com_pos;

  // Get EE positions
  sejong::Vect3 lf_pos3, rf_pos3, body_pos3;
  internal_model->getPosition(q_int, SJLinkID::LK_LEFT_FOOT, lf_pos3);
  internal_model->getPosition(q_int, SJLinkID::LK_RIGHT_FOOT, rf_pos3);  
  internal_model->getPosition(q_int, SJLinkID::LK_BODY, body_pos3);    
  sejong::Vector lf_pos = lf_pos3.head(2); // X, Z
  sejong::Vector rf_pos = rf_pos3.head(2); // X, Z
  sejong::Vector body_ry = body_pos3.tail(1); // Ry   
  sejong::pretty_print(body_ry, std::cout, "body_ry");

  if (std::abs(lf_pos[1]) <= NEAR_ZERO){
    std::cout << "left foot contact active" << std::endl;
  }
  if (std::abs(rf_pos[1]) <= NEAR_ZERO){
    std::cout << "right foot contact active" << std::endl;
  }
 
  // Foot Tasks
  sejong::Matrix J_lf, J_rf;  
  sejong::Matrix J_lf_dot, J_rf_dot;    
  internal_model->getFullJacobian(q_int, SJLinkID::LK_LEFT_FOOT, J_lf);
  internal_model->getFullJacobian(q_int, SJLinkID::LK_RIGHT_FOOT, J_rf);
  internal_model->getFullJacobianDot(q_int, qdot_int, SJLinkID::LK_LEFT_FOOT, J_lf_dot);
  internal_model->getFullJacobianDot(q_int, qdot_int, SJLinkID::LK_RIGHT_FOOT, J_rf_dot);

  sejong::Matrix J_feet(4, NUM_Q);
  J_feet.block(0,0, 2, NUM_Q) = J_lf.block(0, 0, 2, NUM_Q);
  J_feet.block(2,0, 2, NUM_Q) = J_rf.block(0, 0, 2, NUM_Q);  
  sejong::Matrix J_feet_dot(4, NUM_Q);
  J_feet_dot.block(0,0, 2, NUM_Q) = J_lf_dot.block(0, 0, 2, NUM_Q);
  J_feet_dot.block(2,0, 2, NUM_Q) = J_rf_dot.block(0, 0, 2, NUM_Q);  

  sejong::Vector xddot_feet_des(4);
  xddot_feet_des.setZero();
  xddot_feet_des[0] = 0;//-32.0; // LF X direction
  xddot_feet_des[1] = 0;//-32.0; // LF Z direction

  xddot_feet_des[2] = 0;//-42.0; // RF X direction
  xddot_feet_des[3] = 0;//-42.0; // RF Z direction

  // Body Ry Task
  sejong::Matrix J_bry;  
  sejong::Matrix J_bry_dot;    
  internal_model->getFullJacobian(q_int, SJLinkID::LK_BODY, J_bry);
  internal_model->getFullJacobianDot(q_int, qdot_int, SJLinkID::LK_BODY, J_bry_dot);  
  
  sejong::Matrix J_body = J_bry.block(2, 0, 1, NUM_Q);
  sejong::Matrix J_body_dot = J_bry_dot.block(2, 0, 1, NUM_Q);  
  sejong::Vector xddot_body_des(1);
  xddot_body_des[0] = 10*(0 - body_ry[0]);

    // COM Task
  sejong::Vector com_cur(2);
  internal_model->getCoMPosition(q_int, com_pos);

  com_cur[0] = com_pos[0]; // x
  com_cur[1] = com_pos[2]; // z
  sejong::pretty_print(com_cur, std::cout, "COM_pos");

  sejong::Matrix J_com_tmp;
  internal_model->getCoMJacobian(q_int, J_com_tmp);
  sejong::Matrix J_com(2, NUM_Q);
  J_com.block(0,0, 1, NUM_Q) = J_com_tmp.block(0,0, 1, NUM_Q);
  J_com.block(1,0, 1, NUM_Q) = J_com_tmp.block(2,0, 1, NUM_Q);  

  sejong::Vector xddot_com_des(2);
  double omega(2.*M_PI * 1.);
  xddot_com_des[0] = 0.0;
  xddot_com_des[1] = ((0.01 * sin(-omega * state_machine_time_) + 0.29) - com_cur[1]); 


  // Joint Position Task
  sejong::Matrix J_pos(NUM_ACT_JOINT, NUM_Q);
  sejong::Matrix J_pos_dot(NUM_ACT_JOINT, NUM_Q);
  J_pos.setZero();
  J_pos.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = sejong::Matrix::Identity(NUM_ACT_JOINT, NUM_ACT_JOINT);  
  J_pos_dot.setZero();

  sejong::Vector xddot_des_pos(NUM_ACT_JOINT);
  xddot_des_pos.setZero();
  double kp(300.); // 50
  double kd(50.);  // 15
  sejong::Vector jpos_des = jpos_ini_;
  xddot_des_pos = kp*(jpos_des - q_int.tail(NUM_ACT_JOINT)) + kd * ( -qdot_int.tail(NUM_ACT_JOINT));

/*  sejong::Matrix J1 = J_com;
  sejong::Vector x1ddot = xddot_com_des;  */
  sejong::Matrix J1 = J_feet;
  sejong::Matrix J1_dot = J_feet_dot;  
  sejong::Vector x1ddot = xddot_feet_des;    

  sejong::Matrix J2 = J_pos;  
  sejong::Matrix J2_dot = J_pos_dot;  
  sejong::Vector x2ddot = xddot_des_pos;

  sejong::Matrix J3 = J_body;  
  sejong::Matrix J3_dot = J_body_dot;    
  sejong::Vector x3ddot = xddot_body_des;  


  // Prepare Projections
  sejong::Matrix J1_bar;
  _DynConsistent_Inverse(J1, J1_bar);

  sejong::Matrix N1 = sejong::Matrix::Identity(NUM_Q, NUM_Q) - J1_bar*J1;
  sejong::Matrix J2_1 = J2*N1;

  sejong::Matrix J2_1_bar;
  _DynConsistent_Inverse(J2_1, J2_1_bar); 

/*  sejong::Matrix N2_1 = (sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - J2_1_bar*J2_1_bar);
  sejong::Matrix J3_21 = J3*N1*N2_1;
  sejong::Matrix J3_21_bar;
  _DynConsistent_Inverse(J3_21, J3_21_bar);   */

  // Calculate qddot task
  sejong::Vector qddot_1(NUM_Q);
  sejong::Vector qddot_2(NUM_Q);
  sejong::Vector qddot_3(NUM_Q);  
  qddot_1 = J1_bar*(x1ddot - J1_dot*qdot_int);
  qddot_2 = J2_1_bar*(x2ddot - J2_dot*qdot_int - J2*qddot_1);
  //qddot_3 = J3_21_bar*(x3ddot - J3_dot*qdot_int - J3*(qddot_1 + qddot_2));

  sejong::Matrix J_pos_bar;
  _DynConsistent_Inverse(J_pos, J_pos_bar);

  sejong::Vector qddot(NUM_Q);

//  qddot =  J_pos_bar*(xddot_des_pos);
  qddot =  qddot_1 + qddot_2;
  sejong::Vector tau = A_ * qddot + coriolis_ + grav_;

//  gamma_int = qddot.tail(NUM_ACT_JOINT);
  gamma_int = tau.tail(NUM_ACT_JOINT);

  sejong::pretty_print(q_int, std::cout, "q_int");

  // Task 1 Left and Right Foot Accelerations
  // Task 2 Posture Task 
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
  gamma.setZero();
  sejong::Vector x_state(STATE_X_SIZE);
  sejong::Vector u_vec(2); 
  u_vec.setZero();
  x_state.head(NUM_Q) = sp_->Q_;
  x_state.tail(NUM_QDOT) = sp_->Qdot_;

  _get_WBC_command(x_state, u_vec, gamma);

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