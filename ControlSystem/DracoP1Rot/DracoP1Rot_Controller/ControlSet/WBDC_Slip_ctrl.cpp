#include "WBDC_Slip_ctrl.hpp"
#include <math.h>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <DracoP1Rot_Model/Draco_Model.hpp>
#include <Optimizer/Goldfarb/QuadProg++.hh>

WBDC_Slip_ctrl::WBDC_Slip_ctrl(): DracoController(),
                                  count_command_(0),
                                  des_pos_(2),
                                  act_pos_(2),
                                  act_vel_(2),
                                  xddot_cmd_(DIM_NONSLIP_TASK + DIM_MOTION_TASK + DIM_SLIP_TASK)
{

  tau_min_.setZero();
  tau_max_.setZero();
  double torque_limit(175.);
  // double torque_limit(80.);

  for (int i(0); i<NUM_ACT_JOINT; ++i){
    tau_max_(i,0) = torque_limit;
    tau_min_(i,0) = -torque_limit;
  }
  printf("[WBDC Slip Control] Start\n");
}

WBDC_Slip_ctrl::~WBDC_Slip_ctrl(){
}

void WBDC_Slip_ctrl::Initialization(){
  robot_model_->getPosition(sp_->Q_, LK_foot, foot_ini_);
  robot_model_->getPosition(sp_->Q_, LK_body, body_init_);

  des_gap_[0] = body_init_[0] - foot_ini_[0]; // X
  des_gap_[1] = body_init_[1] - foot_ini_[1]; // Z

  start_time_ = sp_->curr_time_;
  phase_ = 10;
}


void WBDC_Slip_ctrl::ComputeTorqueCommand(sejong::Vector & gamma){
  _PreProcessing_Command();
  _body_ctrl(gamma);
  ++count_command_;

  sejong::saveValue(state_machine_time_, "time");
  sejong::saveVector(act_pos_, "act_body_pos");
  sejong::saveVector(des_pos_, "des_body_pos");
  sejong::saveVector(gamma, "torque_cmd");
  sejong::saveVector(xddot_cmd_, "xddot_cmd");
  sejong::saveVector((sejong::Vector)optimization_results_, "opt_result");

  _PostProcessing_Command(gamma);
}

void WBDC_Slip_ctrl::_CopyMassGravityCoriolis(){
  // A, cori, grav
  A_fix_ = A_;
  cori_fix_ = coriolis_;
  grav_fix_ = grav_;
}

void WBDC_Slip_ctrl::_CentroidMatrixSetting(){
  sejong::Matrix Icm_tmp;
  robot_model_->getCentroidInertia(Icm_tmp);
  Icm_ = Icm_tmp;
  sejong::Matrix Jcm_tmp;
  robot_model_->getCentroidJacobian(Jcm_tmp);
  Jcm_ = Jcm_tmp;
  JcmDotQdot_ = Jcm_tmp * Ainv_ * coriolis_;
  // sejong::pretty_print((sejong::Matrix)Icm_, std::cout, "Icm_real");
  // sejong::pretty_print((sejong::Matrix)Jcm_, std::cout, "Jcm_real");
  // sejong::pretty_print(JcmDotQdot_, std::cout, "Jcm * qdot");
}

void WBDC_Slip_ctrl::_body_ctrl(sejong::Vector &gamma){
    gamma.setZero();
  // A, cori, grav
  _CopyMassGravityCoriolis();
  // Foot (Contact constraint)
  sejong::Matrix Jc_tmp;
  robot_model_->getFullJacobian(sp_->Q_, SJLinkID::LK_foot, Jc_tmp);
  Eigen::Matrix<double, 3, NUM_QDOT> Jc;
  Jc = Jc_tmp;

  // Icm, Jcm, \dot{J}_{cm}* \dot{q}
  _CentroidMatrixSetting();

  // Gravity CM
  Eigen::Matrix<double, 3, 1> grav_cm;
  grav_cm.setZero();
  grav_cm[1] = grav_[1];

  // Sf
  Eigen::Matrix<double, 3, 3> Sf;
  Sf.setIdentity();

  sejong::Vect3 foot_pos;
  sejong::Vect3 foot_vel;
  sejong::Vect3 com_pos;
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_foot, foot_pos);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, SJLinkID::LK_foot, foot_vel);
  robot_model_->getCoMPosition(sp_->Q_, com_pos);
  Sf(2,0) = foot_pos[1] - com_pos[2];
  Sf(2,1) = -foot_pos[0] + com_pos[0];

  // x task
  Eigen::Matrix<double, DIM_NONSLIP_TASK + DIM_MOTION_TASK + DIM_SLIP_TASK, 1> x_task;
  // x task: Body
  sejong::Vect3 body_pos, body_vel;
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_body, body_pos);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, SJLinkID::LK_body, body_vel);
  x_task.setZero();
  double kp(100.0);
  double kd(5.0);
  sejong::Vect2 des_vel;
  double amp(0.0);
  double omega(0.5 * 2.*M_PI);
  for(int i(0); i<DIM_MOTION_TASK; ++i){
    // For save
    des_pos_[i] = des_gap_[i] + amp * sin(omega * state_machine_time_);
    des_vel[i] = amp * omega * cos(omega * state_machine_time_);
    act_pos_[i] = (body_pos[i] - foot_pos[i]);
    act_vel_[i] = (body_vel[i] - foot_vel[i]);
    // Task command
    x_task(2 + i, 0) =
      kp * (des_pos_[i] - act_pos_[i]) + kd * (des_vel[i] - act_vel_[i]);
  }

  // sejong::pretty_print(body_pos, std::cout, "body_pos");
  // sejong::pretty_print(body_init_, std::cout, "body_ini");
  // sejong::pretty_print((sejong::Vector)x_task, std::cout, "x task");

  //  B & c
  sejong::Matrix Jtmp;
  sejong::Matrix B_now, B_pre;
  Eigen::Matrix<double, NUM_QDOT, DIM_NONSLIP_TASK + DIM_MOTION_TASK + DIM_SLIP_TASK> B_;
  sejong::Vector c_;
  B_.setZero();
  sejong::Matrix eye(NUM_QDOT, NUM_QDOT);
  eye.setIdentity();

  int prev_idx(0);
  int task_dim = 2;
  // First: vertical & rotation
  sejong::Matrix J1, J1_inv, J1dot;
  robot_model_->getFullJacobian(sp_->Q_, LK_foot, Jtmp);
  J1 = Jtmp.block(1, 0, 2, NUM_QDOT);
  J1dot = Jtmp.block(1, 0, 2, NUM_QDOT);
  _DynConsistent_Inverse(J1, J1_inv);
  sejong::Matrix N_pre = eye - J1_inv * J1;

  robot_model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, LK_foot, Jtmp);
  B_pre = J1_inv;
  c_ = - J1_inv *  J1dot * sp_->Qdot_;
  prev_idx = task_dim;

  // Second: Body
  sejong::Matrix Jfoot;
  robot_model_->getFullJacobian(sp_->Q_, LK_foot, Jfoot);
  // sejong::pretty_print(Jfoot, std::cout, "J foot");
  task_dim = 2;
  sejong::Matrix J21_inv;
  robot_model_->getFullJacobian(sp_->Q_, LK_body, Jtmp);
  sejong::Matrix J2 = Jtmp.block(0,0, 2, NUM_QDOT) - Jfoot.block(0,0, 2, NUM_QDOT);
  robot_model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, LK_body, Jtmp);
  sejong::Matrix J2dot = Jtmp.block(0,0, 2, NUM_QDOT);

  sejong::Matrix J21 = J2 * N_pre;
  _DynConsistent_Inverse(J21, J21_inv);
  N_pre = N_pre * (eye - J21_inv * J21);
  B_now = sejong::Matrix (NUM_QDOT, prev_idx + task_dim);
  B_now.block(0,0, NUM_QDOT, prev_idx) = (eye - J21_inv * J2) * B_pre;
  B_now.block(0, prev_idx, NUM_QDOT, task_dim) = J21_inv;
  B_pre = B_now;
  c_ = ((eye - J21_inv * J2) * c_ - J21_inv * J2dot * sp_->Qdot_);
  prev_idx += task_dim;
  // sejong::pretty_print(N_pre, std::cout, "N pre");

  // 3rd (horizontal)
  task_dim = 1;
  sejong::Matrix J3pre_inv;
  sejong::Matrix J3;
  sejong::Matrix J3dot;
  robot_model_->getFullJacobian(sp_->Q_, LK_foot, Jtmp);
  J3 = Jtmp.block(0,0, 1, NUM_QDOT);
  robot_model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, LK_foot, Jtmp);
  J3dot = Jtmp.block(0,0, 1, NUM_QDOT);

  sejong::Matrix J3pre = J3 * N_pre;
  _DynConsistent_Inverse(J3pre, J3pre_inv);
  N_pre = N_pre * (eye - J3pre_inv * J3pre);
  B_now = sejong::Matrix (NUM_QDOT, prev_idx + task_dim);
  B_now.block(0,0, NUM_QDOT, prev_idx) = (eye - J3pre_inv * J3) * B_pre;
  B_now.block(0, prev_idx, NUM_QDOT, task_dim) = J3pre_inv;
  B_pre = B_now;
  c_ = ((eye - J3pre_inv * J3) * c_ - J3pre_inv * J3dot * sp_->Qdot_);
  prev_idx += task_dim;

  B_ = B_now;
  Eigen::Matrix<double, NUM_QDOT, 1> c_fix = c_;
  // sejong::pretty_print(B_pre, std::cout, "B");

  // wbdc_slip_.MakeTorque(A_fix_, cori_fix_, grav_fix_, Jc,
  //                       Icm_, Jcm_, JcmDotQdot_,
  //                       grav_cm, Sf,
  //                       B_, x_task, 
  //                       c_fix,
  //                       tau_min_, tau_max_, cmd_, optimization_results_);
  Eigen::Matrix<double, DIM_RForce + DIM_MOTION_TASK + 4*DIM_SLIP_TASK, 1> opt_result;
  wbdc_integer_.MakeTorque(A_fix_, cori_fix_, grav_fix_, Jc,
                           Icm_, Jcm_, JcmDotQdot_,
                           grav_cm, Sf,
                           B_, x_task, 
                           c_fix,
                           tau_min_, tau_max_, cmd_, opt_result);
  gamma = cmd_;

  // For save
  xddot_cmd_ = x_task;
}
