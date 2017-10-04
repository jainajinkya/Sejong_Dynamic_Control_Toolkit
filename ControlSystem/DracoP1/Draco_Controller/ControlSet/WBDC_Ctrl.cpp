#include "WBDC_Ctrl.hpp"
#include <math.h>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <Draco_Model/Draco_Model.hpp>

WBDC_Ctrl::WBDC_Ctrl(): DracoController(),
                        count_command_(0),
                        jpos_ini_(NUM_ACT_JOINT),
                        des_pos_(2),
                        act_pos_(2),
                        act_vel_(2),
                        xddot_cmd_(DIM_WBDC_TASK)
{

  tau_min_.setZero();
  tau_max_.setZero();
  double torque_limit(175.);
  // double torque_limit(80.);

  for (int i(0); i<NUM_ACT_JOINT; ++i){
    tau_max_(i,0) = torque_limit;
    tau_min_(i,0) = -torque_limit;
  }
  printf("[WBDC Control] Start\n");
}

WBDC_Ctrl::~WBDC_Ctrl(){
}

void WBDC_Ctrl::Initialization(){
  for (int i(0); i < NUM_ACT_JOINT ; ++i){
    jpos_ini_[i] = sp_->Q_[i + NUM_VIRTUAL];
  }
  robot_model_->getCoMPosition(sp_->Q_, com_init_);
  robot_model_->getPosition(sp_->Q_, LK_foot, foot_ini_);
  robot_model_->getPosition(sp_->Q_, LK_body, body_init_);

  des_gap_[0] = body_init_[0] - foot_ini_[0]; // X
  des_gap_[1] = body_init_[1] - foot_ini_[1]; // Z

  start_time_ = sp_->curr_time_;
  phase_ = 10;
}


void WBDC_Ctrl::ComputeTorqueCommand(sejong::Vector & gamma){
  _PreProcessing_Command();
  // _jpos_ctrl(gamma);
  _body_ctrl(gamma);
  ++count_command_;

  state_machine_time_ = sp_->curr_time_ - start_time_;

  sejong::saveValue(state_machine_time_, "time");
  sejong::saveVector(act_pos_, "act_body_pos");
  sejong::saveVector(des_pos_, "des_body_pos");
  sejong::saveVector(gamma, "torque_cmd");
  sejong::saveVector(xddot_cmd_, "xddot_cmd");
  sejong::saveVector((sejong::Vector)optimization_results_, "opt_result");

  _PostProcessing_Command(gamma);
}

void WBDC_Ctrl::_CopyMassGravityCoriolis(){
  // A, cori, grav
  A_fix_ = A_;
  cori_fix_ = coriolis_;
  grav_fix_ = grav_;
}

void WBDC_Ctrl::_body_ctrl(sejong::Vector & gamma){
  gamma.setZero();
  // A, cori, grav
  _CopyMassGravityCoriolis();
  // Foot (Contact constraint)
  sejong::Matrix Jfoot_tmp;
  robot_model_->getFullJacobian(sp_->Q_, SJLinkID::LK_foot, Jfoot_tmp);
  Eigen::Matrix<double, 3, NUM_QDOT> Jfoot;
  Jfoot = Jfoot_tmp;

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

  // B
  Eigen::Matrix<double, NUM_QDOT, DIM_WBDC_TASK> B;
  sejong::Matrix J1_inv;
  _DynConsistent_Inverse(Jfoot_tmp, J1_inv);
  sejong::Matrix N1;
  N1 = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - J1_inv * Jfoot_tmp;
  sejong::pretty_print(Jfoot_tmp, std::cout, "jfoot tmp");
  sejong::pretty_print(J1_inv, std::cout, "jfoot inv");

  // B: body
  sejong::Matrix Jbody_tmp;
  robot_model_->getFullJacobian(sp_->Q_, LK_body, Jbody_tmp);
  sejong::Matrix J2(Jbody_tmp.block(0,0, 2, NUM_QDOT));
  // B: CoM
  // sejong::Matrix JCoM_tmp;
  // robot_model_->getCoMJacobian(sp_->Q_, JCoM_tmp);
  // sejong::Matrix J2(JCoM_tmp.block(0,0, 2, NUM_QDOT));

  sejong::Matrix J21 = J2 * N1;
  sejong::Matrix J21_inv;
  _DynConsistent_Inverse(J21, J21_inv);
  B.block(0,0, NUM_QDOT, 3) = J1_inv - J21_inv * J2 * J1_inv;
  B.block(0,3, NUM_QDOT, 2) = J21_inv;

  // x task
  Eigen::Matrix<double, DIM_WBDC_TASK, 1> x_task;
  // x task: Body
  sejong::Vect3 body_pos, body_vel;
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_body, body_pos);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, SJLinkID::LK_body, body_vel);
  x_task.setZero();
  double kp(100.0);
  double kd(5.0);

  for(int i(0); i<2; ++i){
    // For save
    des_pos_[i] = des_gap_[i];
    act_pos_[i] = (body_pos[i] - foot_pos[i]);
    act_vel_[i] = (body_vel[i] - foot_vel[i]);
    // Task command
    x_task(DIM_ReactionForce + i, 0) =
      kp * (des_pos_[i] - act_pos_[i]) + kd * (0. - act_vel_[i]);
  }

  sejong::pretty_print(body_pos, std::cout, "body_pos");
  sejong::pretty_print(body_init_, std::cout, "body_ini");
  sejong::pretty_print((sejong::Vector)x_task, std::cout, "x task");

  // selected task
  Eigen::Matrix<int, DIM_WBDC_TASK, 1> selected_task;
  selected_task.setZero();
  selected_task[0] = 1;
  // selected_task[2] = 1;
  // selected_task(DIM_ReactionForce, 0) = 0;
  selected_task[DIM_ReactionForce] = 1;
  selected_task[DIM_ReactionForce+1] = 1;


  // c
  Eigen::Matrix<double, NUM_QDOT, 1> c;
  sejong::Matrix J1dot;
  robot_model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, SJLinkID::LK_foot, J1dot);
  sejong::Matrix J2dot_tmp;
  robot_model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, SJLinkID::LK_body, J2dot_tmp);
  sejong::Matrix J2dot = J2dot_tmp.block(0,0, 2, NUM_QDOT);
  c = (-J1_inv * J1dot - J21_inv * J2dot + J21_inv * J2 * J1_inv * J1dot) * sp_->Qdot_;

  sejong::pretty_print((sejong::Vector)c, std::cout, "c");
  sejong::pretty_print((sejong::Vector)grav_cm, std::cout, "g cm");

  // wbdc_.MakeTorque(A_fix_, cori_fix_, grav_fix_, Jfoot,
  //                  Icm_, Jcm_, JcmDotQdot_,
  //                  grav_cm, Sf,
  //                  B, x_task, selected_task,
  //                  c,
  //                  tau_min_, tau_max_, cmd_, optimization_results_);
  // wbdc_2d_.MakeTorque(A_fix_, cori_fix_, grav_fix_, Jfoot,
  //                     Icm_, Jcm_, JcmDotQdot_,
  //                     grav_cm, Sf,
  //                     B, x_task, selected_task,
  //                     c,
  //                     tau_min_, tau_max_, cmd_, optimization_results_);
  // wbdc_new_.MakeTorque(A_fix_, cori_fix_, grav_fix_, Jfoot,
  //                     Icm_, Jcm_, JcmDotQdot_,
  //                     grav_cm, Sf,
  //                     B, x_task, selected_task,
  //                     c,
  //                     tau_min_, tau_max_, cmd_, optimization_results_);
  wbdc_2d_vir_acc_.MakeTorque(A_fix_, cori_fix_, grav_fix_, Jfoot,
                       Icm_, Jcm_, JcmDotQdot_,
                       grav_cm, Sf,
                       B, x_task, selected_task,
                       c,
                       tau_min_, tau_max_, cmd_, optimization_results_);

  gamma = cmd_;

  // For save
  xddot_cmd_ = x_task;
}



void WBDC_Ctrl::_jpos_ctrl(sejong::Vector & gamma){
  // A, cori, grav
  _CopyMassGravityCoriolis();

  // Foot (Contact constraint)
  sejong::Matrix Jfoot_tmp;
  robot_model_->getFullJacobian(sp_->Q_, SJLinkID::LK_foot, Jfoot_tmp);
  Eigen::Matrix<double, 3, NUM_QDOT> Jc;
  Jc = Jfoot_tmp;
  sejong::pretty_print((sejong::Matrix)Jc, std::cout, "Jc");
  // Icm, Jcm, \dot{J}_{cm}* \dot{q}
  _CentroidMatrixSetting();

  // Gravity CM
  sejong::pretty_print((sejong::Vector)grav_, std::cout, "grav");
  Eigen::Matrix<double, 3, 1> grav_cm;
  grav_cm.setZero();
  grav_cm[1] = grav_[1];

  // Sf
  Eigen::Matrix<double, 3, 3> Sf;
  Sf.setIdentity();

  sejong::Vect3 foot_pos;
  sejong::Vect3 com_pos;
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_foot, foot_pos);// X, Z, Ry
  robot_model_->getCoMPosition(sp_->Q_, com_pos); // X, Y, Z
  sejong::pretty_print(com_pos, std::cout, "com pos");
  sejong::pretty_print(foot_pos, std::cout, "foot pos");

  // Sf(0,2) = 1./(com_pos[2] - foot_pos[1]);
  // Sf(1,2) = 1./(foot_pos[0] - com_pos[0]);
  Sf(2,0) = foot_pos[1] - com_pos[2];
  Sf(2,1) = -foot_pos[0] + com_pos[0];
  sejong::pretty_print((sejong::Matrix)Sf, std::cout, "Sf");

  // B: Foot 
  Eigen::Matrix<double, NUM_QDOT, DIM_WBDC_TASK> B;
  sejong::Matrix J1_inv;
  _DynConsistent_Inverse(Jfoot_tmp, J1_inv);
  sejong::Matrix N1;
  N1 = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - J1_inv * Jfoot_tmp;

  // B: Joint
  sejong::Matrix J2(3, NUM_QDOT);
  J2.setZero();
  J2(2,0) = 1.; J2(3,1) = 1.; J2(4, 2) = 1.;
  sejong::Matrix J21 = J2 * N1;
  sejong::Matrix J21_inv;
  _DynConsistent_Inverse(J21, J21_inv);
  B.block(0,0, NUM_QDOT, 3) = J1_inv - J21_inv * J2 * J1_inv;
  B.block(0,3, NUM_QDOT, 3) = J21_inv;
  sejong::pretty_print((sejong::Matrix)B, std::cout, "B");

  // x task
  Eigen::Matrix<double, DIM_WBDC_TASK, 1> x_task;
  x_task.setZero();
  // x task: joint
  x_task.setZero();
  for (int i(0); i<NUM_ACT_JOINT; ++i){
    x_task(DIM_ReactionForce + i, 0) = 50. * (jpos_ini_[i]  - sp_->Q_[i + NUM_VIRTUAL]) + 2. * (-sp_->Qdot_[NUM_VIRTUAL + i]);
  }

  sejong::pretty_print(sp_->Q_, std::cout, "Q");
  sejong::pretty_print(sp_->Qdot_, std::cout, "Qdot");
  sejong::pretty_print((sejong::Vector)x_task, std::cout, "task cmd");

  // selected task
  Eigen::Matrix<int, DIM_WBDC_TASK, 1> selected_task;
  selected_task.setZero();
  selected_task[DIM_ReactionForce] = 0;
  selected_task[DIM_ReactionForce+2] = 0;

  // c
  Eigen::Matrix<double, NUM_QDOT, 1> c;
  sejong::Matrix J1dot;
  robot_model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, SJLinkID::LK_foot, J1dot);
  c = (-J1_inv * J1dot + J21_inv * J2 * J1_inv * J1dot) * sp_->Qdot_;

  sejong::pretty_print((sejong::Vector)c, std::cout, "c");

  wbdc_.MakeTorque(A_fix_, cori_fix_, grav_fix_, Jc,
                   Icm_, Jcm_, JcmDotQdot_,
                   grav_cm, Sf,
                   B, x_task, selected_task,
                   c,
                   tau_min_, tau_max_, cmd_, optimization_results_);
  gamma = cmd_;
}

void WBDC_Ctrl::_CentroidMatrixSetting(){
  sejong::Matrix Icm_tmp;
  robot_model_->getCentroidInertia(Icm_tmp);
  Icm_ = Icm_tmp;
  sejong::Matrix Jcm_tmp;
  robot_model_->getCentroidJacobian(Jcm_tmp);
  Jcm_ = Jcm_tmp;
  JcmDotQdot_ = Jcm_tmp * Ainv_ * coriolis_;
  sejong::pretty_print((sejong::Matrix)Icm_, std::cout, "Icm_real");
  sejong::pretty_print((sejong::Matrix)Jcm_, std::cout, "Jcm_real");
  sejong::pretty_print(JcmDotQdot_, std::cout, "Jcm * qdot");
}
