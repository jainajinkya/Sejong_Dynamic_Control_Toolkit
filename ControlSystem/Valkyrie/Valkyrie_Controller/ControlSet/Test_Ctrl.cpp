#include "Test_Ctrl.h"
#include <math.h>
#include <stdio.h>
#include <Utils/utilities.h>
#include <Utils/DataManager.h>

#include <Valkyrie_Model/Valkyrie_Model.h>
#include <chrono>

Test_Ctrl::Test_Ctrl(): ValkyrieController(),
                        count_command_(0),
                        jpos_ini_(NUM_ACT_JOINT),
                        jpos_des_(NUM_ACT_JOINT),
                        jvel_des_(NUM_ACT_JOINT),
                        // link_id_(LK_pelvis)
                        // link_id_(LK_leftHipYawLink)
                        // link_id_(LK_leftFoot)
                        link_id_(LK_leftForearmLink),
                        Jcdot_(DIM_RForce, NUM_QDOT)
{
  double torque_lim(10000.);
  for (int i(0); i< NUM_ACT_JOINT; ++i){
    tau_max_[i] = torque_lim;
    tau_min_[i] = -torque_lim;
  }
  selected_task_.setZero();
  // selected_task_[2] = 1; // Right Foot (Rz)
  // selected_task_[3] = 1; // Right Foot (X)
  // selected_task_[4] = 1; // Right Foot (Y)
  // selected_task_[8] = 1; // Left Foot (Rz)
  // selected_task_[9] = 1; // Left Foot (X)
  // selected_task_[10] = 1; // Left Foot (Y)

  double mu(0.3);
  double lx(0.135);
  double ly(0.08);

  Uc_.setZero();
  Uc_(0, 3) = -1;   Uc_(0, 5) = -mu;
  Uc_(1, 3) = 1;    Uc_(1, 5) = -mu;
  Uc_(2, 4) = -1;   Uc_(2, 5) = -mu;
  Uc_(3, 4) = 1;    Uc_(3, 5) = -mu;
  Uc_(4, 0) = -1;   Uc_(4, 5) = -ly;
  Uc_(5, 0) = 1;    Uc_(5, 5) = -ly;
  Uc_(6, 1) = -1;   Uc_(6, 5) = -lx;
  Uc_(7, 1) = 1;    Uc_(7, 5) = -lx;

  Uc_(8, 0) = mu;   Uc_(8, 1) = mu;   Uc_(8, 2) = -1;
  Uc_(8, 3) = -ly;  Uc_(8, 4) = -lx;  Uc_(8, 5) = -(lx + ly) * mu;

  Uc_(9, 0) = mu;   Uc_(9, 1) = -mu;   Uc_(9, 2) = -1;
  Uc_(9, 3) = -ly;  Uc_(9, 4) =  lx;   Uc_(9, 5) = -(lx + ly) * mu;

  Uc_(10, 0) = -mu;   Uc_(10, 1) = mu;   Uc_(10, 2) = -1;
  Uc_(10, 3) = ly;  Uc_(10, 4) = -lx;   Uc_(10, 5) = -(lx + ly) * mu;

  Uc_(11, 0) = -mu;   Uc_(11, 1) = -mu;   Uc_(11, 2) = -1;
  Uc_(11, 3) = ly;  Uc_(11, 4) =  lx;   Uc_(11, 5) = -(lx + ly) * mu;

  Uc_(12, 0) = mu;   Uc_(12, 1) = mu;   Uc_(12, 2) = 1;
  Uc_(12, 3) = ly;  Uc_(12, 4) =  lx;   Uc_(12, 5) = -(lx + ly) * mu;

  Uc_(13, 0) = mu;   Uc_(13, 1) = -mu;   Uc_(13, 2) = 1;
  Uc_(13, 3) = ly;  Uc_(13, 4) = -lx;   Uc_(13, 5) = -(lx + ly) * mu;

  Uc_(14, 0) = -mu;   Uc_(14, 1) = mu;   Uc_(14, 2) = 1;
  Uc_(14, 3) = -ly;  Uc_(14, 4) =  lx;   Uc_(14, 5) = -(lx + ly) * mu;

  Uc_(15, 0) = -mu;   Uc_(15, 1) = -mu;   Uc_(15, 2) = 1;
  Uc_(15, 3) = -ly;  Uc_(15, 4) = -lx;   Uc_(15, 5) = -(lx + ly) * mu;

  // sejong::pretty_print((sejong::Matrix)Uc_, std::cout, "Uc");
  printf("[Test Control] Start\n");
}

Test_Ctrl::~Test_Ctrl(){
}

void Test_Ctrl::Initialization(){
  for (int i(0); i < NUM_ACT_JOINT ; ++i){
    jpos_ini_[i] = sp_->Q_[i + NUM_VIRTUAL];
  }
  robot_model_->getCoMPosition(sp_->Q_, com_init_);
  robot_model_->getOrientation(sp_->Q_, LK_torso, body_ori_ini_);
  robot_model_->getOrientation(sp_->Q_, LK_pelvis, pelvis_ori_ini_);

  robot_model_->getPosition(sp_->Q_, LK_rightForearmLink, rhand_init_);
  robot_model_->getPosition(sp_->Q_, LK_leftForearmLink, lhand_init_);

  start_time_ = sp_->curr_time_;
  phase_ = 10;
}

void Test_Ctrl::ComputeTorqueCommand(sejong::Vector & gamma){
  _PreProcessing_Command();
  _PrepareMatriceVectors();

  // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

#if (DIM_TASK_GLOBAL == DIM_JPOS_TASK)
  _jpos_ctrl(gamma);
#elif (DIM_TASK_GLOBAL == DIM_MULTI_TASK)
  _multi_task_ctrl(gamma);
#elif (DIM_TASK_GLOBAL == DIM_MULTI_TASK2)
  _multi_task_ctrl2(gamma);
#elif (DIM_TASK_GLOBAL == DIM_MULTI_TASK3)
  _multi_task_ctrl_both_hand(gamma);
#endif
  // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
  // std::cout << "All process took me " << time_span1.count()*1000.0 << "ms."<<std::endl;

  ++count_command_;
  state_machine_time_ = sp_->curr_time_ - start_time_;

  sejong::saveValue(state_machine_time_, "time");
  // CoM
  sejong::saveVector(com_pos_des_, "des_com_pos");
  sejong::saveVector(com_vel_des_, "des_com_vel");
  sejong::saveVector(com_pos_, "com_pos");
  sejong::saveVector(com_vel_, "com_vel");
  sejong::saveVector(cam_vel_, "cam_vel");
  // Hand
  sejong::saveVector(lhand_pos_, "left_hand_pos");
  sejong::saveVector(rhand_pos_, "right_hand_pos");
  sejong::saveVector(lhand_vel_, "left_hand_vel");
  sejong::saveVector(rhand_vel_, "right_hand_vel");
  // Hand (des)
  sejong::saveVector(des_lhand_pos_, "des_left_hand_pos");
  sejong::saveVector(des_rhand_pos_, "des_right_hand_pos");
  sejong::saveVector(des_lhand_vel_, "des_left_hand_vel");
  sejong::saveVector(des_rhand_vel_, "des_right_hand_vel");

  // Joint
  sejong::saveVector(sp_->Q_, "config");
  sejong::saveVector(sp_->Qdot_, "qdot");

  sejong::saveVector(jpos_des_, "des_jpos");
  sejong::saveVector(jvel_des_, "des_jvel");
  sejong::saveVector(gamma, "torque_cmd");

  sejong::saveVector((sejong::Vector)optimization_results_, "opt_result");

  _PostProcessing_Command(gamma);
}

void Test_Ctrl::_GenerateBC_MultiTask(){
  //  B & c
  sejong::Matrix B_now, B_pre;
  B_.setZero();
  sejong::Matrix eye(NUM_QDOT, NUM_QDOT);
  eye.setIdentity();

  // First (Jc)
  sejong::Matrix J1_inv;
  _DynConsistent_Inverse(Jc_, J1_inv);
  sejong::Matrix N1 = eye - J1_inv * Jc_;
  B_pre = J1_inv;
  c_ = - J1_inv *  Jcdot_ * sp_->Qdot_;

  // Second (Jcm)
  sejong::Matrix J21_inv;
  sejong::Matrix J2 = Jcm_;
  sejong::Matrix J21 = Jcm_ * N1;
  _DynConsistent_Inverse(J21, J21_inv);
  sejong::Matrix N2 = N1 * (eye - J21_inv * J21);
  B_now = sejong::Matrix (NUM_QDOT, DIM_RForce + DIM_CM);
  B_now.block(0,0, NUM_QDOT, DIM_RForce) = (eye - J21_inv * J2) * B_pre;
  B_now.block(0,DIM_RForce, NUM_QDOT, DIM_CM) = J21_inv;
  B_pre = B_now;
  c_ = ((eye - J21_inv * J2) * c_ - J21_inv * JcmDotQdot_);

  // Last (Joint)
  sejong::Matrix J3pre_inv;
  sejong::Matrix J3(NUM_ACT_JOINT, NUM_QDOT);
  J3.setZero();
  (J3.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT)).setIdentity();
  sejong::Matrix J3pre = J3 * N2;
  _DynConsistent_Inverse(J3pre, J3pre_inv);
  B_now = sejong::Matrix (NUM_QDOT, DIM_MULTI_TASK);
  B_now.block(0,0, NUM_QDOT, DIM_RForce + DIM_CM) = (eye - J3pre_inv * J3) * B_pre;
  B_now.block(0,DIM_RForce + DIM_CM, NUM_QDOT, NUM_ACT_JOINT) = J3pre_inv;
  c_ = ((eye - J3pre_inv * J3)*c_);
  // B_pre = B_now;

  // End
  B_ = B_now;

#ifdef PRINT_OUT_MESSAGE
  sejong::pretty_print((sejong::Matrix)B_, std::cout, "B");
  sejong::pretty_print((sejong::Vector)c_, std::cout, "c");
#endif
}

void Test_Ctrl::_SetJointTaskCMD_Multi(const int st_idx, Eigen::Matrix<double, DIM_TASK_GLOBAL, 1> & x_task){
  double amp(0.0);
  double omega(2.*M_PI*0.2);

  double kp(450.);
  double kd(5.0);

  jpos_des_ = jpos_ini_;
  jvel_des_.setZero();
  sejong::Vector jff(NUM_ACT_JOINT);
  jff.setZero();
  jvel_des_.setZero();

  // Left
  int jctrl_idx = leftShoulderRoll - NUM_VIRTUAL;
  jpos_des_[jctrl_idx] += amp * sin(omega * state_machine_time_);
  jvel_des_[jctrl_idx] += amp * omega * cos(omega * state_machine_time_);
  jff[jctrl_idx] -= amp * omega * omega * sin(omega * state_machine_time_);

  // Right
  jctrl_idx = rightShoulderRoll - NUM_VIRTUAL;
  amp = -amp;
  jpos_des_[jctrl_idx] += amp * sin(omega * state_machine_time_);
  jvel_des_[jctrl_idx] += amp * omega * cos(omega * state_machine_time_);
  jff[jctrl_idx] -= amp * omega * omega * sin(omega * state_machine_time_);

  for (int i(0); i<NUM_ACT_JOINT; ++i){
    x_task[i + st_idx] = jff[i]
      + kp * (jpos_des_[i] - sp_->Q_[i+NUM_VIRTUAL])
      + kd * (jvel_des_[i] - sp_->Qdot_[i + NUM_VIRTUAL]);
  }
}

void Test_Ctrl::_SetCMTaskCMD_Multi(const int st_idx, Eigen::Matrix<double, DIM_TASK_GLOBAL, 1> & x_task){
  // Centroidal Angular Velocity
  int j(0);
  for (int i(st_idx); i<st_idx + 3; ++i){
    x_task[i] = 52. * (0. - cam_vel_[j]);
    ++j;
  }

  // CoM Linear Position Control
  sejong::Vect3 ff;  ff.setZero();
  com_pos_des_ = com_init_;
  com_vel_des_.setZero();

  // double amp(-0.06);
  double amp(0.0);

  double omega(2.*M_PI*0.5);
  int ctrl_idx(2);
  ff[ctrl_idx] += (amp * omega * omega * cos(omega * state_machine_time_));
  com_pos_des_[ctrl_idx] += (amp - amp * cos(omega * state_machine_time_));
  com_vel_des_[ctrl_idx] += (amp * omega * sin(omega * state_machine_time_));

  j = 0;
  double kp_com(150.);
  double kd_com(30.);
  for (int i(st_idx + 3); i<st_idx + 6; ++i){
    x_task[i] = ff[j]
      + kp_com*(com_pos_des_[j] - com_pos_[j])
      + kd_com*(com_vel_des_[j] - com_vel_[j] + sp_->stance_foot_vel_[j]);
      // + kd_com*(com_vel_des_[j] - com_vel_[j]);

    ++j;
  }
#ifdef PRINT_OUT_MESSAGE
  std::cout<<com_des<<std::endl;
  std::cout<<com_pos_<<std::endl;
#endif
}

void Test_Ctrl::_SetRightHandPosTaskCMD(const int st_idx, Eigen::Matrix<double, DIM_TASK_GLOBAL, 1> & x_task){

  des_rhand_pos_ = rhand_init_;
  des_rhand_vel_.setZero();
  double amp(0.0);
  double omega(2.*M_PI*0.5);
  int ctrl_idx(2);
  des_rhand_pos_[ctrl_idx] += amp * sin(omega * state_machine_time_);
  des_rhand_vel_[ctrl_idx] += amp * omega * cos(omega * state_machine_time_);

  int j(0);
  double kp_hand(50.);
  double kd_hand(10.);
  for (int i(st_idx); i<st_idx + 3; ++i){
    x_task[i] =
      kp_hand*(des_rhand_pos_[j] - rhand_pos_[j])
      + kd_hand*(des_rhand_vel_[j] - rhand_vel_[j]);
    ++j;
  }
}

void Test_Ctrl::_SetLeftHandPosTaskCMD(const int st_idx, Eigen::Matrix<double, DIM_TASK_GLOBAL, 1> & x_task){

  des_lhand_pos_ = lhand_init_;
  des_lhand_vel_.setZero();
  double amp(0.0);
  double omega(2.*M_PI*0.5);
  int ctrl_idx(2);
  des_lhand_pos_[ctrl_idx] += amp * sin(omega * state_machine_time_);
  des_lhand_vel_[ctrl_idx] += amp * omega * cos(omega * state_machine_time_);

  // ctrl_idx = 0;
  // pos_des[ctrl_idx] += amp * sin(omega * state_machine_time_);
  // vel_des[ctrl_idx] += amp * omega * cos(omega * state_machine_time_);

  int j(0);
  double kp_hand(50.);
  double kd_hand(10.);
  for (int i(st_idx); i<st_idx + 3; ++i){
    x_task[i] =
      kp_hand*(des_lhand_pos_[j] - lhand_pos_[j])
      + kd_hand*(des_lhand_vel_[j] - lhand_vel_[j]);
    ++j;
  }
}


void Test_Ctrl::_multi_task_ctrl(sejong::Vector & gamma){
  // Sf
  _GenerateSf();

#ifdef TIME_MEASUREMENT_TEST_CTRL
  static int count(0);
  static double time_sum(0.);
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif

  // B, c
#ifdef TIME_MEASUREMENT_TEST_BC
  static int count(0);
  static double time_sum(0.);
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif

  _GenerateBC_MultiTask();

#ifdef TIME_MEASUREMENT_TEST_BC
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
  time_sum += (time_span1.count()*1000.0);
  ++count;
  if (count % 100 == 99){
    double ave_time = time_sum/((double)count);
    std::cout << "Building BC took " << ave_time << "ms."<<std::endl;
    count = 0;
    time_sum = 0.;
  }
#endif



  ////// Task
  Eigen::Matrix<double, DIM_TASK_GLOBAL, 1> x_task;
  // Reaction Force
  for (int i(0); i<DIM_RForce; ++i) x_task[i] = 0.;
  // CM Task
  _SetCMTaskCMD_Multi(DIM_RForce, x_task);
  // Joint
  _SetJointTaskCMD_Multi(DIM_RForce + DIM_CM, x_task);


  wbdc_.MakeTorque( A_fixed_, cori_fixed_, grav_fixed_, Jc_,
                    Icm_, Jcm_, JcmDotQdot_,
                    gcm_, Sf_,
                    B_, x_task, selected_task_,
                    c_, UcR_right_, UcR_left_,
                    tau_min_, tau_max_, cmd_, optimization_results_);

#ifdef TIME_MEASUREMENT_TEST_CTRL
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
  time_sum += (time_span1.count()*1000.0);
  ++count;
  if (count % 100 == 99){
    double ave_time = time_sum/((double)count);
    std::cout << "Total Process took " << ave_time << "ms."<<std::endl;
    count = 0;
    time_sum = 0.;
  }
#endif
  gamma = cmd_;
}

void Test_Ctrl::_GenerateBC(){
  ///////////////////  B
  B_.setZero();
  sejong::Matrix J1_inv;
  _DynConsistent_Inverse(Jc_, J1_inv);
  sejong::Matrix N1 = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - J1_inv * Jc_;

  // J2
  sejong::Matrix J2(NUM_ACT_JOINT, NUM_QDOT);
  J2.setZero();
  (J2.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT)).setIdentity();

  sejong::Matrix J21 = J2 * N1;
  sejong::Matrix J21_inv;
  _DynConsistent_Inverse(J21, J21_inv);

  B_.block(0,0, NUM_QDOT, DIM_RForce) = J1_inv - J21_inv * J2 * J1_inv;
  B_.block(0,DIM_RForce, NUM_QDOT, NUM_ACT_JOINT) = J21_inv;
  // sejong::pretty_print((sejong::Matrix)B_, std::cout, "B");

  ///////////////////  c
  c_ = (-J1_inv * Jcdot_ + J21_inv * J2 * J1_inv * Jcdot_) * sp_->Qdot_;
  // sejong::pretty_print((sejong::Vector)c_, std::cout, "c");
}

void Test_Ctrl::_GenerateSf(){
  // Sf
  (Sf_.block(0,0, 6,6)).setIdentity();
  (Sf_.block(0,6, 6,6)).setIdentity();

  // Right Foot
  sejong::Vect3 P_com_rfoot = rfoot_pos_ - com_pos_;
  Sf_(0, 4) = -P_com_rfoot[2]; Sf_(0, 5) = P_com_rfoot[1];
  Sf_(1, 5) = -P_com_rfoot[0];
  Sf_(1, 3) = P_com_rfoot[2];
  Sf_(2, 3) = -P_com_rfoot[1];  Sf_(2, 4) = P_com_rfoot[0];
  // Left Foot
  sejong::Vect3 P_com_lfoot = lfoot_pos_ - com_pos_;
  Sf_(0, 4 + 6) = -P_com_lfoot[2];  Sf_(0, 5 + 6) = P_com_lfoot[1];
  Sf_(1, 5 + 6) = -P_com_lfoot[0];
  Sf_(1, 3 + 6) = P_com_lfoot[2];
  Sf_(2, 3 + 6) = -P_com_lfoot[1];  Sf_(2, 4 + 6) = P_com_lfoot[0];

#ifdef PRINT_OUT_MESSAGE
  sejong::pretty_print((sejong::Matrix)Sf_, std::cout, "Sf");
#endif
}

void Test_Ctrl::_jpos_ctrl(sejong::Vector & gamma){
  // Sf
  _GenerateSf();
  // B, c
#ifdef TIME_MEASUREMENT_TEST_BC
  static int count(0);
  static double time_sum(0.);
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif

  _GenerateBC();

#ifdef TIME_MEASUREMENT_TEST_BC
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
  time_sum += (time_span1.count()*1000.0);
  ++count;
  if (count % 100 == 99){
    double ave_time = time_sum/((double)count);
    std::cout << "Building BC took " << ave_time << "ms."<<std::endl;
    count = 0;
    time_sum = 0.;
  }
#endif



  // Task
  Eigen::Matrix<double, DIM_TASK_GLOBAL, 1> x_task;

  for (int i(0); i<DIM_RForce; ++i) x_task[i] = 0.;

  double amp(0.0);
  double omega(2.*M_PI*0.8);

  // double kp(300.);
  // double kd(20.0);
  double kp(30.);
  double kd(10.0);

  jpos_des_ = jpos_ini_;
  sejong::Vector jff(NUM_ACT_JOINT);
  jff.setZero();
  jvel_des_.setZero();

  // Left
  int jctrl_idx = leftShoulderRoll - NUM_VIRTUAL;
  jpos_des_[jctrl_idx] += (amp - amp * cos(omega * state_machine_time_));
  jvel_des_[jctrl_idx] += (amp * omega * sin(omega * state_machine_time_));
  jff[jctrl_idx] += amp * omega * omega * cos(omega * state_machine_time_);

  // Right
  jctrl_idx = rightShoulderRoll - NUM_VIRTUAL;
  amp = -amp;
  jpos_des_[jctrl_idx] += (amp - amp * cos(omega * state_machine_time_));
  jvel_des_[jctrl_idx] += (amp * omega * sin(omega * state_machine_time_));
  jff[jctrl_idx] += amp * omega * omega * cos(omega * state_machine_time_);

  for (int i(0); i<NUM_ACT_JOINT; ++i){
    x_task[i + DIM_RForce] = jff[i]
      + kp * (jpos_des_[i] - sp_->Q_[i+NUM_VIRTUAL])
      + kd * (jvel_des_[i] - sp_->Qdot_[i + NUM_VIRTUAL]);
  }


#ifdef TIME_MEASUREMENT_TEST_CTRL
  static int count(0);
  static double time_sum(0.);
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif

  wbdc_.MakeTorque(A_fixed_, cori_fixed_, grav_fixed_, Jc_,
                   Icm_, Jcm_, JcmDotQdot_,
                   gcm_, Sf_,
                   B_, x_task, selected_task_,
                   c_, UcR_right_, UcR_left_,
                   tau_min_, tau_max_, cmd_, optimization_results_);
  
#ifdef TIME_MEASUREMENT_TEST_CTRL
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
  time_sum += (time_span1.count()*1000.0);
  ++count;
  if (count % 100 == 99){
    double ave_time = time_sum/((double)count);
    std::cout << "Optimization took " << ave_time << "ms."<<std::endl;
    count = 0;
    time_sum = 0.;
  }
#endif

  gamma = cmd_;
}

void Test_Ctrl::_PrepareMatriceVectors(){
  // A, cori, grav, Jc
  A_fixed_ = A_;
  cori_fixed_ = coriolis_;
  grav_fixed_ = grav_;
  sejong::Matrix Jtmp;
  robot_model_->getFullJacobian(sp_->Q_, SJLinkID::LK_rightCOP_Frame, Jtmp);
  Jc_.block(0,0, 6, NUM_QDOT) = Jtmp;
  robot_model_->getFullJacobian(sp_->Q_, SJLinkID::LK_leftCOP_Frame, Jtmp);
  Jc_.block(6,0, 6, NUM_QDOT) = Jtmp;
  // JcDot
  robot_model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, SJLinkID::LK_rightCOP_Frame, Jtmp);
  Jcdot_.block(0, 0, 6, NUM_QDOT) = Jtmp;
  robot_model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, SJLinkID::LK_leftCOP_Frame, Jtmp);
  Jcdot_.block(6, 0, 6, NUM_QDOT) = Jtmp;

  // Centroidal Momentum
  sejong::Matrix Icm_tmp;
  robot_model_->getCentroidInertia(Icm_tmp);
  Icm_ = Icm_tmp;
  sejong::Matrix Jcm_tmp;
  robot_model_->getCentroidJacobian(Jcm_tmp);
  Jcm_ = Jcm_tmp;
  JcmDotQdot_ = Jcm_tmp * Ainv_ * coriolis_;

  gcm_.setZero();
  gcm_[5] = grav_[2];

  // Positions
  robot_model_->getCoMPosition(sp_->Q_, com_pos_);
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_rightCOP_Frame, rfoot_pos_);
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_leftCOP_Frame, lfoot_pos_);
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_rightForearmLink, rhand_pos_);
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_leftForearmLink, lhand_pos_);

  // Velocity
  sejong::Vector cent_vel;
  robot_model_->getCentroidVelocity(cent_vel);
  cam_vel_ = cent_vel.head(3);
  com_vel_ = cent_vel.tail(3);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_,
                            SJLinkID::LK_rightForearmLink, rhand_vel_);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_,
                            SJLinkID::LK_leftForearmLink, lhand_vel_);

  // Orientation
  sejong::Quaternion ori;
  Eigen::Matrix3d R;
  // Right
  robot_model_->getOrientation(sp_->Q_, SJLinkID::LK_rightCOP_Frame, ori);
  R = ori.toRotationMatrix();
  sejong::Matrix R_right(6,6);
  R_right.setZero();
  R_right.block(0,0, 3,3) = R.transpose();
  R_right.block(3,3, 3,3) = R.transpose();
  UcR_right_ = Uc_;// * R_right;

  // Left
  robot_model_->getOrientation(sp_->Q_, SJLinkID::LK_leftCOP_Frame, ori);
  R = ori.toRotationMatrix();
  sejong::Matrix R_left(6,6);
  R_left.setZero();
  R_left.block(0,0, 3,3) = R.transpose();
  R_left.block(3,3, 3,3) = R.transpose();
  UcR_left_ = Uc_;// * R_left;


  // std::cout<<"UcR right\n"<<UcR_right_<<std::endl;
  // std::cout<<"UcR left\n"<<UcR_left_<<std::endl;

  /////// Print
  // sejong::pretty_print((sejong::Vector)rfoot_pos_, std::cout, "rfoot_pos");
  // sejong::pretty_print((sejong::Vector)lfoot_pos_, std::cout, "lfoot_pos");
  // sejong::pretty_print((sejong::Matrix)Icm_, std::cout, "Icm_real");
  // sejong::pretty_print_short((sejong::Matrix)Jc_, std::cout, "Jc_real");
  // sejong::pretty_print_short((sejong::Matrix)Jcdot_, std::cout, "JcDot");
  // sejong::pretty_print((sejong::Matrix)Jcm_, std::cout, "Jcm_real");
  // sejong::pretty_print((sejong::Vector)gcm_, std::cout, "gcm_real");
  // sejong::pretty_print((sejong::Vector)JcmDotQdot_, std::cout, "Jcm * qdot");
}
