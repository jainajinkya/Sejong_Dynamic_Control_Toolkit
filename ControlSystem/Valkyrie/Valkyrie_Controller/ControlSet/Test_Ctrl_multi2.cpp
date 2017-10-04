#include "Test_Ctrl.h"
#include <math.h>
#include <stdio.h>
#include <Utils/utilities.h>
#include <Utils/DataManager.h>

#include <Valkyrie_Model/Valkyrie_Model.h>
#include <chrono>


void Test_Ctrl::_GenerateBC_MultiTask2(){
  //  B & c
  sejong::Matrix B_now, B_pre;
  B_.setZero();
  sejong::Matrix eye(NUM_QDOT, NUM_QDOT);
  eye.setIdentity();

  // First (Jc)
  int prev_idx(0);
  sejong::Matrix J1_inv;
  _DynConsistent_Inverse(Jc_, J1_inv);
  sejong::Matrix N_pre = eye - J1_inv * Jc_;
  B_pre = J1_inv;
  c_ = - J1_inv *  Jcdot_ * sp_->Qdot_;
  prev_idx = DIM_RForce;

  // Second (Jcm)
  sejong::Matrix J21_inv;
  sejong::Matrix J2 = Jcm_;
  sejong::Matrix J21 = Jcm_ * N_pre;
  _DynConsistent_Inverse(J21, J21_inv);
  N_pre = N_pre * (eye - J21_inv * J21);
  B_now = sejong::Matrix (NUM_QDOT, prev_idx + DIM_CM);
  B_now.block(0,0, NUM_QDOT, prev_idx) = (eye - J21_inv * J2) * B_pre;
  B_now.block(0, prev_idx, NUM_QDOT, DIM_CM) = J21_inv;
  B_pre = B_now;
  c_ = ((eye - J21_inv * J2) * c_ - J21_inv * JcmDotQdot_);
  prev_idx += DIM_CM;

  // Third (Right Hand)
  sejong::Matrix J3pre_inv;
  sejong::Matrix J3;
  sejong::Matrix J3dot;
  sejong::Matrix Jtmp;
  robot_model_->getFullJacobian(sp_->Q_, LK_rightForearmLink, Jtmp);
  J3 = Jtmp.block(3,0, 3, NUM_QDOT);
  robot_model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, LK_rightForearmLink, Jtmp);
  J3dot = Jtmp.block(3,0, 3, NUM_QDOT);
  sejong::Matrix J3pre = J3 * N_pre;
  _DynConsistent_Inverse(J3pre, J3pre_inv);
  N_pre = N_pre * (eye - J3pre_inv * J3pre);
  B_now = sejong::Matrix (NUM_QDOT, prev_idx + DIM_RIGHT_HAND_POS);
  B_now.block(0,0, NUM_QDOT, prev_idx) = (eye - J3pre_inv * J3) * B_pre;
  B_now.block(0, prev_idx, NUM_QDOT, DIM_RIGHT_HAND_POS) = J3pre_inv;
  B_pre = B_now;
  c_ = ((eye - J3pre_inv * J3) * c_ - J3pre_inv * J3dot * sp_->Qdot_);
  prev_idx += DIM_RIGHT_HAND_POS;

  // Last (Joint)
  sejong::Matrix Jlast_pre_inv;
  sejong::Matrix Jlast(NUM_ACT_JOINT, NUM_QDOT);
  Jlast.setZero();
  (Jlast.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT)).setIdentity();
  sejong::Matrix Jlast_pre = Jlast * N_pre;
  _DynConsistent_Inverse(Jlast_pre, Jlast_pre_inv);
  B_now = sejong::Matrix (NUM_QDOT, prev_idx + NUM_ACT_JOINT);
  B_now.block(0,0, NUM_QDOT, prev_idx) = (eye - Jlast_pre_inv * Jlast) * B_pre;
  B_now.block(0,prev_idx, NUM_QDOT, NUM_ACT_JOINT) = Jlast_pre_inv;
  c_ = ((eye - Jlast_pre_inv * Jlast)*c_);


  // End
  B_ = B_now;
#ifdef PRINT_OUT_MESSAGE
  sejong::pretty_print((sejong::Matrix)B_, std::cout, "B");
  sejong::pretty_print((sejong::Vector)c_, std::cout, "c");
#endif
}


void Test_Ctrl::_multi_task_ctrl2(sejong::Vector & gamma){
  // Sf
  _GenerateSf();
  // B, c
#ifdef TIME_MEASUREMENT_TEST_BC
  static int count(0);
  static double time_sum(0.);
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif

  _GenerateBC_MultiTask2();

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
  // RightHand Task
  _SetRightHandPosTaskCMD(DIM_RForce + DIM_CM, x_task);
  // Joint
  _SetJointTaskCMD_Multi(DIM_RForce + DIM_CM + DIM_RIGHT_HAND_POS,
                         x_task);
#ifdef PRINT_OUT_MESSAGE
  sejong::pretty_print((sejong::Vector)x_task, std::cout, "x task");
#endif

#ifdef TIME_MEASUREMENT_TEST_CTRL
  static int count(0);
  static double time_sum(0.);
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif

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
    std::cout << "Optimization took " << ave_time << "ms."<<std::endl;
    count = 0;
    time_sum = 0.;
  }
#endif
  gamma = cmd_;
}
