#include "Basic_Ctrl.hpp"
#include <chrono>

Basic_Ctrl::Basic_Ctrl():DracoController(),
                         jpos_ini_(NUM_ACT_JOINT)
{
  printf("[Draco P1 Rot] Basic Controller \n");
}
Basic_Ctrl::~Basic_Ctrl(){}

void Basic_Ctrl::Initialization(){
  for (int i(0); i < NUM_ACT_JOINT ; ++i){
    jpos_ini_[i] = sp_->Q_[i + NUM_VIRTUAL];
  }
  start_time_ = sp_->curr_time_;
  phase_ = 10;
}

void Basic_Ctrl::ComputeTorqueCommand(sejong::Vector & gamma){
  _PreProcessing_Command();

  _jpos_ctrl_hanging(gamma);

  _PostProcessing_Command(gamma);

  // _time_test();
}

void Basic_Ctrl::_jpos_ctrl_hanging(sejong::Vector & gamma){
  double kp(10.);
  double kd(2.);
  sejong::Vector qddot(NUM_QDOT); qddot.setZero();

  sejong::Vector jpos_des = jpos_ini_;

  for(int i(0);i<NUM_ACT_JOINT; ++i){
    qddot[i + NUM_VIRTUAL] = kp * (jpos_des[i] - sp_->Q_[i + NUM_VIRTUAL]) + kd * (-sp_->Qdot_[i + NUM_VIRTUAL]);
  }

  sejong::Vector torque = A_ * qddot + grav_;

  gamma = torque.tail(NUM_ACT_JOINT);
}

void Basic_Ctrl::_time_test(){
  int num_iter(2000);

#define A1_row 100
#define A1_col 50
#define B1_col 20

  Eigen::Matrix<double, A1_row, A1_col> A1;
  Eigen::Matrix<double, A1_col, B1_col> B1;
  Eigen::Matrix<double, A1_col, B1_col> B2;
  Eigen::Matrix<double, A1_row, B1_col> result;
  // A1.setZero(); B1.setZero(); result.setZero();
  A1.setRandom();
  B1.setRandom(); B2.setRandom();
  result.setRandom();

  Eigen::MatrixXd A1_dyn(A1_row, A1_col);
  Eigen::MatrixXd B1_dyn(A1_col, B1_col);
  Eigen::MatrixXd B2_dyn(A1_col, B1_col);
  Eigen::MatrixXd result_dyn(A1_row, B1_col);
  // A1_dyn.setZero(); B1_dyn.setZero(); result_dyn.setZero();
  A1_dyn.setRandom();
  B1_dyn.setRandom(); B2_dyn.setRandom();
  result_dyn.setRandom();

  std::chrono::high_resolution_clock::time_point t1;
  std::chrono::high_resolution_clock::time_point t2;
  std::chrono::duration<double> time_span_fixed;
  std::chrono::duration<double> time_span_dyn;

  t1 = std::chrono::high_resolution_clock::now();
  for (int i(0); i<num_iter; ++i){
    result = A1 * (B1 + B2);
    result += result;

    A1.setRandom();
    B1.setRandom(); B2.setRandom();
    result.setRandom();

  }
  t2 = std::chrono::high_resolution_clock::now();
  time_span_fixed = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);


  t1 = std::chrono::high_resolution_clock::now();
  for (int i(0); i<num_iter; ++i){
    result_dyn = A1_dyn * (B1_dyn + B2_dyn);
    result_dyn += result_dyn;

    A1_dyn.setRandom();
    B1_dyn.setRandom(); B2_dyn.setRandom();
    result_dyn.setRandom();

  }
  t2 = std::chrono::high_resolution_clock::now();

  time_span_dyn = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);

  std::cout << "All fixed size matrix: " << time_span_fixed.count()*1000.0 << "ms."<<std::endl;;
  std::cout << "All dynamic size matrix: " << time_span_dyn.count()*1000.0 << "ms."<<std::endl;;

}
