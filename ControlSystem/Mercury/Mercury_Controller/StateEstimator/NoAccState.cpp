#include "NoAccState.hpp"
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>

NoAccState::NoAccState():OriEstimator(),
                         x_(DIM_STATE_NO_ACC_STATE - 3),
                         x_pred_(DIM_STATE_NO_ACC_STATE - 3)
{
  global_ori_.w() = 1.;
  global_ori_.x() = 0.;
  global_ori_.y() = 0.;
  global_ori_.z() = 0.;

  ori_pred_.w() = 1.;
  ori_pred_.x() = 0.;
  ori_pred_.y() = 0.;
  ori_pred_.z() = 0.;

  P_.setIdentity();
  P_pred_.setIdentity();

  x_.setZero();
  x_pred_.setZero();

  Q_.setIdentity();
  R_.setIdentity();
  // Velocity
  // Q_.block<3,3>(0,0) *= 1.0;
  // Q_.block<3,3>(3,3) *= 10.0;
  // Q_.block<3,3>(6,6) *= 0.1;
  R_*=5.0;
}


NoAccState::~NoAccState(){}

void NoAccState::EstimatorInitialization(const std::vector<double> & acc,
                                         const std::vector<double> & ang_vel){
  global_ori_.w() = 1.;
  global_ori_.x() = 0.;
  global_ori_.y() = 0.;
  global_ori_.z() = 0.;

  for(int i(0); i<3; ++i)  global_ang_vel_[i] = ang_vel[i];
}

void NoAccState::setSensorData(const std::vector<double> & acc,
                                 const std::vector<double> & acc_inc,
                                 const std::vector<double> & ang_vel){

  // Orientation
  sejong::Quaternion delt_quat;
  sejong::Vect3 delta_th;
  double theta(0.);
  for(int i(0); i<3; ++i){
    delta_th[i] = (ang_vel[i]) * SERVO_RATE;
    theta += delta_th[i] * delta_th[i];
  }

  delt_quat.w() = cos(theta/2.);
  delt_quat.x() = sin(theta/2.) * delta_th[0]/theta;
  delt_quat.y() = sin(theta/2.) * delta_th[1]/theta;
  delt_quat.z() = sin(theta/2.) * delta_th[2]/theta;

  ori_pred_ = sejong::QuatMultiply(global_ori_, delt_quat);

  for(int i(0); i<3; ++i){
    x_pred_[i] = x_[i] + acc_inc[i] * SERVO_RATE; // Velocity
  }

  // Propagate Covariance
  Eigen::Matrix3d RotMtx(global_ori_);
  F_.setIdentity();
  // sejong::pretty_print((sejong::Matrix)F_, std::cout, "F");
  P_pred_ = F_ * P_ * F_.transpose() + Q_;

  // Update Observation
  sejong::Vect3 grav; grav.setZero();
  grav[2] = 9.81;
  sejong::Vect3 local_acc = grav;
  local_acc = RotMtx.transpose() * local_acc;

  for(int i(0); i<3; ++i) {
    // Sensed
    s_[i] = acc[i];
    s_[i+3] = acc_inc[i];
    // Observation
    h_[i] = local_acc[i];
    h_[i+3] = local_acc[i];
  }
  y_ = s_ - h_;
  sejong::pretty_print((sejong::Vector)s_, std::cout, "s");
  sejong::pretty_print((sejong::Vector)h_, std::cout, "h");

  Eigen::Matrix3d a_g_skew; a_g_skew.setZero();
  sejong::Vect3 state_acc = x_.segment(3,3);
  double g(9.81);
  a_g_skew(0, 1) = -g;   a_g_skew(0, 2) = 0.;
  a_g_skew(1, 0) =  g;   a_g_skew(1, 2) = 0.;
  a_g_skew(2, 0) = 0.;   a_g_skew(2, 1) = 0.;

  H_.setZero();
  H_.block<3,3>(0,3) = RotMtx.transpose() * a_g_skew;
  H_.block<3,3>(3,3) = RotMtx.transpose() * a_g_skew;

  // sejong::pretty_print((sejong::Matrix)a_g_skew, std::cout, "ag skew");
  // sejong::pretty_print((sejong::Matrix)H_, std::cout, "H");

  sejong::Matrix S = R_ + H_ * P_pred_ * H_.transpose();
  // sejong::Matrix K = P_pred_ * H_.transpose() * S.inverse();
  sejong::Matrix Sinv;
  sejong::pseudoInverse(S, 0.0001, Sinv);
  sejong::Matrix K = P_pred_ * H_.transpose() * Sinv;

  sejong::Vector delta = K*y_;
  x_pred_ += delta.head(3);

  sejong::Vect3 delta_ori = delta.tail(3);
  sejong::Quaternion quat_delta;
  sejong::convert(delta_ori, quat_delta);
  ori_pred_ = sejong::QuatMultiply(quat_delta, ori_pred_);

  // sejong::pretty_print(x_, std::cout, "x");
  // sejong::pretty_print(S, std::cout, "S");
  // sejong::pretty_print(K, std::cout, "K");
  sejong::pretty_print(delta, std::cout, "delta");
  sejong::pretty_print(ori_pred_, std::cout, "ori updated");

  sejong::Matrix eye(DIM_STATE_NO_ACC_STATE, DIM_STATE_NO_ACC_STATE);
  eye.setIdentity();
  P_pred_ = (eye - K * H_) * P_pred_;

  // Set Angular Velocity
  _SetGlobalAngularVelocity(ang_vel);
  global_ori_ = ori_pred_;
  P_ = P_pred_;
  x_ = x_pred_;
}


void NoAccState::_SetGlobalAngularVelocity(const std::vector<double> & ang_vel){
  sejong::Quaternion ang_quat;
  ang_quat.w() = 0.;
  ang_quat.x() = ang_vel[0];
  ang_quat.y() = ang_vel[1];
  ang_quat.z() = ang_vel[2];

  sejong::Quaternion quat_dot = sejong::QuatMultiply(global_ori_, ang_quat, false);
  quat_dot = sejong::QuatMultiply(quat_dot, global_ori_.inverse(), false);

  global_ang_vel_[0] = quat_dot.x();
  global_ang_vel_[1] = quat_dot.y();
  global_ang_vel_[2] = quat_dot.z();

}
