#include "BasicAccumulation.hpp"
#include <Configuration.h>

BasicAccumulation::BasicAccumulation():OriEstimator(){
  global_ori_.w() = 1.;
  global_ori_.x() = 0.;

}
BasicAccumulation::~BasicAccumulation(){}

void BasicAccumulation::EstimatorInitialization(const std::vector<double> & acc,
                                         const std::vector<double> & ang_vel){
  global_ori_.w() = 1.;
  global_ori_.x() = 0.;
  global_ori_.y() = 0.;
  global_ori_.z() = 0.;

  for(int i(0); i<3; ++i)  global_ang_vel_[i] = ang_vel[i];
}

void BasicAccumulation::setSensorData(const std::vector<double> & acc,
                                      const std::vector<double> & acc_inc,
                                      const std::vector<double> & ang_vel){
  // Orientation
  sejong::Quaternion delt_quat;
  sejong::Vect3 delta_th;
  double theta(0.);
  for(int i(0); i<3; ++i){
    delta_th[i] = ang_vel[i] * SERVO_RATE;
    theta += delta_th[i] * delta_th[i];
  }

  delt_quat.w() = cos(theta/2.);
  delt_quat.x() = sin(theta/2.) * delta_th[0]/theta;
  delt_quat.y() = sin(theta/2.) * delta_th[1]/theta;
  delt_quat.z() = sin(theta/2.) * delta_th[2]/theta;

  global_ori_ = sejong::QuatMultiply(global_ori_, delt_quat);

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
