#include "OriEst_IMU.hpp"

OriEst_IMU::OriEst_IMU(){
  ori_imu_.w() = 1.;

}


void OriEst_IMU::EstimatorInitialization(const std::vector<double> & acc,
                                         const std::vector<double> & ang_vel){
  ori_imu_.w() = 1.;
  ori_imu_.x() = 0.;
  ori_imu_.y() = 0.;
  ori_imu_.z() = 0.;

  for(int i(0); i<3; ++i)  global_ang_vel[i] = ang_vel[i];
  
}
void OriEst_IMU::setSensorData(const std::vector<double> & acc,
                               const std::vector<double> & ang_vel){
  
}
void OriEst_IMU::getEstimatedState(sejong::Quaternion & ori,
                                   sejong::Vect3 & global_ang_vel){
  
}
