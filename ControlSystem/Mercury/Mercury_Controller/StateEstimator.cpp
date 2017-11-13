#include "StateEstimator.hpp"
#include "StateProvider.hpp"
#include <Utils/utilities.hpp>
#include <Robot_Model/RobotModel.hpp>
#include <Filter/filters.hpp>

StateEstimator::StateEstimator(): jvel_filter_(NUM_ACT_JOINT){
  sp_ = StateProvider::GetStateProvider();
  robot_model_ = RobotModel::GetRobotModel();

  for(int i(0); i<NUM_ACT_JOINT; ++i){
    jvel_filter_[i] = new digital_lp_filter(2*M_PI*200, SERVO_RATE);
  }
}

StateEstimator::~StateEstimator(){
}

void StateEstimator::Initialization(_DEF_SENSOR_DATA_){
  sp_->Q_.setZero();
  sp_->Qdot_.setZero();
  sp_->Q_[NUM_QDOT] = 1.;

  // Joint Set
  for (int i(0); i<NUM_ACT_JOINT; ++i){
    sp_->Q_[NUM_VIRTUAL + i] = jpos[i];
    sp_->Qdot_[NUM_VIRTUAL + i] = jvel[i];
  }
  // IMU
  for(int i(0);i<3; ++i){
    sp_->Qdot_[i + 3] = imu_ang_vel[i];
  }
  sp_->body_ori_.w() = 1.;
  sp_->body_ori_.x() = 0.;
  sp_->body_ori_.y() = 0.;
  sp_->body_ori_.z() = 0.;
  // Local Frame Setting
  sejong::Vect3 foot_pos, foot_vel;
  robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
  robot_model_->getPosition(sp_->Q_, sp_->stance_foot_, foot_pos);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, sp_->stance_foot_, foot_vel);
  sp_->Q_[0] = -foot_pos[0];
  sp_->Q_[1] = -foot_pos[1];
  sp_->Q_[2] = -foot_pos[2];
  sp_->Qdot_[0] = -foot_vel[0];
  sp_->Qdot_[1] = -foot_vel[1];
  sp_->Qdot_[2] = -foot_vel[2];

  sp_->global_pos_local_.head(2) = foot_pos.head(2);
  robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
  sp_->SaveCurrentData();
}

void StateEstimator::Update(_DEF_SENSOR_DATA_){
  sp_->Q_.setZero();
  sp_->Qdot_.setZero();
  sp_->Q_[NUM_QDOT] = 1.;

  for (int i(0); i<NUM_ACT_JOINT; ++i){
    sp_->Q_[NUM_VIRTUAL + i] = jpos[i];
    jvel_filter_[i]->input(jpos[i]);
    sp_->Qdot_[NUM_VIRTUAL + i] = jvel[i];
    // sp_->Qdot_[NUM_VIRTUAL + i] = jvel_filter_[i]->output();
  }
  // Orientation
  sejong::Quaternion delt_quat;
  sejong::Vect3 delta_th;
  double theta(0.);
  for(int i(0); i<3; ++i){
    delta_th[i] = imu_ang_vel[i] * SERVO_RATE;
    theta += delta_th[i] * delta_th[i];
  }

  delt_quat.w() = cos(theta/2.);
  delt_quat.x() = sin(theta/2.) * delta_th[0]/theta;
  delt_quat.y() = sin(theta/2.) * delta_th[1]/theta;
  delt_quat.z() = sin(theta/2.) * delta_th[2]/theta;

  sp_->body_ori_ = sejong::QuatMultiply(sp_->body_ori_, delt_quat);
  // sejong::pretty_print(sp_->body_ori_, std::cout, "body ori");

  sp_->Q_[3] = sp_->body_ori_.x();
  sp_->Q_[4] = sp_->body_ori_.y();
  sp_->Q_[5] = sp_->body_ori_.z();
  sp_->Q_[NUM_QDOT] = sp_->body_ori_.w();


  sejong::Quaternion imu_ang_quat;
  imu_ang_quat.w() = 0.;
  imu_ang_quat.x() = imu_ang_vel[0];
  imu_ang_quat.y() = imu_ang_vel[1];
  imu_ang_quat.z() = imu_ang_vel[2];

  sejong::Quaternion quat_dot = sejong::QuatMultiply(sp_->body_ori_, imu_ang_quat, false);
  quat_dot = sejong::QuatMultiply(quat_dot, sp_->body_ori_.inverse(), false);
  // sejong::pretty_print(imu_ang_quat, std::cout, "imu ang vel");
  // sejong::pretty_print(quat_dot, std::cout, "global quat dot");

  sp_->Qdot_[3] = quat_dot.x();
  sp_->Qdot_[4] = quat_dot.y();
  sp_->Qdot_[5] = quat_dot.z();

  // Foot position based offset
  sejong::Vect3 foot_pos, foot_vel;
  robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
  robot_model_->getPosition(sp_->Q_, sp_->stance_foot_, foot_pos);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, sp_->stance_foot_, foot_vel);
  sp_->Q_[0] = -foot_pos[0];
  sp_->Q_[1] = -foot_pos[1];
  sp_->Q_[2] = -foot_pos[2];
  sp_->Qdot_[0] = -foot_vel[0];
  sp_->Qdot_[1] = -foot_vel[1];
  sp_->Qdot_[2] = -foot_vel[2];

  robot_model_->UpdateModel(sp_->Q_, sp_->Qdot_);
  sp_->SaveCurrentData();
}
