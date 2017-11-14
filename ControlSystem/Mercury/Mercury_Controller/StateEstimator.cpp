#include "StateEstimator.hpp"
#include "StateProvider.hpp"
#include <Utils/utilities.hpp>
#include <Robot_Model/RobotModel.hpp>
#include <Filter/filters.hpp>
// #include <StateEstimator/OriEstAccObs.hpp>
#include <StateEstimator/BasicAccumulation.hpp>


StateEstimator::StateEstimator(){
  sp_ = StateProvider::GetStateProvider();
  robot_model_ = RobotModel::GetRobotModel();

  ori_est_ = new BasicAccumulation();
}

StateEstimator::~StateEstimator(){
  delete ori_est_;
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
  ori_est_->EstimatorInitialization(imu_acc, imu_ang_vel);

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
    sp_->Qdot_[NUM_VIRTUAL + i] = jvel[i];
  }
  ori_est_->setSensorData(imu_acc, imu_ang_vel, imu_ang_vel);
  ori_est_->getEstimatedState(sp_->body_ori_, sp_->body_ang_vel_);

  sp_->Q_[3] = sp_->body_ori_.x();
  sp_->Q_[4] = sp_->body_ori_.y();
  sp_->Q_[5] = sp_->body_ori_.z();
  sp_->Q_[NUM_QDOT] = sp_->body_ori_.w();

  for(int i(0); i<3; ++i)
    sp_->Qdot_[i+3] = sp_->body_ang_vel_[i];

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
