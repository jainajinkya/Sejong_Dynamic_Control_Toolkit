#include "JPosTask.hpp"
#include <Configuration.h>
#include <StateProvider.hpp>
#include <Robot_Model/RobotModel.hpp>

#include <Utils/utilities.hpp>

JPosTask::JPosTask():WBDC_Task(NUM_ACT_JOINT),
                     Kp_(200.0),
                     Kd_(25.0)
{
  sp_ = StateProvider::GetStateProvider();
  model_ = RobotModel::GetRobotModel();
  Jt_ = sejong::Matrix(NUM_ACT_JOINT, NUM_QDOT);
  JtDotQdot_ = sejong::Vector(NUM_ACT_JOINT);
  // printf("[JPos Task] Constructed\n");
}

JPosTask::~JPosTask(){}

bool JPosTask::_UpdateCommand(void* pos_des,
                              const sejong::Vector & vel_des,
                              const sejong::Vector & acc_des){
  sejong::Vector* pos_cmd = (sejong::Vector*)pos_des;

  for(int i(0); i<NUM_ACT_JOINT; ++i){
    op_cmd_[i] = acc_des[i] + Kp_ * ((*pos_cmd)[i] - sp_->Q_[NUM_VIRTUAL + i]) + Kd_ * (vel_des[i] - sp_->Qdot_[NUM_VIRTUAL + i]);
  }
  // sejong::pretty_print(op_cmd_, std::cout, "op cmd");
  // sejong::pretty_print(*pos_cmd, std::cout, "pos cmd");
  // sejong::pretty_print(sp_->JPos_pos_, std::cout, "body pos");
  return true;
}

bool JPosTask::_UpdateTaskJacobian(){
  Jt_.setZero();
  (Jt_.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT)).setIdentity();

  return true;
}

bool JPosTask::_UpdateTaskJDotQdot(){
  JtDotQdot_.setZero();
  return true;
}
