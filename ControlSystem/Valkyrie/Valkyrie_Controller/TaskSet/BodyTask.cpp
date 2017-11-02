#include "BodyTask.hpp"
#include <Configuration.h>
#include <StateProvider.hpp>
#include <Robot_Model/RobotModel.hpp>

#include <Utils/utilities.hpp>

BodyTask::BodyTask(int dim):WBDC_Task(dim),
                            Kp_(100.0),
                            Kd_(10.0)
{
  sp_ = StateProvider::GetStateProvider();
  model_ = RobotModel::GetRobotModel();

  // printf("[Body Task] Constructed\n");
}

BodyTask::~BodyTask(){}

bool BodyTask::_UpdateCommand(void* pos_des,
                              const sejong::Vector & vel_des,
                              const sejong::Vector & acc_des){
  sejong::Vect3 pos, vel;
  model_->getPosition(sp_->Q_, LK_torso, pos);
  model_->getVelocity(sp_->Q_, sp_->Qdot_, LK_torso, vel);

  sejong::Vect3* pos_cmd = (sejong::Vect3*)pos_des;
  op_cmd_ = acc_des + Kp_ * (*pos_cmd - pos) + Kd_ * (vel_des - vel);

  // sejong::pretty_print(op_cmd_, std::cout, "op cmd");
  // sejong::pretty_print(*pos_cmd, std::cout, "pos cmd");
  // sejong::pretty_print(sp_->Body_pos_, std::cout, "body pos");
  return true;
}

bool BodyTask::_UpdateTaskJacobian(){
  sejong::Matrix Jbody, Jfoot;

  model_->getFullJacobian(sp_->Q_, SJLinkID::LK_torso, Jbody);
  // model_->getFullJacobian(sp_->Q_, SJLinkID::LK_foot, Jfoot);
  // Jt_ = Jbody - Jfoot;
  Jt_ = Jbody;
  return true;
}

bool BodyTask::_UpdateTaskJDotQdot(){
  sejong::Matrix Jdot;
  model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, SJLinkID::LK_torso, Jdot);
  JtDotQdot_ = Jdot * sp_->Qdot_;

  return true;
}
