#include "BodyTask.hpp"
#include <Configuration.h>
#include <StateProvider.hpp>
#include <Mercury_Model/MercuryModel.hpp>

#include <Utils/utilities.hpp>

BodyTask::BodyTask(int dim):WBDC_Task(dim),
                            Kp_(100.0),
                            Kd_(10.0)
{
  sp_ = StateProvider::GetStateProvider();
  model_ = MercuryModel::GetMercuryModel();

  // printf("[Body Task] Constructed\n");
}

BodyTask::~BodyTask(){}

bool BodyTask::_UpdateCommand(void* pos_des,
                              const sejong::Vector & vel_des,
                              const sejong::Vector & acc_des){
  sejong::Vect3* pos_cmd = (sejong::Vect3*)pos_des;
  op_cmd_ = acc_des + Kp_ * (*pos_cmd - sp_->Body_pos_) + Kd_ * (vel_des - sp_->Body_vel_);

  // sejong::pretty_print(op_cmd_, std::cout, "op cmd");
  // sejong::pretty_print(*pos_cmd, std::cout, "pos cmd");
  // sejong::pretty_print(sp_->Body_pos_, std::cout, "body pos");
  return true;
}

bool BodyTask::_UpdateTaskJacobian(){
  sejong::Matrix Jbody, Jfoot;

  model_->getFullJacobian(sp_->Q_, SJLinkID::LK_Body, Jbody);
  model_->getFullJacobian(sp_->Q_, SJLinkID::LK_RFOOT, Jfoot);
  Jt_ = Jbody - Jfoot;
  return true;
}

bool BodyTask::_UpdateTaskJDotQdot(){
  sejong::Matrix Jdot;
  model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, SJLinkID::LK_Body, Jdot);
  JtDotQdot_ = Jdot * sp_->Qdot_;

  return true;
}
