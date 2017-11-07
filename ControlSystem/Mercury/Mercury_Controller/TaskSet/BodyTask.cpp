#include "BodyTask.hpp"
#include <Configuration.h>
#include <StateProvider.hpp>
#include <Robot_Model/RobotModel.hpp>

#include <Utils/utilities.hpp>

BodyTask::BodyTask():WBDC_Task(4),
                     Kp_(100.0),
                     Kd_(10.0)
{
  sp_ = StateProvider::GetStateProvider();
  model_ = RobotModel::GetRobotModel();
  Jt_ = sejong::Matrix::Zero(dim_task_, NUM_QDOT);
  printf("[Body Task] Constructed\n");
}

BodyTask::~BodyTask(){}


bool BodyTask::_UpdateCommand(void* pos_des,
                              const sejong::Vector & vel_des,
                              const sejong::Vector & acc_des){
  sejong::Vector* pos_cmd = (sejong::Vector*)pos_des;
  sejong::Vect3 com_pos, com_vel;
  model_->getCoMPosition(sp_->Q_, com_pos);
  model_->getCoMVelocity(sp_->Q_, sp_->Qdot_, com_vel);
  sejong::Vector pos(dim_task_);
  sejong::Vector vel(dim_task_);
  op_cmd_ = sejong::Vector::Zero(dim_task_);

  // CoM Height
  op_cmd_[0] = acc_des[0] + Kp_ * ((*pos_cmd)[0] - com_pos[2]) + Kd_ * (vel_des[0] - com_vel[2]);

  // Orientation
  for(int i(0); i<3; ++i){
    op_cmd_[i+1] = acc_des[i+1] + Kp_ * 0. + Kd_ * (vel_des[i+1] - sp_->Qdot_[i+3]);
  }

  sejong::pretty_print(op_cmd_, std::cout, "op cmd");
  sejong::pretty_print(*pos_cmd, std::cout, "pos cmd");
  sejong::pretty_print(com_pos, std::cout, "body pos");
  return true;
}

bool BodyTask::_UpdateTaskJacobian(){
  sejong::Matrix Jbody, Jcom, Jfoot;

  model_->getCoMJacobian(sp_->Q_, Jcom);
  model_->getFullJacobian(sp_->Q_, sp_->stance_foot_, Jfoot);

  // TODO
  Jt_.block(0,0, 1, NUM_QDOT) = Jcom.block(2,0,1, NUM_QDOT) - Jfoot.block(5, 0, 1, NUM_QDOT);
  Jt_(1, 3) = 1.;
  Jt_(2, 4) = 1.;
  Jt_(3, 5) = 1.;
  sejong::pretty_print(Jt_, std::cout, "Jt Body");
  return true;
}

bool BodyTask::_UpdateTaskJDotQdot(){
  sejong::Matrix Jdot;
  // model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, SJLinkID::LK_Body, Jdot);
  JtDotQdot_ = Jdot * sp_->Qdot_;
  JtDotQdot_ = sejong::Vector::Zero(dim_task_);
  return true;
}
