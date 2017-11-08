#include "BodyTask.hpp"
#include <Configuration.h>
#include <StateProvider.hpp>
#include <Robot_Model/RobotModel.hpp>

#include <Utils/utilities.hpp>

BodyTask::BodyTask():WBDC_Task(4),
                     Kp_(20.0),
                     Kd_(5.0)
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
  sejong::Quaternion curr_quat;
  curr_quat.w() = sp_->Q_[NUM_QDOT];
  curr_quat.x() = sp_->Q_[3];
  curr_quat.y() = sp_->Q_[4];
  curr_quat.z() = sp_->Q_[5];

  sejong::Quaternion des_quat;
  des_quat.w() = (*pos_cmd)[1];
  des_quat.x() = (*pos_cmd)[2];
  des_quat.y() = (*pos_cmd)[3];
  des_quat.z() = (*pos_cmd)[4];

  sejong::Quaternion err_quat = sejong::QuatMultiply(des_quat, curr_quat.inverse());
  // sejong::Quaternion err_quat = sejong::QuatMultiply(curr_quat, des_quat);

  sejong::Vect3 ori_err;
  sejong::convert(err_quat, ori_err);

  double Kp_ori(300.0);
  double Kd_ori(15.0);

  for(int i(0); i<3; ++i){
    op_cmd_[i+1] = acc_des[i+1] + Kp_ori * ori_err[i] + Kd_ori * (vel_des[i+1] - sp_->Qdot_[i+3]);
    // op_cmd_[i+1] = acc_des[i+1] + Kd_ * (vel_des[i+1] - sp_->Qdot_[i+3]);

  }

  // sejong::pretty_print(op_cmd_, std::cout, "op cmd");
  // sejong::pretty_print(*pos_cmd, std::cout, "pos cmd");
  // sejong::pretty_print(com_pos, std::cout, "body pos");
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

  return true;
}

bool BodyTask::_UpdateTaskJDotQdot(){
  sejong::Matrix Jdot;
  // model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, SJLinkID::LK_Body, Jdot);
  JtDotQdot_ = Jdot * sp_->Qdot_;
  JtDotQdot_ = sejong::Vector::Zero(dim_task_);
  return true;
}
