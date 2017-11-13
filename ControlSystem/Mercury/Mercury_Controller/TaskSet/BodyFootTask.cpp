#include "BodyFootTask.hpp"
#include <Configuration.h>
#include <StateProvider.hpp>
#include <Robot_Model/RobotModel.hpp>

#include <Utils/utilities.hpp>

BodyFootTask::BodyFootTask(int swing_foot):WBDC_Task(9),
                                           swing_foot_(swing_foot)
{
  Kp_vec_ = sejong::Vector(dim_task_);
  Kd_vec_ = sejong::Vector(dim_task_);

  for(int i(0); i<dim_task_; ++i){
    Kp_vec_[i] = 100.0;
    Kd_vec_[i] = 10.0;
  }

  sp_ = StateProvider::GetStateProvider();
  model_ = RobotModel::GetRobotModel();
  Jt_ = sejong::Matrix::Zero(dim_task_, NUM_QDOT);
  // printf("[BodyFoot Task] Constructed\n");
}

BodyFootTask::~BodyFootTask(){}

bool BodyFootTask::_UpdateCommand(void* pos_des,
                                    const sejong::Vector & vel_des,
                                    const sejong::Vector & acc_des){

  sejong::Vector* pos_cmd = (sejong::Vector*)pos_des;
  sejong::Vect3 com_pos, com_vel;
  model_->getCoMPosition(sp_->Q_, com_pos);
  model_->getCoMVelocity(sp_->Q_, sp_->Qdot_, com_vel);
  op_cmd_ = sejong::Vector::Zero(dim_task_);

  for(int i(0); i<3; ++i){
    op_cmd_[i] = acc_des[i] + Kp_vec_[i] * ((*pos_cmd)[i] - com_pos[i]) + Kd_vec_[i] * (vel_des[i] - com_vel[i]);
  }

  // Orientation
  sejong::Quaternion curr_quat;
  curr_quat.w() = sp_->Q_[NUM_QDOT];
  curr_quat.x() = sp_->Q_[3];
  curr_quat.y() = sp_->Q_[4];
  curr_quat.z() = sp_->Q_[5];
  
  sejong::Quaternion des_quat;
  des_quat.w() = (*pos_cmd)[3];
  des_quat.x() = (*pos_cmd)[4];
  des_quat.y() = (*pos_cmd)[5];
  des_quat.z() = (*pos_cmd)[6];

  sejong::Quaternion err_quat = sejong::QuatMultiply(des_quat, curr_quat.inverse());

  sejong::Vect3 ori_err;
  sejong::convert(err_quat, ori_err);
  // sejong::pretty_print(err_quat, std::cout, "err quat");
  // sejong::pretty_print(ori_err, std::cout, "so3 err");

  for(int i(0); i<3; ++i){
    op_cmd_[i+3] = acc_des[i+3] + Kp_vec_[i+3] * ori_err[i] + Kd_vec_[i+3] * (vel_des[i+1] - sp_->Qdot_[i+3]);
  }

  // Foot Position
  sejong::Vect3 foot_pos, foot_vel;
  model_->getPosition(sp_->Q_, swing_foot_, foot_pos);
  model_->getVelocity(sp_->Q_, sp_->Qdot_, swing_foot_, foot_vel);

  for(int i(0); i<3; ++i){
    op_cmd_[i+6] = acc_des[i+6] + Kp_vec_[i+6] * ((*pos_cmd)[i+7] - foot_pos[i]) + Kd_vec_[i+6] * (vel_des[i+6] - foot_vel[i]);
  }

  // sejong::pretty_print(op_cmd_, std::cout, "op cmd");
  // sejong::pretty_print(*pos_cmd, std::cout, "pos cmd");
  // sejong::pretty_print(foot_pos, std::cout, "foot pos");
  // sejong::pretty_print(acc_des, std::cout, "acc des");
  // sejong::pretty_print(vel_des, std::cout, "vel des");

  return true;
}

bool BodyFootTask::_UpdateTaskJacobian(){
  sejong::Matrix Jbody, Jcom, Jfoot;

  model_->getCoMJacobian(sp_->Q_, Jcom);
  model_->getFullJacobian(sp_->Q_, sp_->stance_foot_, Jfoot);

  // TODO
  Jt_.block(0,0, 3, NUM_QDOT) = Jcom - Jfoot.block(3, 0, 3, NUM_QDOT);
  Jt_(3, 3) = 1.;
  Jt_(4, 4) = 1.;
  Jt_(5, 5) = 1.;

  sejong::Matrix Jswing;
  model_->getFullJacobian(sp_->Q_, swing_foot_, Jswing);
  Jt_.block(6, 0, 3, NUM_QDOT) = Jswing.block(3,0,3, NUM_QDOT) - Jfoot.block(3, 0, 3, NUM_QDOT);

  // sejong::pretty_print(Jswing, std::cout, "Jswing");
  // sejong::pretty_print(Jfoot, std::cout, "Jfoot");
  // sejong::pretty_print(Jt_, std::cout, "Jt BodyFoot");
  return true;
}

bool BodyFootTask::_UpdateTaskJDotQdot(){
  // TODO
  JtDotQdot_ = sejong::Vector::Zero(dim_task_);

  sejong::Matrix Jfoot_dot;
  model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, swing_foot_, Jfoot_dot);
  JtDotQdot_.tail(3) = Jfoot_dot.block(3, 0, 3, NUM_QDOT) * sp_->Qdot_;

  return true;
}

