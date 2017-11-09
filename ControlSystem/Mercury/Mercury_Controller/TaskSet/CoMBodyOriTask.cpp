#include "CoMBodyOriTask.hpp"
#include <Configuration.h>
#include <StateProvider.hpp>
#include <Robot_Model/RobotModel.hpp>

#include <Utils/utilities.hpp>

CoMBodyOriTask::CoMBodyOriTask():WBDC_Task(6),
                                 Kp_(200.0),
                                 Kd_(25.0)
{
  sp_ = StateProvider::GetStateProvider();
  model_ = RobotModel::GetRobotModel();
  Jt_ = sejong::Matrix::Zero(dim_task_, NUM_QDOT);
  // printf("[CoMBodyOri Task] Constructed\n");
}

CoMBodyOriTask::~CoMBodyOriTask(){}

bool CoMBodyOriTask::_UpdateCommand(void* pos_des,
                                    const sejong::Vector & vel_des,
                                    const sejong::Vector & acc_des){

  sejong::Vector* pos_cmd = (sejong::Vector*)pos_des;
  sejong::Vect3 com_pos, com_vel;
  model_->getCoMPosition(sp_->Q_, com_pos);
  model_->getCoMVelocity(sp_->Q_, sp_->Qdot_, com_vel);
  op_cmd_ = sejong::Vector::Zero(dim_task_);

  for(int i(0); i<3; ++i){
    op_cmd_[i] = acc_des[i] + Kp_ * ((*pos_cmd)[i] - com_pos[i]) + Kd_ * (vel_des[i] - com_vel[i]);
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

  double Kp_ori(300.0);
  double Kd_ori(30.0);

  for(int i(0); i<3; ++i){
    op_cmd_[i+3] = acc_des[i+3] + Kp_ori * ori_err[i] + Kd_ori * (vel_des[i+1] - sp_->Qdot_[i+3]);
    // op_cmd_[i+1] = acc_des[i+1] + Kd_ * (vel_des[i+1] - sp_->Qdot_[i+3]);
  }

  // sejong::pretty_print(op_cmd_, std::cout, "op cmd");
  // sejong::pretty_print(*pos_cmd, std::cout, "pos cmd");
  // sejong::pretty_print(com_pos, std::cout, "body pos");
  return true;
}

bool CoMBodyOriTask::_UpdateTaskJacobian(){
  sejong::Matrix Jbody, Jcom, Jfoot;

  model_->getCoMJacobian(sp_->Q_, Jcom);
  model_->getFullJacobian(sp_->Q_, sp_->stance_foot_, Jfoot);

  // TODO
  Jt_.block(0,0, 3, NUM_QDOT) = Jcom - Jfoot.block(3, 0, 3, NUM_QDOT);
  Jt_(3, 3) = 1.;
  Jt_(4, 4) = 1.;
  Jt_(5, 5) = 1.;
  // sejong::pretty_print(Jt_, std::cout, "Jt CoMBodyOri");
  return true;
}

bool CoMBodyOriTask::_UpdateTaskJDotQdot(){
  sejong::Matrix Jdot;
  // model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, SJLinkID::LK_CoMBodyOri, Jdot);
  // TODO
  JtDotQdot_ = Jdot * sp_->Qdot_;
  JtDotQdot_ = sejong::Vector::Zero(dim_task_);
  return true;
}
