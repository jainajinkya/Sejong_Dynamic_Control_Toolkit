#include "Mercury_Dyn_environment.hpp"
#include "common/utils.h"
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <ControlSystem/Mercury/Mercury_Controller/interface.hpp>
#include <ControlSystem/Mercury/Mercury_Controller/StateProvider.hpp>
#include <ParamHandler/ParamHandler.hpp>

// #define SENSOR_NOISE
#define SENSOR_DELAY 0 // Sensor_delay* SERVO_RATE (sec) = time delay 


Mercury_Dyn_environment::Mercury_Dyn_environment():
  num_substep_rendering_(15)
{
  _ParamterSetup();

  m_Mercury = new Mercury(Vec3(0.0, 0.0, 0.0), srSystem::FIXED, srJoint::TORQUE);
  m_Space = new srSpace();
  m_ground = new Ground();

  interface_ = new interface();

  m_Space->AddSystem(m_ground->BuildGround());
  m_Space->AddSystem((srSystem*)m_Mercury);
  m_Space->DYN_MODE_PRESTEP();


  m_Space->SET_USER_CONTROL_FUNCTION_2(ContolFunction);
  m_Space->SetTimestep(SERVO_RATE);
  m_Space->SetGravity(0.0,0.0,-9.8);
  m_Space->SetNumberofSubstepForRendering(num_substep_rendering_);
}

void Mercury_Dyn_environment::ContolFunction( void* _data ) {
  static int count(0);

  Mercury_Dyn_environment* pDyn_env = (Mercury_Dyn_environment*)_data;
  Mercury* robot = pDyn_env->m_Mercury;

  double alternate_time = SIM_SERVO_RATE * count;
  std::vector<double> jpos(6);
  std::vector<double> jvel(6);
  std::vector<double> jtorque(6);
  std::vector<double> imu_acc(3);
  std::vector<double> imu_ang_vel(3);
  bool rfoot_contact(false);
  bool lfoot_contact(false);
  std::vector<double> torque_command(robot->num_act_joint_, 0.);

  // IMU data
  se3 imu_se3_vel = robot->link_[robot->link_idx_map_.find("imu")->second]->GetVel();
  se3 imu_se3_acc = robot->link_[robot->link_idx_map_.find("imu")->second]->GetAcc();
  SE3 imu_frame = robot->link_[robot->link_idx_map_.find("imu")->second]->GetFrame();
  SO3 imu_ori = robot->link_[robot->link_idx_map_.find("imu")->second]->GetOrientation();

  for(int i(0); i<3; ++i){
    imu_ang_vel[i] = imu_se3_vel[i];
  }

  Eigen::Matrix3d Rot;
  Rot<<
    imu_frame(0,0), imu_frame(0,1), imu_frame(0,2),
    imu_frame(1,0), imu_frame(1,1), imu_frame(1,2),
    imu_frame(2,0), imu_frame(2,1), imu_frame(2,2);
  Eigen::Matrix<double, 3, 1> ang_vel;
  ang_vel<<imu_se3_vel[0], imu_se3_vel[1], imu_se3_vel[2];
  sejong::Vect3 global_ang_vel = Rot * ang_vel;

  bool b_printout(false);
  if(b_printout){
    printf("imu info: \n");
    std::cout<<imu_se3_vel<<std::endl;
    std::cout<<imu_se3_acc<<std::endl;
    std::cout<<imu_frame<<std::endl;

    printf("global ang vel\n");
    std::cout<<global_ang_vel<<std::endl;

    Eigen::Quaterniond quat(Rot);
    printf("quat global:\n");
    std::cout<<quat.w()<<std::endl;
    std::cout<<quat.vec()<<std::endl;
  }


  // Right
  for(int i(0); i< 3; ++i){
    jpos[i] = robot->r_joint_[i]->m_State.m_rValue[0];
    jvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
    jtorque[i] = robot->r_joint_[i]->m_State.m_rValue[3];
  }
  // Left
  for(int i(0); i< 3; ++i){
    jpos[i+3] = robot->r_joint_[i+4]->m_State.m_rValue[0];
    jvel[i+3] = robot->r_joint_[i+4]->m_State.m_rValue[1];
    jtorque[i+3] = robot->r_joint_[i+4]->m_State.m_rValue[3];
  }

  pDyn_env->interface_->GetCommand(alternate_time, jpos, jvel, jtorque, imu_acc, imu_ang_vel, rfoot_contact, lfoot_contact, torque_command);

  for(int i(0); i<3; ++i){
    robot->vp_joint_[i]->m_State.m_rCommand = 0.0;
    robot->vr_joint_[i]->m_State.m_rCommand = 0.0;
  }

  // Right
  for(int i(0); i<3; ++i){
    robot->r_joint_[i]->m_State.m_rCommand = torque_command[i];
  }
  // Left
  for(int i(0); i<3; ++i){
    robot->r_joint_[i+4]->m_State.m_rCommand = torque_command[i+3];
  }
  // Ankle Passive
  double kp(0.02);
  double kd(0.002);
  double des_pos(-0.5);
    robot->r_joint_[3]->m_State.m_rCommand = kp * (des_pos - robot->r_joint_[3]->m_State.m_rValue[0]) - kd* robot->r_joint_[3]->m_State.m_rValue[1];
  robot->r_joint_[7]->m_State.m_rCommand = kp * (des_pos - robot->r_joint_[7]->m_State.m_rValue[0]) - kd* robot->r_joint_[7]->m_State.m_rValue[1];

  // robot->r_joint_[3]->m_State.m_rCommand = 0.;
  // robot->r_joint_[7]->m_State.m_rCommand = 0.;

  pDyn_env->_FixXY();
  // std::map<std::string, int>::iterator iter = robot->r_joint_idx_map_.begin();
  // while(iter != robot->r_joint_idx_map_.end()){
  //   std::cout<<iter->first<<std::endl;
  //   ++iter;
  // }
  ++count;
}
void Mercury_Dyn_environment::_FixXY(){
  double pos,vel;

  double kp(2000.0);
  double kd(300.0);

  int idx(0);
  pos = m_Mercury->vp_joint_[idx]->m_State.m_rValue[0];
  vel = m_Mercury->vp_joint_[idx]->m_State.m_rValue[1];
  m_Mercury->vp_joint_[idx]->m_State.m_rCommand = -kp * pos - kd * vel;

  idx = 1;
  pos = m_Mercury->vp_joint_[idx]->m_State.m_rValue[0];
  vel = m_Mercury->vp_joint_[idx]->m_State.m_rValue[1];

  m_Mercury->vp_joint_[idx]->m_State.m_rCommand = -kp * pos - kd * vel;

}

void Mercury_Dyn_environment::Rendering_Fnc(){}


void Mercury_Dyn_environment::_ParamterSetup(){
  ParamHandler handler(CONFIG_PATH"sr_sim_setting.yaml");

  handler.getInteger("num_substep_rendering", num_substep_rendering_);
}
