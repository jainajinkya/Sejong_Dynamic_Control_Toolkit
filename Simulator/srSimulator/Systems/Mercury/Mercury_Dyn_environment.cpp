#include "Mercury_Dyn_environment.hpp"
#include "common/utils.h"
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <ControlSystem/Mercury/Mercury_Controller/interface.hpp>
#include <ControlSystem/Mercury/Mercury_Controller/StateProvider.hpp>

// #define Measure_Time
#ifdef Measure_Time
#include <chrono>
using namespace std::chrono;
#endif

// #define SENSOR_NOISE
#define SENSOR_DELAY 0 // Sensor_delay* SERVO_RATE (sec) = time delay 


Mercury_Dyn_environment::Mercury_Dyn_environment(){
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
  m_Space->SetNumberofSubstepForRendering(15);
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

  // std::map<std::string, int>::iterator iter = robot->r_joint_idx_map_.begin();
  // while(iter != robot->r_joint_idx_map_.end()){
  //   std::cout<<iter->first<<std::endl;
  //   ++iter;
  // }
  ++count;
}


void Mercury_Dyn_environment::Rendering_Fnc(){}
