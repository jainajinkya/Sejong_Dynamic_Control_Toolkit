#include "Mercury.hpp"
#include <time.h>
#include <iostream>
#include <stdio.h>
#include "Configuration.h"

Mercury::Mercury(const  Vec3 & location, BASELINKTYPE base_link_type, srJoint::ACTTYPE joint_type):SystemGenerator(){
  BuildRobot(location, base_link_type, joint_type, "urdf/mercury.urdf");
  printf("[Mercury] END of Mercury assemble\n");
}

void Mercury::_SetCollision(){
}

void Mercury::_SetInitialConf()
{
  for(int i(0);i<3; ++i){
    vp_joint_[i]->m_State.m_rValue[0] = 0.;
    vr_joint_[i]->m_State.m_rValue[0] = 0.;
  }
  vp_joint_[2]->m_State.m_rValue[0] = 1.3;

  for(int i(0); i<num_r_joint_; ++i){
    r_joint_[i]->m_State.m_rValue[0] = 0.;
  }
  KIN_UpdateFrame_All_The_Entity();
}

void Mercury::_SetJointLimit(){
  
}
