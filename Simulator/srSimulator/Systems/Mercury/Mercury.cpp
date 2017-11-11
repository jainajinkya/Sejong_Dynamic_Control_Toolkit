#include "Mercury.hpp"
#include <time.h>
#include <iostream>
#include <stdio.h>
#include "Configuration.h"

Mercury::Mercury(const  Vec3 & location, BASELINKTYPE base_link_type, srJoint::ACTTYPE joint_type):SystemGenerator()
{
  BuildRobot(location, base_link_type, joint_type, "urdf/mercury.urdf");
  printf("[Mercury] END of Mercury assemble\n");
}
Mercury::~Mercury(){
}
void Mercury::_SetCollision(){
  collision_.resize(3);

  for(int i(0); i<2; ++i){
    collision_[i] = new srCollision();
    collision_[i]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
    collision_[i]->GetGeomInfo().SetDimension(0.2, 0.01, 0.01);
    collision_[i]->SetLocalFrame(EulerZYX(Vec3(0.,0.,0.), Vec3(0.,0., -0.018)));
  }

  link_[link_idx_map_.find("rfoot")->second]->AddCollision(collision_[0]);
  link_[link_idx_map_.find("lfoot")->second]->AddCollision(collision_[1]);

  double fric(1.8);
  link_[link_idx_map_.find("rfoot")->second]->SetFriction(fric);
  link_[link_idx_map_.find("lfoot")->second]->SetFriction(fric);

  double damp(0.01);
  link_[link_idx_map_.find("rfoot")->second]->SetDamping(damp);
  link_[link_idx_map_.find("lfoot")->second]->SetDamping(damp);

  double restit(0.0);
  link_[link_idx_map_.find("rfoot")->second]->SetRestitution(restit);
  link_[link_idx_map_.find("lfoot")->second]->SetRestitution(restit);


  // Link
  collision_[2] = new srCollision();
  collision_[2]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
  collision_[2]->GetGeomInfo().SetDimension(0.05, 0.1, 0.05);
  collision_[2]->SetLocalFrame(EulerZYX(Vec3(0.,0., 0.), Vec3(0., 0., -0.80)));
  link_[link_idx_map_.find("body")->second]->AddCollision(collision_[2]);
  link_[link_idx_map_.find("body")->second]->SetFriction(fric);

}

void Mercury::_SetInitialConf()
{
  for(int i(0);i<3; ++i){
    vp_joint_[i]->m_State.m_rValue[0] = 0.;
    vr_joint_[i]->m_State.m_rValue[0] = 0.;
  }
  // 1: similar to actual
  // 3: stand up ready to walk
  int pos(1);

  switch (pos){
  case 0:
    vp_joint_[2]->m_State.m_rValue[0] = 1.3;

    for(int i(0); i<num_r_joint_; ++i){
      r_joint_[i]->m_State.m_rValue[0] = 0.;
    }
    break;
  case 1:
    vp_joint_[2]->m_State.m_rValue[0] = 0.85;

    for(int i(0); i<2; ++i){
      r_joint_[4*i]->m_State.m_rValue[0] = 0.;
      r_joint_[4*i + 1]->m_State.m_rValue[0] = -0.95;
      r_joint_[4*i + 2]->m_State.m_rValue[0] = 1.85;
      r_joint_[4*i + 3]->m_State.m_rValue[0] = -1.0;
    }
    break;

  case 2:
    vp_joint_[2]->m_State.m_rValue[0] = 1.3;
    //right abduction
    r_joint_[0]->m_State.m_rValue[0] = -0.2;
    //right hip
    r_joint_[1]->m_State.m_rValue[0] = -1.0;
    //right knee
    r_joint_[2]->m_State.m_rValue[0] = 1.8;
    //right ankle
    r_joint_[3]->m_State.m_rValue[0] = 0.;

    //left abduction
    r_joint_[4]->m_State.m_rValue[0] = 0.05;
    //left hip
    r_joint_[5]->m_State.m_rValue[0] = 0.3;
    //left knee
    r_joint_[6]->m_State.m_rValue[0] = 1.0;
    //left ankle
    r_joint_[7]->m_State.m_rValue[0] = 0.;

  case 3:
    vp_joint_[2]->m_State.m_rValue[0] = 1.05;
    //right abduction
    r_joint_[0]->m_State.m_rValue[0] = 0.0;
    //right hip
    r_joint_[1]->m_State.m_rValue[0] = -0.6;
    //right knee
    r_joint_[2]->m_State.m_rValue[0] = 1.2;
    //right ankle
    r_joint_[3]->m_State.m_rValue[0] = -0.6;

    //left abduction
    r_joint_[4]->m_State.m_rValue[0] = 0.0;
    //left hip
    r_joint_[5]->m_State.m_rValue[0] = -0.6;
    //left knee
    r_joint_[6]->m_State.m_rValue[0] = 1.2;
    //left ankle
    r_joint_[7]->m_State.m_rValue[0] = -0.6;

    break;

  }
  KIN_UpdateFrame_All_The_Entity();
}

void Mercury::_SetJointLimit(){
  
}
