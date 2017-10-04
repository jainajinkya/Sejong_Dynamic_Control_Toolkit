#include "Valkyrie_Left_Leg.h"
#include <rbdl/urdfreader.h>
#include <Configuration.h>
#include <stdio.h>

using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics;

Valkyrie_Left_Leg::Valkyrie_Left_Leg(){
  model_ = new Model();

  if (!Addons::URDFReadFromFile (ModelPath"urdf/r5_urdf_left_leg.urdf", model_, false)) {
    std::cerr << "Error loading model ./r5_urdf_left_leg.urdf" << std::endl;
    abort();
  }
  bodyid_ = model_->GetBodyId("leftCOP_Frame");
  printf("[Valkyrie Left Leg Model] Contructed\n");
}

Valkyrie_Left_Leg::~Valkyrie_Left_Leg(){
  delete model_;
}

void Valkyrie_Left_Leg::UpdateKinematics(const sejong::Vector & q, const sejong::Vector & qdot ){
  UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
}

Valkyrie_Left_Leg* Valkyrie_Left_Leg::GetValkyrieLeftLeg(){
  static Valkyrie_Left_Leg valkyrie_left_leg_;
  return & valkyrie_left_leg_;
}

void Valkyrie_Left_Leg::getLeftFootPosition(sejong::Vect3 & lfoot_pos){
  VectorNd q_dummy;
  // lfoot_pos = CalcBodyToBaseCoordinates(*model_, q_dummy, bodyid_, model_->mBodies[bodyid_].mCenterOfMass, false);
  lfoot_pos = CalcBodyToBaseCoordinates(*model_, q_dummy, bodyid_, model_->mFixedBodies[bodyid_ - model_->fixed_body_discriminator].mCenterOfMass, false);
}

void Valkyrie_Left_Leg::getLeftFootVelocity(sejong::Vect3 & lfoot_vel){
  VectorNd q_dummy;

  lfoot_vel = CalcPointVelocity ( *model_, q_dummy, q_dummy, bodyid_, model_->mFixedBodies[bodyid_ - model_->fixed_body_discriminator].mCenterOfMass, false);
}

void Valkyrie_Left_Leg::getLeftFootAngVel(sejong::Vect3 & ang_vel){
  VectorNd q_dummy;

  sejong::Vector vel = CalcPointVelocity6D(*model_, q_dummy, q_dummy, bodyid_,
                                           model_->mFixedBodies[bodyid_ - model_->fixed_body_discriminator].mCenterOfMass, false);
  ang_vel = vel.head(3);
}

void Valkyrie_Left_Leg::getLeftFootOrientation(sejong::Quaternion & ori){
  VectorNd q_dummy;

  Matrix3d R;
  R = CalcBodyWorldOrientation( *model_, q_dummy, bodyid_, false);
  ori = R.transpose();
  if(ori.w() < 0.){
    ori.w() *= (-1.);
    ori.x() *= (-1.);
    ori.y() *= (-1.);
    ori.z() *= (-1.);
  }
}
