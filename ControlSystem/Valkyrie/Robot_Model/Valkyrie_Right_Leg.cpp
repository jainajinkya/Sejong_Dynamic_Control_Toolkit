#include "Valkyrie_Right_Leg.hpp"
#include <rbdl/urdfreader.h>
#include <Configuration.h>

using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics;

Valkyrie_Right_Leg::Valkyrie_Right_Leg(){
  model_ = new Model();

  if (!Addons::URDFReadFromFile (THIS_COM"ControlSystem/Valkyrie/Valkyrie_urdf/r5_urdf_right_leg.urdf", model_, false)) {
    std::cerr << "Error loading model ./r5_urdf_right_leg.urdf" << std::endl;
    abort();
  }
  bodyid_ = model_->GetBodyId("rightCOP_Frame");
  printf("[Valkyrie Right Leg Model] Constructed\n ");
}

Valkyrie_Right_Leg::~Valkyrie_Right_Leg(){
  delete model_;
}

void Valkyrie_Right_Leg::UpdateKinematics(const sejong::Vector & q, const sejong::Vector & qdot ){
  UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
}

Valkyrie_Right_Leg* Valkyrie_Right_Leg::GetValkyrieRightLeg(){
  static Valkyrie_Right_Leg valkyrie_right_leg;
  return & valkyrie_right_leg;
}

void Valkyrie_Right_Leg::getRightFootPosition(sejong::Vect3 & lfoot_pos){
  VectorNd q_dummy;
  lfoot_pos = CalcBodyToBaseCoordinates(*model_, q_dummy, bodyid_, model_->mFixedBodies[bodyid_ - model_->fixed_body_discriminator].mCenterOfMass, false);
}

void Valkyrie_Right_Leg::getRightFootVelocity(sejong::Vect3 & lfoot_vel){
  VectorNd q_dummy;

  lfoot_vel = CalcPointVelocity ( *model_, q_dummy, q_dummy, bodyid_, model_->mFixedBodies[bodyid_ -  model_->fixed_body_discriminator].mCenterOfMass, false);
}

void Valkyrie_Right_Leg::getRightFootAngVel(sejong::Vect3 & ang_vel){
  VectorNd q_dummy;

  sejong::Vector vel = CalcPointVelocity6D(*model_, q_dummy, q_dummy, bodyid_,
                                           model_->mFixedBodies[bodyid_ -  model_->fixed_body_discriminator].mCenterOfMass, false);
  ang_vel = vel.head(3);
}

void Valkyrie_Right_Leg::getRightFootOrientation(sejong::Quaternion & ori){
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
