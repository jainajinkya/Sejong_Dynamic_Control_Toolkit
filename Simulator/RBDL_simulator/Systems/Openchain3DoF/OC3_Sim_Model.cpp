#include "OC3_Sim_Model.hpp"
#include <Utils/utilities.hpp>
#include <stdio.h>
#include <Configuration.h>
#include <RBDL_Sim_Configuration.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

OC3_Sim_Model* OC3_Sim_Model::GetOC3_Sim_Model(){
  static OC3_Sim_Model oc3_sim_model_;
  return & oc3_sim_model_;
}

OC3_Sim_Model::OC3_Sim_Model(){
  model_ = new Model();
  rbdl_check_api_version (RBDL_API_VERSION);
  model_->gravity = Vector3d (0.,0.,  -9.81);

  Matrix3d inertia;
  Vector3d com_pos, link_length;
  com_pos.setZero(); link_length.setZero();
  Joint joint_ry = Joint (JointTypeRevolute, Vector3d (0., 1., 0.) );
  Body link1, link2, link3, j2position, j3position, link_ee;

  //////////////////////////////////////////////////////
  ///                   Parameters                   ///
  //////////////////////////////////////////////////////
  double mass (1.7);
  com_pos[0] = 0.15;
  link_length[0] = 0.3;
  // xx, yy, zz, xy, yz, zx
  inertia <<
    0.001, 0.0, 0.0,
    0.0, 0.2, 0.0,
    0.0, 0.0, 0.2;

  //////////////////////////////////////////////////////
  ///                 Assemble Model                 ///
  //////////////////////////////////////////////////////

  // link 1
  link1 = Body (mass, com_pos, inertia);
  int link1_id = model_->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_ry, link1, "link1");
  printf("link 1 id: %i \n", link1_id);

  // link 2
  link2 = Body (mass, com_pos, inertia);
  int link2_id = model_->AddBody(link1_id, Xtrans(link_length), joint_ry, link1, "link2");
  printf("link 2 id: %i \n", link2_id);

  // link 3
  link3 = Body (mass, com_pos, inertia);
  int link3_id = model_->AddBody(link2_id, Xtrans(link_length), joint_ry, link1, "link3");
  printf("link 3 id: %i \n", link2_id);

  //////////////////////////////////////////////////////
  // Fixed Joint Assemble
  Joint fixed_joint = Joint(JointTypeFixed);
  com_pos.setZero();
  Vector3d gyration_radii; gyration_radii.setZero();
  double mass_zero(0.000000001);

  // Fixed Joint (J2)
  j2position = Body(mass_zero, com_pos, gyration_radii);
  int j2_id = model_->AddBody(link1_id, Xtrans(link_length), fixed_joint, link_ee, "j2_pos");
  printf("link j2 pos id: %i \n", j2_id);

  // Fixed Joint (J3)
  j3position = Body(mass_zero, com_pos, gyration_radii);
  int j3_id = model_->AddBody(link2_id, Xtrans(link_length), fixed_joint, link_ee, "j3_pos");
  printf("link j3 pos id: %i \n", j3_id);

  // Fixed Joint (EE)
  link_ee = Body(mass_zero, com_pos, gyration_radii);
  int ee_id = model_->AddBody(link3_id, Xtrans(link_length), fixed_joint, link_ee, "link_ee");
  printf("link ee id: %i \n", ee_id);

  //////////////////////////////////////////////////////
  ///            End of Assemble Model               ///
  //////////////////////////////////////////////////////
  printf("[OC3 Model] Contructed\n");
}

OC3_Sim_Model::~OC3_Sim_Model(){
  delete model_;
}
void OC3_Sim_Model::UpdateModel(const Vector & q, const Vector & qdot){
  UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
  // Mass Matrix
  A_ = Matrix::Zero(model_->qdot_size, model_->qdot_size);
  CompositeRigidBodyAlgorithm(*model_, q, A_, false);

  Vector ZeroQdot = Vector::Zero(model_->qdot_size);
  // Gravity
  Vector grav_tmp = sejong::Vector::Zero(model_->qdot_size);
  InverseDynamics(*model_, q, ZeroQdot, ZeroQdot, grav_tmp);
  grav_ = grav_tmp;
  // Coriolis
  Vector coriolis_tmp = sejong::Vector::Zero(model_->qdot_size);
  InverseDynamics(*model_, q, qdot, ZeroQdot, coriolis_tmp);

  coriolis_ = coriolis_tmp - grav_;
}


bool OC3_Sim_Model::getMassInertia(sejong::Matrix & A) {
  A = A_;
  return true;
}

bool OC3_Sim_Model::getGravity(Vector & grav) {
  grav = grav_;
  return true;
}

bool OC3_Sim_Model::getCoriolis(Vector & coriolis) {
  coriolis = coriolis_;
  return true;
}

void OC3_Sim_Model::getFullJacobian(int link_id, sejong::Matrix & J) const {
  Matrix Jtmp = Matrix::Zero(6, model_->qdot_size);
  sejong::Vector q;

  unsigned int bodyid = _find_body_idx(link_id);
  Vector3d zero_vector = Vector3d::Zero();

  if(bodyid >=model_->fixed_body_discriminator){
    CalcPointJacobian6D(*model_, q, bodyid,
                        model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass,
                        Jtmp, false);
  }
  else{
    CalcPointJacobian6D(*model_, q, bodyid,
                        model_->mBodies[bodyid].mCenterOfMass,
                        Jtmp, false);
  }
  // X, Z, Ry
  J.block(0,0, 1, NUM_QDOT) = Jtmp.block(3, 0, 1, NUM_QDOT); // X
  J.block(1,0, 1, NUM_QDOT) = Jtmp.block(5, 0, 1, NUM_QDOT); // Z
  J.block(2,0, 1, NUM_QDOT) = Jtmp.block(1, 0, 1, NUM_QDOT); // Ry
}

void OC3_Sim_Model::getFullJacobianDot(int link_id, sejong::Matrix & Jdot) const {
  sejong::Vector q, qdot;

  Matrix Jdot_analytic = Matrix::Zero(6, model_->qdot_size);
  unsigned int bodyid = _find_body_idx(link_id);

  if(bodyid >=model_->fixed_body_discriminator){
    CalcPointJacobianDot(*model_, q, qdot, bodyid,
                         model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass,
                         Jdot_analytic, true);
  }
  else{
    CalcPointJacobianDot(*model_, q, qdot, bodyid,
                         model_->mBodies[bodyid].mCenterOfMass,
                         Jdot_analytic, true);
  }

  // X, Z, Ry
  Jdot.block(0,0, 1, NUM_QDOT) = Jdot_analytic.block(3, 0, 1, NUM_QDOT); // X
  Jdot.block(1,0, 1, NUM_QDOT) = Jdot_analytic.block(5, 0, 1, NUM_QDOT); // Z
  Jdot.block(2,0, 1, NUM_QDOT) = Jdot_analytic.block(1, 0, 1, NUM_QDOT); // Ry
}

void OC3_Sim_Model::getPos(int link_id, Vect3 & pos) {
  sejong::Vector q;

  // X, Z, Ry
  sejong::Vect3 pos_tmp;
  // Position
  Vector3d offset;
  int bodyid = _find_body_idx(link_id);
  if(bodyid >=model_->fixed_body_discriminator){
    offset = model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass;
  }
  else{
    offset =  model_->mBodies[bodyid].mCenterOfMass;
  }

  pos_tmp = CalcBodyToBaseCoordinates(*model_, q, _find_body_idx(link_id), offset, false);

  // Orientation
  Matrix3d R = CalcBodyWorldOrientation( *model_, q, _find_body_idx(link_id), false);
  sejong::Quaternion ori;
  ori = R.transpose();
  if(ori.w() < 0.){
    ori.w() *= (-1.);
    ori.x() *= (-1.);
    ori.y() *= (-1.);
    ori.z() *= (-1.);
  }

  pos[0] = pos_tmp[0];
  pos[1] = pos_tmp[2];
  pos[2] = 2. * asin(ori.y());
}

void OC3_Sim_Model::getVel(int link_id, Vect3 & vel) {
  sejong::Vector q, qdot;

  Vector3d offset;
  int bodyid = _find_body_idx(link_id);

  Vector vel_tmp;
  if(bodyid >=model_->fixed_body_discriminator){
    vel_tmp = CalcPointVelocity6D(*model_, q, qdot, bodyid,
                                  model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass, false);
  }
  else{
    vel_tmp = CalcPointVelocity6D(*model_, q, qdot, bodyid,
                              model_->mBodies[bodyid].mCenterOfMass, false);
  }
  vel[0] = vel_tmp[3];
  vel[1] = vel_tmp[5];
  vel[2] = vel_tmp[1];
}

unsigned int OC3_Sim_Model::_find_body_idx(int id) const {
  switch(id){
  case LK_SIM_L1:
    return model_->GetBodyId("link1");
  case LK_SIM_L2:
    return model_->GetBodyId("link2");
  case LK_SIM_L3:
    return model_->GetBodyId("link3");
  case LK_SIM_J2:
    return model_->GetBodyId("j2_pos");
  case LK_SIM_J3:
    return model_->GetBodyId("j3_pos");
  case LK_SIM_EE:
    return model_->GetBodyId("link_ee");
  default:
    printf("[Warnning] Incorrect link id\n");
    return 0;
  }
}
