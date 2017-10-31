#include "Walker2D_Sim_Model.hpp"
#include <Utils/utilities.hpp>
#include <stdio.h>
#include <Configuration.h>
#include <RBDL_Sim_Configuration.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Walker2D_Sim_Model* Walker2D_Sim_Model::GetWalker2D_Sim_Model(){
  static Walker2D_Sim_Model walker2d_sim_model_;
  return & walker2d_sim_model_;
}

Walker2D_Sim_Model::Walker2D_Sim_Model(){
  model_ = new Model();
  rbdl_check_api_version (RBDL_API_VERSION);
  model_->gravity = Vector3d (0.,0.,  -9.81);

  Matrix3d inertia;
  Vector3d com_pos, com_pos_body, link_length, link_length_body;
  com_pos.setZero(); link_length.setZero();
  com_pos_body.setZero(); link_length_body.setZero();
  // Joint
  Joint vjoint_x = Joint(JointTypePrismatic, Vector3d (1., 0., 0.)  );
  Joint vjoint_z = Joint(JointTypePrismatic, Vector3d (0., 0., 1.)  );
  Joint joint_ry = Joint (JointTypeRevolute, Vector3d (0., 1., 0.) );
  Joint fixed_joint = Joint(JointTypeFixed);

  Body vlk_x, vlk_z, lk_body, lk_lthigh, lk_lshank, lk_rthigh, lk_rshank, lk_lfoot, lk_rfoot, lk_hip, lk_body_ee, lk_lknee, lk_rknee;

  //////////////////////////////////////////////////////
  ///                   Parameters                   ///
  //////////////////////////////////////////////////////
  double mass (1.7);
  double mass_zero(0.000000001);
  Vector3d com_pos_zero; com_pos_zero.setZero();
  Vector3d gyration_radii_zero; gyration_radii_zero.setZero();

  com_pos[2] = -0.15;
  link_length[2] = -0.3;
  com_pos_body[2] = 0.2;
  link_length_body[2] = 0.37;
  // xx, yy, zz, xy, yz, zx
  inertia <<
    0.001, 0.0, 0.0,
    0.0, 0.2, 0.0,
    0.0, 0.0, 0.2;

  //////////////////////////////////////////////////////
  ///                 Assemble Model                 ///
  //////////////////////////////////////////////////////
  // ground to Virtual X link
  vlk_x = Body (mass_zero, com_pos_zero, gyration_radii_zero);
  int vlx_id = model_->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), vjoint_x, vlk_x, "virtual_x");
  printf("body X virtual id: %i \n", vlx_id);

  // Virtual X link to Virtual Z link
  vlk_z = Body (mass_zero, com_pos_zero, gyration_radii_zero);
  int vlz_id = model_->AddBody(vlx_id, Xtrans(Vector3d(0., 0., 0.)), vjoint_z, vlk_z, "virtual_z");
  printf("body Z virtual id: %i \n", vlz_id);

  // Virtual Z link to Body
  lk_body = Body (mass, com_pos_body, inertia);
  int body_id = model_->AddBody(vlz_id, Xtrans(Vector3d(0.,0.,0.)), joint_ry, lk_body, "body");
  printf("Body id: %i \n", body_id);

  // Body to Left Thigh
  lk_lthigh = Body (mass, com_pos, inertia);
  int lthigh_id = model_->AddBody(body_id, Xtrans(Vector3d(0.,0.,0.)),
                                  joint_ry, lk_lthigh, "leftThigh");
  printf("Left Thigh id: %i \n", lthigh_id);

  // Left Thigh to Left Shank
  lk_lshank = Body (mass, com_pos, inertia);
  int lshank_id = model_->AddBody(lthigh_id, Xtrans(link_length),
                                  joint_ry, lk_lshank, "leftShank");
  printf("Left Shank id: %i \n", lshank_id);

  // Body to Right Thigh
  lk_rthigh = Body (mass, com_pos, inertia);
  int rthigh_id = model_->AddBody(body_id, Xtrans(Vector3d(0.,0.,0.)),
                                  joint_ry, lk_rthigh, "rightThigh");
  printf("Right Thigh id: %i \n", rthigh_id);

  // Right Thigh to Right Shank
  lk_rshank = Body (mass, com_pos, inertia);
  int rshank_id = model_->AddBody(rthigh_id, Xtrans(link_length),
                                  joint_ry, lk_rshank, "rightShank");
  printf("Right Shank id: %i \n", rshank_id);

  ///////////////////////////////////////////////////////////////
  // Fixed Joint (Hip)
  lk_hip = Body(mass_zero, com_pos_zero, gyration_radii_zero);
  int hip_id = model_->AddBody(body_id, Xtrans(Vector3d(0.,0.,0.)), fixed_joint, lk_hip, "hip");
  printf("Hip id: %i \n", hip_id);

  // Fixed Joint (Body EE)
  lk_body_ee = Body(mass_zero, com_pos_zero, gyration_radii_zero);
  int body_ee_id = model_->AddBody(body_id, Xtrans(link_length_body), fixed_joint, lk_body_ee, "body_ee");
  printf("Body ee id: %i \n", body_ee_id);

  // Fixed Joint (Left Knee)
  lk_lknee = Body(mass_zero, com_pos_zero, gyration_radii_zero);
  int lknee_id = model_->AddBody(lthigh_id, Xtrans(link_length), fixed_joint, lk_lknee, "leftKnee");
  printf("Left Knee id: %i \n", lknee_id);

  // Fixed Joint (Right Knee)
  lk_rknee = Body(mass_zero, com_pos_zero, gyration_radii_zero);
  int rknee_id = model_->AddBody(rthigh_id, Xtrans(link_length), fixed_joint, lk_rknee, "rightKnee");
  printf("Right Knee id: %i \n", rknee_id);

  // Fixed Joint (Left Foot)
  lk_lfoot = Body(mass_zero, com_pos_zero, gyration_radii_zero);
  int lfoot_id = model_->AddBody(lshank_id, Xtrans(link_length), fixed_joint, lk_lfoot, "leftFoot");
  printf("Left Foot id: %i \n", lfoot_id);

  // Fixed Joint (Right Foot)
  lk_rfoot = Body(mass_zero, com_pos_zero, gyration_radii_zero);
  int rfoot_id = model_->AddBody(rshank_id, Xtrans(link_length), fixed_joint, lk_rfoot, "rightFoot");
  printf("Right Foot id: %i \n", rfoot_id);

  //////////////////////////////////////////////////////
  ///            End of Assemble Model               ///
  //////////////////////////////////////////////////////

  printf("[Walker2D Model] Contructed\n");
}

Walker2D_Sim_Model::~Walker2D_Sim_Model(){
  delete model_;
}
void Walker2D_Sim_Model::UpdateModel(const Vector & q, const Vector & qdot){
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


bool Walker2D_Sim_Model::getMassInertia(sejong::Matrix & A) {
  A = A_;
  return true;
}

bool Walker2D_Sim_Model::getGravity(Vector & grav) {
  grav = grav_;
  return true;
}

bool Walker2D_Sim_Model::getCoriolis(Vector & coriolis) {
  coriolis = coriolis_;
  return true;
}

void Walker2D_Sim_Model::getFullJacobian(int link_id, sejong::Matrix & J) const {
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

void Walker2D_Sim_Model::getFullJacobianDot(int link_id, sejong::Matrix & Jdot) const {
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

void Walker2D_Sim_Model::getPos(int link_id, Vect3 & pos) {
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

void Walker2D_Sim_Model::getVel(int link_id, Vect3 & vel) {
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

unsigned int Walker2D_Sim_Model::_find_body_idx(int id) const {
  switch(id){
  case LK_SIM_BODY:
    return model_->GetBodyId("body");
  case LK_SIM_LEFT_THIGH:
    return model_->GetBodyId("leftThigh");
  case LK_SIM_LEFT_SHANK:
    return model_->GetBodyId("leftShank");
  case LK_SIM_RIGHT_THIGH:
    return model_->GetBodyId("rightThigh");
  case LK_SIM_RIGHT_SHANK:
    return model_->GetBodyId("rightShank");
  case LK_SIM_HIP:
    return model_->GetBodyId("hip");
  case LK_SIM_BODY_EE:
    return model_->GetBodyId("body_ee");
  case LK_SIM_LEFT_KNEE:
    return model_->GetBodyId("leftKnee");
  case LK_SIM_RIGHT_KNEE:
    return model_->GetBodyId("rightKnee");
  case LK_SIM_LEFT_FOOT:
    return model_->GetBodyId("leftFoot");
  case LK_SIM_RIGHT_FOOT:
    return model_->GetBodyId("rightFoot");
  default:
    printf("[Warnning] Incorrect link id\n");
    return 0;
  }
}
