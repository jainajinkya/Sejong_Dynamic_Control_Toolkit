#include "RobotModel.hpp"
#include "Mercury_Dyn_Model.hpp"
#include "Mercury_Kin_Model.hpp"
#include "rbdl/urdfreader.h"
#include <Utils/utilities.hpp>

#include <stdio.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

RobotModel* RobotModel::GetRobotModel(){
  static RobotModel draco_model_;
  return & draco_model_;
}

RobotModel::RobotModel(){
  model_ = new Model();
  rbdl_check_api_version (RBDL_API_VERSION);

  if (!Addons::URDFReadFromFile (THIS_COM"ControlSystem/Mercury/Mercury_urdf/mercury.urdf", model_, false)) {
    std::cerr << "Error loading model ./mercury.urdf" << std::endl;
    abort();
  }

  dyn_model_ = new Mercury_Dyn_Model(model_);
  kin_model_ = new Mercury_Kin_Model(model_);

  printf("[Mercury Model] Contructed\n");
}

RobotModel::~RobotModel(){
  delete dyn_model_;
  delete kin_model_;
  delete model_;
}

void RobotModel::UpdateModel(const Vector & q, const Vector & qdot){
  UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
  dyn_model_->UpdateDynamics(q, qdot);
  kin_model_->UpdateKinematics(q, qdot);
}

void RobotModel::getCentroidInertia(sejong::Matrix & Icent){
  sejong::Matrix Icm_tmp;
  kin_model_->getCentroidInertia(Icm_tmp);
}

void RobotModel::getCentroidJacobian(sejong::Matrix & Jcent){
  Jcent.setZero();
  kin_model_->getCentroidJacobian(Jcent);
}

void RobotModel::UpdateKinematics(const Vector & q, const Vector &qdot){
  UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
}

bool RobotModel::getInverseMassInertia(sejong::Matrix & Ainv) {
  return dyn_model_->getInverseMassInertia(Ainv);
}

bool RobotModel::getMassInertia(sejong::Matrix & A) {
  return dyn_model_->getMassInertia(A);
}

bool RobotModel::getGravity(Vector & grav) {
  return dyn_model_->getGravity(grav);
}

bool RobotModel::getCoriolis(Vector & coriolis) {
  return dyn_model_->getCoriolis(coriolis);
}

void RobotModel::getFullJacobian(const Vector & q, int link_id, sejong::Matrix & J) const {
  J.setZero();
  kin_model_->getJacobian(q, link_id, J);
}
void RobotModel::getFullJacobianDot(const Vector & q, const Vector & qdot, int link_id, sejong::Matrix & Jdot) const {
  kin_model_->getJacobianDot6D_Analytic(q, qdot, link_id, Jdot);
}

void RobotModel::getPosition(const Vector & q,
                            int link_id, Vect3 & pos) {
    kin_model_->getPosition(q, link_id, pos);
}
void RobotModel::getOrientation(const Vector & q,
                               int link_id, sejong::Quaternion & ori) {
    kin_model_->getOrientation(q, link_id, ori);
}
void RobotModel::getVelocity(const Vector & q, const Vector &qdot,
                            int link_id, Vect3 & vel) {
    kin_model_->getVelocity(q, qdot, link_id, vel);
}
void RobotModel::getAngVel(const Vector & q, const Vector & qdot,
                          int link_id, Vect3 & ang_vel){
    kin_model_->getAngVel(q, qdot, link_id, ang_vel);
}

void RobotModel::getCoMJacobian(const Vector & q, sejong::Matrix & J){
    J = sejong::Matrix::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(q, J);
}

void RobotModel::getCoMPosition(const Vector & q, Vect3 & com_pos, bool update){
  com_pos = kin_model_->com_pos_;
}

void RobotModel::getCoMVelocity(const Vector & q, const Vector & qdot, Vect3 & com_vel){
    kin_model_->getCoMVel(q, qdot, com_vel);
}
void RobotModel::getCentroidVelocity(sejong::Vector & centroid_vel){
  centroid_vel = kin_model_->centroid_vel_;
}
