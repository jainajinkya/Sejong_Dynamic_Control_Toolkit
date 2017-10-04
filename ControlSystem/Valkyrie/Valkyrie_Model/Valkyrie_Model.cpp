#include "Valkyrie_Model.h"
#include "Valkyrie_Dyn_Model.h"
#include "Valkyrie_Kin_Model.h"
#include "rbdl/urdfreader.h"
#include "Utils/utilities.h"

#include <stdio.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

ValkyrieModel* ValkyrieModel::GetValkyrieModel(){
    static ValkyrieModel valkyrie_model_;
    return & valkyrie_model_;
}

ValkyrieModel::ValkyrieModel(){
    model_ = new Model();

    if (!Addons::URDFReadFromFile (ModelPath"/urdf/r5_urdf_rbdl.urdf", model_, false)) {
        std::cerr << "Error loading model ./r5_urdf_rbdl.urdf" << std::endl;
        abort();
    }

    dyn_model_ = new Valkyrie_Dyn_Model(model_);
    kin_model_ = new Valkyrie_Kin_Model(model_);

    printf("[Valkyrie Model] Contructed\n");
}

ValkyrieModel::~ValkyrieModel(){
    delete dyn_model_;
    delete kin_model_;
    delete model_;
}
void ValkyrieModel::UpdateModel(const Vector & q, const Vector & qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
    dyn_model_->UpdateDynamics(q, qdot);
    kin_model_->UpdateKinematics(q, qdot);
}

void ValkyrieModel::getCentroidInertia(sejong::Matrix & Icent){
    kin_model_->getCentroidInertia(Icent);
}

void ValkyrieModel::getCentroidJacobian(sejong::Matrix & Jcent){
  Jcent.setZero();
    kin_model_->getCentroidJacobian(Jcent);
}

void ValkyrieModel::UpdateKinematics(const Vector & q, const Vector &qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
}

bool ValkyrieModel::getInverseMassInertia(sejong::Matrix & Ainv) {
    return dyn_model_->getInverseMassInertia(Ainv);
}

bool ValkyrieModel::getMassInertia(sejong::Matrix & A) {
    return dyn_model_->getMassInertia(A);
}

bool ValkyrieModel::getGravity(Vector & grav) {
    return dyn_model_->getGravity(grav);
}

bool ValkyrieModel::getCoriolis(Vector & coriolis) {
    return dyn_model_->getCoriolis(coriolis);
}

void ValkyrieModel::getFullJacobian(const Vector & q, int link_id, sejong::Matrix & J) const {
  J.setZero();
    kin_model_->getJacobian(q, link_id, J);
}
void ValkyrieModel::getFullJacobianDot(const Vector & q, const Vector & qdot, int link_id, sejong::Matrix & Jdot) const {
  kin_model_->getJacobianDot6D_Analytic(q, qdot, link_id, Jdot);
}

void ValkyrieModel::getPosition(const Vector & q,
                            int link_id, Vect3 & pos) {
    kin_model_->getPosition(q, link_id, pos);
}
void ValkyrieModel::getOrientation(const Vector & q,
                               int link_id, sejong::Quaternion & ori) {
    kin_model_->getOrientation(q, link_id, ori);
}
void ValkyrieModel::getVelocity(const Vector & q, const Vector &qdot,
                            int link_id, Vect3 & vel) {
    kin_model_->getVelocity(q, qdot, link_id, vel);
}
void ValkyrieModel::getAngVel(const Vector & q, const Vector & qdot,
                          int link_id, Vect3 & ang_vel){
    kin_model_->getAngVel(q, qdot, link_id, ang_vel);
}

void ValkyrieModel::getCoMJacobian(const Vector & q, sejong::Matrix & J){
    J = sejong::Matrix::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(q, J);
}

void ValkyrieModel::getCoMPosition(const Vector & q, Vect3 & com_pos, bool update){
  com_pos = kin_model_->com_pos_;
    // kin_model_->getCoMPos(q, com_pos, update);
}

void ValkyrieModel::getCoMVelocity(const Vector & q, const Vector & qdot, Vect3 & com_vel){
    kin_model_->getCoMVel(q, qdot, com_vel);
}
void ValkyrieModel::getCentroidVelocity(sejong::Vector & centroid_vel){
  centroid_vel = kin_model_->centroid_vel_;
  // kin_model_->getCentroidVelocity(centroid_vel);
}
