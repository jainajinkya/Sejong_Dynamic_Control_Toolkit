#include "Valkyrie_Dyn_Model.h"

#include <Utils/utilities.h>

#include <sys/types.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <Utils/pseudo_inverse.hpp>

using namespace RigidBodyDynamics::Math;

Valkyrie_Dyn_Model::Valkyrie_Dyn_Model(RigidBodyDynamics::Model* model){
    model_ = model;
}

Valkyrie_Dyn_Model::~Valkyrie_Dyn_Model(){
}

bool Valkyrie_Dyn_Model::getMassInertia(Matrix & a){
    a = A_;
    return true;
}

bool Valkyrie_Dyn_Model::getInverseMassInertia(Matrix & ainv){
    ainv = Ainv_;
    return true;
}

bool Valkyrie_Dyn_Model::getGravity(Vector &  grav){
    grav = grav_;
    return true;
}
bool Valkyrie_Dyn_Model::getCoriolis(Vector & coriolis){
    coriolis = coriolis_;
    return true;
}

void Valkyrie_Dyn_Model::UpdateDynamics(const sejong::Vector & q, const sejong::Vector & qdot){
    // Mass Matrix
    A_ = Matrix::Zero(model_->qdot_size, model_->qdot_size);
    CompositeRigidBodyAlgorithm(*model_, q, A_, false);

    // Ainv_ = A_.inverse();
    sejong::pseudoInverse(A_, 1.e-10, Ainv_, 0);

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
