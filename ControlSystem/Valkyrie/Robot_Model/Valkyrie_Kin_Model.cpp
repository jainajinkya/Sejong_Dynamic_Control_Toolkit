#include "Valkyrie_Kin_Model.hpp"

#include <Utils/pseudo_inverse.hpp>
#include <Utils/utilities.hpp>

#include <sys/types.h>
#include <unistd.h>
#include <sys/syscall.h>

using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics;

Valkyrie_Kin_Model::Valkyrie_Kin_Model( RigidBodyDynamics::Model* model){
    model_ = model;
    Ig_ = Matrix::Zero(6,6);
    Jg_ = Matrix::Zero(6, model_->qdot_size);
}

Valkyrie_Kin_Model::~Valkyrie_Kin_Model(){
}
void Valkyrie_Kin_Model::UpdateKinematics(const sejong::Vector & q, const sejong::Vector & qdot){
  _UpdateCentroidFrame(q, qdot);
}

void Valkyrie_Kin_Model::_UpdateCentroidFrame(const sejong::Vector & q, const sejong::Vector & qdot){
    double mass;
    Vector3d zero_vector;
    zero_vector.setZero();

    Vector3d com_pos;
    Vector3d cm;
    Vector3d link_pos;
    Vector3d p_g;

    getCoMPos(q, com_pos, false);
    com_pos_ = com_pos;

    Matrix Xg_inv = Matrix::Zero(6, 6);
    Ig_.setZero();
    Matrix Ag = Matrix::Zero(6, model_->qdot_size);

    Matrix I = Matrix::Zero(6, 6);
    Matrix Jsp = Matrix::Zero(6, model_->qdot_size);

    int start_idx = _find_body_idx(LK_pelvis);
    Matrix3d p;
    Matrix3d cmm;
    Matrix3d R;

    for (int i(start_idx); i<model_->mBodies.size(); ++i){
        R = CalcBodyWorldOrientation(*model_, q, i, false);

        link_pos = CalcBodyToBaseCoordinates ( *model_, q, i, zero_vector, false);

        Jsp.setZero();
        CalcBodySpatialJacobian( *model_, q, i, Jsp, false);

        mass = model_->mBodies[i].mMass;
        I.setZero();
        cm = model_->mBodies[i].mCenterOfMass;
        cmm <<
            0.0, -cm[2], cm[1],
            cm[2], 0.0, -cm[0],
            -cm[1], cm[0], 0.0;
        I.setZero();
        I.block(0, 0, 3, 3) = model_->mBodies[i].mInertia + mass * cmm * cmm.transpose();
        I.block(0,3, 3,3) = mass * cmm;
        I.block(3,0, 3,3) = -mass * cmm;
        I.block(3, 3, 3, 3) = mass * Matrix::Identity(3,3);

        p_g = R * (com_pos - link_pos);
        p << 0.0, -p_g[2], p_g[1],
            p_g[2], 0.0, -p_g[0],
            -p_g[1], p_g[0], 0.0;

        Xg_inv.block(0,0, 3,3) = R;
        Xg_inv.block(3,3, 3,3) = R;
        Xg_inv.block(3,0, 3,3) = p * R;
        Ig_ = Ig_ + Xg_inv.transpose() * I * Xg_inv;
        Ag = Ag + Xg_inv.transpose() * I * Jsp;
    }
    Jg_ = Ig_.inverse() * Ag;

    centroid_vel_ = Jg_ * qdot;
    // sejong::Vector centroid_momentum = Ig_ * centroid_vel_;
    // sejong::pretty_print(centroid_vel_, std::cout, "centroid vel");
    // sejong::pretty_print(centroid_momentum, std::cout, "centroid momentum");

    // double rbdl_mass;
    // Math::Vector3d rbdl_com, rbdl_com_vel, rbdl_ang_momentum;
    // RigidBodyDynamics::Utils::CalcCenterOfMass(*model_, q, qdot, rbdl_mass, rbdl_com, &rbdl_com_vel, &rbdl_ang_momentum, false);
    // printf("[rbd] mass: %f\n", rbdl_mass);
    // sejong::pretty_print(rbdl_com_vel, std::cout, "[rbdl] com_vel");
    // sejong::pretty_print(rbdl_ang_momentum, std::cout, "[rbdl] ang_momentum");
    // sejong::pretty_print(com_vel, std::cout, "[rbdl] com_vel");

}


void Valkyrie_Kin_Model::getCoMJacobian(const sejong::Vector & Q, sejong::Matrix & Jcom) const {
    Vector3d zero_vector = Vector3d::Zero();

    Jcom = Matrix::Zero(3, model_->qdot_size);
    MatrixNd J(3, model_->qdot_size);

    double mass;
    double tot_mass(0.0);
    int start_idx = _find_body_idx(LK_pelvis);

    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;
        // CoM Jacobian Update
        J.setZero();
        CalcPointJacobian(*model_, Q, i, model_->mBodies[i].mCenterOfMass, J, false);
        Jcom +=  mass * J;
        tot_mass += mass;
    }
    Jcom /= tot_mass;
}

void Valkyrie_Kin_Model::getCoMPos(const sejong::Vector & q, sejong::Vect3 & CoM_pos, bool update)const {
    Vector3d zero_vector = Vector3d::Zero();

    CoM_pos.setZero();
    Vector3d link_pos;

    int start_idx = _find_body_idx(LK_pelvis);
    double mass;
    double tot_mass(0.0);
    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;

        // CoM position Update
        link_pos = CalcBodyToBaseCoordinates ( *model_, q, i,  model_->mBodies[i].mCenterOfMass, update);
        CoM_pos += mass * link_pos;
        tot_mass += mass;
    }
    CoM_pos /= tot_mass;
}

void Valkyrie_Kin_Model::getCoMVel(const sejong::Vector & q, const sejong::Vector & qdot, sejong::Vect3 & CoM_vel) const {

    int start_idx = _find_body_idx(LK_pelvis);
    CoM_vel = sejong::Vector::Zero(3);
    Vector3d link_vel;

    Vector3d zero_vector = Vector3d::Zero();
    double mass;
    double tot_mass(0.0);
    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;

        // CoM velocity Update
        link_vel = CalcPointVelocity ( *model_, q, qdot, i, model_->mBodies[i].mCenterOfMass, false);
        CoM_vel += mass * link_vel;
        tot_mass += mass;
    }
    CoM_vel /= tot_mass;
}

void Valkyrie_Kin_Model::getPosition(const Vector & q, int link_id, Vect3 & pos){
    Vector3d zero;
    // zero << 0.0, 0.0, 0.0;
    int bodyid = _find_body_idx(link_id);
    if(bodyid >=model_->fixed_body_discriminator){
        zero = model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass;
    }
    else{
        zero =  model_->mBodies[bodyid].mCenterOfMass;
    }

    pos = CalcBodyToBaseCoordinates(*model_, q, _find_body_idx(link_id), zero, false);
    // pos = CalcBaseToBodyCoordinates(*model_, q, _find_body_idx(link_id), zero, false);

}

void Valkyrie_Kin_Model::getOrientation(const Vector & q, int link_id, sejong::Quaternion & ori){
    Matrix3d R;
    R = CalcBodyWorldOrientation( *model_, q, _find_body_idx(link_id), false);
    ori = R.transpose();
    //std::cout<<"mat R : \n"<<R<<std::endl;
    //sejong::pretty_print(ori,std::cout,"quat");
    if(ori.w() < 0.){
      ori.w() *= (-1.);
      ori.x() *= (-1.);
      ori.y() *= (-1.);
      ori.z() *= (-1.);
    }
    // if(ori.z() > 0.8){
    //   exit(0);
    //   ori.z() *= (-1.);
    // }
}

void Valkyrie_Kin_Model::getVelocity(const Vector & q, const Vector &qdot,
                                 int link_id, Vect3 & vel){
    Vector3d zero;
    // zero << 0.0, 0.0, 0.0;
    int bodyid = _find_body_idx(link_id);
    if(bodyid >=model_->fixed_body_discriminator){
        zero = model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass;
    }
    else{
        zero =  model_->mBodies[bodyid].mCenterOfMass;
    }

    vel = CalcPointVelocity ( *model_, q, qdot, _find_body_idx(link_id), zero, false);

}
void Valkyrie_Kin_Model::getAngVel(const Vector & q, const Vector & qdot,
                             int link_id, Vect3 & ang_vel){
    unsigned int bodyid = _find_body_idx(link_id);
    Vector vel;
    if(bodyid >=model_->fixed_body_discriminator){
        vel = CalcPointVelocity6D(*model_, q, qdot, bodyid,
                            model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass, false);
    }
    else{
        vel = CalcPointVelocity6D(*model_, q, qdot, bodyid,
                            model_->mBodies[bodyid].mCenterOfMass, false);
    }
    ang_vel = vel.head(3);
}

void Valkyrie_Kin_Model::getJacobian(const Vector & q, int link_id, Matrix &J){

    J = Matrix::Zero(6, model_->qdot_size);

    unsigned int bodyid = _find_body_idx(link_id);
    Vector3d zero_vector = Vector3d::Zero();

    if(bodyid >=model_->fixed_body_discriminator){
        CalcPointJacobian6D(*model_, q, bodyid,
                            model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass,
                            J, false);
    }
    else{
        CalcPointJacobian6D(*model_, q, bodyid,
                            model_->mBodies[bodyid].mCenterOfMass,
                            J, false);
    }
    // Matrix3d R;
    // R = CalcBodyWorldOrientation(*model_, q, bodyid, false);
    // J.block(0,0, 3, model_->qdot_size) = R.transpose() * J.block(0,0, 3, model_->qdot_size);
}


void Valkyrie_Kin_Model::getJacobianDot6D_Analytic(const Vector & q, const Vector & qdot, int link_id, Matrix & J){
  J = Matrix::Zero(6, model_->qdot_size);

  unsigned int bodyid = _find_body_idx(link_id);

  if(bodyid >=model_->fixed_body_discriminator){
    CalcPointJacobianDot(*model_, q, qdot, bodyid,
                         model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass,
                         J, true);
  }
  else{
    // printf("2-1\n");
    CalcPointJacobianDot(*model_, q, qdot, bodyid,
                         model_->mBodies[bodyid].mCenterOfMass,
                         J, true);
    // printf("2-3\n");
  }
}


unsigned int Valkyrie_Kin_Model::_find_body_idx(int id) const {
    switch(id){
    case LK_pelvis:
      return model_->GetBodyId("pelvis");
    case LK_torso:
      return model_->GetBodyId("torso");
    case LK_leftCOP_Frame:
        return model_->GetBodyId("leftCOP_Frame");
    case LK_rightCOP_Frame:
        return model_->GetBodyId("rightCOP_Frame");

    case LK_leftFootOutFront:
        return model_->GetBodyId("leftFootOutFront");
    case LK_leftFootOutBack:
        return model_->GetBodyId("leftFootOutBack");
    case LK_leftFootInBack:
        return model_->GetBodyId("leftFootInBack");
    case LK_leftFootInFront:
        return model_->GetBodyId("leftFootInFront");

    case LK_rightFootOutFront:
        return model_->GetBodyId("rightFootOutFront");
    case LK_rightFootOutBack:
        return model_->GetBodyId("rightFootOutBack");
    case LK_rightFootInBack:
        return model_->GetBodyId("rightFootInBack");
    case LK_rightFootInFront:
        return model_->GetBodyId("rightFootInFront");
    }
    return (unsigned int)(id + 2);
}
