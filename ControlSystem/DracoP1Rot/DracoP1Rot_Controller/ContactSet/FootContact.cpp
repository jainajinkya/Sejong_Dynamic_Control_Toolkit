#include "FootContact.hpp"
#include <DracoP1Rot_Model/DracoModel.hpp>
#include <StateProvider.hpp>

FootContact::FootContact(int dim):WBDC_ContactSpec(dim)
{
  model_ = DracoModel::GetDracoModel();
  sp_ = StateProvider::GetStateProvider();
}
FootContact::~FootContact(){
}
bool FootContact::_UpdateJc(){
  model_->getFullJacobian(sp_->Q_, SJLinkID::LK_foot, Jc_);
  return true;
}
bool FootContact::_UpdateJcDotQdot(){
  sejong::Matrix JcDot;
  model_->getFullJacobianDot(sp_->Q_,sp_->Qdot_,  SJLinkID::LK_foot, JcDot);
  JcDotQdot_ = JcDot * sp_->Qdot_;
  return true;

}

bool FootContact::_UpdateUf(){
  double mu(0.3);
  double l1(0.12);
  double l2(0.08);
  Uf_ = sejong::Matrix::Zero(5, dim_contact_);
  Uf_(0, 1) = 1.; // Fz > 0
  Uf_(1, 0) = -1.; Uf_(1,1) = mu; // -Fx + mu * Fz > 0
  Uf_(2, 0) = 1.;  Uf_(2, 1) = mu; // Fx + mu * Fz > 0
  Uf_(3, 1) = 1.;  Uf_(3, 2) = 1./l1; // Fz + tau_y/l1 > 0
  Uf_(4, 1) = 1.;  Uf_(4, 2) = -1./l2; // Fz - tau_y/l2 > 0

  return true;
}

bool FootContact::_UpdateInequalityVector(){
  ieq_vec_ = sejong::Vector::Zero(5);
  return true;
}
