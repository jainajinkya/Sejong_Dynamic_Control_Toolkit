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
  double mu(0.8);
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

bool FootContact::_UpdateWf(){
  sejong::Vect3 com_pos;
  sejong::Vect3 foot_pos;
  model_->getCoMPosition(sp_->Q_, com_pos);
  model_->getPosition(sp_->Q_, SJLinkID::LK_foot, foot_pos);
  Wf_ = sejong::Matrix::Zero(1, dim_contact_);
  sejong::pretty_print(com_pos, std::cout, "com pos");
  sejong::pretty_print(foot_pos, std::cout, "foot pos");

  Wf_(0, 0) = -(com_pos[2] - foot_pos[1]); // Z * Fx
  Wf_(0, 1) = (com_pos[0] - foot_pos[0]); // X * Fz
  Wf_(0, 2) = 1.;

  return true;
}
