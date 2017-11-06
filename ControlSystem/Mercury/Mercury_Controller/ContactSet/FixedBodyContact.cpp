#include "FixedBodyContact.hpp"
#include <Robot_Model/RobotModel.hpp>
#include <StateProvider.hpp>

FixedBodyContact::FixedBodyContact():WBDC_ContactSpec(6)
{
  model_ = RobotModel::GetRobotModel();
  sp_ = StateProvider::GetStateProvider();
}
FixedBodyContact::~FixedBodyContact(){ }

bool FixedBodyContact::_UpdateJc(){
  model_->getFullJacobian(sp_->Q_, SJLinkID::LK_Body, Jc_);
  return true;
}

bool FixedBodyContact::_UpdateJcDotQdot(){
  sejong::Matrix JcDot;
  model_->getFullJacobianDot(sp_->Q_,sp_->Qdot_,  SJLinkID::LK_Body, JcDot);
  JcDotQdot_ = JcDot * sp_->Qdot_;
  return true;
}

bool FixedBodyContact::_UpdateUf(){
  Uf_ = sejong::Matrix::Zero(1, dim_contact_);
  return true;
}
