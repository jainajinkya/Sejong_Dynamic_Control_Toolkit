#include "SingleContact.hpp"
#include <Robot_Model/RobotModel.hpp>
#include <StateProvider.hpp>

SingleContact::SingleContact(int pt):WBDC_ContactSpec(3),
                                     contact_pt_(pt)
{
  model_ = RobotModel::GetRobotModel();
  sp_ = StateProvider::GetStateProvider();
}

SingleContact::~SingleContact(){
}
bool SingleContact::_UpdateJc(){
  sejong::Matrix Jtmp;
  model_->getFullJacobian(sp_->Q_, contact_pt_, Jtmp);
  Jc_ = Jtmp.block(3, 0, 3, NUM_QDOT);
  return true;
}
bool SingleContact::_UpdateJcDotQdot(){
  sejong::Matrix JcDot;
  model_->getFullJacobianDot(sp_->Q_,sp_->Qdot_,  SJLinkID::LK_RFOOT, JcDot);
  JcDotQdot_ = JcDot.block(3, 0, 3, NUM_QDOT) * sp_->Qdot_;
  return true;

}

bool SingleContact::_UpdateUf(){
  double mu (0.8);
  Uf_ = sejong::Matrix::Zero(5, dim_contact_);
  Uf_(0, 2) = 1.; // Fz >= 0

  Uf_(1, 0) = 1.; Uf_(1, 2) = mu; // Fx >= - mu * Fz
  Uf_(2, 0) = -1.; Uf_(2, 2) = mu; // Fx <= mu * Fz

  Uf_(3, 1) = 1.; Uf_(3, 2) = mu; // Fy >= - mu * Fz
  Uf_(4, 1) = -1.; Uf_(4, 2) = mu; // Fy <=  mu * Fz
  return true;
}
