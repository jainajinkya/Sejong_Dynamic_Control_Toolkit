#include "DoubleContact.hpp"
#include <Robot_Model/RobotModel.hpp>
#include <StateProvider.hpp>
#include <Utils/utilities.hpp>

DoubleContact::DoubleContact():WBDC_ContactSpec(6)
{
  model_ = RobotModel::GetRobotModel();
  sp_ = StateProvider::GetStateProvider();
  Jc_ = sejong::Matrix(6, NUM_QDOT);
  // printf("[Double Contact] Constructed\n");
}

DoubleContact::~DoubleContact(){}

bool DoubleContact::_UpdateJc(){
  sejong::Matrix Jtmp;
  model_->getFullJacobian(sp_->Q_, SJLinkID::LK_RFOOT, Jtmp);
  Jc_.block(0, 0, 3, NUM_QDOT) = Jtmp.block(3, 0, 3, NUM_QDOT);

  model_->getFullJacobian(sp_->Q_, SJLinkID::LK_LFOOT, Jtmp);
  Jc_.block(3, 0, 3, NUM_QDOT) = Jtmp.block(3, 0, 3, NUM_QDOT);

  // sejong::pretty_print(Jc_, std::cout, "double] Jc");
  return true;
}
bool DoubleContact::_UpdateJcDotQdot(){
  sejong::Matrix JcDot(dim_contact_, NUM_QDOT);
  sejong::Matrix jcdot_tmp;
  // Right
  model_->getFullJacobianDot(sp_->Q_,sp_->Qdot_,  SJLinkID::LK_RFOOT, jcdot_tmp);
  JcDot.block(0, 0, 3, NUM_QDOT) = jcdot_tmp.block(3, 0, 3, NUM_QDOT);
  // Left
  model_->getFullJacobianDot(sp_->Q_,sp_->Qdot_,  SJLinkID::LK_LFOOT, jcdot_tmp);
  JcDot.block(3, 0, 3, NUM_QDOT) = jcdot_tmp.block(3, 0, 3, NUM_QDOT);

  // sejong::pretty_print(JcDot, std::cout,  "JcDot");
  JcDotQdot_ = JcDot * sp_->Qdot_;
  return true;
}

bool DoubleContact::_UpdateUf(){
  double mu(0.8);

  int size_u(5);
  Uf_ = sejong::Matrix::Zero(size_u*2, dim_contact_);

  sejong::Matrix U;
  _setU(mu, U);
  Uf_.block(0, 0, size_u, 3) = U;
  Uf_.block(size_u, 3, size_u, 3) = U;
  return true;
}

void DoubleContact::_setU(double mu, sejong::Matrix & U){
  U = sejong::Matrix::Zero(5, 3);

  U(0, 2) = 1.;

  U(1, 0) = 1.; U(1, 2) = mu;
  U(2, 0) = -1.; U(2, 2) = mu;

  U(3, 1) = 1.; U(3, 2) = mu;
  U(4, 1) = -1.; U(4, 2) = mu;
}
