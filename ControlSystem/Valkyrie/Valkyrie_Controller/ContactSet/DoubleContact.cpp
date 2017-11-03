#include "DoubleContact.hpp"
#include <Robot_Model/RobotModel.hpp>
#include <StateProvider.hpp>
#include <Utils/utilities.hpp>

DoubleContact::DoubleContact(int dim):WBDC_ContactSpec(dim)
{
  model_ = RobotModel::GetRobotModel();
  sp_ = StateProvider::GetStateProvider();
  Jc_ = sejong::Matrix(dim, NUM_QDOT);

}
DoubleContact::~DoubleContact(){
}
bool DoubleContact::_UpdateJc(){
  sejong::Matrix Jtmp;
  model_->getFullJacobian(sp_->Q_, SJLinkID::LK_rightCOP_Frame, Jtmp);
  Jc_.block(0, 0, 6, NUM_QDOT) = Jtmp;

  model_->getFullJacobian(sp_->Q_, SJLinkID::LK_leftCOP_Frame, Jtmp);
  Jc_.block(6, 0, 6, NUM_QDOT) = Jtmp;

  // sejong::pretty_print(Jc_, std::cout, "double] Jc");
  return true;
}
bool DoubleContact::_UpdateJcDotQdot(){
  sejong::Matrix JcDot(dim_contact_, NUM_QDOT);
  sejong::Matrix jcdot_tmp;
  // Right
  model_->getFullJacobianDot(sp_->Q_,sp_->Qdot_,  SJLinkID::LK_rightCOP_Frame, jcdot_tmp);
  JcDot.block(0, 0, 6, NUM_QDOT) = jcdot_tmp;
  // Left
  model_->getFullJacobianDot(sp_->Q_,sp_->Qdot_,  SJLinkID::LK_leftCOP_Frame, jcdot_tmp);
  JcDot.block(6, 0, 6, NUM_QDOT) = jcdot_tmp;

  JcDotQdot_ = JcDot * sp_->Qdot_;
  return true;
}

bool DoubleContact::_UpdateUf(){
  double mu(1.3);
  double X(0.1);
  double Y(0.05);

  Uf_ = sejong::Matrix::Zero(9*2, dim_contact_);

  sejong::Matrix U;
  _setU(X, Y, mu, U);
  Uf_.block(0,0, 9, 6) = U;
  Uf_.block(9, 6, 9, 6) = U;
  return true;
}

void DoubleContact::_setU(double x, double y, double mu, sejong::Matrix & U){
  U = sejong::Matrix::Zero(9, 6);

  U(0, 5) = 1.;

  U(1, 3) = 1.; U(1, 5) = mu;
  U(2, 3) = -1.; U(2, 5) = mu;

  U(3, 4) = 1.; U(3, 5) = mu;
  U(4, 4) = -1.; U(4, 5) = mu;

  U(5, 0) = 1.; U(5, 5) = y;
  U(6, 0) = -1.; U(6, 5) = y;

  U(7, 1) = 1.; U(7, 5) = x;
  U(8, 1) = -1.; U(8, 5) = x;

  // // Tau
  // U(8, 0) = -mu; U(8, 1) = -mu; U(8, 2) = 1;
  // U(8, 3) = y; U(8, 4) = x; U(8, 5) = (x + y)*mu;

  // U(9, 0) = -mu; U(9, 1) = mu; U(9, 2) = 1;
  // U(9, 3) = y; U(9, 4) = -x; U(9, 5) = (x + y)*mu;

  // U(10, 0) = mu; U(10, 1) = -mu; U(10, 2) = 1;
  // U(10, 3) = -y; U(10, 4) = x; U(10, 5) = (x + y)*mu;

  // U(11, 0) = mu; U(11, 1) = mu; U(11, 2) = 1;
  // U(11, 3) = -y; U(11, 4) = -x; U(11, 5) = (x + y)*mu;
  // /////////////////////////////////////////////////
  // U(12, 0) = -mu; U(12, 1) = -mu; U(12, 2) = -1;
  // U(12, 3) = -y; U(12, 4) = -x; U(12, 5) = (x + y)*mu;

  // U(13, 0) = -mu; U(13, 1) = mu; U(13, 2) = -1;
  // U(13, 3) = -y; U(13, 4) = x; U(13, 5) = (x + y)*mu;

  // U(14, 0) = mu; U(14, 1) = -mu; U(14, 2) = -1;
  // U(14, 3) = y; U(14, 4) = -x; U(14, 5) = (x + y)*mu;

  // U(15, 0) = mu; U(15, 1) = mu; U(15, 2) = -1;
  // U(15, 3) = y; U(15, 4) = x; U(15, 5) = (x + y)*mu;
  // ////////////////////////////////////////////////////

}
