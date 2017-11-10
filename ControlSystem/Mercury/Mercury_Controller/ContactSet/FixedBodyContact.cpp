#include "FixedBodyContact.hpp"
#include <Robot_Model/RobotModel.hpp>
#include <StateProvider.hpp>
#include <Utils/utilities.hpp>

FixedBodyContact::FixedBodyContact():WBDC_ContactSpec(6)
{
  model_ = RobotModel::GetRobotModel();
  sp_ = StateProvider::GetStateProvider();
  Jc_ = sejong::Matrix::Zero(dim_contact_, NUM_QDOT);
}
FixedBodyContact::~FixedBodyContact(){ }

bool FixedBodyContact::_UpdateJc(){
  for(int i(0);i<dim_contact_; ++i)  Jc_(i,i) = 1.;
  return true;
}

bool FixedBodyContact::_UpdateJcDotQdot(){
  JcDotQdot_ = sejong::Vector::Zero(dim_contact_);
  return true;
}

bool FixedBodyContact::_UpdateUf(){
  Uf_ = sejong::Matrix::Zero(1, dim_contact_);
  return true;
}

bool FixedBodyContact::_UpdateInequalityVector(){
  ieq_vec_ = sejong::Vector::Zero(1);
  return true;
}
