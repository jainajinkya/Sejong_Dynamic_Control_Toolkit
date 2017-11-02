#include "ValkyrieTest.hpp"
#include "ValkyrieController.hpp"

ValkyrieTest::ValkyrieTest():b_first_visit_(true){
}

ValkyrieTest::~ValkyrieTest(){
}

void ValkyrieTest::getTorqueInput(sejong::Vector & gamma){
  if(b_first_visit_){
    state_list_[phase_]->FirstVisit();
    b_first_visit_ = false;
  }

  state_list_[phase_]->OneStep(gamma);

  if(state_list_[phase_]->EndOfPhase()){
    state_list_[phase_]->LastVisit();
    phase_ = _NextPhase(phase_);
    b_first_visit_ = true;
  }
}
