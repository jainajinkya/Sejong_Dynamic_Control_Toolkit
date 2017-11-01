#include "MercuryTest.hpp"
#include "MercuryController.hpp"

MercuryTest::MercuryTest():b_first_visit_(true){
}

MercuryTest::~MercuryTest(){
}

void MercuryTest::getTorqueInput(sejong::Vector & gamma){
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
