#include "DracoTest.hpp"
#include "DracoController.hpp"

DracoTest::DracoTest(){
}

DracoTest::~DracoTest(){
}

void DracoTest::getTorqueInput(sejong::Vector & gamma){
  state_list_[phase_]->FirstVisit();
  state_list_[phase_]->OneStep(gamma);
  if(state_list_[phase_]->EndOfPhase()){
    state_list_[phase_]->LastVisit();
    phase_ = _NextPhase(phase_);
  }
}
