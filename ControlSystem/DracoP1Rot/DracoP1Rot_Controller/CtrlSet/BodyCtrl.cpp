#include "BodyCtrl.hpp"

BodyCtrl::BodyCtrl(): end_time_(1000000.){
  printf("[Body Controller] Constructed\n");
}
BodyCtrl::~BodyCtrl(){
  
}


void BodyCtrl::OneStep(sejong::Vector & gamma){
  gamma.setZero();
}

void BodyCtrl::FirstVisit(){
  
}

void BodyCtrl::LastVisit(){
  
}

bool BodyCtrl::EndOfPhase(){
  if(state_machine_time_ > end_time_){
    return true;
  }
  return false;
}
void BodyCtrl::CtrlInitialization(std::string setting_file_name){
  
}
