#include "BodyCtrlTest.hpp"
#include <CtrlSet/BodyCtrl.hpp>

BodyCtrlTest::BodyCtrlTest():Test(){
  phase_ = BCPhase::initial_transition;
  state_list_.clear();

  body_ini_tran_ = new BodyCtrl();
  body_shake_ctrl_ = new BodyCtrl();
  body_up_ctrl_ = new BodyCtrl();
  body_stay_ = new BodyCtrl();

  state_list_.push_back(body_ini_tran_);
  state_list_.push_back(body_shake_ctrl_);
  state_list_.push_back(body_up_ctrl_);
  state_list_.push_back(body_stay_);
}

BodyCtrlTest::~BodyCtrlTest(){
  for(int i(0); i<state_list_.size(); ++i){
    delete state_list_[i];
  }
}

void BodyCtrlTest::TestInitialization(){
  // Yaml file name
  body_ini_tran_->CtrlInitialization("body_initial_transition");
  body_shake_ctrl_->CtrlInitialization("body_shake_ctrl");
  body_up_ctrl_->CtrlInitialization("body_up_ctrl");
  body_stay_->CtrlInitialization("body_stay");
}

int BodyCtrlTest::_NextPhase(const int & phase){
  // if (phase == NUM_PHASE) return BCPhase::initial_transition;
  if (phase == NUM_PHASE) return BCPhase::stay_up;
  else return (phase +1);
}
