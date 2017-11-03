#include "JointCtrlTest.hpp"
#include <CtrlSet/JPosCtrl.hpp>

JointCtrlTest::JointCtrlTest():Test(){
  phase_ = 0;
  state_list_.clear();

  jpos_ctrl_ = new JPosCtrl();
  state_list_.push_back(jpos_ctrl_);

  printf("[Joint Ctrl Test] Constructed\n");
}
JointCtrlTest::~JointCtrlTest(){
  delete jpos_ctrl_;
}
void JointCtrlTest::TestInitialization(){
  jpos_ctrl_->CtrlInitialization("blank");
}

int JointCtrlTest::_NextPhase(const int & phase){
  return 0;
}
