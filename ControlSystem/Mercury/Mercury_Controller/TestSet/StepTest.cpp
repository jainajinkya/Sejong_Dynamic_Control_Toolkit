#include "StepTest.hpp"
#include <StateProvider.hpp>
#include <CtrlSet/CoMzRxRyRzCtrl.hpp>
#include <CtrlSet/JPosCtrl.hpp>
#include <CtrlSet/BodyFootCtrl.hpp>
#include <CtrlSet/TransitionCtrl.hpp>

StepTest::StepTest():Test(){

  sp_ = StateProvider::GetStateProvider();
  sp_->stance_foot_ = SJLinkID::LK_LFOOT;

  phase_ = STPhase::double_contact_1;
  state_list_.clear();

  jpos_ctrl_ = new JPosCtrl();
  body_up_ctrl_ = new CoMzRxRyRzCtrl();
  body_fix_ctrl_ = new CoMzRxRyRzCtrl();
  // Right
  right_swing_start_trans_ctrl_ = new TransitionCtrl(SJLinkID::LK_RFOOT);
  right_swing_ctrl_ = new BodyFootCtrl(SJLinkID::LK_RFOOT);
  right_swing_end_trans_ctrl_ = new TransitionCtrl(SJLinkID::LK_RFOOT);
  // Left
  left_swing_start_trans_ctrl_ = new TransitionCtrl(SJLinkID::LK_LFOOT);
  left_swing_ctrl_ = new BodyFootCtrl(SJLinkID::LK_LFOOT);
  left_swing_end_trans_ctrl_ = new TransitionCtrl(SJLinkID::LK_LFOOT);

  state_list_.push_back(jpos_ctrl_);
  state_list_.push_back(body_up_ctrl_);
  state_list_.push_back(body_fix_ctrl_);
  state_list_.push_back(right_swing_start_trans_ctrl_);
  state_list_.push_back(right_swing_ctrl_);
  state_list_.push_back(right_swing_end_trans_ctrl_);
  state_list_.push_back(body_fix_ctrl_);
  state_list_.push_back(left_swing_start_trans_ctrl_);
  state_list_.push_back(left_swing_ctrl_);
  state_list_.push_back(left_swing_end_trans_ctrl_);

  printf("[Step Test] Constructed\n");
}

StepTest::~StepTest(){
  for(int i(0); i<state_list_.size(); ++i){
    delete state_list_[i];
  }
}

void StepTest::TestInitialization(){
  // Yaml file name
  jpos_ctrl_->CtrlInitialization("set_initial_position");
  body_up_ctrl_->CtrlInitialization("move_to_target_height");
  body_fix_ctrl_->CtrlInitialization("fix_des_pos");
}

int StepTest::_NextPhase(const int & phase){
  int next_phase = phase + 1;
  if(phase == double_contact_1) sp_->stance_foot_ = SJLinkID::LK_LFOOT;
  if(phase == double_contact_2) sp_->stance_foot_ = SJLinkID::LK_RFOOT;

  if(phase == NUM_STPHASE) return STPhase::double_contact_1;
  else return next_phase;
}

