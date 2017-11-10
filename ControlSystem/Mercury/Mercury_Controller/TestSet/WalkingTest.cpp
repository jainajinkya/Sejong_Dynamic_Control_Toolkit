#include "WalkingTest.hpp"
#include <StateProvider.hpp>
#include <CtrlSet/CoMzRxRyRzCtrl.hpp>
#include <CtrlSet/JPosCtrl.hpp>
#include <CtrlSet/BodyFootPlanningCtrl.hpp>
#include <CtrlSet/TransitionCtrl.hpp>
#include <ParamHandler/ParamHandler.hpp>

WalkingTest::WalkingTest():Test(){

  sp_ = StateProvider::GetStateProvider();
  sp_->stance_foot_ = SJLinkID::LK_LFOOT;

  phase_ = WKPhase::wk_lift_up;
  state_list_.clear();

  jpos_ctrl_ = new JPosCtrl();
  body_up_ctrl_ = new CoMzRxRyRzCtrl();
  body_fix_ctrl_ = new CoMzRxRyRzCtrl();
  // Right
  right_swing_start_trans_ctrl_ = new TransitionCtrl(SJLinkID::LK_RFOOT, false);
  right_swing_ctrl_ = new BodyFootPlanningCtrl(SJLinkID::LK_RFOOT);
  right_swing_end_trans_ctrl_ = new TransitionCtrl(SJLinkID::LK_RFOOT, true);
  // Left
  left_swing_start_trans_ctrl_ = new TransitionCtrl(SJLinkID::LK_LFOOT, false);
  left_swing_ctrl_ = new BodyFootPlanningCtrl(SJLinkID::LK_LFOOT);
  left_swing_end_trans_ctrl_ = new TransitionCtrl(SJLinkID::LK_LFOOT, true);

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

  _SettingParameter();

  printf("[Walking Test] Constructed\n");
}

WalkingTest::~WalkingTest(){
  for(int i(0); i<state_list_.size(); ++i){
    delete state_list_[i];
  }
}

void WalkingTest::TestInitialization(){
  // Yaml file name
  jpos_ctrl_->CtrlInitialization("set_initial_jpos");
  body_up_ctrl_->CtrlInitialization("move_to_target_height");
  body_fix_ctrl_->CtrlInitialization("fix_des_pos");

  // Transition
  right_swing_start_trans_ctrl_->CtrlInitialization("trans");
  right_swing_end_trans_ctrl_->CtrlInitialization("trans");
  left_swing_start_trans_ctrl_->CtrlInitialization("trans");
  left_swing_end_trans_ctrl_->CtrlInitialization("trans");

  // Swing
  right_swing_ctrl_->CtrlInitialization("right_walking_swing");
  left_swing_ctrl_->CtrlInitialization("left_walking_swing");
}

int WalkingTest::_NextPhase(const int & phase){
  int next_phase = phase + 1;
  printf("next phase: %i\n", next_phase);
  if(phase == WKPhase::wk_double_contact_1) {
    printf("One swing done: Next Right Leg Swing\n");
    sp_->stance_foot_ = SJLinkID::LK_LFOOT;
  }
  if(phase == WKPhase::wk_double_contact_2){
    printf("One swing done: Next Left Leg Swing\n");
    sp_->stance_foot_ = SJLinkID::LK_RFOOT;
  }
  if(next_phase == NUM_WALKING_PHASE) {
    printf("one step is done\n");
    return WKPhase::wk_double_contact_1;
  }
  else{ return next_phase; }
}

void WalkingTest::_SettingParameter(){
  // Setting Parameters
  ParamHandler handler(CONFIG_PATH"step_test.yaml");

  double tmp;
  std::vector<double> tmp_vec;
  std::string tmp_str;

  //// Timing Setup
  handler.getValue("com_lifting_time", tmp);
  ((CoMzRxRyRzCtrl*)body_up_ctrl_)->setStanceTime(tmp);

  // Stance Time
  handler.getValue("stance_time", tmp);
  ((CoMzRxRyRzCtrl*)body_fix_ctrl_)->setStanceTime(tmp);

  // Swing Time
  handler.getValue("swing_time", tmp);
  ((BodyFootPlanningCtrl*)right_swing_ctrl_)->setSwingTime(tmp);
  ((BodyFootPlanningCtrl*)left_swing_ctrl_)->setSwingTime(tmp);

  // Transition Time
  handler.getValue("st_transition_time", tmp);
  ((TransitionCtrl*)right_swing_start_trans_ctrl_)->setTransitionTime(tmp);
  ((TransitionCtrl*)right_swing_end_trans_ctrl_)->setTransitionTime(tmp);
  ((TransitionCtrl*)left_swing_start_trans_ctrl_)->setTransitionTime(tmp);
  ((TransitionCtrl*)left_swing_end_trans_ctrl_)->setTransitionTime(tmp);

  handler.getString("planner_name", tmp_str);
}
