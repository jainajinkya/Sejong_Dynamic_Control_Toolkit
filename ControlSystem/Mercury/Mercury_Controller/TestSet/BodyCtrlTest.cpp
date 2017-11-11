#include "BodyCtrlTest.hpp"

#include <CtrlSet/JPosTargetCtrl.hpp>
#include <CtrlSet/ContactTransBodyCtrl.hpp>
#include <CtrlSet/CoMzRxRyRzCtrl.hpp>
#include <ParamHandler/ParamHandler.hpp>

BodyCtrlTest::BodyCtrlTest():Test(){
  phase_ = BCPhase::initial_jpos;
  state_list_.clear();

  jpos_ctrl_ = new JPosTargetCtrl();
  body_up_ctrl_ = new ContactTransBodyCtrl();
  body_fix_ctrl_ = new CoMzRxRyRzCtrl();
  body_shake_ctrl_ = new CoMzRxRyRzCtrl();

  state_list_.push_back(jpos_ctrl_);
  state_list_.push_back(body_up_ctrl_);
  state_list_.push_back(body_fix_ctrl_);
  state_list_.push_back(body_shake_ctrl_);

  _SettingParameter();

  printf("[Body Control Test] Constructed\n");
}

BodyCtrlTest::~BodyCtrlTest(){
  for(int i(0); i<state_list_.size(); ++i){
    delete state_list_[i];
  }
}

void BodyCtrlTest::TestInitialization(){
  // Yaml file name
  jpos_ctrl_->CtrlInitialization("CTRL_set_initial_jpos");
  body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
  body_fix_ctrl_->CtrlInitialization("CTRL_fix_des_pos");
  body_shake_ctrl_->CtrlInitialization("CTRL_body_shake_ctrl");
}

int BodyCtrlTest::_NextPhase(const int & phase){
  int next_phase = phase + 1;
  printf("next phase: %i\n", next_phase);
  if (next_phase == NUM_BC_PHASE) {
    return BCPhase::stay_up;
  }
  else return next_phase;
}


void BodyCtrlTest::_SettingParameter(){
  ParamHandler handler(CONFIG_PATH"TEST_body_ctrl.yaml");

  double tmp;
  std::vector<double> tmp_vec;

  handler.getVector("initial_jpos", tmp_vec);
  ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);

  // CoM Height
  handler.getValue("com_height", tmp);
  ((ContactTransBodyCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
  ((CoMzRxRyRzCtrl*)body_fix_ctrl_)->setStanceHeight(tmp);
  ((CoMzRxRyRzCtrl*)body_shake_ctrl_)->setStanceHeight(tmp);

  //// Timing Setup
  handler.getValue("jpos_initialization_time", tmp);
  ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);
  handler.getValue("com_lifting_time", tmp);
  ((ContactTransBodyCtrl*)body_up_ctrl_)->setStanceTime(tmp);

  // Stance Time
  handler.getValue("body_stay_time", tmp);
  ((CoMzRxRyRzCtrl*)body_fix_ctrl_)->setStanceTime(tmp);
  handler.getValue("body_ctrl_time", tmp);
  ((CoMzRxRyRzCtrl*)body_shake_ctrl_)->setStanceTime(tmp);

  // Motion Setup
  handler.getVector("amp", tmp_vec);
  ((CoMzRxRyRzCtrl*)body_shake_ctrl_)->setAmp(tmp_vec);
  handler.getVector("frequency", tmp_vec);
  ((CoMzRxRyRzCtrl*)body_shake_ctrl_)->setFrequency(tmp_vec);
  handler.getVector("phase", tmp_vec);
  ((CoMzRxRyRzCtrl*)body_shake_ctrl_)->setPhase(tmp_vec);
}
