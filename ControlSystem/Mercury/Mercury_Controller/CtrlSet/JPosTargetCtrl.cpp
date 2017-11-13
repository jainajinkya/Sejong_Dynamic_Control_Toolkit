#include "JPosTargetCtrl.hpp"
#include <StateProvider.hpp>
#include <TaskSet/JPosTask.hpp>
#include <ContactSet/FixedBodyContact.hpp>
#include <WBDC/WBDC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/utilities.hpp>

JPosTargetCtrl::JPosTargetCtrl():Controller(),
                                 jpos_target_(NUM_ACT_JOINT),
                                 end_time_(1000.0)
{
  jpos_task_ = new JPosTask();
  fixed_body_contact_ = new FixedBodyContact();
  wbdc_ = new WBDC(act_list_);
  wbdc_data_ = new WBDC_ExtraData();

  wbdc_data_->tau_min = sejong::Vector(NUM_ACT_JOINT);
  wbdc_data_->tau_max = sejong::Vector(NUM_ACT_JOINT);
  for(int i(0); i<NUM_ACT_JOINT; ++i){
    wbdc_data_->tau_max[i] = 50.0;
    wbdc_data_->tau_min[i] = -50.0;
  }
}

JPosTargetCtrl::~JPosTargetCtrl(){
  delete jpos_task_;
  delete fixed_body_contact_;
  delete wbdc_;
  delete wbdc_data_;
}

void JPosTargetCtrl::OneStep(sejong::Vector & gamma){
  _PreProcessing_Command();

  gamma.setZero();
  _fixed_body_contact_setup();
  _jpos_task_setup();
  _jpos_ctrl(gamma);

  _PostProcessing_Command(gamma);
}

void JPosTargetCtrl::_jpos_ctrl(sejong::Vector & gamma){
  wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
  wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);
}

void JPosTargetCtrl::_jpos_task_setup(){
  sejong::Vector jpos_des = jpos_ini_;
  sejong::Vector jvel_des(NUM_ACT_JOINT); jvel_des.setZero();
  sejong::Vector jacc_des(NUM_ACT_JOINT); jacc_des.setZero();

  for(int i(0); i<NUM_ACT_JOINT; ++i){
    jpos_des[i] = sejong::smooth_changing(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
    jvel_des[i] = sejong::smooth_changing_vel(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
    jacc_des[i] = sejong::smooth_changing_acc(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);

  }
  jpos_task_->UpdateTask(&(jpos_des), jvel_des, jacc_des);

  std::vector<bool> relaxed_op(jpos_task_->getDim(), false);
  jpos_task_->setRelaxedOpCtrl(relaxed_op);
  task_list_.push_back(jpos_task_);
}

void JPosTargetCtrl::_fixed_body_contact_setup(){
  fixed_body_contact_->UpdateContactSpec();
  contact_list_.push_back(fixed_body_contact_);
  wbdc_data_->cost_weight = sejong::Vector::Zero(fixed_body_contact_->getDim());

  for(int i(0); i<fixed_body_contact_->getDim(); ++i){
    wbdc_data_->cost_weight[i] = 1.;
  }
}

void JPosTargetCtrl::FirstVisit(){
  ctrl_start_time_ = sp_->curr_time_;
}

void JPosTargetCtrl::LastVisit(){
}

bool JPosTargetCtrl::EndOfPhase(){
  if(state_machine_time_ > end_time_){
    return true;
  }
  return false;
}
void JPosTargetCtrl::CtrlInitialization(const std::string & setting_file_name){
  jpos_ini_ = sp_->Q_.segment(NUM_VIRTUAL, NUM_ACT_JOINT);
  // ParamHandler handle(CONFIG_PATH + setting_file_name + ".yaml");

  // std::vector<double> tmp_vec;
  // handle.getVector("target_jpos", tmp_vec);
  // for(int i(0); i<NUM_ACT_JOINT; ++i) jpos_target_[i] = tmp_vec[i];
}

void JPosTargetCtrl::setTargetPosition(const std::vector<double>& jpos){
  for(int i(0); i<NUM_ACT_JOINT; ++i){
    jpos_target_[i] = jpos[i];
    // printf("%i th jpos: %f\n", i, jpos[i]);
  }
}
