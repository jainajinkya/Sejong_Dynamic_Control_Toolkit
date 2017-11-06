#include "JPosCtrl.hpp"
#include <StateProvider.hpp>
#include <TaskSet/JPosTask.hpp>
#include <ContactSet/DoubleContact.hpp>
#include <WBDC/WBDC.hpp>


JPosCtrl::JPosCtrl():Controller(){
  jpos_task_ = new JPosTask(NUM_ACT_JOINT);
  double_contact_ = new DoubleContact(12);
  wbdc_ = new WBDC(act_list_);
  wbdc_data_ = new WBDC_ExtraData();

  wbdc_data_->tau_min = sejong::Vector(NUM_ACT_JOINT);
  wbdc_data_->tau_max = sejong::Vector(NUM_ACT_JOINT);
  for(int i(0); i<NUM_ACT_JOINT; ++i){
    wbdc_data_->tau_max[i] = 1800.0;
    wbdc_data_->tau_min[i] = -1800.0;
  }
}

JPosCtrl::~JPosCtrl(){
  delete jpos_task_;
  delete double_contact_;
}

void JPosCtrl::OneStep(sejong::Vector & gamma){
  _PreProcessing_Command();

  gamma.setZero();
  _double_contact_setup();
  _jpos_task_setup();
  _jpos_ctrl(gamma);

  _PostProcessing_Command(gamma);
}

void JPosCtrl::_jpos_ctrl(sejong::Vector & gamma){
  wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
  wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);
}

void JPosCtrl::_jpos_task_setup(){
  sejong::Vector jpos_des = jpos_ini_;
  sejong::Vector jvel_des(NUM_ACT_JOINT); jvel_des.setZero();
  sejong::Vector jacc_des(NUM_ACT_JOINT); jacc_des.setZero();

  double amp (0.2);
  double omega (2. * M_PI * 1.0);
  int jidx = SJJointID::leftShoulderPitch;
  jpos_des[jidx] += amp * sin(omega * state_machine_time_);
  jvel_des[jidx] = amp * omega * cos(omega * state_machine_time_);
  jacc_des[jidx] = -amp * omega * omega * sin(omega * state_machine_time_);
  jidx = SJJointID::torsoYaw;
  jpos_des[jidx] += amp * sin(omega * state_machine_time_);
  jvel_des[jidx] = amp * omega * cos(omega * state_machine_time_);
  jacc_des[jidx] = -amp * omega * omega * sin(omega * state_machine_time_);
  jpos_task_->UpdateTask(&(jpos_des), jvel_des, jacc_des);

  // set relaxed op direction
  // cost weight setupn
  // bool b_height_relax(false);
  std::vector<bool> relaxed_op(jpos_task_->getDim(), true);
  // relaxed_op[SJJointID::torsoYaw] = true;
  
  int prev_size(wbdc_data_->cost_weight.rows());
  wbdc_data_->cost_weight.conservativeResize( prev_size + NUM_ACT_JOINT);
  for(int i(0); i<NUM_ACT_JOINT; ++i){
    wbdc_data_->cost_weight[prev_size + i] = 500.;
  }


  // wbdc_data_->cost_weight[prev_size+1] = 500.;
  jpos_task_->setRelaxedOpCtrl(relaxed_op);

  // Push back to task list
  task_list_.push_back(jpos_task_);
  // sejong::pretty_print(wbdc_data_->cost_weight,std::cout, "cost weight");
}

void JPosCtrl::_double_contact_setup(){
  double_contact_->UpdateContactSpec();
  contact_list_.push_back(double_contact_);
  wbdc_data_->cost_weight = sejong::Vector::Zero(double_contact_->getDim());

  for(int i(0); i<double_contact_->getDim(); ++i){
    wbdc_data_->cost_weight[i] = 1.;
  }
  wbdc_data_->cost_weight[5] = 0.0001;
  wbdc_data_->cost_weight[11] = 0.0001;
}


void JPosCtrl::FirstVisit(){
  ctrl_start_time_ = sp_->curr_time_;
}

void JPosCtrl::LastVisit(){
}

bool JPosCtrl::EndOfPhase(){
  return false;
}
void JPosCtrl::CtrlInitialization(std::string setting_file_name){
  jpos_ini_ = sp_->Q_.segment(NUM_VIRTUAL, NUM_ACT_JOINT);
}
