#include "BodyCtrl.hpp"
#include <Configuration.h>
#include <StateProvider.hpp>
#include <TaskSet/BodyTask.hpp>
#include <ContactSet/DoubleContact.hpp>
#include <WBDC/WBDC.hpp>
#include <Robot_Model/RobotModel.hpp>
#include <chrono>

// #define WBDC_COMPUTATION_TIME

BodyCtrl::BodyCtrl(): end_time_(1000000.),
                      body_pos_ini_(4)
{
  body_task_ = new BodyTask();
  double_contact_ = new DoubleContact();
  wbdc_ = new WBDC(act_list_);

  wbdc_data_ = new WBDC_ExtraData();
  wbdc_data_->tau_min = sejong::Vector(NUM_ACT_JOINT);
  wbdc_data_->tau_max = sejong::Vector(NUM_ACT_JOINT);

  for(int i(0); i<NUM_ACT_JOINT; ++i){
    wbdc_data_->tau_max[i] = 180.0;
    wbdc_data_->tau_min[i] = -180.0;
  }
  // printf("[Body Controller] Constructed\n");
}

BodyCtrl::~BodyCtrl(){
  delete body_task_;
  delete double_contact_;
}

void BodyCtrl::OneStep(sejong::Vector & gamma){
  // printf("[Body Ctrl] Onestep\n");
  _PreProcessing_Command();

  gamma.setZero();
  _double_contact_setup();
  _body_task_setup();
  _body_ctrl(gamma);

  _PostProcessing_Command(gamma);
}

void BodyCtrl::_body_ctrl(sejong::Vector & gamma){
  wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);

#ifdef WBDC_COMPUTATION_TIME
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif

  wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);

#ifdef WBDC_COMPUTATION_TIME
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
  std::cout << "All process took me " << time_span1.count()*1000.0 << "ms."<<std::endl;;
#endif

}

void BodyCtrl::_body_task_setup(){
  sejong::Vector pos_des(body_task_->getDim());
  sejong::Vector vel_des(body_task_->getDim());
  sejong::Vector acc_des(body_task_->getDim());
  pos_des.setZero(); vel_des.setZero(); acc_des.setZero();

  // CoM Height
  pos_des[0] = ini_com_pos_[2];
  body_task_->UpdateTask(&(pos_des),
                         vel_des,
                         acc_des);

  // set relaxed op direction
  // cost weight setup
  // bool b_height_relax(false);
  bool b_height_relax(true);
  if(b_height_relax){
    std::vector<bool> relaxed_op(body_task_->getDim(), false);
    relaxed_op[0] = true; // Z
    relaxed_op[1] = true; // Rx
    relaxed_op[2] = true; // Ry
    relaxed_op[3] = true; // Rz
    body_task_->setRelaxedOpCtrl(relaxed_op);

    int prev_size(wbdc_data_->cost_weight.rows());
    wbdc_data_->cost_weight.conservativeResize( prev_size + 4);
    wbdc_data_->cost_weight[prev_size] = 500.;
    wbdc_data_->cost_weight[prev_size+1] = 500.;
    wbdc_data_->cost_weight[prev_size+2] = 500.;
    wbdc_data_->cost_weight[prev_size+3] = 500.;
  }

  // Push back to task list
  task_list_.push_back(body_task_);
  // sejong::pretty_print(wbdc_data_->cost_weight,std::cout, "cost weight");
}
void BodyCtrl::_double_contact_setup(){
  double_contact_->UpdateContactSpec();
  contact_list_.push_back(double_contact_);
  wbdc_data_->cost_weight = sejong::Vector::Zero(double_contact_->getDim());
  for(int i(0); i<double_contact_->getDim(); ++i){
    wbdc_data_->cost_weight[i] = 1.;
  }
  wbdc_data_->cost_weight[2] = 0.0001;
  wbdc_data_->cost_weight[5] = 0.0001;
}

void BodyCtrl::FirstVisit(){
  ctrl_start_time_ = sp_->curr_time_;
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
  robot_model_->getCoMPosition(sp_->Q_, ini_com_pos_);
  sejong::pretty_print(ini_com_pos_, std::cout, "ini com");
}
