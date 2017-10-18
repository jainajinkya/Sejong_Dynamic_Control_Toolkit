#include "BodyCtrl.hpp"
#include <Configuration.h>
#include <StateProvider.hpp>
#include <TaskSet/BodyTask.hpp>
#include <ContactSet/FootContact.hpp>
#include <WBDC/WBDC.hpp>
#include <DracoP1Rot_Model/DracoModel.hpp>

BodyCtrl::BodyCtrl(): end_time_(1000000.){
  body_task_ = new BodyTask(3);
  foot_contact_ = new FootContact(3);
  wbdc_ = new WBDC(act_list_);
  wbdc_data_ = new WBDC_ExtraData();
  wbdc_data_->tau_min = sejong::Vector(NUM_ACT_JOINT);
  wbdc_data_->tau_max = sejong::Vector(NUM_ACT_JOINT);
  for(int i(0); i<NUM_ACT_JOINT; ++i){
    wbdc_data_->tau_max[i] = 1000.0;
    wbdc_data_->tau_min[i] = -1000.0;
  }
  printf("[Body Controller] Constructed\n");
}
BodyCtrl::~BodyCtrl(){
  delete body_task_;
  delete foot_contact_;
}


void BodyCtrl::OneStep(sejong::Vector & gamma){
  _PreProcessing_Command();

  gamma.setZero();
  _foot_contact_setup();
  _body_task_setup();
  _body_ctrl(gamma);

  _PostProcessing_Command(gamma);
}

void BodyCtrl::_body_ctrl(sejong::Vector & gamma){
  sejong::Matrix Icm, Jcm;
  robot_model_->getCentroidInertia(Icm);
  robot_model_->getCentroidJacobian(Jcm);
  wbdc_data_->Icam = Icm.block(2, 2, 1, 1);
  wbdc_data_->Jcam = Jcm.block(2,0, 1, NUM_QDOT);
  wbdc_data_->JcamDotQdot = (Jcm * Ainv_ * coriolis_).tail(1);

  sejong::pretty_print(Icm, std::cout, "Icm");
  sejong::pretty_print(Jcm, std::cout, "Jcm");
  sejong::pretty_print(wbdc_data_->Icam, std::cout, "Icam");
  sejong::pretty_print(wbdc_data_->Jcam, std::cout, "Jcam");
  sejong::pretty_print(wbdc_data_->JcamDotQdot, std::cout, "JcamDot qdot");

  wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
  wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);
}

void BodyCtrl::_body_task_setup(){
  sp_->Body_pos_des_ = body_pos_ini_;
  sp_->Body_vel_des_.setZero();
  sp_->Body_acc_des_.setZero();

  double amp(0.2);
  double omega(2.*M_PI * 1.);

  printf("state time: %f\n", state_machine_time_);
  sp_->Body_pos_des_[2] += amp * sin(omega * state_machine_time_);
  sp_->Body_vel_des_[2] = amp * omega * cos(omega * state_machine_time_);
  sp_->Body_acc_des_[2] = amp * omega * omega * sin(omega * state_machine_time_);

  body_task_->UpdateTask(&(sp_->Body_pos_des_),
                         (sejong::Vector)sp_->Body_vel_des_,
                         (sejong::Vector)sp_->Body_acc_des_);



  // set relaxed op direction
  // cost weight setup
  bool b_height_relax(false);
  // bool b_height_relax(true);
  if(b_height_relax){
    std::vector<bool> relaxed_op(body_task_->getDim(), false);
    relaxed_op[0] = true; // Z
    body_task_->setRelaxedOpCtrl(relaxed_op);

    int prev_size(wbdc_data_->cost_weight.rows());
    wbdc_data_->cost_weight.conservativeResize( prev_size + 1);
    wbdc_data_->cost_weight[prev_size] = 100.;
  }

  // Push back to task list
  task_list_.push_back(body_task_);
  sejong::pretty_print(wbdc_data_->cost_weight,std::cout, "cost weight");
}
void BodyCtrl::_foot_contact_setup(){
  foot_contact_->UpdateContactSpec();
  contact_list_.push_back(foot_contact_);
  wbdc_data_->cost_weight = sejong::Vector::Zero(3);

  wbdc_data_->cost_weight[0] = 1.;
  wbdc_data_->cost_weight[1] = 0.0001;
  wbdc_data_->cost_weight[2] = 1.;
}

void BodyCtrl::_jpos_ctrl(sejong::Vector & gamma){
  gamma = grav_.tail(NUM_ACT_JOINT);
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
  body_pos_ini_ = sp_->Body_pos_;
}
