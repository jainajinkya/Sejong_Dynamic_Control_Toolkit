#include "CoMzRxRyRzCtrl.hpp"
#include <Configuration.h>
#include <StateProvider.hpp>
#include <TaskSet/CoMBodyOriTask.hpp>
#include <ContactSet/DoubleContact.hpp>
#include <WBDC/WBDC.hpp>
#include <Robot_Model/RobotModel.hpp>
#include <ParamHandler/ParamHandler.hpp>

CoMzRxRyRzCtrl::CoMzRxRyRzCtrl(): Controller(),
                                  dim_ctrl_(6),
                                  end_time_(100.),
                                  body_pos_ini_(4),
                                  b_set_height_target_(false)
{
  amp_ = sejong::Vector::Zero(dim_ctrl_);
  freq_= sejong::Vector::Zero(dim_ctrl_);
  phase_= sejong::Vector::Zero(dim_ctrl_);

  body_task_ = new CoMBodyOriTask();
  double_contact_ = new DoubleContact();
  wbdc_ = new WBDC(act_list_);

  wbdc_data_ = new WBDC_ExtraData();
  wbdc_data_->tau_min = sejong::Vector(NUM_ACT_JOINT);
  wbdc_data_->tau_max = sejong::Vector(NUM_ACT_JOINT);

  for(int i(0); i<NUM_ACT_JOINT; ++i){
    wbdc_data_->tau_max[i] = 100.0;
    wbdc_data_->tau_min[i] = -100.0;
  }
}

CoMzRxRyRzCtrl::~CoMzRxRyRzCtrl(){
  delete body_task_;
  delete double_contact_;
  delete wbdc_;
  delete wbdc_data_;
}

void CoMzRxRyRzCtrl::OneStep(sejong::Vector & gamma){
  _PreProcessing_Command();

  gamma.setZero();
  _double_contact_setup();
  _body_task_setup();
  _body_ctrl(gamma);

  _PostProcessing_Command(gamma);
}

void CoMzRxRyRzCtrl::_body_ctrl(sejong::Vector & gamma){
  wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
  wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);

  for(int i(0); i<6; ++i)
    sp_->reaction_forces_[i] = wbdc_data_->opt_result_[i];
}

void CoMzRxRyRzCtrl::_body_task_setup(){
  sejong::Vector pos_des(3 + 4);
  sejong::Vector vel_des(body_task_->getDim());
  sejong::Vector acc_des(body_task_->getDim());
  pos_des.setZero(); vel_des.setZero(); acc_des.setZero();

  // CoM Pos
  pos_des.head(3) = ini_com_pos_;
  if(b_set_height_target_)
    pos_des[2] = des_com_height_;

  double amp, omega, phase;
  for(int i(0); i<3; ++i){
    omega = 2. * M_PI * freq_[i];
    amp = amp_[i];
    phase = phase_[i];

    pos_des[i] += amp * sin(omega * state_machine_time_ + phase);
    vel_des[i] = amp * omega * cos(omega * state_machine_time_ + phase);
    acc_des[i] = -amp * omega * omega * sin(omega * state_machine_time_ + phase);
  }

  // Orientation
  sejong::Vect3 rpy_des;
  rpy_des.setZero();

  for(int i(0); i<3; ++i){
    omega = 2. * M_PI * freq_[i+3];
    amp = amp_[i+3];
    phase = phase_[i+3];

    rpy_des[i] += amp * sin(omega * state_machine_time_ + phase);
    vel_des[i + 3] = amp * omega * cos(omega * state_machine_time_ + phase);
    acc_des[i + 3] = -amp * omega * omega * sin(omega * state_machine_time_ + phase);
  }
  sejong::Quaternion quat_des;
  sejong::convert(rpy_des, quat_des);

  pos_des[3] = quat_des.w();
  pos_des[4] = quat_des.x();
  pos_des[5] = quat_des.y();
  pos_des[6] = quat_des.z();

  body_task_->UpdateTask(&(pos_des), vel_des, acc_des);

  // set relaxed op direction
  // cost weight setup
  // bool b_height_relax(false);
  std::vector<bool> relaxed_op(body_task_->getDim(), true);
  body_task_->setRelaxedOpCtrl(relaxed_op);

  int prev_size(wbdc_data_->cost_weight.rows());
  wbdc_data_->cost_weight.conservativeResize( prev_size + dim_ctrl_);
  wbdc_data_->cost_weight[prev_size] = 0.0001;
  wbdc_data_->cost_weight[prev_size+1] = 0.0001;
  wbdc_data_->cost_weight[prev_size+2] = 10.;
  wbdc_data_->cost_weight[prev_size+3] = 10.;
  wbdc_data_->cost_weight[prev_size+4] = 10.;
  wbdc_data_->cost_weight[prev_size+5] = 0.001;

  // Push back to task list
  task_list_.push_back(body_task_);
  // sejong::pretty_print(wbdc_data_->cost_weight,std::cout, "cost weight");
}
void CoMzRxRyRzCtrl::_double_contact_setup(){
  double_contact_->UpdateContactSpec();
  contact_list_.push_back(double_contact_);
  wbdc_data_->cost_weight = sejong::Vector::Zero(double_contact_->getDim());
  for(int i(0); i<double_contact_->getDim(); ++i){
    wbdc_data_->cost_weight[i] = 1.;
  }
  wbdc_data_->cost_weight[2] = 0.0001;
  wbdc_data_->cost_weight[5] = 0.0001;
}

void CoMzRxRyRzCtrl::FirstVisit(){
  // printf("[CoMzRxRyRz] Start\n");
  ctrl_start_time_ = sp_->curr_time_;
}

void CoMzRxRyRzCtrl::LastVisit(){
  // printf("[CoMzRxRyRz] End\n");
}

void CoMzRxRyRzCtrl::setAmp(const std::vector<double> & amp){
  for(int i(0); i< dim_ctrl_; ++i) amp_[i] = amp[i];
}
void CoMzRxRyRzCtrl::setFrequency(const std::vector<double> & freq){
  for(int i(0); i< dim_ctrl_; ++i) freq_[i] = freq[i];

}
void CoMzRxRyRzCtrl::setPhase(const std::vector<double> & phase){
  for(int i(0); i< dim_ctrl_; ++i) phase_[i] = phase[i];
}

bool CoMzRxRyRzCtrl::EndOfPhase(){
  if(state_machine_time_ > end_time_){
    return true;
  }
  return false;
}
void CoMzRxRyRzCtrl::CtrlInitialization(const std::string & setting_file_name){
  robot_model_->getCoMPosition(sp_->Q_, ini_com_pos_);
  std::vector<double> tmp_vec;

  ParamHandler handler(CONFIG_PATH + setting_file_name + ".yaml");

  // Feedback Gain
  handler.getVector("Kp", tmp_vec);
  for(int i(0); i<tmp_vec.size(); ++i){
    ((CoMBodyOriTask*)body_task_)->Kp_vec_[i] = tmp_vec[i];
  }
  handler.getVector("Kd", tmp_vec);
  for(int i(0); i<tmp_vec.size(); ++i){
    ((CoMBodyOriTask*)body_task_)->Kd_vec_[i] = tmp_vec[i];
  }
}
