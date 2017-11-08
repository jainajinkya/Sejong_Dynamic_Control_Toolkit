#include "CoMzRxRyRzCtrl.hpp"
#include <Configuration.h>
#include <StateProvider.hpp>
#include <TaskSet/CoMBodyOriTask.hpp>
#include <ContactSet/DoubleContact.hpp>
#include <WBDC/WBDC.hpp>
#include <Robot_Model/RobotModel.hpp>

CoMzRxRyRzCtrl::CoMzRxRyRzCtrl(): Controller(),
                                  end_time_(100000000),
                                  body_pos_ini_(4)
{
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
}

void CoMzRxRyRzCtrl::_body_task_setup(){
  sejong::Vector pos_des(3 + 4);
  sejong::Vector vel_des(body_task_->getDim());
  sejong::Vector acc_des(body_task_->getDim());
  pos_des.setZero(); vel_des.setZero(); acc_des.setZero();

  // CoM Pos
  pos_des.head(3) = ini_com_pos_;
  // Orientation
  sejong::Vect3 rpy_des;
  sejong::Quaternion quat_des;
  rpy_des.setZero();

  double amp(0.2);
  double omega(2. * M_PI * 0.5);
  int rot_idx(0);
  // Roll
  rpy_des[rot_idx] += amp * sin(omega * state_machine_time_);
  vel_des[rot_idx + 3] = amp * omega * cos(omega * state_machine_time_);
  acc_des[rot_idx + 3] = -amp* omega * omega * sin(omega * state_machine_time_);

  // Pitch
  rot_idx = 1;
  amp = 0.0;
  omega = 2. * M_PI * 0.1;
  rpy_des[rot_idx] += amp * sin(omega * state_machine_time_);
  vel_des[rot_idx + 3] = amp * omega * cos(omega * state_machine_time_);
  acc_des[rot_idx + 3] = -amp* omega * omega * sin(omega * state_machine_time_);

  sejong::convert(rpy_des, quat_des);
  pos_des[3] = quat_des.w();
  pos_des[4] = quat_des.x();
  pos_des[5] = quat_des.y();
  pos_des[6] = quat_des.z();

  body_task_->UpdateTask(&(pos_des), vel_des, acc_des);

  // set relaxed op direction
  // cost weight setup
  // bool b_height_relax(false);
  bool b_height_relax(true);
  if(b_height_relax){
    std::vector<bool> relaxed_op(body_task_->getDim(), true);
    // relaxed_op[0] = true; // X
    // relaxed_op[1] = true; // Y
    // relaxed_op[2] = true; // Z
    // relaxed_op[3] = true; // Rx
    // relaxed_op[4] = true; // Ry
    // relaxed_op[5] = true; // Rz

    body_task_->setRelaxedOpCtrl(relaxed_op);

    int prev_size(wbdc_data_->cost_weight.rows());
    wbdc_data_->cost_weight.conservativeResize( prev_size + 6);
    wbdc_data_->cost_weight[prev_size] = 1.;
    wbdc_data_->cost_weight[prev_size+1] = 1.;
    wbdc_data_->cost_weight[prev_size+2] = 10.;
    wbdc_data_->cost_weight[prev_size+3] = 10.;
    wbdc_data_->cost_weight[prev_size+4] = 10.;
    wbdc_data_->cost_weight[prev_size+5] = 1.;
  }

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
  ctrl_start_time_ = sp_->curr_time_;
}

void CoMzRxRyRzCtrl::LastVisit(){
}

bool CoMzRxRyRzCtrl::EndOfPhase(){
  if(state_machine_time_ > end_time_){
    return true;
  }
  return false;
}
void CoMzRxRyRzCtrl::CtrlInitialization(std::string setting_file_name){
  robot_model_->getCoMPosition(sp_->Q_, ini_com_pos_);
}
