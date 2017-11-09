#include "BodyFootCtrl.hpp"
#include <Configuration.h>
#include <StateProvider.hpp>
#include <TaskSet/BodyFootTask.hpp>
#include <ContactSet/SingleContact.hpp>
#include <WBDC/WBDC.hpp>
#include <Robot_Model/RobotModel.hpp>
#include <ParamHandler/ParamHandler.hpp>


BodyFootCtrl::BodyFootCtrl(int swing_foot):Controller(),
                                           swing_foot_(swing_foot)
{
  body_foot_task_ = new BodyFootTask(swing_foot);
  if(swing_foot == LK_LFOOT) single_contact_ = new SingleContact(LK_RFOOT);
  else if(swing_foot == LK_RFOOT) single_contact_ = new SingleContact(LK_LFOOT);
  else printf("[Warnning] swing foot is not foot: %i\n", swing_foot);
  wbdc_ = new WBDC(act_list_);

  wbdc_data_ = new WBDC_ExtraData();
  wbdc_data_->tau_min = sejong::Vector(NUM_ACT_JOINT);
  wbdc_data_->tau_max = sejong::Vector(NUM_ACT_JOINT);

  for(int i(0); i<NUM_ACT_JOINT; ++i){
    wbdc_data_->tau_max[i] = 100.0;
    wbdc_data_->tau_min[i] = -100.0;
  }
  // printf("[BodyFoot Controller] Constructed\n");
}

BodyFootCtrl::~BodyFootCtrl(){
  delete body_foot_task_;
  delete single_contact_;
}

void BodyFootCtrl::OneStep(sejong::Vector & gamma){
  _PreProcessing_Command();

  gamma.setZero();
  _single_contact_setup();
  _task_setup();
  _body_foot_ctrl(gamma);

  _PostProcessing_Command(gamma);
}

void BodyFootCtrl::_body_foot_ctrl(sejong::Vector & gamma){
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

void BodyFootCtrl::_task_setup(){
  sejong::Vector pos_des(3 + 4 + 3);
  sejong::Vector vel_des(body_foot_task_->getDim());
  sejong::Vector acc_des(body_foot_task_->getDim());
  pos_des.setZero(); vel_des.setZero(); acc_des.setZero();

  // CoM Pos
  pos_des.head(3) = ini_com_pos_;
  // Orientation
  sejong::Vect3 rpy_des;
  sejong::Quaternion quat_des;
  rpy_des.setZero();

  sejong::convert(rpy_des, quat_des);
  pos_des[3] = quat_des.w();
  pos_des[4] = quat_des.x();
  pos_des[5] = quat_des.y();
  pos_des[6] = quat_des.z();


  // set relaxed op direction
  // cost weight setup
  // bool b_height_relax(false);
  bool b_height_relax(true);
  if(b_height_relax){
    std::vector<bool> relaxed_op(body_foot_task_->getDim(), false);
    relaxed_op[0] = true; // X
    relaxed_op[1] = true; // Y
    relaxed_op[2] = true; // Z
    relaxed_op[3] = true; // Rx
    relaxed_op[4] = true; // Ry
    relaxed_op[5] = true; // Rz

    body_foot_task_->setRelaxedOpCtrl(relaxed_op);

    int prev_size(wbdc_data_->cost_weight.rows());
    wbdc_data_->cost_weight.conservativeResize( prev_size + 6);
    wbdc_data_->cost_weight[prev_size] = 0.0001;
    wbdc_data_->cost_weight[prev_size+1] = 0.0001;
    wbdc_data_->cost_weight[prev_size+2] = 1000.;
    wbdc_data_->cost_weight[prev_size+3] = 10.;
    wbdc_data_->cost_weight[prev_size+4] = 10.;
    wbdc_data_->cost_weight[prev_size+5] = 0.001;
  }

  // Foot Position Task
  double pos[3];
  double vel[3];
  double acc[3];
  // printf("time: %f\n", state_machine_time_);
  foot_traj_.getCurvePoint(state_machine_time_, pos);
  foot_traj_.getCurveDerPoint(state_machine_time_, 1, vel);
  foot_traj_.getCurveDerPoint(state_machine_time_, 2, acc);
  // printf("pos:%f, %f, %f\n", pos[0], pos[1], pos[2]);

  for(int i(0); i<3; ++i){
    pos_des[i + 7] = pos[i];
    vel_des[i + 6] = vel[i];
    acc_des[i + 6] = acc[i];
  }
  // sejong::pretty_print(vel_des, std::cout, "[Ctrl] vel des");
  // Push back to task list
  body_foot_task_->UpdateTask(&(pos_des), vel_des, acc_des);
  task_list_.push_back(body_foot_task_);
  // sejong::pretty_print(wbdc_data_->cost_weight,std::cout, "cost weight");
}
void BodyFootCtrl::_single_contact_setup(){
  single_contact_->UpdateContactSpec();
  contact_list_.push_back(single_contact_);
  wbdc_data_->cost_weight = sejong::Vector::Zero(single_contact_->getDim());
  for(int i(0); i<single_contact_->getDim(); ++i){
    wbdc_data_->cost_weight[i] = 1.;
  }
  wbdc_data_->cost_weight[2] = 0.0001;
}

void BodyFootCtrl::FirstVisit(){
  printf("[Body Foot Ctrl] Start\n");

  robot_model_->getPosition(sp_->Q_, swing_foot_, ini_foot_pos_);
  
  sejong::pretty_print(ini_foot_pos_, std::cout, "ini foot pos");
  sejong::pretty_print(target_foot_pos_, std::cout, "target foot pos");

  target_foot_pos_[0] = sp_->Q_[0];

  ctrl_start_time_ = sp_->curr_time_;
  // Trajectory Setup
  double init[9];
  double fin[9];
  double** middle_pt = new double*[1];
  middle_pt[0] = new double[3];

  for(int i(0); i<3; ++i){
    init[i] = ini_foot_pos_[i];
    fin[i] = target_foot_pos_[i];
    middle_pt[0][i] = (ini_foot_pos_[i] + target_foot_pos_[i])/2.0;
  }
  middle_pt[0][2] = swing_height_;

  // Initial and final velocity & acceleration
  for(int i(3); i<9; ++i){
    init[i] = 0.;
    fin[i] = 0.;
  }
  foot_traj_.SetParam(init, fin, middle_pt, end_time_);

  delete [] *middle_pt;
  delete [] middle_pt;
}

void BodyFootCtrl::LastVisit(){
}

bool BodyFootCtrl::EndOfPhase(){
  if(state_machine_time_ > end_time_){
    printf("[Body Foot Ctrl] End\n");
    return true;
  }
  return false;
}
void BodyFootCtrl::CtrlInitialization(std::string setting_file_name){
  robot_model_->getCoMPosition(sp_->Q_, ini_com_pos_);
  robot_model_->getPosition(sp_->Q_, swing_foot_, ini_foot_pos_);

  // Setting Parameters
  ParamHandler handler(CONFIG_PATH + setting_file_name + ".yaml");

  handler.getValue("swing_height", swing_height_);

  // Target foot location
  handler.getBoolean("compute_target", b_compute_target_);

  if(!b_compute_target_){
    std::vector<double> tmp_vec;
    handler.getVector("target_location", tmp_vec);
    for(int i(0);i<3; ++i){
      target_foot_pos_[i] = tmp_vec[i];
    }
  }
}
