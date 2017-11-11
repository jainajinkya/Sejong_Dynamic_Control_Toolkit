#include "BodyFootPlanningCtrl.hpp"
#include <Configuration.h>
#include <StateProvider.hpp>
#include <TaskSet/BodyFootTask.hpp>
#include <ContactSet/SingleContact.hpp>
#include <WBDC/WBDC.hpp>
#include <Robot_Model/RobotModel.hpp>
#include <ParamHandler/ParamHandler.hpp>


BodyFootPlanningCtrl::BodyFootPlanningCtrl(int swing_foot):
  Controller(),
  swing_foot_(swing_foot),
  update_time_(0.),
  num_planning_(0),
  planning_frequency_(0.)
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

BodyFootPlanningCtrl::~BodyFootPlanningCtrl(){
  delete body_foot_task_;
  delete single_contact_;
  delete wbdc_;
  delete wbdc_data_;
}

void BodyFootPlanningCtrl::OneStep(sejong::Vector & gamma){
  _PreProcessing_Command();

  gamma.setZero();
  _single_contact_setup();
  _task_setup();
  _body_foot_ctrl(gamma);

  _PostProcessing_Command(gamma);
}

void BodyFootPlanningCtrl::_body_foot_ctrl(sejong::Vector & gamma){
  wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
  wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);

  int offset(0);
  if(swing_foot_ == LK_RFOOT) offset = 3;
  for(int i(0); i<3; ++i)
    sp_->reaction_forces_[i + offset] = wbdc_data_->opt_result_[i];
}

void BodyFootPlanningCtrl::_task_setup(){
  sejong::Vector pos_des(3 + 4 + 3);
  sejong::Vector vel_des(body_foot_task_->getDim());
  sejong::Vector acc_des(body_foot_task_->getDim());
  pos_des.setZero(); vel_des.setZero(); acc_des.setZero();

  // CoM Pos
  pos_des.head(3) = ini_com_pos_;
  if(b_set_height_target_) pos_des[2] = des_com_height_;

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

  // _CheckPlanning();

  double traj_time = state_machine_time_ - update_time_;
  // Foot Position Task
  double pos[3];
  double vel[3];
  double acc[3];

  // printf("time: %f\n", state_machine_time_);
  foot_traj_.getCurvePoint(traj_time, pos);
  foot_traj_.getCurveDerPoint(traj_time, 1, vel);
  foot_traj_.getCurveDerPoint(traj_time, 2, acc);
  // printf("pos:%f, %f, %f\n", pos[0], pos[1], pos[2]);

  for(int i(0); i<3; ++i){
    curr_foot_pos_des_[i] = pos[i];
    curr_foot_vel_des_[i] = vel[i];
    curr_foot_acc_des_[i] = acc[i];

    pos_des[i + 7] = curr_foot_pos_des_[i];
    vel_des[i + 6] = curr_foot_vel_des_[i];
    acc_des[i + 6] = curr_foot_acc_des_[i];
  }
  // sejong::pretty_print(vel_des, std::cout, "[Ctrl] vel des");
  // Push back to task list
  body_foot_task_->UpdateTask(&(pos_des), vel_des, acc_des);
  task_list_.push_back(body_foot_task_);
  // sejong::pretty_print(wbdc_data_->cost_weight,std::cout, "cost weight");
}

void BodyFootPlanningCtrl::_CheckPlanning(){
  if( state_machine_time_ > end_time_/(planning_frequency_ + 1) * (num_planning_ + 1.) ){
    _Replanning();
    ++num_planning_;
  }
}

void BodyFootPlanningCtrl::_Replanning(){
  sejong::Vect3 com_pos, com_vel;
  robot_model_->getCoMPosition(sp_->Q_, com_pos);
  robot_model_->getCoMVelocity(sp_->Q_, sp_->Qdot_, com_vel);

  double time_modification(0.);
  double new_nx_loc;
  bool b_time_change;
  sejong::Vect3 target_pos; target_pos.setZero();

  // X
  planner_.getNextFootLocation(com_pos[0], com_vel[0], 0.,
                               end_time_-state_machine_time_, t_prime_x_,
                               b_time_change, time_modification,
                               new_nx_loc);
  target_pos[0] = new_nx_loc;
  if(b_time_change) end_time_ += time_modification;

  // Y
  planner_.getNextFootLocation(com_pos[1], com_vel[1], 0.,
                               end_time_-state_machine_time_, t_prime_y_,
                               b_time_change, time_modification,
                               new_nx_loc);
  target_pos[1] = new_nx_loc;

  if(b_time_change){
    end_time_ += time_modification;
    planner_.getNextFootLocation(com_pos[0], com_vel[0], 0.,
                                 end_time_-state_machine_time_,t_prime_x_,
                                 b_time_change, time_modification,
                                 new_nx_loc);
    target_pos[0] = new_nx_loc;
  }

  sejong::pretty_print(target_pos, std::cout, "next foot loc");
  printf("end_time:%f\n", end_time_);

  _SetBspline(curr_foot_pos_des_, curr_foot_vel_des_, curr_foot_acc_des_, target_pos);
}

void BodyFootPlanningCtrl::_single_contact_setup(){
  single_contact_->UpdateContactSpec();
  contact_list_.push_back(single_contact_);
  wbdc_data_->cost_weight = sejong::Vector::Zero(single_contact_->getDim());
  for(int i(0); i<single_contact_->getDim(); ++i){
    wbdc_data_->cost_weight[i] = 1.;
  }
  wbdc_data_->cost_weight[2] = 0.0001;
}

void BodyFootPlanningCtrl::FirstVisit(){
  // printf("[Body Foot Ctrl] Start\n");

  robot_model_->getPosition(sp_->Q_, swing_foot_, curr_foot_pos_des_);
  ctrl_start_time_ = sp_->curr_time_;
  state_machine_time_ = 0.;

  // sejong::pretty_print(target_foot_pos_, std::cout, "target foot pos");
  sejong::Vect3 zero;
  zero.setZero();
  default_target_loc_[0] = sp_->Q_[0];
  _SetBspline(curr_foot_pos_des_, zero, zero, default_target_loc_);
  num_planning_ = 0;
}

void BodyFootPlanningCtrl::_SetBspline(const sejong::Vect3 & st_pos,
                                       const sejong::Vect3 & st_vel,
                                       const sejong::Vect3 & st_acc,
                                       const sejong::Vect3 & target_pos){
  // Trajectory Setup
  double init[9];
  double fin[9];
  double** middle_pt = new double*[1];
  middle_pt[0] = new double[3];

  double portion = (1./end_time_) * (end_time_/2. - state_machine_time_);
  printf("portion: %f\n", portion);
  // Initial and final position & velocity & acceleration
  for(int i(0); i<3; ++i){
    // Initial
    init[i] = st_pos[i];
    init[i+3] = st_vel[i];
    init[i+6] = st_acc[i];
    // Final
    fin[i] = target_pos[i];
    fin[i+3] = 0.;
    fin[i+6] = 0.;

    if(portion > 0.)
      middle_pt[0][i] = (st_pos[i] + target_pos[i])*portion;
    else
      middle_pt[0][i] = (st_pos[i] + target_pos[i])/2.;
  }
  if(portion > 0.)  middle_pt[0][2] = swing_height_;

  foot_traj_.SetParam(init, fin, middle_pt, end_time_);

  update_time_ = state_machine_time_;

  delete [] *middle_pt;
  delete [] middle_pt;
}


void BodyFootPlanningCtrl::LastVisit(){
}

bool BodyFootPlanningCtrl::EndOfPhase(){
  if(state_machine_time_ > end_time_){
    // printf("[Body Foot Ctrl] End\n");
    return true;
  }
  return false;
}

void BodyFootPlanningCtrl::CtrlInitialization(std::string setting_file_name){
  robot_model_->getCoMPosition(sp_->Q_, ini_com_pos_);
  std::vector<double> tmp_vec;

  // Setting Parameters
  ParamHandler handler(CONFIG_PATH + setting_file_name + ".yaml");
  handler.getValue("swing_height", swing_height_);
  handler.getValue("planning_frequency", planning_frequency_);

  handler.getVector("default_target_foot_location", tmp_vec);
  for(int i(0); i<3; ++i){
    default_target_loc_[i] = tmp_vec[i];
  }

  // Feedback Gain
  handler.getVector("Kp", tmp_vec);
  for(int i(0); i<tmp_vec.size(); ++i){
    ((BodyFootTask*)body_foot_task_)->Kp_vec_[i] = tmp_vec[i];
  }
  handler.getVector("Kd", tmp_vec);
  for(int i(0); i<tmp_vec.size(); ++i){
    ((BodyFootTask*)body_foot_task_)->Kd_vec_[i] = tmp_vec[i];
  }
}
