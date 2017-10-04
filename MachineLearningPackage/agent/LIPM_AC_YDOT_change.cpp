#include "LIPM_AC_YDOT_change.h"
#include <Utils/utilities.h>
#include <stdio.h>

void LIPM_AC_YDOT_change::_GetAction(const double* state,
                                     double* action){

  double rbf_feature[NUM_RBF_FEATURE];
  rbf_generic_.getGradient(state, rbf_feature);

  double feature[NUM_VALUE_FEATURE];
  for(int i(0);i<NUM_RBF_FEATURE; ++i){
    feature[i] = rbf_feature[i];
  }
  feature[NUM_RBF_FEATURE] = 1.;

  for(int k(0);k<DIM_THETA_HALF; ++k){
    mean_[k] = 0.;
    variance_[k] = 0.;

    for(int i(0);i<NUM_ACTION_FEATURE; ++i){
      mean_[k] += theta_[k][i] * feature[i];
      variance_[k] += theta_[DIM_THETA_HALF + k][i] * feature[i];
    }
  }
  for(int i(0);i<AC_DIM_ACTION; ++i){
    if(exp(variance_[i]) < 0.07){
      printf("small enough variance!!\n");
      lets_finish_learning_ = true;
    }
  }

  for (int i(0);i<AC_DIM_ACTION; ++i){
    if(sejong::MinMaxCheck(mean_[i], action_min_[i], action_max_[i]) ){
      printf("%d th action hit the limit: %f\n", i, mean_[i]);
      printf("variance: %f, %f, %f \n", exp(variance_[0]), exp(variance_[1]), exp(variance_[2]));

      lets_finish_learning_ = true;
      // lets_skip_learning_ = true;
    }
    action[i] = sejong::generator_truncated_white_noise(mean_[i], exp(variance_[i]), action_min_[i], action_max_[i]);
  }

  static int count(0);
  ++count;
  if(count % 500 == 0){
    printf("mean: %f, %f, %f \n", mean_[0], mean_[1], mean_[2]);

    printf("variance: %f, %f, %f \n", exp(variance_[0]), exp(variance_[1]), exp(variance_[2]));
  }
}


void LIPM_AC_YDOT_change::_TestLearnedPolicy(){
  printf("\n\n\n\n\n ****** TEST LEARNED POLICY ****** \n");
  _MultipleTrialsTest();
}

void LIPM_AC_YDOT_change::_MultipleTrialsTest(){
  // Set initial state list
  std::vector<sejong::Vector> ini_state_list;
  _SetTestStateList(ini_state_list);
  int num_trials = ini_state_list.size();

  // Test each initial state
  double ini_state[AC_DIM_STATE]; // (y, xdot, ydot)

  for(int i(0); i<num_trials; ++i){

    for(int j(0); j<AC_DIM_STATE; ++j){
      ini_state[j] = ini_state_list[i][j];
    }
    _SingleStraightWalkingTest(ini_state);
  }
}

void LIPM_AC_YDOT_change::_time_to_apex(const double* com_pos, const double* com_vel){
  double omega = sqrt(9.81/ lipm_env_->GetHeight());

  double A (0.5 * ( com_pos[0] + 1./omega * com_vel[0]));
  double B (0.5 * ( com_pos[0] - 1./omega * com_vel[0]));

  t_apex_ = 1./(2.*omega) * log(-B/A);
  printf("apex time: %f\n", t_apex_);
  if(t_apex_ < 0.){
    printf("[LIPM AC RBF, Warning] t apex is negative\n");
    exit(0);
  }
}

void LIPM_AC_YDOT_change::_SingleStraightWalkingTest(const double * ini_state){
  static int trial_idx(0);
  ++trial_idx;

  int num_test(37);
  double com_offset[2];
  com_offset[0] = 0.;
  com_offset[1] = 0.;

  double dt(0.02);
  double com_pos[2];
  double com_vel[2];
  double local_xp, local_yp, t_switch;

  double state[AC_DIM_STATE];
  double nx_state[AC_DIM_STATE];
  double action[AC_DIM_ACTION];

  int _num_step(0);
  bool b_Terminal(false);
  double reward;
  double theta(0.);

  double global_switch_com_pos[2];
  double global_switch_com_vel[2];

  global_switch_com_pos[0] = 0.;
  global_switch_com_pos[1] = ini_state[0];
  global_switch_com_vel[0] = ini_state[1];
  global_switch_com_vel[1] = ini_state[2];

  double global_com_pos[2];
  double global_com_vel[2];
  double local_com_pos[2];
  double local_com_vel[2];

  bool b_flip;
  bool b_printout(false);
  bool b_printout_advance(false);

  char foot_file_name[100];
  sprintf(foot_file_name, "%dth_com_offset", trial_idx);
  char com_pos_file_name[100];
  sprintf(com_pos_file_name, "%dth_com_pos", trial_idx);
  char com_vel_file_name[100];
  sprintf(com_vel_file_name, "%dth_com_vel", trial_idx);
  char time_file_name[100];
  sprintf(time_file_name, "%dth_time", trial_idx);

  double global_time(0.);
  while(!b_Terminal && _num_step < num_test){
    // printf("theta: %f\n", theta);
    sejong::saveVector(com_offset, foot_file_name, 2);
    // sejong::saveVector(global_switch_com_pos, "com_switch", 2);
    // sejong::saveVector(global_switch_com_vel, "com_switch_vel", 2);

    ++_num_step;
    // Rotate Switching Statea To Local Frame
    _Change_Global_To_Local(global_switch_com_pos,
                            global_switch_com_vel,
                            com_offset,
                            theta,
                            local_com_pos,
                            local_com_vel);
    sejong::pretty_print(local_com_pos, "local_com pos 1",2);
    sejong::pretty_print(local_com_vel, "local_com vel 1",2);

    _SetLIPM_Param_With_Initial(local_com_pos, local_com_vel);
    if(b_printout){
      sejong::pretty_print(global_switch_com_pos, "global sw pos", 2);
      sejong::pretty_print(global_switch_com_vel, "global sw vel", 2);
    }
    double curr_time(0.);
    // Advance Local CoM state to x=0
    _time_to_apex(local_com_pos, local_com_vel);

    printf("in apex time, curr time: %f, %f\n", t_apex_, curr_time);

    while(curr_time < t_apex_){
      _GetState(local_com_pos, local_com_vel, curr_time);

      global_time += dt;
      sejong::saveValue(global_time, time_file_name);

      // sejong::pretty_print(local_com_pos, "local_com pos",2);
      // sejong::pretty_print(local_com_vel, "local_com vel",2);

      curr_time += dt;
      // For Save
      _Change_Local_To_Global(local_com_pos, local_com_vel, com_offset,
                              theta, global_com_pos, global_com_vel);

      sejong::saveVector(global_com_pos, com_pos_file_name, 2);
      sejong::saveVector(global_com_vel, com_vel_file_name, 2);
    }

    _GetState(local_com_pos, local_com_vel, t_apex_);
    sejong::pretty_print(local_com_pos, "local_com pos",2);
    sejong::pretty_print(local_com_vel, "local_com vel",2);

    curr_time = t_apex_;
    // For Save
    _Change_Local_To_Global(local_com_pos, local_com_vel, com_offset,
                            theta, global_com_pos, global_com_vel);

    sejong::saveVector(global_com_pos, com_pos_file_name, 2);
    sejong::saveVector(global_com_vel, com_vel_file_name, 2);
    // End of Advancement of Local CoM to x=0

    b_flip = _Change_Local_to_ActionState(local_com_pos, local_com_vel, state);
    // Get Action (mean)
    _GetLearned_Action(state, action);

    // Find xp, yp, t_switch
    b_Terminal = _Find_Foot_Switching_Time(state, action, b_flip, local_xp, local_yp, t_switch);

    // Advance State to Switching Time
    double t(dt);
    while (t < t_switch){
      global_time += dt;
      sejong::saveValue(global_time, time_file_name);

      _GetState(local_com_pos, local_com_vel, curr_time + t);

      _Change_Local_To_Global(local_com_pos, local_com_vel, com_offset,
                              theta, global_com_pos, global_com_vel);

      sejong::saveVector(global_com_pos, com_pos_file_name, 2);
      sejong::saveVector(global_com_vel, com_vel_file_name, 2);
      t += dt;
    }
    _GetState(local_com_pos, local_com_vel, curr_time + t_switch);

    _Change_Local_To_Global(local_com_pos, local_com_vel, com_offset,
                            theta, global_com_pos, global_com_vel);

    sejong::saveVector(global_com_pos, com_pos_file_name, 2);
    sejong::saveVector(global_com_vel, com_vel_file_name, 2);

    // Update
    sejong::Copy(global_com_pos, global_switch_com_pos, 2);
    sejong::Copy(global_com_vel, global_switch_com_vel, 2);
    double local_foot[2];
    double rotated_foot[2];
    local_foot[0] = local_xp;

    if(b_flip) local_foot[1] = -local_yp;
    else local_foot[1] = local_yp;

    _2D_Rotate(local_foot, theta, rotated_foot);
    com_offset[0] += rotated_foot[0];
    com_offset[1] += rotated_foot[1];
  }

}
void LIPM_AC_YDOT_change::_GetState(double * com_pos, double * com_vel, double curr_time){
  double omega = sqrt(9.81/ lipm_env_->GetHeight());
  for(int i(0);i<2; ++i){
    com_pos[i] = A_[i] *exp(omega * curr_time) + B_[i] * exp(-omega * curr_time);
    com_vel[i] = A_[i] * omega * exp(omega * curr_time) - B_[i] * omega * exp(-omega * curr_time);
  }

}


void LIPM_AC_YDOT_change::_SetLIPM_Param_With_Initial(const double * local_pos_ini, const double * vel_ini){
  double omega = sqrt(9.81/ lipm_env_->GetHeight());
  for(int i(0); i<2; ++i){
    A_[i] = 0.5 * (local_pos_ini[i] + vel_ini[i] / omega);
    B_[i] = 0.5 * (local_pos_ini[i] - vel_ini[i] / omega);
  }
}


///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
LIPM_AC_YDOT_change::~LIPM_AC_YDOT_change(){
  delete [] w_;
  delete [] w_eligi_;

  for(int i(0);i<DIM_THETA; ++i){
    delete [] theta_[i];
    delete [] theta_eligi_[i];
    delete [] policy_gradient_[i];
  }
  delete [] theta_;
  delete [] theta_eligi_;
  delete [] policy_gradient_;
}

void LIPM_AC_YDOT_change::_GetValueFeature(const double* state,
                                           double* psi){
  double rbf_feature[NUM_RBF_FEATURE];
  rbf_generic_.getGradient(state, rbf_feature);

  for(int i(0);i<NUM_RBF_FEATURE; ++i){
    psi[i] = rbf_feature[i];
  }
  psi[NUM_RBF_FEATURE] = 1.;
}

void LIPM_AC_YDOT_change::_GetGradientLogPolicy(const double* state,
                                                const double* action,
                                                double** gradient){
  double rbf_feature[NUM_RBF_FEATURE];
  rbf_generic_.getGradient(state, rbf_feature);

  double feature[NUM_VALUE_FEATURE];
  for(int i(0);i<NUM_RBF_FEATURE; ++i){
    feature[i] = rbf_feature[i];
  }
  feature[NUM_RBF_FEATURE] = 1.;

  double mean_coeff[AC_DIM_ACTION];
  double var_coeff[AC_DIM_ACTION];

  _mean_var_gradient_coeff(feature, action, mean_coeff, var_coeff);
  // printf("mean coeff: %f, %f\n", mean_coeff[0], mean_coeff[1]);
  for(int i(0);i<DIM_THETA_HALF; ++i){
    for(int j(0); j<NUM_ACTION_FEATURE; ++j){
      gradient[i][j] = mean_coeff[i] * feature[j];
    }
  }
  for(int i(DIM_THETA_HALF);i<DIM_THETA; ++i){
    for(int j(0); j<NUM_ACTION_FEATURE; ++j){
      gradient[i][j] = (-feature[j] + var_coeff[i] * feature[j]);
    }
  }
}

void LIPM_AC_YDOT_change::_mean_var_gradient_coeff(const double * feature, const double * action, double * mean_coeff, double * var_coeff){

  double theta_var_feature, theta_mean_feature;

  for(int i(0);i<DIM_THETA_HALF; ++i){
    theta_mean_feature = 0.;
    theta_var_feature = 0.;

    for(int k(0); k<NUM_ACTION_FEATURE; ++k){
      theta_mean_feature += theta_[i][k] * feature[k];
      theta_var_feature += theta_[i + DIM_THETA_HALF][k] * feature[k];
    }
    // printf("%d th theta mean feature: %f\n", i, theta_mean_feature);
    // printf("%d th theta var feature: %f\n", i, theta_var_feature);


    mean_coeff[i] = exp(-2. * theta_var_feature) * (action[i] - theta_mean_feature);
    var_coeff[i] = exp(-2. * theta_var_feature) * pow(action[i] - theta_mean_feature, 2.);
  }
  // printf("mean coeff: %f, %f\n", mean_coeff[0], mean_coeff[1]);
  // printf("var coeff: %f, %f\n", var_coeff[0], var_coeff[1]);

}

void LIPM_AC_YDOT_change::_DoTest(const double* ini_state){
  
}

void LIPM_AC_YDOT_change::GetPolicy(sejong::Vector & policy){
  // (t_s, xp, yp)
  
}
void LIPM_AC_YDOT_change::_SingleTrialTest(const double * ini_state_input){
  double reward;

  double ini_state[AC_DIM_STATE];
  sejong::Copy(ini_state_input, ini_state, AC_DIM_STATE);
  double state[AC_DIM_STATE];
  double nx_state[AC_DIM_STATE];
  double action[AC_DIM_ACTION];


  bool b_Terminal(false);
  double pos[2]; double rot_pos[2];
  double vel[2]; double rot_vel[2];
  double rotated_state[3];
  double theta(0.*M_PI/180.);
  // rotation
  pos[0] = 0.;
  pos[1] = ini_state[0];
  _2D_Rotate(pos, theta, rot_pos);
  vel[0] = ini_state[1];
  vel[1] = ini_state[2];
  _2D_Rotate(vel, theta, rot_vel);

  ini_state[0] = rot_pos[1];
  ini_state[1] = rot_vel[0];
  ini_state[2] = rot_vel[1];

  sejong::Copy(ini_state, state, AC_DIM_STATE);
  int num_step(0);
  while(!b_Terminal){
    // while(true){
    // Get Action (mean)
    double rbf_feature[NUM_RBF_FEATURE];
    rbf_generic_.getGradient(state, rbf_feature);

    double feature[NUM_VALUE_FEATURE];
    for(int i(0);i<NUM_RBF_FEATURE; ++i){
      feature[i] = rbf_feature[i];
    }
    feature[NUM_RBF_FEATURE] = 1.;

    for(int k(0);k<DIM_THETA_HALF; ++k){
      action[k] = 0.;

      for(int i(0);i<NUM_ACTION_FEATURE; ++i){
        action[k] += theta_[k][i] * feature[i];
      }
    }
    // Transition
    b_Terminal = lipm_env_->Transition(state, action, reward, nx_state, true);

    ++num_step;
    if(num_step < 4){
      printf("reward: %f\n", reward);
      sejong::pretty_print(action, "action", dim_action_);
      sejong::pretty_print(state, "state", dim_state_);
      sejong::pretty_print(nx_state, "nx_state", dim_state_);
      printf("num_step: %d\n", num_step);
    }
    sejong::Copy(nx_state, state, AC_DIM_STATE);


    if(num_step > 35){
      break;
    }
  }  
}


void LIPM_AC_YDOT_change::_GetLearned_Action(const double* state, double* action){
  double rbf_feature[NUM_RBF_FEATURE];
  rbf_generic_.getGradient(state, rbf_feature);

  double feature[NUM_VALUE_FEATURE];
  for(int i(0);i<NUM_RBF_FEATURE; ++i){
    feature[i] = rbf_feature[i];
  }
  feature[NUM_RBF_FEATURE] = 1.;

  for(int k(0);k<DIM_THETA_HALF; ++k){
    action[k] = 0.;

    for(int i(0);i<NUM_ACTION_FEATURE; ++i){
      action[k] += theta_[k][i] * feature[i];
    }
  }
}


void LIPM_AC_YDOT_change::_Change_Global_To_Local(const double* global_pos, const double* global_vel, const double* offset, const double& theta, double* local_pos, double * local_vel ){
  double off_pos[2];
  off_pos[0] = global_pos[0] - offset[0];
  off_pos[1] = global_pos[1] - offset[1];
  _2D_Rotate(off_pos, -theta, local_pos);
  _2D_Rotate(global_vel, -theta, local_vel);
}

void LIPM_AC_YDOT_change::_Change_Local_To_Global(const double* local_pos, const double* local_vel, const double* offset, const double& theta, double* global_pos, double * global_vel ){
  _2D_Rotate(local_pos, theta, global_pos);
  _2D_Rotate(local_vel, theta, global_vel);

  global_pos[0] += offset[0];
  global_pos[1] += offset[1];
}

bool LIPM_AC_YDOT_change::_Change_Local_to_ActionState(const double * com_pos, const double* com_vel, double * state){
  // Check com_pos[0] == 0.
  if(fabs(com_pos[0])> 2.e-4){
    printf("[Error] com pos is not zero: %f \n", com_pos[0]);
    exit(0);
  }

  if(com_pos[1] > 0.){
    state[0] = com_pos[1];
    state[1] = com_vel[0];
    state[2] = com_vel[1];
    return false;
  }
  if(com_pos[1] < 0.){
    state[0] = -com_pos[1];
    state[1] = com_vel[0];
    state[2] = -com_vel[1];
    return true;
  }
  return false;
}

bool LIPM_AC_YDOT_change::_Find_Foot_Switching_Time(const double *state, const double* action, const bool & b_flip, double & local_xp, double & local_yp, double & t_switch){
  double nx_state[AC_DIM_STATE];
  double reward;

  bool b_terminal = lipm_env_->Transition(state, action, reward, nx_state);
  sejong::pretty_print(state, "state", AC_DIM_STATE);
  sejong::pretty_print(action, "action", AC_DIM_ACTION);
  sejong::pretty_print(nx_state, "nx_state", AC_DIM_STATE);

  local_xp = action[0];
  ((LIPM_3D_AC_YDOT_System*)lipm_env_)->Get_Yp_SwitchingTime( local_yp, t_switch);

  return b_terminal;
}
void LIPM_AC_YDOT_change::_FindZeroCoM_X(double* com_pos, double* com_vel){
  double omega = sqrt(9.81/ lipm_env_->GetHeight());

  double A (0.5 * ( com_pos[0] + 1./omega * com_vel[0]));
  double B (0.5 * ( com_pos[0] - 1./omega * com_vel[0]));

  double t = 1./(2.*omega) * log(-B/A);
  // X
  com_pos[0] = 0.;
  com_vel[0] = omega * (A * exp(omega * t) - B * exp(-omega * t));
  // Y
  A = (0.5 * ( com_pos[1] + 1./omega * com_vel[1]));
  B = (0.5 * ( com_pos[1] - 1./omega * com_vel[1]));
  com_pos[1] = A * exp(omega * t) + B * exp(-omega * t);
  com_vel[1] = omega * (A * exp(omega * t) - B * exp(-omega * t));
}
