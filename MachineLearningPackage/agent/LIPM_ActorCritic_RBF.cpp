#include "LIPM_ActorCritic_RBF.h"
#include <Utils/utilities.h>
#include <stdio.h>

LIPM_ActorCritic_RBF::LIPM_ActorCritic_RBF():
  LIPM_ActorCritic_Learner()
{
  lipm_env_ = new LIPM_AC_RBF_System();

  dim_state_ = AC_DIM_STATE;
  dim_action_ = AC_DIM_ACTION;
  dim_theta_ = DIM_THETA;
  dim_theta_half_ = DIM_THETA_HALF;
  num_value_feature_ = NUM_VALUE_FEATURE;
  num_action_feature_ = NUM_ACTION_FEATURE;

  w_ = new double[NUM_VALUE_FEATURE];
  w_eligi_ = new double[NUM_VALUE_FEATURE];

  theta_ = new double*[DIM_THETA];
  theta_eligi_ = new double*[DIM_THETA];
  policy_gradient_ = new double*[DIM_THETA];

  for(int i(0);i<DIM_THETA; ++i){
    theta_[i] = new double [NUM_ACTION_FEATURE];
    theta_eligi_[i] = new double [NUM_ACTION_FEATURE];

    sejong::SetArrayZero(theta_[i], NUM_ACTION_FEATURE);
    sejong::SetArrayZero(theta_eligi_[i], NUM_ACTION_FEATURE);

    policy_gradient_[i] = new double[NUM_ACTION_FEATURE];
  }
  for(int i(0);i<NUM_VALUE_FEATURE; ++i){
    w_[i] = 0.1 * rand()/(double)RAND_MAX;
  }

  for(int k(0);k<DIM_THETA; ++k){
    for(int i(0); i<NUM_VALUE_FEATURE; ++i){
      theta_[k][i] = 0.001 * rand()/(double)RAND_MAX;
    }
  }
  theta_[0][NUM_VALUE_FEATURE-1] = 0.2;
  theta_[1][NUM_VALUE_FEATURE-1] = 0.3;

  theta_[2][NUM_VALUE_FEATURE-1] = -0.1;
  theta_[3][NUM_VALUE_FEATURE-1] = -0.1;

  ///// Parameters
  gamma_ = 0.9;
  alpha_ = 0.000005;
  beta_ =  0.000005;
  lambda_w_ = 0.8;
  lambda_theta_ = 0.8;

  I_ = 1.0;

  ///// Action Offset
  action_offset_[0] = 0.0;
  action_offset_[1] = 0.0;

  ///// Set Min & Max
  // xp
  action_min_[0] = 0.01;
  action_max_[0] = 0.55;
  // apex velocity
  action_min_[1] = X_VEL_MIN + 0.03;
  action_max_[1] = X_VEL_MAX - 0.03;
  // action_min_[1] = 0.1;
  // action_max_[1] = 0.40;

  ///// RBF Setting
  double ** mean  = new double*[NUM_RBF_FEATURE];
  for(int i(0);i<NUM_RBF_FEATURE; ++i){
    mean[i] = new double[AC_DIM_STATE];
  }

  // y: 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19, 0.20, 0.21, 0.22 (22)

  // xdot: 0.010, 0.025, 0.040, 0.055, 0.070, 0.085, 0.100, 0.115, 0.130,
  //       0.145, 0.160, 0.175, 0.190, 0.205, 0.220, 0.235, 0.250, 0.265,
  //       0.280, 0.295, 0.310, 0.325, 0.340, 0.355, 0.370, 0.385, 0.400,
  //       0.415, 0.430, 0.445, 0.460, 0.475, 0.490 (33)

  // ydot:
  // -0.320, -0.310, -0.300, -0.290, -0.280, -0.270, -0.260, -0.250,
  // -0.240, -0.230, -0.220, -0.210, -0.200, -0.190, -0.180, -0.170,
  // -0.160, -0.150, -0.140, -0.130, -0.120, -0.110, -0.100, -0.090,
  // -0.080, -0.070, -0.060, -0.050, -0.040, -0.030, -0.020, -0.010,
  //  0.0
  // 0.320, 0.310, 0.300, 0.290, 0.280, 0.270, 0.260, 0.250,
  // 0.240, 0.230, 0.220, 0.210, 0.200, 0.190, 0.180, 0.170,
  // 0.160, 0.150, 0.140, 0.130, 0.120, 0.110, 0.100, 0.090,
  // 0.080, 0.070, 0.060, 0.050, 0.040, 0.030, 0.020, 0.010,

  for (int k(0); k<NUM_Y_POS_GRID; ++k){
    for(int i(0); i<NUM_X_VEL_GRID; ++i){
      for(int j(0); j<NUM_Y_VEL_GRID; ++j){
        mean[NUM_X_VEL_GRID * NUM_Y_VEL_GRID*k + NUM_Y_VEL_GRID * i + j][0] = Y_POS_MIN + Y_POS_RES*k;
        mean[NUM_X_VEL_GRID * NUM_Y_VEL_GRID*k + NUM_Y_VEL_GRID * i + j][1] = X_VEL_MIN + X_VEL_RES*i;
        mean[NUM_X_VEL_GRID * NUM_Y_VEL_GRID*k + NUM_Y_VEL_GRID * i  + j][2] = Y_VEL_MIN + Y_VEL_RES*j;
      }
    }
  }
  // Print Mean List
  // for(int i(0); i<NUM_RBF_FEATURE; ++i){
  //   sejong::pretty_print(mean[i], "mean list", AC_DIM_STATE);
  // } exit(0);

  double* sigma = new double[NUM_RBF_FEATURE];
  for(int i(0); i<NUM_RBF_FEATURE; ++i){
    sigma[i] = RBF_SIGMA;
  }
  // Set mean & sigma
  rbf_generic_.setMeanSigma(mean, sigma);

  for(int i(0); i<NUM_RBF_FEATURE; ++i){
    delete [] mean[i];
  }
  delete [] mean;
  delete [] sigma;
}
void LIPM_ActorCritic_RBF::_GetAction(const double* state,
                                      double* action){

  double rbf_feature[NUM_RBF_FEATURE];
  rbf_generic_.getGradient(state, rbf_feature);

  double feature[NUM_VALUE_FEATURE];
  for(int i(0);i<NUM_RBF_FEATURE; ++i){
    feature[i] = rbf_feature[i];
  }
  feature[NUM_RBF_FEATURE] = 1.;
  // sejong::pretty_print(feature, "generic feature", NUM_VALUE_FEATURE);

  for(int k(0);k<DIM_THETA_HALF; ++k){
    mean_[k] = 0.;
    variance_[k] = 0.;

    for(int i(0);i<NUM_ACTION_FEATURE; ++i){
      mean_[k] += theta_[k][i] * feature[i];
      variance_[k] += theta_[DIM_THETA_HALF + k][i] * feature[i];
    }
  }
#if PRINT_MESSAGE
  sejong::pretty_print(mean_, "mean", AC_DIM_ACTION);
#endif
  for(int i(0);i<AC_DIM_ACTION; ++i){
#if PRINT_MESSAGE
    printf("%dth variance: %f\n", i, exp(variance_[i]));
#endif
    // printf("%dth variance: %f\n", i, exp(variance_[i]));

    // if(exp(variance_[i]) < 0.025){
    if(exp(variance_[i]) < 0.03){
      printf("small enough variance!!\n");
      lets_finish_learning_ = true;
    }
  }

  for (int i(0);i<AC_DIM_ACTION; ++i){
    if(sejong::MinMaxCheck(mean_[i], action_min_[i], action_max_[i]) ){
      printf("%d th action hit the limit: %f\n", i, mean_[i]);
      printf("variance: %f, %f \n", exp(variance_[0]), exp(variance_[1]));

      lets_finish_learning_ = true;
      // exit(0);
    }

    action[i] = sejong::generator_truncated_white_noise(mean_[i], exp(variance_[i]), action_min_[i], action_max_[i]);

    // action[i] = sejong::generator_white_noise(mean_[i], exp(variance_[i]));
  }
#if PRINT_MESSAGE
  sejong::pretty_print(action, "action", AC_DIM_ACTION);
#endif
}


void LIPM_ActorCritic_RBF::_TestLearnedPolicy(){
  printf("\n\n\n\n\n ****** TEST LEARNED POLICY ****** \n");

  double ini_state[AC_DIM_STATE]; // (y, xdot, ydot)

  // Original setup
  ini_state[0] = 0.095;
  ini_state[1] = 0.50;
  ini_state[2] = 0.0;

  // 1
  // ini_state[0] = 0.145;
  // ini_state[1] = 0.55;
  // ini_state[2] = 0.05;

  //2
  // ini_state[0] = 0.105;
  // ini_state[1] = 0.5;
  // ini_state[2] = 0.08;

  //3
  // ini_state[0] = 0.055;
  // ini_state[1] = 0.5;
  // ini_state[2] = -0.1;

  // _SingleTrialTest(ini_state);
  _MultipleTrialsTest();
}

void LIPM_ActorCritic_RBF::_MultipleTrialsTest(){
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

void LIPM_ActorCritic_RBF::_time_to_apex(const double* com_pos, const double* com_vel){
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

void LIPM_ActorCritic_RBF::_SingleStraightWalkingTest(const double * ini_state){
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
    sejong::pretty_print(state, "state", AC_DIM_STATE);
    sejong::pretty_print(action, "action", AC_DIM_ACTION);

    // Find xp, yp, t_switch
    b_Terminal = _Find_Foot_Switching_Time(state, action, b_flip, local_xp, local_yp, t_switch);

    // Advance State to Switching Time
    double t(dt);
    while (t < t_switch){
      global_time += dt;
      sejong::saveValue(global_time, time_file_name);

      // _Advance_OneStep(local_com_pos, local_com_vel, dt);
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
void LIPM_ActorCritic_RBF::_GetState(double * com_pos, double * com_vel, double curr_time){
  double omega = sqrt(9.81/ lipm_env_->GetHeight());
  for(int i(0);i<2; ++i){
    com_pos[i] = A_[i] *exp(omega * curr_time) + B_[i] * exp(-omega * curr_time);
    com_vel[i] = A_[i] * omega * exp(omega * curr_time) - B_[i] * omega * exp(-omega * curr_time);
  }

}


void LIPM_ActorCritic_RBF::_SetLIPM_Param_With_Initial(const double * local_pos_ini, const double * vel_ini){
  double omega = sqrt(9.81/ lipm_env_->GetHeight());
  for(int i(0); i<2; ++i){
    A_[i] = 0.5 * (local_pos_ini[i] + vel_ini[i] / omega);
    B_[i] = 0.5 * (local_pos_ini[i] - vel_ini[i] / omega);
  }
}

void LIPM_ActorCritic_RBF::_SetTestStateList(std::vector<sejong::Vector> & ini_state_list){
  sejong::Vector test_state(3);

  // original
  test_state[0] = 0.055;
  test_state[1] = 0.2;
  test_state[2] = 0.0;
  ini_state_list.push_back(test_state);

  // right push
  test_state[0] = 0.120;
  test_state[1] = 0.33;
  test_state[2] = 0.13;
  ini_state_list.push_back(test_state);

  // left push
  test_state[0] = 0.07;
  test_state[1] = 0.18;
  test_state[2] = -0.05;
  ini_state_list.push_back(test_state);

  // forward push
  test_state[0] = 0.085;
  test_state[1] = 0.25;
  test_state[2] = -0.005;
  ini_state_list.push_back(test_state);

  // backward push
  // test_state[0] = 0.095;
  // test_state[1] = 0.4;
  // test_state[2] = 0.0;
  // ini_state_list.push_back(test_state);
}
