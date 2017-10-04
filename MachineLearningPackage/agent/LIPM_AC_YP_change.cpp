#include "LIPM_AC_YP_change.h"
#include <Utils/utilities.h>
#include <stdio.h>

LIPM_AC_YP_change::LIPM_AC_YP_change():
  LIPM_ActorCritic_Learner()
{
  lipm_env_ = new LIPM_3D_AC_YP_System();

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
    w_[i] = 0. * rand()/(double)RAND_MAX;
  }

  for(int k(0);k<DIM_THETA; ++k){
    for(int i(0); i<NUM_VALUE_FEATURE; ++i){
      theta_[k][i] = 0.00 * rand()/(double)RAND_MAX;
    }
  }
  theta_[0][NUM_VALUE_FEATURE-1] = 0.3;
  theta_[1][NUM_VALUE_FEATURE-1] = 0.22;
  theta_[2][NUM_VALUE_FEATURE-1] = 0.4;

  theta_[3][NUM_VALUE_FEATURE-1] = -3.0;
  theta_[4][NUM_VALUE_FEATURE-1] = -3.0;
  theta_[5][NUM_VALUE_FEATURE-1] = -3.0;

  ///// Parameters
  gamma_ = 0.9;
  alpha_ = 0.000001;
  beta_ =  0.000001;
  lambda_w_ = 0.90;
  lambda_theta_ = 0.90;

  I_ = 1.0;

  ///// action offset
  action_offset_[0] = 0.0;
  action_offset_[1] = 0.0;

  ///// set min & max
  // xp
  action_min_[0] = 0.01;
  action_max_[0] = 0.5;
  // apex velocity
  action_min_[1] = X_VEL_MIN + 0.03;
  action_max_[1] = 0.7;
  // Yp
  action_min_[2] = 0.05;
  action_max_[2] = 0.6;

  ///// RBF SETTING
  double ** mean  = new double*[NUM_RBF_FEATURE];
  for(int i(0);i<NUM_RBF_FEATURE; ++i){
    mean[i] = new double[AC_DIM_STATE];
  }

  for (int k(0); k<NUM_Y_POS_GRID; ++k){
    for(int i(0); i<NUM_X_VEL_GRID; ++i){
      for(int j(0); j<NUM_Y_VEL_GRID; ++j){
        mean[NUM_X_VEL_GRID * NUM_Y_VEL_GRID*k + NUM_Y_VEL_GRID * i + j][0] = Y_POS_MIN + Y_POS_RES*k;
        mean[NUM_X_VEL_GRID * NUM_Y_VEL_GRID*k + NUM_Y_VEL_GRID * i + j][1] = X_VEL_MIN + X_VEL_RES*i;
        mean[NUM_X_VEL_GRID * NUM_Y_VEL_GRID*k + NUM_Y_VEL_GRID * i  + j][2] = Y_VEL_MIN + Y_VEL_RES*j;
      }
    }
  }

  double* sigma = new double[NUM_RBF_FEATURE];
  for(int i(0); i<NUM_RBF_FEATURE; ++i){
    sigma[i] = RBF_SIGMA;
  }
  // set mean & sigma
  rbf_generic_.setMeanSigma(mean, sigma);

  for(int i(0); i<NUM_RBF_FEATURE; ++i){
    delete [] mean[i];
  }
  delete [] mean;
  delete [] sigma;
}

void LIPM_AC_YP_change::_GetAction(const double* state,
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
    if(exp(variance_[i]) < 0.003){
      printf("small enough variance!!\n");
      lets_finish_learning_ = true;
    }
  }
  static int count(0);
  ++count;
  if(count % 500 == 0){
    printf("mean: %f, %f, %f \n", mean_[0], mean_[1], mean_[2]);

    printf("variance: %f, %f, %f \n", exp(variance_[0]), exp(variance_[1]), exp(variance_[2]));
  }


  for (int i(0);i<AC_DIM_ACTION; ++i){
    if(sejong::MinMaxCheck(mean_[i], action_min_[i], action_max_[i]) ){
      printf("%d th action hit the limit: %f\n", i, mean_[i]);
      printf("variance: %f, %f, %f \n", exp(variance_[0]), exp(variance_[1]), exp(variance_[2]));

      // lets_finish_learning_ = true;
      lets_skip_learning_ = true;
    }

    action[i] = sejong::generator_truncated_white_noise(mean_[i], exp(variance_[i]), action_min_[i], action_max_[i]);

    // action[i] = sejong::generator_white_noise(mean_[i], exp(variance_[i]));
  }
}


void LIPM_AC_YP_change::_TestLearnedPolicy(){
  printf("\n\n\n\n\n ****** TEST LEARNED POLICY ****** \n");
  _MultipleTrialsTest();
}

void LIPM_AC_YP_change::_MultipleTrialsTest(){
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

void LIPM_AC_YP_change::_time_to_apex(const double* com_pos, const double* com_vel){
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

void LIPM_AC_YP_change::_SingleStraightWalkingTest(const double * ini_state){
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
    sejong::saveVector(com_offset, foot_file_name, 2);

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
    // Find the time until x=0
    _time_to_apex(local_com_pos, local_com_vel);
    printf("in apex time, curr time: %f, %f\n", t_apex_, curr_time);

    // Advance Until Apex state
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
    printf("t switch: %f \n", t_switch);
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
void LIPM_AC_YP_change::_GetState(double * com_pos, double * com_vel, double curr_time){
  double omega = sqrt(9.81/ lipm_env_->GetHeight());
  for(int i(0);i<2; ++i){
    com_pos[i] = A_[i] *exp(omega * curr_time) + B_[i] * exp(-omega * curr_time);
    com_vel[i] = A_[i] * omega * exp(omega * curr_time) - B_[i] * omega * exp(-omega * curr_time);
  }

}


void LIPM_AC_YP_change::_SetLIPM_Param_With_Initial(const double * local_pos_ini, const double * vel_ini){
  double omega = sqrt(9.81/ lipm_env_->GetHeight());
  for(int i(0); i<2; ++i){
    A_[i] = 0.5 * (local_pos_ini[i] + vel_ini[i] / omega);
    B_[i] = 0.5 * (local_pos_ini[i] - vel_ini[i] / omega);
  }
}

void LIPM_AC_YP_change::_SetTestStateList(std::vector<sejong::Vector> & ini_state_list){
  sejong::Vector test_state(3);
  sejong::Vector normal_state(3);

  // original
  normal_state[0] = 0.056;
  normal_state[1] = 0.2;
  normal_state[2] = 0.0;
  ini_state_list.push_back(normal_state);

  double impulse(60.); // 60 N* sec (600 N / 0.1 sec)
  double body_mass(136.); // 136 kg
  double dt(0.1); // 0.1 sec
  double acc = impulse / body_mass / dt;

  double omega = sqrt(9.81/ lipm_env_->GetHeight());

  for(int i(0); i< 13; ++i){
    double angle = 90./180. * M_PI - 15./180. * M_PI * i;

    double acc_x = acc * cos (angle);
    double acc_y = acc * sin (angle);
    printf("acc: %f, %f\n", acc_x, acc_y);
    // X
    double Ax (0.5 * (1./(omega * omega) * acc_x + 1./omega * normal_state[1]));
    double Bx (0.5 * (1./(omega * omega) * acc_x - 1./omega * normal_state[1]));
    // Y
    double Ay (0.5 * ( normal_state[0] + 1./(omega * omega) * acc_y + 1./omega * normal_state[2]));
    double By (0.5 * ( normal_state[0] + 1./(omega * omega) * acc_y - 1./omega * normal_state[2]));


    double vel_change_x =  omega* (Ax * exp(omega * dt) - Bx * exp(-omega * dt));
    double vel_change_y =  omega* (Ay * exp(omega * dt) - By * exp(-omega * dt));

    double pos_change_x = Ax * exp(omega * dt) + Bx * exp(-omega * dt) - 1./(omega * omega) * acc_x;
    double pos_change_y = Ay * exp(omega * dt) + By * exp(-omega * dt) - 1./(omega * omega) * acc_y;

    double disturbed_pos[2];
    double disturbed_vel[2];
    // Pos
    disturbed_pos[0] = pos_change_x;
    disturbed_pos[1] = pos_change_y;
    // Vel
    disturbed_vel[0] = vel_change_x;
    disturbed_vel[1] = vel_change_y;

    // printf("disturbed state: %f, %f, %f, %f\n",
    //        disturbed_pos[0], disturbed_pos[1],
    //        disturbed_vel[0], disturbed_vel[1]);

    _FindZeroCoM_X(disturbed_pos, disturbed_vel);
    // printf("disturbed state come back: %f, %f, %f, %f\n",
    //        disturbed_pos[0], disturbed_pos[1],
    //        disturbed_vel[0], disturbed_vel[1]);
    // printf("\n");

    // Check
    if(fabs(disturbed_pos[0]) > 0.000001){
      printf("disturbed x is not zero: %f, %f, %f, %f\n",
             disturbed_pos[0], disturbed_pos[1],
             disturbed_vel[0], disturbed_vel[1]);
      exit(0);
    }

    test_state[0] = disturbed_pos[1];
    test_state[1] = disturbed_vel[0];
    test_state[2] = disturbed_vel[1];
    ini_state_list.push_back(test_state);
  }

  for(int i(0); i<ini_state_list.size(); ++i){
    sejong::pretty_print(ini_state_list[i], std::cout, "ini_state");
    printf("\n");
  }
}

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
LIPM_AC_YP_change::~LIPM_AC_YP_change(){
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
void LIPM_AC_YP_change::_Sampling_Initial_State(double * ini_state){
  // double h = lipm_env_->GetHeight();
  // double slope = sqrt(9.81/h);
  // // y, xdot, ydot
  // ini_state[1] = X_VEL_MIN+(X_VEL_MAX - X_VEL_MIN)*rand()/(double)RAND_MAX;
  // do {
  //   ini_state[0] = Y_POS_MIN+(Y_POS_MAX - Y_POS_MIN)*rand()/(double)RAND_MAX;
  //   ini_state[2] = Y_VEL_MIN+(Y_VEL_MAX - Y_VEL_MIN)*rand()/(double)RAND_MAX;
  //   // sejong::pretty_print(ini_state, "initial state", AC_DIM_STATE);
  // }while( !((ini_state[2] < slope * ini_state[0]) && (ini_state[2] > -slope * ini_state[0])) );
  sejong::Vector test_state(3);
  sejong::Vector normal_state(3);

  // original
  normal_state[0] = 0.056;
  normal_state[1] = 0.2;
  normal_state[2] = 0.0;

  double impulse(60.); // 60 N* sec (600 N / 0.1 sec)
  double body_mass(136.); // 136 kg
  double dt(0.1); // 0.1 sec
  double acc = impulse / body_mass / dt;
  double omega = sqrt(9.81/ lipm_env_->GetHeight());

  static int count(0);
  ++count;

  int trial = count%13;

  double angle = 90./180. * M_PI - 15./180. * M_PI * trial;

  double acc_x = acc * cos (angle);
  double acc_y = acc * sin (angle);
  // X
  double Ax (0.5 * (1./(omega * omega) * acc_x + 1./omega * normal_state[1]));
  double Bx (0.5 * (1./(omega * omega) * acc_x - 1./omega * normal_state[1]));
  // Y
  double Ay (0.5 * ( normal_state[0] + 1./(omega * omega) * acc_y + 1./omega * normal_state[2]));
  double By (0.5 * ( normal_state[0] + 1./(omega * omega) * acc_y - 1./omega * normal_state[2]));

  double vel_change_x =  omega* (Ax * exp(omega * dt) - Bx * exp(-omega * dt));
  double vel_change_y =  omega* (Ay * exp(omega * dt) - By * exp(-omega * dt));
  
  double pos_change_x = Ax * exp(omega * dt) + Bx * exp(-omega * dt) - 1./(omega * omega) * acc_x;
  double pos_change_y = Ay * exp(omega * dt) + By * exp(-omega * dt) - 1./(omega * omega) * acc_y;
  
  double disturbed_pos[2];
  double disturbed_vel[2];
  // Pos
  disturbed_pos[0] = pos_change_x;
  disturbed_pos[1] = pos_change_y;
  // Vel
  disturbed_vel[0] = vel_change_x;
  disturbed_vel[1] = vel_change_y;
  _FindZeroCoM_X(disturbed_pos, disturbed_vel);

  test_state[0] = disturbed_pos[1];
  test_state[1] = disturbed_vel[0];
  test_state[2] = disturbed_vel[1];
  // push left
  for (int i(0); i < 3; ++i){
    ini_state[i] = test_state[i] + 0.02 * rand()/(double)RAND_MAX;
  }

  // push forward
  // ini_state[0] = 0.05815 + 0.02 * rand()/(double)RAND_MAX;
  // ini_state[1] = 0.6295 + 0.02 * rand()/(double)RAND_MAX;
  // ini_state[2] = -0.09615 + 0.02 * rand()/(double)RAND_MAX;

}

void LIPM_AC_YP_change::_GetValueFeature(const double* state,
                                            double* psi){
  double rbf_feature[NUM_RBF_FEATURE];
  rbf_generic_.getGradient(state, rbf_feature);

  for(int i(0);i<NUM_RBF_FEATURE; ++i){
    psi[i] = rbf_feature[i];
  }
  psi[NUM_RBF_FEATURE] = 1.;
}

void LIPM_AC_YP_change::_GetGradientLogPolicy(const double* state,
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

void LIPM_AC_YP_change::_mean_var_gradient_coeff(const double * feature, const double * action, double * mean_coeff, double * var_coeff){

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

void LIPM_AC_YP_change::_DoTest(const double* ini_state){
  
}

void LIPM_AC_YP_change::GetPolicy(sejong::Vector & policy){
  // (t_s, xp, yp)
  
}

void LIPM_AC_YP_change::_GetLearned_Action(const double* state, double* action){
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


void LIPM_AC_YP_change::_Change_Global_To_Local(const double* global_pos, const double* global_vel, const double* offset, const double& theta, double* local_pos, double * local_vel ){
  double off_pos[2];
  off_pos[0] = global_pos[0] - offset[0];
  off_pos[1] = global_pos[1] - offset[1];
  _2D_Rotate(off_pos, -theta, local_pos);
  _2D_Rotate(global_vel, -theta, local_vel);
}

void LIPM_AC_YP_change::_Change_Local_To_Global(const double* local_pos, const double* local_vel, const double* offset, const double& theta, double* global_pos, double * global_vel ){
  _2D_Rotate(local_pos, theta, global_pos);
  _2D_Rotate(local_vel, theta, global_vel);

  global_pos[0] += offset[0];
  global_pos[1] += offset[1];
}

bool LIPM_AC_YP_change::_Change_Local_to_ActionState(const double * com_pos, const double* com_vel, double * state){
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

bool LIPM_AC_YP_change::_Find_Foot_Switching_Time(const double *state, const double* action, const bool & b_flip, double & local_xp, double & local_yp, double & t_switch){
  double nx_state[AC_DIM_STATE];
  double reward;

  bool b_terminal = lipm_env_->Transition(state, action, reward, nx_state);
  sejong::pretty_print(state, "state", AC_DIM_STATE);
  sejong::pretty_print(action, "action", AC_DIM_ACTION);
  sejong::pretty_print(nx_state, "nx_state", AC_DIM_STATE);

  local_xp = action[0];
  ((LIPM_3D_AC_YP_System*)lipm_env_)->Get_Yp_SwitchingTime( local_yp, t_switch);

  return b_terminal;
}

void LIPM_AC_YP_change::_FindZeroCoM_X(double* com_pos, double* com_vel){
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
