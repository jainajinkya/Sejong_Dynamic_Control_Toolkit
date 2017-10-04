#include "LIPM_AC_RBF_Switching.h"
#include <Utils/utilities.h>
#include <stdio.h>

LIPM_AC_RBF_Switching::LIPM_AC_RBF_Switching():
  LIPM_ActorCritic_Learner()
{
  lipm_env_ = new LIPM_AC_RBF_Switching_System();

  dim_state_ = AC_SW_DIM_STATE;
  dim_action_ = AC_SW_DIM_ACTION;
  dim_theta_ = DIM_SW_THETA;
  dim_theta_half_ = DIM_SW_THETA_HALF;
  num_value_feature_ = NUM_SW_VALUE_FEATURE;
  num_action_feature_ = NUM_SW_ACTION_FEATURE;

  w_ = new double[NUM_SW_VALUE_FEATURE];
  w_eligi_ = new double[NUM_SW_VALUE_FEATURE];

  theta_ = new double*[DIM_SW_THETA];
  theta_eligi_ = new double*[DIM_SW_THETA];
  policy_gradient_ = new double*[DIM_SW_THETA];

  for(int i(0);i<DIM_SW_THETA; ++i){
    theta_[i] = new double [NUM_SW_ACTION_FEATURE];
    theta_eligi_[i] = new double [NUM_SW_ACTION_FEATURE];

    sejong::SetArrayZero(theta_[i], NUM_SW_ACTION_FEATURE);
    sejong::SetArrayZero(theta_eligi_[i], NUM_SW_ACTION_FEATURE);

    policy_gradient_[i] = new double[NUM_SW_ACTION_FEATURE];
  }
  for(int i(0);i<NUM_SW_VALUE_FEATURE; ++i){
    w_[i] = 0.1 * rand()/(double)RAND_MAX;
  }

  for(int k(0);k<DIM_SW_THETA; ++k){
    for(int i(0); i<NUM_SW_VALUE_FEATURE; ++i){
      theta_[k][i] = 0.001 * rand()/(double)RAND_MAX;
    }
  }
  theta_[0][NUM_SW_VALUE_FEATURE-1] = 0.1;
  theta_[1][NUM_SW_VALUE_FEATURE-1] = 0.1;

  ///// Parameters
  gamma_ = 0.9;
  alpha_ = 0.00001;
  beta_ =  0.00002;
  lambda_w_ = 0.8;
  lambda_theta_ = 0.8;

  I_ = 1.0;

  ///// Set Min & Max
  // xp
  action_min_[0] = 0.01;
  action_max_[0] = 0.40;
  // apex velocity
  action_min_[1] = 0.05;
  action_max_[1] = 0.35;

  ///// RBF Setting
  double ** mean  = new double*[NUM_SW_RBF_FEATURE];
  for(int i(0);i<NUM_SW_RBF_FEATURE; ++i){
    mean[i] = new double[AC_SW_DIM_STATE];
  }

  // x: -0.25, -0.15, -0.1, -0.05, 0.0 (5)
  // y:  0.05, 0.1, 0.15, 0.2, 0.25 (5)
  // xdot: 0.10, 0.2, 0.3, 0.4, 0.5, 0.6 (6)
  // ydot:  -0.55, -0.45, -0.35, -0.25, -0.15 (5)
  for(int ll(0); ll < NUM_X_SW_POS_GRID; ++ll){
    for (int k(0); k<NUM_Y_SW_POS_GRID; ++k){
      for(int i(0); i<NUM_X_SW_VEL_GRID; ++i){
        for(int j(0); j<NUM_Y_SW_VEL_GRID; ++j){
          mean[NUM_Y_SW_POS_GRID * NUM_X_SW_VEL_GRID * NUM_Y_SW_VEL_GRID * ll +
               NUM_X_SW_VEL_GRID * NUM_Y_SW_VEL_GRID*k +
               NUM_Y_SW_VEL_GRID * i +
               j][0] = X_SW_POS_MIN + X_SW_POS_RES*ll;

          mean[NUM_Y_SW_POS_GRID * NUM_X_SW_VEL_GRID * NUM_Y_SW_VEL_GRID * ll +
               NUM_X_SW_VEL_GRID * NUM_Y_SW_VEL_GRID*k +
               NUM_Y_SW_VEL_GRID * i +
               j][1] = Y_SW_POS_MIN + Y_SW_POS_RES*k;

          mean[NUM_Y_SW_POS_GRID * NUM_X_SW_VEL_GRID * NUM_Y_SW_VEL_GRID * ll +
               NUM_X_SW_VEL_GRID * NUM_Y_SW_VEL_GRID*k +
               NUM_Y_SW_VEL_GRID * i +
               j][2] = X_SW_VEL_MIN + X_SW_VEL_RES*i;

          mean[NUM_Y_SW_POS_GRID * NUM_X_SW_VEL_GRID * NUM_Y_SW_VEL_GRID * ll +
               NUM_X_SW_VEL_GRID * NUM_Y_SW_VEL_GRID*k +
               NUM_Y_SW_VEL_GRID * i  +
               j][3] = Y_SW_VEL_MIN + Y_SW_VEL_RES*j;
        }
      }
    }
  }
  // Print Mean List
  // for(int i(0); i<NUM_SW_RBF_FEATURE; ++i){
  //   sejong::pretty_print(mean[i], "mean list", AC_SW_DIM_STATE);
  // } exit(0);

  double* sigma = new double[NUM_SW_RBF_FEATURE];
  for(int i(0); i<NUM_SW_RBF_FEATURE; ++i){
    sigma[i] = 0.06;
  }
  // Set mean & sigma
  rbf_generic_.setMeanSigma(mean, sigma);

  for(int i(0); i<NUM_SW_RBF_FEATURE; ++i){
    delete [] mean[i];
  }
  delete [] mean;
  delete [] sigma;
}

LIPM_AC_RBF_Switching::~LIPM_AC_RBF_Switching(){
  delete [] w_;
  delete [] w_eligi_;

  for(int i(0);i<DIM_SW_THETA; ++i){
    delete [] theta_[i];
    delete [] theta_eligi_[i];
    delete [] policy_gradient_[i];
  }
  delete [] theta_;
  delete [] theta_eligi_;
  delete [] policy_gradient_;
}
void LIPM_AC_RBF_Switching::_Sampling_Initial_State(double * ini_state){
  double h = lipm_env_->GetHeight();
  double slope = sqrt(9.81/h);

  // x, xdot
  do {
    ini_state[0] = X_SW_POS_MIN+(X_SW_POS_MAX - X_SW_POS_MIN)*rand()/(double)RAND_MAX;
    ini_state[2] = X_SW_VEL_MIN+(X_SW_VEL_MAX - X_SW_VEL_MIN)*rand()/(double)RAND_MAX;
    // sejong::pretty_print(ini_state, "initial state", AC_SW_DIM_STATE);
  }while( !((ini_state[2] > slope * ini_state[0]) && (ini_state[2] > -slope * ini_state[0])) );

  // y, ydot
  do {
    ini_state[1] = Y_SW_POS_MIN+(Y_SW_POS_MAX - Y_SW_POS_MIN)*rand()/(double)RAND_MAX;
    ini_state[3] = Y_SW_VEL_MIN+(Y_SW_VEL_MAX - Y_SW_VEL_MIN)*rand()/(double)RAND_MAX;
    // sejong::pretty_print(ini_state, "initial state", AC_SW_DIM_STATE);
  }while( !((ini_state[3] < slope * ini_state[1]) && (ini_state[3] > -slope * ini_state[1])) );

}

void LIPM_AC_RBF_Switching::_GetValueFeature(const double* state,
                                            double* psi){
  double rbf_feature[NUM_SW_RBF_FEATURE];
  rbf_generic_.getGradient(state, rbf_feature);

  for(int i(0);i<NUM_SW_RBF_FEATURE; ++i){
    psi[i] = rbf_feature[i];
  }
  psi[NUM_SW_RBF_FEATURE] = 1.;
}

void LIPM_AC_RBF_Switching::_GetGradientLogPolicy(const double* state,
                                                 const double* action,
                                                 double** gradient){
  double rbf_feature[NUM_SW_RBF_FEATURE];
  rbf_generic_.getGradient(state, rbf_feature);

  double feature[NUM_SW_VALUE_FEATURE];
  for(int i(0);i<NUM_SW_RBF_FEATURE; ++i){
    feature[i] = rbf_feature[i];
  }
  feature[NUM_SW_RBF_FEATURE] = 1.;

  double mean_coeff[AC_SW_DIM_ACTION];
  double var_coeff[AC_SW_DIM_ACTION];

  _mean_var_gradient_coeff(feature, action, mean_coeff, var_coeff);
  // printf("mean coeff: %f, %f\n", mean_coeff[0], mean_coeff[1]);
  for(int i(0);i<DIM_SW_THETA_HALF; ++i){
    for(int j(0); j<NUM_SW_ACTION_FEATURE; ++j){
      gradient[i][j] = mean_coeff[i] * feature[j];
    }
  }
  for(int i(DIM_SW_THETA_HALF);i<DIM_SW_THETA; ++i){
    for(int j(0); j<NUM_SW_ACTION_FEATURE; ++j){
      gradient[i][j] = (-feature[j] + var_coeff[i] * feature[j]);
    }
  }
}

void LIPM_AC_RBF_Switching::_mean_var_gradient_coeff(const double * feature, const double * action, double * mean_coeff, double * var_coeff){

  double theta_var_feature, theta_mean_feature;

  for(int i(0);i<DIM_SW_THETA_HALF; ++i){
    theta_mean_feature = 0.;
    theta_var_feature = 0.;

    for(int k(0); k<NUM_SW_ACTION_FEATURE; ++k){
      theta_mean_feature += theta_[i][k] * feature[k];
      theta_var_feature += theta_[i + DIM_SW_THETA_HALF][k] * feature[k];
    }
    // printf("%d th theta mean feature: %f\n", i, theta_mean_feature);
    // printf("%d th theta var feature: %f\n", i, theta_var_feature);


    mean_coeff[i] = exp(-2. * theta_var_feature) * (action[i] - theta_mean_feature);
    var_coeff[i] = exp(-2. * theta_var_feature) * pow(action[i] - theta_mean_feature, 2.);
  }
  // printf("mean coeff: %f, %f\n", mean_coeff[0], mean_coeff[1]);
  // printf("var coeff: %f, %f\n", var_coeff[0], var_coeff[1]);

}

void LIPM_AC_RBF_Switching::_GetAction(const double* state,
                                      double* action){

  double rbf_feature[NUM_SW_RBF_FEATURE];
  rbf_generic_.getGradient(state, rbf_feature);

  double feature[NUM_SW_VALUE_FEATURE];
  for(int i(0);i<NUM_SW_RBF_FEATURE; ++i){
    feature[i] = rbf_feature[i];
  }
  feature[NUM_SW_RBF_FEATURE] = 1.;
  // sejong::pretty_print(feature, "generic feature", NUM_SW_VALUE_FEATURE);

  for(int k(0);k<DIM_SW_THETA_HALF; ++k){
    mean_[k] = 0.;
    variance_[k] = 0.;

    for(int i(0);i<NUM_SW_ACTION_FEATURE; ++i){
      mean_[k] += theta_[k][i] * feature[i];
      variance_[k] += theta_[DIM_SW_THETA_HALF + k][i] * feature[i];
    }
  }

  sejong::pretty_print(mean_, "mean", AC_SW_DIM_ACTION);
  for(int i(0);i<AC_SW_DIM_ACTION; ++i){
    printf("%dth variance: %f\n", i, exp(variance_[i]));
    if(exp(variance_[i]) < 0.04){
      printf("small enough variance!!\n");
      lets_finish_learning_ = true;
    }
  }

  for (int i(0);i<AC_SW_DIM_ACTION; ++i){
    if(sejong::MinMaxCheck(mean_[i], action_min_[i], action_max_[i]) ){
      printf("%d th action hit the limit: %f\n", i, mean_[i]);
      exit(0);
    }
    action[i] = sejong::generator_truncated_white_noise(mean_[i], exp(variance_[i]), action_min_[i], action_max_[i]);

    // action[i] = sejong::generator_white_noise(mean_[i], exp(variance_[i]));
  }
  sejong::pretty_print(action, "action", AC_SW_DIM_ACTION);
}

void LIPM_AC_RBF_Switching::_DoTest(const double* ini_state){
  
}

void LIPM_AC_RBF_Switching::GetPolicy(sejong::Vector & policy){
  // (t_s, xp, yp)
  
}
void LIPM_AC_RBF_Switching::_TestLearnedPolicy(){
  double reward;

  double ini_state[AC_SW_DIM_STATE]; // (y, xdot, ydot)
  double state[AC_SW_DIM_STATE];
  double nx_state[AC_SW_DIM_STATE];
  double action[AC_SW_DIM_ACTION];
  // ini_state[0] = -0.08;
  // ini_state[1] = 0.08;
  // ini_state[2] = 0.35;
  // ini_state[3] = -0.16;

  double pos[2];
  double vel[2];
  pos[0] = -0.105;
  pos[1] = 0.17;
  vel[0] = 0.4159;
  vel[1] = -0.41;

  double rot_pos[2];
  double rot_vel[2];
  double theta(-12. * M_PI/180.);

  _2D_Rotate(pos, -theta, rot_pos);
  _2D_Rotate(vel, -theta, rot_vel);

  ini_state[0] = rot_pos[0];
  ini_state[1] = rot_pos[1];
  ini_state[2] = rot_vel[0];
  ini_state[3] = rot_vel[1];


  // ini_state[0] = -0.01;
  // ini_state[1] = -0.052;
  // ini_state[2] = 0.05;
  // ini_state[3] = -0.17;


  bool b_Terminal(false);

  sejong::Copy(ini_state, state, AC_SW_DIM_STATE);
  int num_step(0);
  while(!b_Terminal){
    // Get Action (mean)
    double rbf_feature[NUM_SW_RBF_FEATURE];
    rbf_generic_.getGradient(state, rbf_feature);

    double feature[NUM_SW_VALUE_FEATURE];
    for(int i(0);i<NUM_SW_RBF_FEATURE; ++i){
      feature[i] = rbf_feature[i];
    }
    feature[NUM_SW_RBF_FEATURE] = 1.;

    for(int k(0);k<DIM_SW_THETA_HALF; ++k){
      action[k] = 0.;

      for(int i(0);i<NUM_SW_ACTION_FEATURE; ++i){
        action[k] += theta_[k][i] * feature[i];
      }
    }
    // Transition
    b_Terminal = lipm_env_->Transition(state, action, reward, nx_state);

    ++num_step;
    if(num_step < 20){
      printf("reward: %f\n", reward);
      sejong::pretty_print(action, "action", dim_action_);
      sejong::pretty_print(state, "state", dim_state_);
      sejong::pretty_print(nx_state, "nx_state", dim_state_);
      printf("num_step: %d\n", num_step);
    }
    sejong::Copy(nx_state, state, AC_SW_DIM_STATE);


    if(num_step > 50){
      exit(0);
    }
  }
}
