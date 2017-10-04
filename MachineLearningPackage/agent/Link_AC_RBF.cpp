#include "Link_AC_RBF.h"
#include <Utils/utilities.h>
#include <stdio.h>

Link_AC_RBF::Link_AC_RBF(Linkage_System* link_sys):
  Link_ActCritic_Learner(link_sys)
{
  ///// Parameters
  gamma_ = 0.9;
  alpha_ = 0.001; // Action
  beta_ = 0.003; // Value
  lambda_w_ = 0.05;
  lambda_theta_ = 0.05;

  I_ = 1.0;

  ///// Set Min & Max
  action_min_[0] = -3.;
  action_max_[0] = 3.;

  ///// RBF Setting
  double ** mean  = new double*[NUM_LINK_RBF];
  for(int i(0);i<NUM_LINK_RBF; ++i){
    mean[i] = new double[DIM_LINK_STATE];
  }

  // x: -0.15, -0.1, -0.05, 0.0, 0.05, 0.1, 0.15 (7)
  // xdot: -0.35, -0.25, -0.15, -0.05, 0.05, 0.15, 0.25, 0.35 (8)
  for (int k(0); k<NUM_POS_GRID; ++k){
    for(int i(0); i<NUM_VEL_GRID; ++i){
      mean[NUM_POS_GRID*k + i][0] = POS_MIN + POS_RES*k;
      mean[NUM_POS_GRID*k + i][1] = VEL_MIN + VEL_RES*i;
    }
  }

  // // Print Mean List
  // for(int i(0); i<NUM_LINK_RBF; ++i){
  //   sejong::pretty_print(mean[i], "mean list", 2);
  // }

  double* sigma = new double[NUM_LINK_RBF];
  for(int i(0); i<NUM_LINK_RBF; ++i){
    sigma[i] = 0.15;
  }
  // Set mean & sigma
  rbf_generic_.setMeanSigma(mean, sigma);

  for(int i(0); i<NUM_LINK_RBF; ++i){
    delete [] mean[i];
  }
  delete [] mean;
  delete [] sigma;
}

Link_AC_RBF::~Link_AC_RBF(){}

void Link_AC_RBF::_GetValueFeature(const double* state,
                                            double* psi){
  double rbf_feature[NUM_LINK_RBF];
  rbf_generic_.getGradient(state, rbf_feature);

  for(int i(0);i<NUM_LINK_RBF; ++i){
    psi[i] = rbf_feature[i];
  }
  psi[NUM_LINK_RBF] = 1.;
}

void Link_AC_RBF::_GetGradientLogPolicy(const double* state,
                                        const double* action,
                                        double** gradient){

  double rbf_feature[NUM_LINK_RBF];
  rbf_generic_.getGradient(state, rbf_feature);

  double feature[NUM_LINK_FEATURE];
  for(int i(0);i<NUM_LINK_RBF; ++i){
    feature[i] = rbf_feature[i];
  }
  feature[NUM_LINK_RBF] = 1.;

  double mean_coeff[DIM_LINK_ACTION];
  double var_coeff[DIM_LINK_ACTION];

  _mean_var_gradient_coeff(feature, action, mean_coeff, var_coeff);
  // printf("mean coeff: %f, %f\n", mean_coeff[0], mean_coeff[1]);
  for(int i(0);i<DIM_LINK_THETA_HALF; ++i){
    for(int j(0); j<NUM_LINK_FEATURE; ++j){
      gradient[i][j] = mean_coeff[i] * feature[j];
    }
  }
  for(int i(DIM_LINK_THETA_HALF);i<DIM_LINK_THETA; ++i){
    for(int j(0); j<NUM_LINK_FEATURE; ++j){
      gradient[i][j] = (-feature[j] + var_coeff[i] * feature[j]);
    }
  }
}

void Link_AC_RBF::_mean_var_gradient_coeff(const double * feature, const double * action, double * mean_coeff, double * var_coeff){

  double theta_var_feature, theta_mean_feature;

  for(int i(0);i<DIM_LINK_THETA_HALF; ++i){
    theta_mean_feature = 0.;
    theta_var_feature = 0.;

    for(int k(0); k<NUM_LINK_FEATURE; ++k){
      theta_mean_feature += theta_[i][k] * feature[k];
      theta_var_feature += theta_[i + DIM_LINK_THETA_HALF][k] * feature[k];
    }
    // printf("%d th theta mean feature: %f\n", i, theta_mean_feature);
    // printf("%d th theta var feature: %f\n", i, theta_var_feature);

    mean_coeff[i] = exp(-2. * theta_var_feature) * (action[i] - theta_mean_feature);
    var_coeff[i] = exp(-2. * theta_var_feature) * pow(action[i] - theta_mean_feature, 2.);
  }
  // printf("mean coeff: %f, %f\n", mean_coeff[0], mean_coeff[1]);
  // printf("var coeff: %f, %f\n", var_coeff[0], var_coeff[1]);
}

void Link_AC_RBF::_GetAction(const double* state,
                             double* action){

  double rbf_feature[NUM_LINK_RBF];
  rbf_generic_.getGradient(state, rbf_feature);

  double feature[NUM_LINK_FEATURE];
  for(int i(0);i<NUM_LINK_RBF; ++i){
    feature[i] = rbf_feature[i];
  }
  feature[NUM_LINK_RBF] = 1.;
  // sejong::pretty_print(feature, "generic feature", NUM_LINK_FEATURE);

  for(int k(0);k<DIM_LINK_THETA_HALF; ++k){
    mean_[k] = 0.;
    variance_[k] = 0.;

    for(int i(0);i<NUM_LINK_FEATURE; ++i){
      mean_[k] += theta_[k][i] * feature[i];
      variance_[k] += theta_[DIM_LINK_THETA_HALF + k][i] * feature[i];
    }
  }
  for(int i(0); i<DIM_LINK_ACTION; ++i){
    if(exp(variance_[i]) > 4.15){
      printf("%dth variance: %f\n", i, exp(variance_[i]));
    }
  }

  // sejong::pretty_print(mean_, "mean", DIM_LINK_ACTION);
  for(int i(0);i<DIM_LINK_ACTION; ++i){
    if(exp(variance_[i]) < 0.05){
      printf("%dth variance: %f\n", i, exp(variance_[i]));
      exit(0);
    }
  }

  for (int i(0);i<DIM_LINK_ACTION; ++i){
    if(sejong::MinMaxCheck(mean_[i], action_min_[i], action_max_[i]) ){
      printf("%d th action hit the limit: %f\n", i, mean_[i]);
      exit(0);
    }
    action[i] = sejong::generator_truncated_white_noise(mean_[i], exp(variance_[i]), action_min_[i], action_max_[i]);

    // action[i] = sejong::generator_white_noise(mean_[i], exp(variance_[i]));
  }
  // sejong::pretty_print(action, "action", DIM_LINK_ACTION);
}
