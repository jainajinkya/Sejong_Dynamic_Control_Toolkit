#include "LIPM_ActorCritic_Learner.h"
#include <Utils/utilities.h>
#include <environment/LIPM_3D_System.h>
#include <Configuration.h>
#include <iostream>
#include <fstream>

#define LEARNING 1

LIPM_ActorCritic_Learner::LIPM_ActorCritic_Learner():
  Agent(),
  lets_finish_learning_(false),
  lets_skip_learning_(false),
  skip_count_(0)
{
}

LIPM_ActorCritic_Learner::~LIPM_ActorCritic_Learner(){
}

bool LIPM_ActorCritic_Learner::DoLearning(const sejong::Vector & ini_state){

#if LEARNING
  ini_state_ = ini_state;
  // int num_learning(300000);
  // int num_learning(1000000);
  int num_learning(500);

  double* state = new double[dim_state_];
  double* state_save = new double[dim_state_];
  double* nx_state = new double[dim_state_];
  double* action = new double[dim_action_];
  double reward(0.);

  double* psi_nx = new double[num_value_feature_];
  double* psi = new double[num_value_feature_];

  int max_num_step(0);
  for(int iii(0); iii<num_learning; ++iii){
    _Sampling_Initial_State(state);
#if PRINT_MESSAGE
    sejong::pretty_print(state, "initial state", dim_state_);
#endif
    I_ = 1.0;

    sejong::SetArrayZero(w_eligi_, num_value_feature_);
    for(int i_th(0); i_th<dim_theta_; ++i_th){
      sejong::SetArrayZero(theta_eligi_[i_th], num_action_feature_);
    }

    bool b_Terminal(false);
    int num_step(0);
    if(skip_count_ > 5){
      break;
    }
    while(!b_Terminal) {
      _GetAction(state, action);
      if(lets_skip_learning_){
        lets_skip_learning_ = false;
        ++skip_count_;
        break;
      } else {
        skip_count_ = 0;
      }
      // TEST
      lipm_env_->num_step_ = 0;//num_step;
      // Transition
      b_Terminal = lipm_env_->Transition(state, action, reward, nx_state);
      _GetValueFeature(state, psi);
      _GetValueFeature(nx_state, psi_nx);
      _GetGradientLogPolicy(state, action, policy_gradient_);

      double delta;
      if(b_Terminal){
        delta = reward - sejong::Dot(w_, psi, num_value_feature_);
        double tmp = sejong::Dot(w_, psi, num_value_feature_);
#if PRINT_MESSAGE
        printf("curr value: %f \n", tmp);
#endif
      }else {
        delta = reward + gamma_ * sejong::Dot(w_, psi_nx, num_value_feature_) - sejong::Dot(w_, psi, num_value_feature_);
      }
#if PRINT_MESSAGE
      printf("delta, reward: %f, %f \n", delta, reward);
#endif
      for(int i(0); i<num_value_feature_; ++i){
        w_eligi_[i] = lambda_w_ * w_eligi_[i] + I_ * psi[i];
      }

      for(int i(0); i<dim_theta_; ++i){
        for(int j(0); j<num_action_feature_; ++j){
          theta_eligi_[i][j] = lambda_theta_ * theta_eligi_[i][j] + I_ * policy_gradient_[i][j];
        }
      }

      for(int i(0); i<num_value_feature_; ++i){
        w_[i] += beta_ * delta * w_eligi_[i];
      }

      for (int i(0); i<dim_theta_; ++i){
        for(int j(0); j<num_action_feature_; ++j){
          theta_[i][j] -= alpha_ * delta * theta_eligi_[i][j];
        }
      }
#if PRINT_MESSAGE
      bool b_printout(true);
#else
      bool b_printout(false);
#endif

      if(b_printout){
        sejong::pretty_print(action, "action", dim_action_);
        sejong::pretty_print(state, "state", dim_state_);
        sejong::pretty_print(nx_state, "nx_state", dim_state_);
      }
      sejong::Copy(state, state_save, dim_state_);
      sejong::Copy(nx_state, state, dim_state_);
      I_ *= gamma_;
#if PRINT_MESSAGE
      // _Print_Feature(psi, psi_nx);
      // _Print_Coefficient();
#endif

      ++num_step;
      if(num_step >max_num_step) {
        max_num_step = num_step;
      }
      if(num_step > 1000){
        _Print_Coefficient();
        // break;
      }
    }
    if(iii%100 == 1){
      printf("%i th learning, num step, max: %i, %i \n\n", iii, num_step, max_num_step);
      sejong::pretty_print(action, "action", dim_action_);
      sejong::pretty_print(state_save, "state", dim_state_);
      sejong::pretty_print(nx_state, "nx_state", dim_state_);
    }

    if(lets_finish_learning_){
      break;
    }
  }
  printf("max num step: %i\n", max_num_step);
  // _Print_Coefficient();
  _Save_Coefficient();
  _TestLearnedPolicy();

  return true;
#else
  _Read_Coefficient();
  _TestLearnedPolicy();


#endif
  return true;
}


void LIPM_ActorCritic_Learner::_EnvrionmentTEST(const double* ini_state){
  // double* action = new double[2];
  double action[2];

  action[0] = 0.2;
  action[1] = 0.1;
  double reward;
  double* nx_state = new double[4];
  double state[4];
  sejong::Copy(ini_state, state, 4);
  bool b_Terminal;
  for(int i(0);i<10; ++i){
    b_Terminal = lipm_env_->Transition(state, action, reward, nx_state);
    sejong::Copy(nx_state, state, 4);
  }
  sejong::pretty_print(ini_state, "ini_state", 4);
  sejong::pretty_print(nx_state, "nx_state", 4);
  printf("is terminal, reward: %i, %f\n", b_Terminal, reward);
}

double LIPM_ActorCritic_Learner::GetHeight(){
  return lipm_env_->GetHeight();
}

void LIPM_ActorCritic_Learner::_Print_Coefficient(){
  sejong::pretty_print(w_,  "w", num_value_feature_);

  for(int i(0); i<dim_theta_; ++i){
    sejong::pretty_print(theta_[i], "theta", num_action_feature_);
  }
  // for(int i(0); i<dim_theta_; ++i){
  //   sejong::pretty_print(theta_eligi_[i], "theta_eligi_", num_action_feature_);
  // }

}
void LIPM_ActorCritic_Learner::_Print_Feature(const double * curr_feature,
                                              const double * next_feature){
  sejong::pretty_print(curr_feature, "current value feature", num_value_feature_);
  sejong::pretty_print(next_feature, "next value feature", num_value_feature_);
}

void LIPM_ActorCritic_Learner::_Save_Coefficient(){
  sejong::saveVector(w_,  "w", num_value_feature_, true);

  for(int i(0); i<dim_theta_; ++i){
    sejong::saveVector(theta_[i], "theta", num_action_feature_, true);
  }
}
void LIPM_ActorCritic_Learner::_Read_Coefficient(){
  std::ifstream f_theta, f_w;

  f_theta.open(THIS_COM"parameter_data/theta.txt");
  f_w.open(THIS_COM"parameter_data/w.txt");

  int i(0);
  int j(0);
  while(!f_theta.eof()){
    f_theta>>theta_[i][j];
    ++j;
    if(j == num_action_feature_){
      j=0;
      ++i;
    }
  }
  i = 0;
  while(!f_w.eof()){
    f_w>>w_[i];
    ++i;
  }
  // sejong::pretty_print(w_,  "w", num_value_feature_);

  // for(int i(0); i<dim_theta_; ++i){
  //   sejong::pretty_print(theta_[i], "theta", num_action_feature_);
  // }
  
}

void LIPM_ActorCritic_Learner::_2D_Rotate(const double * vec, double theta, double* rotated_vec){
  rotated_vec[0] = vec[0] * cos(theta) - vec[1] * sin(theta);
  rotated_vec[1] = vec[0] * sin(theta) + vec[1] * cos(theta);
}
