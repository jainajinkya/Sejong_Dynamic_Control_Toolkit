#include "LIPM_Act_Only_Learner.h"
#include <environment/LIPM_3D_Act_Only.h>
#include <Utils/utilities.h>
#include "LIPM_ActOnly_Tester.h"
#include <stdio.h>

LIPM_Act_Only_Learner::LIPM_Act_Only_Learner(LIPM_3D_Act_Only* lipm_env):
  Agent(),
  lipm_env_(lipm_env),
  best_reward_(-1.e9)
{
  lipm_tester_ = new LIPM_ActOnly_Tester();
}

LIPM_Act_Only_Learner::~LIPM_Act_Only_Learner(){
}
double LIPM_Act_Only_Learner::GetHeight() { return lipm_env_->GetHeight(); }

bool LIPM_Act_Only_Learner::DoLearning(const sejong::Vector & ini_state){
  ini_state_ = ini_state;
  sejong::Vector gradient(dim_act_);

  int num_trials(0);
  int learning_limit(100);
  srand(time(NULL));

  bool all_set(false);
  int num_pos, num_neg, num_zero;
  int num_ok;

  while(num_trials < learning_limit){
    // Generate delta list
    all_set = false;

    while(!all_set){
      _GenerateDeltaList(delta_list_);

      num_ok = 0;
      for(int i(0); i<dim_act_; ++i){
        num_pos = 0;
        num_neg = 0;
        num_zero = 0;

        for (int j(0); j<num_allocation_; ++j){
          if( delta_list_[j][i] == 1 ){
            ++num_pos;
          } else if( delta_list_[j][i] == -1){
            ++num_neg;
          } else {
            ++num_zero;
          }
        }
        if( (num_pos > 0) && (num_neg > 0) && (num_zero > 0)){
          ++num_ok;
        }
      }
      if(num_ok == dim_act_) all_set = true;
    }
    // _PrintDeltaList();

    // Try each action and save reward
    _TestAction(policy_, delta_list_, reward_list_);
    // _PrintRewardList();
    // Find Policy Gradient
    if ( _FindGradient(delta_list_, reward_list_, gradient) ){
      // Update Policy
      policy_ += gradient;
    } else {
      printf("learning is completed\n");
      break;
    }
    if( num_trials % 100 == 0){
      printf("policy: \n");
      for(int i(0); i<step_horizon_; ++i){
        printf("%f, %f, %f \n", policy_[3*i], policy_[3*i +1], policy_[3*i +2]);
      }
      printf("num_trials: %d\n", num_trials);
    }
    ++num_trials;
  }

  printf("Final policy: \n");
  for(int i(0); i<step_horizon_; ++i){
    printf("%f, %f, %f \n", best_policy_[3*i], best_policy_[3*i +1], best_policy_[3*i +2]);
  }
  lipm_tester_->Build_Test_Information(lipm_env_, this);
}

bool LIPM_Act_Only_Learner::_GenerateDeltaList(std::vector< std::vector<int> > & delta_list){
  // _PrintDeltaList();
  for(int i(0); i<num_allocation_; ++i){
    for(int k(0); k<dim_act_; ++k){
      delta_list[i][k] = _rand();
    }
  }
  return true;
}

int LIPM_Act_Only_Learner::_rand(){
  int delta(0);
  std::rand();
  double rand = std::rand()/((double) RAND_MAX)  * 1.5;

  // printf("rand: %f\n", rand);
  if(rand > 1.0  ){
    delta = 1;
  } else if(rand > 0.5){
    delta = -1;
  }
  return delta;
}

bool LIPM_Act_Only_Learner::_FindGradient(const std::vector<std::vector<int> > & delta_list,
                                          const std::vector<double> & reward_list,
                                          sejong::Vector & grad){
  grad = sejong::Vector::Zero(dim_act_);

  double sum_pos_reward(0.);
  double sum_neg_reward(0.);
  double sum_zero_reward(0.);

  double ave_pos_reward, ave_neg_reward, ave_zero_reward;

  int num_pos, num_neg, num_zero;

  for(int i(0); i<dim_act_; ++i){
    num_pos = 0;
    num_neg = 0;
    num_zero = 0;

    for (int j(0); j<num_allocation_; ++j){
      if( delta_list[j][i] == 1 ){
        ++num_pos;
        sum_pos_reward += reward_list_[j];
      } else if( delta_list[j][i] == -1){
        ++num_neg;
        sum_neg_reward += reward_list_[j];
      } else {
        ++num_zero;
        sum_zero_reward += reward_list_[j];
      }
    }
    ave_pos_reward = sum_pos_reward/ ((double)num_pos);
    ave_neg_reward = sum_neg_reward/ ((double)num_neg);
    ave_zero_reward = sum_zero_reward/ ((double)num_zero);

    if( (ave_zero_reward > ave_pos_reward) &&
        (ave_zero_reward > ave_neg_reward) ){
      grad[i] = 0.;
    } else {
      grad[i] = ave_pos_reward - ave_neg_reward;
    }
    num_pos = 0;
    num_neg = 0;
    num_zero = 0;

    sum_pos_reward = 0.;
    sum_neg_reward = 0.;
    sum_zero_reward = 0.;
  }
  double size_grad = grad.norm();
  if(size_grad < 0.0001){
    return false;
  }
  grad.normalize();
  grad *= etha_;

  return true;
}


////////////////////////////////////////////////
void LIPM_Act_Only_Learner::_PrintDeltaList(){
  for(int i(0); i< dim_act_; ++i){
    for(int j(0); j<num_allocation_; ++j){
      printf("%d, \t", delta_list_[j][i]);
    }
    printf("\n");
  }
}

void LIPM_Act_Only_Learner::_PrintRewardList(){
  for(int j(0); j<num_allocation_; ++j){
    printf("%f, \t", reward_list_[j]);
  }
  printf("\n");
}
void LIPM_Act_Only_Learner::_ShortenState(const sejong::Vector & full_state,
                                          sejong::Vector & shorten_state){
  shorten_state = sejong::Vector::Zero(4);
  // (x, y, z, xdot, ydot, zdot) --> (x, y, xdot, ydot)
  shorten_state[0] = full_state[0];
  shorten_state[1] = full_state[1];

  shorten_state[2] = full_state[3];
  shorten_state[3] = full_state[4];
}

void LIPM_Act_Only_Learner::_ExtendState(const sejong::Vector & shorten_state,
                                         sejong::Vector & full_state){
  full_state = sejong::Vector::Zero(6);
  // (x, y, z, xdot, ydot, zdot) <-- (x, y, xdot, ydot)
  full_state[0] = shorten_state[0];
  full_state[1] = shorten_state[1];
  full_state[2] = lipm_env_->GetHeight();

  full_state[3] = shorten_state[2];
  full_state[4] = shorten_state[3];
}
