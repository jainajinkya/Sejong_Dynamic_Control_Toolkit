#include "LIPM_ActOnly_Traj.h"
#include <environment/LIPM_3D_Act_Only.h>

LIPM_ActOnly_Traj::LIPM_ActOnly_Traj(LIPM_3D_Act_Only* lipm_env):
  LIPM_Act_Only_Learner(lipm_env)
{
  etha_ = 0.01;
  step_horizon_ = 4;

  dim_act_ = 3 * step_horizon_;

  // initial policy
  policy_ = sejong::Vector::Zero(dim_act_);
  for(int i(0); i<step_horizon_; ++i){
    policy_[3*i + 0] = 0.7;
    policy_[3*i + 1] = 0.17;

    if(i % 2 == 0 ){
      policy_[3*i + 2] = 0.3;
    } else {
      policy_[3*i + 2] = -0.3;
    }
  }

  // discrete action generation
  dim_allocation_ = 5;
  num_allocation_ = 200;

  delta_list_.resize(num_allocation_);
  reward_list_.resize(num_allocation_, 0.);
  for(int i(0); i<num_allocation_; ++i){
    delta_list_[i].resize(dim_act_, 0);
  }

  // Test Step size
  epsilon_.resize(3, 0.05);
  epsilon_[0] = 0.01;
}

LIPM_ActOnly_Traj::~LIPM_ActOnly_Traj(){
  
}

bool LIPM_ActOnly_Traj::_TestAction(const sejong::Vector & pivot_action,
                                    const std::vector< std::vector<int > > & delta_list,
                                    std::vector<double> & reward_list){
  double sum_reward(0.);
  sejong::Vector state, nx_state, action;


  for(int k(0); k < num_allocation_; ++k){

    state = ini_state_;
    for(int i(0); i < step_horizon_; ++i){
      double reward(0.);

      action = pivot_action.segment(3*i, 3);

      for(int j(0); j<3; ++j){
        action[j] += (double)delta_list[k][3*i + j] * epsilon_[j];
      }

      bool b_terminal = lipm_env_->Transition(state, action, reward, nx_state);
      sum_reward += reward;
      if(b_terminal){
        break;
      } else {
        sum_reward += 5.;
      }
      state = nx_state;
    }
    reward_list[k] = sum_reward;
    sum_reward = 0.;
  }
  return true;
}
