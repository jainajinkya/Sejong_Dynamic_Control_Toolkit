#include "Mace_Q_Learning.h"
#include <Utils/utilities.h>
#include <iostream>
#include <stdio.h>

Mace_Q_learning::Mace_Q_learning(Env_Mace* mace_env):Mace_learner(mace_env),
                                                     alpha_(0.01),
                                                     gamma_(0.9){

}

Mace_Q_learning::~Mace_Q_learning(){}

bool Mace_Q_learning::DoLearning(const sejong::Vector & ini_state){
  bool is_terminal;
  sejong::Vector state;
  sejong::Vector nx_state;
  sejong::Vector action(DIM_STATE);
  double reward(0.);

  std::map<std::vector<double>, double>::iterator iter;
  std::vector<double> aug_state_action;

  int num_learning(2000);
  int count(0);
  int max_count(55);
  for (int i(0); i< num_learning; ++i){
    state = ini_state;
    is_terminal = false;
    while(!is_terminal){
      int idx = rand() % NUM_ACT;
      _getAction(idx, action);
      is_terminal = mace_env_->Transition(state, action, reward, nx_state);
      _getAugmentedStateAction(state, action, aug_state_action);
  
      iter = q_value_.find(aug_state_action);
      if(iter == q_value_.end()){
        q_value_[aug_state_action] = 0.;
        iter = q_value_.find(aug_state_action);
        // sejong::pretty_print(iter->first, "new state");
      } else {
        // sejong::pretty_print(iter->first, "existing state");
      }
      // std::cout<<"q value: "<<iter->second<<std::endl;
      sejong::Vector opt_action;
      double max_Q = _FindMAX_Q(nx_state, opt_action);
      if(is_terminal){
        max_Q = 0.0;
      }
      // std::cout<<"max Q: "<<max_Q<<std::endl;
      iter->second += alpha_ * (reward + gamma_ * max_Q - iter->second);
      state = nx_state;

      ++count;
    }
    count = 0;
  }
  _DoTest(ini_state);
  return false;
}
void Mace_Q_learning::_getAugmentedStateAction(const sejong::Vector & state,
                             const sejong::Vector & action,
                             std::vector<double> & aug_state_action){
  aug_state_action.resize(DIM_STATE + DIM_ACTION);
  for (int i(0); i < DIM_STATE; ++i) {
    aug_state_action[i] = state[i];
  }
  for (int i(0); i< DIM_ACTION; ++i){
    aug_state_action[DIM_STATE + i] = action[i];
  }
}

double Mace_Q_learning::_FindMAX_Q(const sejong::Vector & state, sejong::Vector & opt_action){
  std::map<std::vector<double>, double>::iterator iter;
  std::vector<double> aug_state_action;
  sejong::Vector action;

  double max_value(-10000.0);
  for (int i(0); i<NUM_ACT; ++i){
    _getAction(i, action);
    _getAugmentedStateAction(state, action, aug_state_action);
    iter = q_value_.find(aug_state_action);
    if(iter == q_value_.end()){
      q_value_[aug_state_action] = 0.;
      iter = q_value_.find(aug_state_action);
      // printf("new q value: %f \n", iter->second);
    }
    if(iter->second > max_value){
      max_value = iter->second;
      opt_action = action;
    }
  }
  return max_value;
}
