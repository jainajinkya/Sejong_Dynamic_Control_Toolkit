#include "Mace_Learner.h"
#include <Utils/utilities.h>
#include <iostream>
#include <stdio.h>

Mace_learner::Mace_learner(Env_Mace* mace_env):Agent(){
  mace_env_ = mace_env;
}

Mace_learner::~Mace_learner(){}

void Mace_learner::_DoTest(const sejong::Vector & ini_state){
  sejong::Vector state = ini_state;
  sejong::Vector nx_state;
  sejong::Vector opt_action;
  bool is_terminal(false);
  double reward;

  int count(0);
  int max_count(100);

  double sum_reward (0.0);
  while(!is_terminal){
    _FindMAX_Q(state, opt_action);
    sejong::pretty_print(state, std::cout, "state");
    sejong::pretty_print(opt_action, std::cout, "opt_action");
    is_terminal = mace_env_->Transition(state, opt_action, reward, nx_state);
    state = nx_state;
    ++count;
    if(count>max_count) break;
    sum_reward += reward;
    printf("sum reward: %f \n", sum_reward);

  }
  printf("sum reward: %f \n", sum_reward);
}

void Mace_learner::_getAction(int idx, sejong::Vector & action){
  action = sejong::Vector::Zero(DIM_ACTION);
  switch(idx){
  case Up:
    action[0] = 1.;
    break;
  case Down:
    action[0] = -1.;
    break;
  case Right:
    action[1] = 1.;
    break;
  case Left:
    action[1] = -1.;
    break;
  case Stay:
    break;
  }
}

