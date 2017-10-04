#ifndef MACE_TEST_H
#define MACE_TEST_H

#include <environment/Mace.h>
#include <agent/Mace_Q_Learning.h>
#include <agent/Mace_feature_learning.h>
#include <stdio.h>


void MaceTest(){
  Environment* env = new Env_Mace();

  // Agent Selection
  // Agent* agent = new Mace_Q_learning(dynamic_cast<Env_Mace*>(env));
  Agent* agent = new Mace_feature_learning(dynamic_cast<Env_Mace*>(env));

  // Initial State
  sejong::Vector ini_state(2);
  ini_state[0] = 3;
  ini_state[1] = 4;

  agent->DoLearning(ini_state);
}

#endif
