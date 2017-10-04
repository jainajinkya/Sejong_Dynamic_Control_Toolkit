#include "LIPM_ActOnly_Tester.h"
#include <Utils/utilities.h>
#include <Utils/wrap_eigen.hpp>
#include "LIPM_Act_Only_Learner.h"
#include <environment/LIPM_3D.h>
#include <stdio.h>

LIPM_ActOnly_Tester::LIPM_ActOnly_Tester():
  LIPM_Tester()
{
  
}
LIPM_ActOnly_Tester::~LIPM_ActOnly_Tester(){}

void LIPM_ActOnly_Tester::Build_Test_Information(LIPM_3D* lipm, Agent* agent){
  LIPM_Act_Only_Learner * _agent = dynamic_cast<LIPM_Act_Only_Learner*>(agent);
  // Each Initial Time
  ini_time_list_.push_back(0.);

  // Each Pivot point in global frame
  sejong::Vector pivot(3);
  pivot.setZero();
  pivot_list_.push_back(pivot);

  // Each Initial State in global frame
  sejong::Vector state(6);
  state.setZero();
  sejong::Vector tmp_state;
  _agent->GetInitialState(tmp_state);
  state[0] = tmp_state[0];
  state[1] = tmp_state[1];
  state[2] = _agent->GetHeight();
  state[3] = tmp_state[2];
  state[4] = tmp_state[3];
  ini_state_list_.push_back(state);

  sejong::Vector policy;
  _agent->GetPolicy(policy);

  int size = policy.size();
  int num_step = size/3;

  sejong::Vector tmp_local_ini_state(4);
  sejong::Vector tmp_local_curr_state(4);
  printf("num step: %d\n", num_step);
  for(int i(0); i<num_step; ++i){
    // Time
    ini_time_list_.push_back(policy[3*i] + ini_time_list_[i]);
    // Pivot
    pivot[0] = policy[3*i + 1] + pivot_list_[i][0];
    pivot[1] = policy[3*i + 2] + pivot_list_[i][1];
    pivot[2] = 0.;
    pivot_list_.push_back(pivot);

    // State
    tmp_local_ini_state[0] = state[0] - pivot_list_[i][0];
    tmp_local_ini_state[1] = state[1] - pivot_list_[i][1];
    tmp_local_ini_state[2] = state[3];
    tmp_local_ini_state[3] = state[4];
    lipm->getState(tmp_local_ini_state, policy[3*i], tmp_local_curr_state);
    state[0] = tmp_local_curr_state[0] + pivot_list_[i][0];
    state[1] = tmp_local_curr_state[1] + pivot_list_[i][1];
    state[3] = tmp_local_curr_state[2];
    state[4] = tmp_local_curr_state[3];

    ini_state_list_.push_back(state);
  }
  sejong::pretty_print(ini_time_list_, "time list");
  sejong::printVectorSequence(pivot_list_, "pivot_list");
  sejong::printVectorSequence(ini_state_list_, "state_list");
}
