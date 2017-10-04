#include "Mace_feature_learning.h"
#include <Utils/utilities.h>
#include <iostream>
#include <stdio.h>

Mace_feature_learning::Mace_feature_learning(Env_Mace* mace_env):Mace_learner(mace_env){
  alpha_ = 0.01;
  gamma_ = 0.9;
  epsilon_ = 7.0;
  lambda_ = 0.3;
  mace_env_->getTerminalPlace(terminal_loc_list_);



  num_feature_ = terminal_loc_list_.size() + 1;
  weight_ = sejong::Vector::Zero(num_feature_);
  eligibility_ = sejong::Vector::Zero(num_feature_);

  for (int i(0); i<num_feature_; ++i){
    weight_[i] = 0.0;
  }

  for(int i(0); i<num_feature_-1; ++i){
    printf("%i th terminal location: (%i, %i)\n",i,
           terminal_loc_list_[i][0],
           terminal_loc_list_[i][1]);
  }

}

Mace_feature_learning::~Mace_feature_learning(){}

void Mace_feature_learning::_getFeatureValue(const sejong::Vector & state,
                                               const sejong::Vector & action,
                                               sejong::Vector & feature_value){
  double reward;
  sejong::Vector nx_state;
  mace_env_->Transition(state, action, reward, nx_state);

  feature_value = sejong::Vector::Zero(num_feature_);
  double dist(0.0);
  for (int i(0); i<num_feature_-1; ++i){
    dist =
      sqrt(pow(terminal_loc_list_[i][0] - nx_state[0], 2) +
           pow(terminal_loc_list_[i][1] - nx_state[1], 2));

    feature_value[i] = dist/6.4;
  }
  feature_value[num_feature_-1] = //10.0; //((double)count_)/10.0;
    sqrt(pow(ini_state_[0] - nx_state[0], 2) +
         pow(ini_state_[1] - nx_state[1], 2));
    // sqrt(pow(state[0] - nx_state[0], 2) +
    //      pow(state[1] - nx_state[1], 2));
}

bool Mace_feature_learning::DoLearning(const sejong::Vector & ini_state){
  int num_learning (60);
  sejong::Vector action;
  sejong::Vector state = ini_state;
  ini_state_ = ini_state;
  sejong::Vector nx_state;
  sejong::Vector feature_value;

  bool is_terminal(false);
  double reward;

  for (int i(0); i<num_learning; ++i){
    state = ini_state;
    is_terminal = false;

    count_ = 0;
    // epsilon_ += 1.0;
    while(!is_terminal){
      if(count_ % (int)epsilon_ == 1){
        _FindMAX_Q(state, action);
        sejong::pretty_print(action, std::cout, "action");
      } else {
        int idx = rand() % NUM_ACT;
        _getAction(idx, action);
      }
      
      ++count_;
      // sejong::pretty_print(action, std::cout, "action");
      double Q_value = _getQValue(state, action);
      // printf("Q value: %f \n", Q_value);

      // sejong::pretty_print(state, std::cout, "curr state");
      _getFeatureValue(state, action, feature_value);
      // sejong::pretty_print(feature_value, std::cout, "feature");
      is_terminal = mace_env_->Transition(state, action, reward, nx_state);
      sejong::Vector opt_action;
      double max_Q = _FindMAX_Q(nx_state, opt_action);
      if(is_terminal){
        max_Q = 0.0;
      }
      double error = reward + gamma_ * max_Q - Q_value;

      // Feature Representation
      // for (int i(0); i<num_feature_; ++i){
      //   weight_[i] += alpha_ * error * feature_value[i];
      // }

      // Eligibility Trace
      eligibility_ = lambda_ * gamma_ * eligibility_ + feature_value;

      for (int i(0); i<num_feature_; ++i){
        weight_[i] += alpha_ * error * eligibility_[i];
      }
      
      // sejong::pretty_print(weight_, std::cout, "weight");

      state = nx_state;
    }
    sejong::pretty_print(weight_, std::cout, "updated weight");
  }
  sejong::pretty_print(weight_, std::cout, "** Final weight **");

  count_ = 0;
  _DoTest(ini_state);
  return false;
}

double Mace_feature_learning::_getQValue(const sejong::Vector & state,
                                         const sejong::Vector & action){
  sejong::Vector feature_value;
  _getFeatureValue(state, action, feature_value);

  double sum(0.0);

  for (int i(0); i<num_feature_; ++i){
    sum += weight_[i] * feature_value[i];
  }
  return sum;
}
double Mace_feature_learning::_FindMAX_Q(const sejong::Vector & state,
                                         sejong::Vector & opt_action){
  double max_value(-10000.0);
  sejong::Vector action;
  for (int i(0); i<NUM_ACT; ++i){
    _getAction(i, action);
    double q = _getQValue(state, action);
    if( q > max_value){
      max_value = q;
      opt_action = action;
    }
  }
  return max_value;
}
