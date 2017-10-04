#include "LIPM_ActOnly_Phase.h"
#include <environment/LIPM_3D_Act_Only.h>
#include <Utils/utilities.h>

LIPM_ActOnly_Phase::LIPM_ActOnly_Phase(LIPM_3D_Act_Only* lipm_env):
  LIPM_Act_Only_Learner(lipm_env),
  check_time_step_(0.01),
  a_(0.05),
  b_(0.35),
  y_offset_(0.123),
  terminal_cost_(500.)
{
  etha_ = 0.01;
  step_horizon_ = 4;

  dim_act_ = 3 * step_horizon_;

  // initial policy
  policy_ = sejong::Vector::Zero(dim_act_);
  for(int i(0); i<step_horizon_; ++i){
    policy_[3*i + 0] = 0.61;
    policy_[3*i + 1] = 0.14;

    if(i % 2 == 0 ){
      policy_[3*i + 2] = 0.2978;
      if(i > 1) policy_[3*i + 2] = 0.3433;
    } else {
      policy_[3*i + 2] = -0.3433;
    }
  }
  policy_[3] = 0.56;


  policy_ = sejong::Vector::Zero(dim_act_);
  for(int i(0); i<step_horizon_; ++i){
    policy_[3*i + 0] = 0.61;
    policy_[3*i + 1] = 0.14;

    if(i % 2 == 0 ){
      policy_[3*i + 2] = 0.24;
    } else {
      policy_[3*i + 2] = -0.2433;
    }
  }

  best_policy_ = policy_;

  // discrete action generation
  num_allocation_ = 5000;

  delta_list_.resize(num_allocation_);
  reward_list_.resize(num_allocation_, 0.);
  for(int i(0); i<num_allocation_; ++i){
    delta_list_[i].resize(dim_act_, 0);
  }

  // Test Step size
  epsilon_.resize(3, 0.003);
  epsilon_[0] = 0.001;

  _build_phase_path();
}

LIPM_ActOnly_Phase::~LIPM_ActOnly_Phase(){
}

bool LIPM_ActOnly_Phase::_TestAction(const sejong::Vector & pivot_action,
                                     const std::vector< std::vector<int > > & delta_list,
                                     std::vector<double> & reward_list){
  static int count(0);
  ++count;
  printf("%d th test action\n", count);

  double sum_reward(0.);
  double t_switch, time, xp, yp, tmp_reward, x_offset, y_offset;
  sejong::Vector ini_state, nx_state, tmp_state, global_nx_state;
  sejong::Vector shorten_ini_state, shorten_nx_state;
  sejong::Vector foot(3);
  foot.setZero();

  sejong::Vector action(3);

  for(int k(0); k < num_allocation_; ++k){
    ini_state = ini_state_;
    x_offset = 0.;
    y_offset = 0.;

    for(int i(0); i < step_horizon_; ++i){

      t_switch = pivot_action[3*i] + (double)delta_list[k][3*i] * epsilon_[0];
      xp = pivot_action[3*i + 1] + (double)delta_list[k][3*i + 1] * epsilon_[1];
      yp = pivot_action[3*i + 2] + (double)delta_list[k][3*i + 2] * epsilon_[2];
      action[0] = t_switch; action[1] = xp; action[2] = yp;
      // sejong::pretty_print(action, std::cout, "action");

      // Add Distance in this step
      double reward(0.);
      time = 0.;
      // sejong::pretty_print(ini_state, std::cout, "ini");
      while( time < t_switch ){
        lipm_env_->getState(ini_state, time, nx_state);
        // sejong::pretty_print(nx_state, std::cout, "nx");
        global_nx_state = nx_state;
        global_nx_state[0] += x_offset;
        global_nx_state[1] += y_offset;
        reward -= _CalcDistanceFromDesired(global_nx_state);
        time += check_time_step_;
        // printf("time, reward: %f, %f \n", time, reward);
      }
      // if( i > 0) exit(0);

      sum_reward += reward;
      bool isTerminal = lipm_env_->Transition(ini_state, action, tmp_reward, tmp_state);
      if(isTerminal){
        sum_reward -= terminal_cost_ * (step_horizon_ - i);
        break;
      } else {
        sum_reward += 5.;
      }

      ini_state = nx_state;
      ini_state[0] -= xp;
      ini_state[1] -= yp;
      x_offset += xp;
      y_offset += yp;
    }
    reward_list[k] = sum_reward;
    sum_reward = 0.;
  }
  // Check Current Policy Reward
  _CheckCurrentPolicy(pivot_action);
  return true;
}

void LIPM_ActOnly_Phase::_CheckCurrentPolicy(const sejong::Vector & pivot_action){
  double sum_reward(0.);
  double t_switch, time, xp, yp, tmp_reward, x_offset, y_offset;
  sejong::Vector ini_state, nx_state, tmp_state, global_nx_state;
  sejong::Vector shorten_ini_state, shorten_nx_state;
  sejong::Vector foot(3);
  foot.setZero();

  sejong::Vector action(3);

  ini_state = ini_state_;
  x_offset = 0.;
  y_offset = 0.;

  for(int i(0); i < step_horizon_; ++i){

    t_switch = pivot_action[3*i];
    xp = pivot_action[3*i + 1];
    yp = pivot_action[3*i + 2];
    action[0] = t_switch; action[1] = xp; action[2] = yp;

    // Add Distance in this step
    double reward(0.);
    time = 0.;
    while( time < t_switch ){
      lipm_env_->getState(ini_state, time, nx_state);

      global_nx_state = nx_state;
      global_nx_state[0] += x_offset;
      global_nx_state[1] += y_offset;

      reward -=  _CalcDistanceFromDesired(global_nx_state);
      time += check_time_step_;
    }
    sum_reward += reward;
    bool isTerminal = lipm_env_->Transition(ini_state, action, tmp_reward, tmp_state);
    if(isTerminal){
      sum_reward -= terminal_cost_ * (step_horizon_ - i);
      break;
    } else {
      sum_reward += 50.;
    }

    ini_state = nx_state;
    ini_state[0] -= xp;
    ini_state[1] -= yp;
    x_offset += xp;
    y_offset += yp;
  }
  printf("current policy reward: %f \n", sum_reward);
  if(sum_reward > best_reward_){
    best_reward_ = sum_reward;
    best_policy_ = pivot_action;
  }
}

void LIPM_ActOnly_Phase::_build_phase_path(){
  double init[3] = {0.05, 0.06, 0.06};
  double fin[3] = {0.2, 0., 0.};
  double** middle = NULL;
  spline_.SetParam(init, fin, middle, 3.0);

  // Test
  // sejong::Vector test_vec(2);
  // double x;
  // double xdot[1];
  // for(int i(0); i<100; ++i){
  //   x = i * 3./100.;
  //   test_vec[0] = x;
  //   spline_.getCurvePoint(x, xdot);
  //   test_vec[1] = xdot[0];
  //   sejong::saveVector(test_vec, "b_spline_test");
  // }
  // exit(0);
}

double LIPM_ActOnly_Phase::_CalcDistanceFromDesired(const sejong::Vector & state){
  double x = state[0];
  double y = state[1];
  double xdot = state[2];
  double ydot = state[3];

  double y_err = (y- y_offset_)*(y - y_offset_)/(a_*a_)
    + ydot*ydot/(b_*b_) - 1.;
  // printf("y_error : %f\n", y_err);

  double xdot_des[1];
  spline_.getCurvePoint(x, xdot_des);
  // printf("curve point: %f\n", xdot_des[0]);
  // double x_err = xdot - xdot_des[0];
  double x_err = 0.2 - xdot;

  return (5.0 * x_err * x_err +  y_err * y_err);
  // return y_err * y_err;

}
