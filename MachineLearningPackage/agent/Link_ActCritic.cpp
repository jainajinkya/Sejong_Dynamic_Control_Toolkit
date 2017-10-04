#include "Link_ActCritic.h"
#include <Utils/utilities.h>
#include <stdio.h>


Link_ActCritic_Learner::Link_ActCritic_Learner(Linkage_System* link_sys):
  Learner(),
  num_grid_(20)
{
  link_sys_ = link_sys;
  theta_ = new double*[DIM_LINK_THETA];
  theta_eligi_ = new double*[DIM_LINK_THETA];
  policy_gradient_ = new double*[DIM_LINK_THETA];

  for(int i(0);i<DIM_LINK_THETA; ++i){
    theta_[i] = new double [NUM_LINK_FEATURE];
    theta_eligi_[i] = new double [NUM_LINK_FEATURE];
    policy_gradient_[i] = new double[NUM_LINK_FEATURE];
  }

  res_pos_ = (POS_MAX - POS_MIN)/(double)num_grid_;
  res_vel_ = (VEL_MAX - VEL_MIN)/(double)num_grid_;

  double state_test[DIM_LINK_STATE];
  for(int i(0); i<num_grid_; ++i){
    for(int j(0); j<num_grid_; ++j){
      state_test[0] = POS_MIN + res_pos_ * i;
      state_test[1] = VEL_MIN + res_vel_ * j;
      sejong::saveVector(state_test, "input_data", DIM_LINK_STATE);
    }
  }
}

Link_ActCritic_Learner::~Link_ActCritic_Learner(){
  for(int i(0);i<DIM_LINK_THETA; ++i){
    delete [] theta_[i];
    delete [] theta_eligi_[i];
    delete [] policy_gradient_[i];
  }
  delete [] theta_;
  delete [] theta_eligi_;
  delete [] policy_gradient_;
}

bool Link_ActCritic_Learner::DoLearning(const double * ini_state){

  int num_learning(10000);
  double state[DIM_LINK_STATE];
  double nx_state[DIM_LINK_STATE];

  double action[DIM_LINK_ACTION];
  double reward(0.);

  double psi_nx[NUM_LINK_FEATURE];
  double psi[NUM_LINK_FEATURE];

  for(int i(0);i<NUM_LINK_FEATURE; ++i){
    w_[i] = 0. * rand()/(double)RAND_MAX;
  }

  for(int k(0);k<DIM_LINK_THETA_HALF; ++k){
    for(int i(0); i<NUM_LINK_FEATURE; ++i){
      theta_[k][i] = 0.0 * rand()/(double)RAND_MAX;
    }
  }

  for(int k(DIM_LINK_THETA_HALF); k<DIM_LINK_THETA; ++k){
    for(int i(0); i<NUM_LINK_FEATURE; ++i){
      theta_[k][i] = 0.0 * rand()/(double)RAND_MAX;
    }
  }
  // theta_[2][NUM_LINK_FEATURE-1] = -0.8;
  // theta_[3][NUM_LINK_FEATURE-1] = -0.8;
  int num_process(0);
  for(int iii(0); iii<num_learning; ++iii){
    // for(int k(0); k<DIM_LINK_STATE; ++k){
    //   state[k] = ini_state[k] - 0.03 + 0.06 * rand()/(double)RAND_MAX;
    // }
    state[0] = POS_MIN + (POS_MAX - POS_MIN) * rand()/(double)RAND_MAX;
    state[1] = VEL_MIN + (VEL_MAX - VEL_MIN) * rand()/(double)RAND_MAX;

    I_ = 1.0;

    sejong::SetArrayZero(w_eligi_, NUM_LINK_FEATURE);
    for(int i_th(0);i_th<DIM_LINK_THETA; ++i_th){
      sejong::SetArrayZero(theta_eligi_[i_th], NUM_LINK_FEATURE);
    }

    bool b_Terminal(false);
    num_process = 0;

    while(!b_Terminal) {
      _GetAction(state, action);
      b_Terminal = link_sys_->Transition(state, action, reward, nx_state);
      _GetValueFeature(state, psi);
      _GetValueFeature(nx_state, psi_nx);
      _GetGradientLogPolicy(state, action, policy_gradient_);

      double delta;
      if(b_Terminal){
        delta = reward - sejong::Dot(w_, psi, NUM_LINK_FEATURE);
        double tmp = sejong::Dot(w_, psi, NUM_LINK_FEATURE);
        printf("curr value: %f \n", tmp);
      }else {
        delta = reward + gamma_ * sejong::Dot(w_, psi_nx, NUM_LINK_FEATURE) - sejong::Dot(w_, psi, NUM_LINK_FEATURE);
      }

      for(int i(0); i<NUM_LINK_FEATURE; ++i){
        w_eligi_[i] = lambda_w_ * w_eligi_[i] + I_ * psi[i];
      }

      for(int i(0); i<DIM_LINK_THETA; ++i){
        for(int j(0); j<NUM_LINK_FEATURE; ++j){
          theta_eligi_[i][j] = lambda_theta_ * theta_eligi_[i][j] + I_ * policy_gradient_[i][j];
        }
      }

      for(int i(0); i<NUM_LINK_FEATURE; ++i){
        w_[i] += beta_ * delta * w_eligi_[i];
      }


      for (int i(0); i<DIM_LINK_THETA; ++i){
        for(int j(0); j<NUM_LINK_FEATURE; ++j){
          theta_[i][j] += alpha_ * delta * theta_eligi_[i][j];
        }
      }

      bool b_printout(false);
      if(num_process % 500 ==1){
        b_printout = true;
      }

      if(b_printout){
        printf("delta, reward: %f, %f \n", delta, reward);
        sejong::pretty_print(action, "action", DIM_LINK_ACTION);
        sejong::pretty_print(state, "state", DIM_LINK_STATE);
        sejong::pretty_print(nx_state, "nx_state", DIM_LINK_STATE);
        printf("%d th learning, %d th process\n", iii, num_process);
        printf("\n");
      }

      sejong::Copy(nx_state, state, DIM_LINK_STATE);
      I_ *= gamma_;

      // _Print_Feature(psi, psi_nx);
      // _Print_Coefficient();
      ++num_process;

      if(num_process > 1000){
        break;
      }

    }
    _SaveValue_Action(iii);
  }
  _Print_Coefficient();

  return true;
}

void Link_ActCritic_Learner::_SaveValue_Action(int num_learning){
  // Value Function Printing
  double psi_test[NUM_LINK_FEATURE];

  if(num_learning%30 == 1){
    double state_test[DIM_LINK_STATE];
    // Save Value Function
    char file_name[50];
    char file_name_act[50];
    char file_name_variance[50];
    sprintf(file_name, "%dth_Value", num_learning);
    sprintf(file_name_act, "%dth_Action", num_learning);
    sprintf(file_name_variance, "%dth_ActionVariance", num_learning);

    for(int i(0); i<num_grid_; ++i){
      for(int j(0); j<num_grid_; ++j){
        state_test[0] = POS_MIN + res_pos_ * i;
        state_test[1] = VEL_MIN + res_vel_ * j;

        _GetValueFeature(state_test, psi_test);
        double tmp = sejong::Dot(w_, psi_test, NUM_LINK_FEATURE);
        double tmp2 = sejong::Dot(theta_[0], psi_test, NUM_LINK_FEATURE);
        double tmp3 = sejong::Dot(theta_[1], psi_test, NUM_LINK_FEATURE);

        sejong::saveValue(tmp, file_name);
        sejong::saveValue(tmp2, file_name_act);
        sejong::saveValue(exp(tmp3), file_name_variance);
      }
    }
  }
}

void Link_ActCritic_Learner::_Print_Coefficient(){
  sejong::pretty_print(w_,  "w", NUM_LINK_FEATURE);

  for(int i(0); i<DIM_LINK_THETA; ++i){
    sejong::pretty_print(policy_gradient_[i], "policy_gradient", NUM_LINK_FEATURE);
  }
  for(int i(0); i<DIM_LINK_THETA; ++i){
    sejong::pretty_print(theta_[i], "theta", NUM_LINK_FEATURE);
  }
  // for(int i(0); i<DIM_LINK_THETA; ++i){
  //   sejong::pretty_print(theta_eligi_[i], "theta_eligi_", NUM_LINK_FEATURE);
  // }

}
void Link_ActCritic_Learner::_Print_Feature(const double * curr_feature,
                                            const double * next_feature){
  sejong::pretty_print(curr_feature, "current value feature", NUM_LINK_FEATURE);
  sejong::pretty_print(next_feature, "next value feature", NUM_LINK_FEATURE);
}
