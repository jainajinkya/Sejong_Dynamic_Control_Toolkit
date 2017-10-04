#include "LIPM_ActorCritic_RBF.h"
#include <Utils/utilities.h>
#include <stdio.h>


LIPM_ActorCritic_RBF::~LIPM_ActorCritic_RBF(){
  delete [] w_;
  delete [] w_eligi_;

  for(int i(0);i<DIM_THETA; ++i){
    delete [] theta_[i];
    delete [] theta_eligi_[i];
    delete [] policy_gradient_[i];
  }
  delete [] theta_;
  delete [] theta_eligi_;
  delete [] policy_gradient_;
}
void LIPM_ActorCritic_RBF::_Sampling_Initial_State(double * ini_state){
  double h = lipm_env_->GetHeight();
  double slope = sqrt(9.81/h);
  // y, xdot, ydot
  ini_state[1] = X_VEL_MIN+(X_VEL_MAX - X_VEL_MIN)*rand()/(double)RAND_MAX;
  do {
    ini_state[0] = Y_POS_MIN+(Y_POS_MAX - Y_POS_MIN)*rand()/(double)RAND_MAX;
    ini_state[2] = Y_VEL_MIN+(Y_VEL_MAX - Y_VEL_MIN)*rand()/(double)RAND_MAX;
    // sejong::pretty_print(ini_state, "initial state", AC_DIM_STATE);
  }while( !((ini_state[2] < slope * ini_state[0]) && (ini_state[2] > -slope * ini_state[0])) );

}

void LIPM_ActorCritic_RBF::_GetValueFeature(const double* state,
                                            double* psi){
  double rbf_feature[NUM_RBF_FEATURE];
  rbf_generic_.getGradient(state, rbf_feature);

  for(int i(0);i<NUM_RBF_FEATURE; ++i){
    psi[i] = rbf_feature[i];
  }
  psi[NUM_RBF_FEATURE] = 1.;
}

void LIPM_ActorCritic_RBF::_GetGradientLogPolicy(const double* state,
                                                 const double* action,
                                                 double** gradient){
  double rbf_feature[NUM_RBF_FEATURE];
  rbf_generic_.getGradient(state, rbf_feature);

  double feature[NUM_VALUE_FEATURE];
  for(int i(0);i<NUM_RBF_FEATURE; ++i){
    feature[i] = rbf_feature[i];
  }
  feature[NUM_RBF_FEATURE] = 1.;

  double mean_coeff[AC_DIM_ACTION];
  double var_coeff[AC_DIM_ACTION];

  _mean_var_gradient_coeff(feature, action, mean_coeff, var_coeff);
  // printf("mean coeff: %f, %f\n", mean_coeff[0], mean_coeff[1]);
  for(int i(0);i<DIM_THETA_HALF; ++i){
    for(int j(0); j<NUM_ACTION_FEATURE; ++j){
      gradient[i][j] = mean_coeff[i] * feature[j];
    }
  }
  for(int i(DIM_THETA_HALF);i<DIM_THETA; ++i){
    for(int j(0); j<NUM_ACTION_FEATURE; ++j){
      gradient[i][j] = (-feature[j] + var_coeff[i] * feature[j]);
    }
  }
}

void LIPM_ActorCritic_RBF::_mean_var_gradient_coeff(const double * feature, const double * action, double * mean_coeff, double * var_coeff){

  double theta_var_feature, theta_mean_feature;

  for(int i(0);i<DIM_THETA_HALF; ++i){
    theta_mean_feature = 0.;
    theta_var_feature = 0.;

    for(int k(0); k<NUM_ACTION_FEATURE; ++k){
      theta_mean_feature += theta_[i][k] * feature[k];
      theta_var_feature += theta_[i + DIM_THETA_HALF][k] * feature[k];
    }
    // printf("%d th theta mean feature: %f\n", i, theta_mean_feature);
    // printf("%d th theta var feature: %f\n", i, theta_var_feature);


    mean_coeff[i] = exp(-2. * theta_var_feature) * (action[i] - theta_mean_feature);
    var_coeff[i] = exp(-2. * theta_var_feature) * pow(action[i] - theta_mean_feature, 2.);
  }
  // printf("mean coeff: %f, %f\n", mean_coeff[0], mean_coeff[1]);
  // printf("var coeff: %f, %f\n", var_coeff[0], var_coeff[1]);

}

void LIPM_ActorCritic_RBF::_DoTest(const double* ini_state){
  
}

void LIPM_ActorCritic_RBF::GetPolicy(sejong::Vector & policy){
  // (t_s, xp, yp)
  
}
void LIPM_ActorCritic_RBF::_SingleTrialTest(const double * ini_state_input){
  double reward;

  double ini_state[AC_DIM_STATE];
  sejong::Copy(ini_state_input, ini_state, AC_DIM_STATE);
  double state[AC_DIM_STATE];
  double nx_state[AC_DIM_STATE];
  double action[AC_DIM_ACTION];


  bool b_Terminal(false);
  double pos[2]; double rot_pos[2];
  double vel[2]; double rot_vel[2];
  double rotated_state[3];
  double theta(0.*M_PI/180.);
  // rotation
  pos[0] = 0.;
  pos[1] = ini_state[0];
  _2D_Rotate(pos, theta, rot_pos);
  vel[0] = ini_state[1];
  vel[1] = ini_state[2];
  _2D_Rotate(vel, theta, rot_vel);

  ini_state[0] = rot_pos[1];
  ini_state[1] = rot_vel[0];
  ini_state[2] = rot_vel[1];

  sejong::Copy(ini_state, state, AC_DIM_STATE);
  int num_step(0);
  while(!b_Terminal){
  // while(true){
    // Get Action (mean)
    double rbf_feature[NUM_RBF_FEATURE];
    rbf_generic_.getGradient(state, rbf_feature);

    double feature[NUM_VALUE_FEATURE];
    for(int i(0);i<NUM_RBF_FEATURE; ++i){
      feature[i] = rbf_feature[i];
    }
    feature[NUM_RBF_FEATURE] = 1.;

    for(int k(0);k<DIM_THETA_HALF; ++k){
      action[k] = 0.;

      for(int i(0);i<NUM_ACTION_FEATURE; ++i){
        action[k] += theta_[k][i] * feature[i];
      }
    }
    // Transition
    b_Terminal = lipm_env_->Transition(state, action, reward, nx_state, true);

    ++num_step;
    if(num_step < 4){
      printf("reward: %f\n", reward);
      sejong::pretty_print(action, "action", dim_action_);
      sejong::pretty_print(state, "state", dim_state_);
      sejong::pretty_print(nx_state, "nx_state", dim_state_);
      printf("num_step: %d\n", num_step);
    }
    sejong::Copy(nx_state, state, AC_DIM_STATE);


    if(num_step > 35){
      break;
    }
  }  
}


void LIPM_ActorCritic_RBF::_GetLearned_Action(const double* state, double* action){
  double rbf_feature[NUM_RBF_FEATURE];
  rbf_generic_.getGradient(state, rbf_feature);

  double feature[NUM_VALUE_FEATURE];
  for(int i(0);i<NUM_RBF_FEATURE; ++i){
    feature[i] = rbf_feature[i];
  }
  feature[NUM_RBF_FEATURE] = 1.;

  for(int k(0);k<DIM_THETA_HALF; ++k){
    action[k] = 0.;

    for(int i(0);i<NUM_ACTION_FEATURE; ++i){
      action[k] += theta_[k][i] * feature[i];
    }
  }
}


void LIPM_ActorCritic_RBF::_Change_Global_To_Local(const double* global_pos, const double* global_vel, const double* offset, const double& theta, double* local_pos, double * local_vel ){
  double off_pos[2];
  off_pos[0] = global_pos[0] - offset[0];
  off_pos[1] = global_pos[1] - offset[1];
  _2D_Rotate(off_pos, -theta, local_pos);
  _2D_Rotate(global_vel, -theta, local_vel);
}

void LIPM_ActorCritic_RBF::_Change_Local_To_Global(const double* local_pos, const double* local_vel, const double* offset, const double& theta, double* global_pos, double * global_vel ){
  _2D_Rotate(local_pos, theta, global_pos);
  _2D_Rotate(local_vel, theta, global_vel);

  global_pos[0] += offset[0];
  global_pos[1] += offset[1];
}

bool LIPM_ActorCritic_RBF::_Change_Local_to_ActionState(const double * com_pos, const double* com_vel, double * state){
  // Check com_pos[0] == 0.
  if(fabs(com_pos[0])> 2.e-4){
    printf("[Error] com pos is not zero: %f \n", com_pos[0]);
    exit(0);
  }

  if(com_pos[1] > 0.){
    state[0] = com_pos[1];
    state[1] = com_vel[0];
    state[2] = com_vel[1];
    return false;
  }
  if(com_pos[1] < 0.){
    state[0] = -com_pos[1];
    state[1] = com_vel[0];
    state[2] = -com_vel[1];
    return true;
  }
  return false;
}

void LIPM_ActorCritic_RBF::_Advance_OneStep(double * pos, double * vel, const double & dt){
  double acc_x (9.81/lipm_env_->GetHeight() * pos[0]);
  double acc_y (9.81/lipm_env_->GetHeight() * pos[1]);
  pos[0] = pos[0] + vel[0] * dt + 0.5 * acc_x * dt * dt;
  pos[1] = pos[1] + vel[1] * dt + 0.5 * acc_y * dt * dt;
  vel[0] = vel[0] + acc_x * dt;
  vel[1] = vel[1] + acc_y * dt;
}

bool LIPM_ActorCritic_RBF::_Find_Foot_Switching_Time(const double *state, const double* action, const bool & b_flip, double & local_xp, double & local_yp, double & t_switch){
  double nx_state[AC_DIM_STATE];
  double reward;

  bool b_terminal = lipm_env_->Transition(state, action, reward, nx_state);
  local_xp = action[0];
  ((LIPM_AC_RBF_System*)lipm_env_)->Get_Yp_SwitchingTime( local_yp, t_switch);

  return b_terminal;
}
