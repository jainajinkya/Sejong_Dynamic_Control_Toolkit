#ifndef LIPM_ACTOR_CRITIC_FEATURE_RBF_SWITCHING_STATE
#define LIPM_ACTOR_CRITIC_FEATURE_RBF_SWITCHING_STATE

#include "LIPM_ActorCritic_Learner.h"
#include <features/RadialBasisFunction.h>
#include <environment/LIPM_3D_AC_RBF_Switching_System.h>

// When the agent type change, this must be changed too
#define NUM_SW_VALUE_FEATURE (NUM_X_SW_POS_GRID * NUM_Y_SW_POS_GRID * NUM_X_SW_VEL_GRID * NUM_Y_SW_VEL_GRID + 1)
#define NUM_SW_ACTION_FEATURE NUM_SW_VALUE_FEATURE
#define DIM_SW_THETA 2*AC_SW_DIM_ACTION // (first: mean x AC_SW_DIM_ACTION, last: variance x AC_SW_DIM_ACTION)
#define DIM_SW_THETA_HALF AC_SW_DIM_ACTION

#define NUM_SW_RBF_FEATURE (NUM_SW_VALUE_FEATURE-1)

// Action: xp, v_apex
class LIPM_AC_RBF_Switching: public LIPM_ActorCritic_Learner{
 public:
  LIPM_AC_RBF_Switching();
  virtual ~LIPM_AC_RBF_Switching();

  virtual void GetPolicy(sejong::Vector & policy);
 protected:
  double mean_[AC_SW_DIM_ACTION];
  double variance_[AC_SW_DIM_ACTION];

  double action_min_[AC_SW_DIM_ACTION];
  double action_max_[AC_SW_DIM_ACTION];

  double state_min_[AC_SW_DIM_STATE];
  double state_max_[AC_SW_DIM_STATE];

  RadialBasisFunction<AC_SW_DIM_STATE, NUM_SW_RBF_FEATURE> rbf_generic_;

  virtual void _GetValueFeature(const double* state,
                                double* psi);

  virtual void _GetGradientLogPolicy(const double* state,
                                     const double* action,
                                     double** gradient);
  virtual void _GetAction(const double* state,
                          double* action);
  virtual void _DoTest(const double* ini_state);
  virtual void _Sampling_Initial_State(double* ini_state);
  virtual void _TestLearnedPolicy();

  void _mean_var_gradient_coeff(const double * feature, const double * action, double * mean_coeff, double * var_coeff);
};
#endif
