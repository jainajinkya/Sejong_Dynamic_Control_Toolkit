#ifndef LIPM_ACTOR_CRITIC_YP_CHANGE
#define LIPM_ACTOR_CRITIC_YP_CHANGE

#include "LIPM_ActorCritic_Learner.h"
#include <features/RadialBasisFunction.h>
#include <environment/LIPM_3D_AC_YP_System.h>

// When the agent type change, this must be changed too
#define NUM_VALUE_FEATURE (NUM_Y_POS_GRID * NUM_X_VEL_GRID * NUM_Y_VEL_GRID + 1)
#define NUM_ACTION_FEATURE NUM_VALUE_FEATURE
#define DIM_THETA 2*AC_DIM_ACTION //(first: mean x AC_DIM_ACTION, last: variance x AC_DIM_ACTION)
#define DIM_THETA_HALF AC_DIM_ACTION

#define NUM_RBF_FEATURE (NUM_VALUE_FEATURE-1)

// Action: xp, v_apex
class LIPM_AC_YP_change: public LIPM_ActorCritic_Learner{
 public:
  LIPM_AC_YP_change();
  virtual ~LIPM_AC_YP_change();

  virtual void GetPolicy(sejong::Vector & policy);
 protected:
  double mean_[AC_DIM_ACTION];
  double variance_[AC_DIM_ACTION];

  double action_min_[AC_DIM_ACTION];
  double action_max_[AC_DIM_ACTION];

  double state_min_[AC_DIM_STATE];
  double state_max_[AC_DIM_STATE];

  double action_offset_[AC_DIM_ACTION];

  RadialBasisFunction<AC_DIM_STATE, NUM_RBF_FEATURE> rbf_generic_;

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

  void _SingleStraightWalkingTest(const double * ini_state);
  void _MultipleTrialsTest();
  void _SetTestStateList(std::vector<sejong::Vector> & ini_state_list);

  void _GetLearned_Action(const double* state, double* action);
  void _Change_Global_To_Local(const double* global_pos, const double* global_vel, const double* offset, const double& theta, double* local_pos, double * local_vel );
  void _Change_Local_To_Global(const double* local_pos, const double* local_vel, const double* offset, const double& theta, double* global_pos, double * global_vel );
  bool _Change_Local_to_ActionState(const double * com_pos, const double* com_vel, double * state);
  void _FindZeroCoM_X(double* com_pos, double* com_vel);

  bool _Find_Foot_Switching_Time(const double *state, const double* action, const bool & b_flip, double & local_xp, double & local_yp, double & t_switch);

  void _mean_var_gradient_coeff(const double * feature, const double * action, double * mean_coeff, double * var_coeff);


  ///// For the TEST
  // Get the Position
  void _SetLIPM_Param_With_Initial(const double * pos_ini, const double * vel_ini);
  double A_[2];
  double B_[2];
  double t_apex_;
  void _GetState(double * com_pos, double * com_vel, double curr_time);
  void _time_to_apex(const double* com_pos, const double* com_vel);
};
#endif
