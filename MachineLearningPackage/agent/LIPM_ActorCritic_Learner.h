#ifndef LIPM_ACTOR_CRITIC_LEARNER
#define LIPM_ACTOR_CRITIC_LEARNER

#include "Agent.h"

#define PRINT_MESSAGE 0

class LIPM_3D_System;
// Actor & Critic
class LIPM_ActorCritic_Learner: public Agent{
public:
  LIPM_ActorCritic_Learner();
  virtual ~LIPM_ActorCritic_Learner();

  virtual bool DoLearning(const sejong::Vector & ini_state);
  void GetInitialState(sejong::Vector & ini_state){
    ini_state = ini_state_;
  }
  virtual void GetPolicy(sejong::Vector & policy) = 0;
  double GetHeight();
  /* const LIPM_Tester * GetLIPM_Tester(){ return lipm_tester_; } */

 protected:
  bool lets_finish_learning_;
  bool lets_skip_learning_;
  int skip_count_;

  int dim_state_;
  int dim_action_;
  int dim_theta_;
  int dim_theta_half_;
  int num_value_feature_;
  int num_action_feature_;

  sejong::Vector ini_state_;
  LIPM_3D_System* lipm_env_;
  double gamma_;
  double alpha_;
  double beta_;

  double lambda_w_;
  double lambda_theta_;

  double I_;

  // Value
  double* w_;
  double* w_eligi_;

  // Action
  /* double theta_[DIM_THETA][NUM_ACTION_FEATURE]; */
  /* double theta_eligi_[DIM_THETA][NUM_ACTION_FEATURE]; */
  double ** theta_;
  double ** theta_eligi_;
  double ** policy_gradient_;

  virtual void _GetValueFeature(const double* state,
                                double * psi) = 0;

  virtual void _GetGradientLogPolicy(const double* state,
                                     const double* action,
                                     double** gradient) = 0;

  virtual void _GetAction(const double* state,
                          double* action) = 0;
  virtual void _DoTest(const double* ini_state) = 0;
  virtual void _Sampling_Initial_State(double* ini_state) = 0;
  virtual void _TestLearnedPolicy() = 0;

  void _EnvrionmentTEST(const double * state);
  void _Print_Coefficient();
  void _Print_Feature(const double * curr_feature,
                      const double * next_feature);
  void _Save_Coefficient();
  void _Read_Coefficient();
  void _2D_Rotate(const double * vec, double theta, double* rotated_vec);

};
#endif
