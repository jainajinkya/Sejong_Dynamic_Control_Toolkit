#ifndef LINK_ACTOR_CRITIC_LEARNER
#define LINK_ACTOR_CRITIC_LEARNER

#include "Learner.h"
#include <environment/Link_Dyn_Sys.h>

#define NUM_LINK_FEATURE (NUM_POS_GRID*NUM_VEL_GRID + 1)
#define DIM_LINK_THETA 2*DIM_LINK_ACTION // (first: mean x DIM_ACTION, last: variance x DIM_ACTION)
#define DIM_LINK_THETA_HALF DIM_LINK_ACTION


class Link_ActCritic_Learner : public Learner{
 public:
  Link_ActCritic_Learner(Linkage_System*);
  virtual ~Link_ActCritic_Learner();

  virtual bool DoLearning(const double * ini_state);
 protected:
  Linkage_System* link_sys_;
  double gamma_;
  double alpha_;
  double beta_;

  double lambda_w_;
  double lambda_theta_;

  double I_;

  double w_[NUM_LINK_FEATURE];
  double w_eligi_[NUM_LINK_FEATURE];

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

  void _Print_Coefficient();
  void _Print_Feature(const double * curr_feature,
                      const double * next_feature);
  void _SaveValue_Action(int num_learning);

  int num_grid_;
  double res_pos_;
  double res_vel_;
};

#endif
