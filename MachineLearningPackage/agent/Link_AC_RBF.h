#ifndef LINK_SYSTEM_ACTOR_CRITIC_RADIAL_BASIS_FUNCTION
#define LINK_SYSTEM_ACTOR_CRITIC_RADIAL_BASIS_FUNCTION

#include "Link_ActCritic.h"
#include <features/RadialBasisFunction.h>

#define NUM_LINK_RBF (NUM_LINK_FEATURE -1)

class Link_AC_RBF : public Link_ActCritic_Learner{
 public:
  Link_AC_RBF(Linkage_System* );
  virtual ~Link_AC_RBF();

 protected:
  double mean_[DIM_LINK_ACTION];
  double variance_[DIM_LINK_ACTION];

  double action_min_[DIM_LINK_ACTION];
  double action_max_[DIM_LINK_ACTION];

  double state_min_[DIM_LINK_STATE];
  double state_max_[DIM_LINK_STATE];


  RadialBasisFunction<DIM_LINK_STATE, NUM_LINK_RBF> rbf_generic_;

  virtual void _GetValueFeature(const double* state,
                                double* psi);

  virtual void _GetGradientLogPolicy(const double* state,
                                     const double* action,
                                     double** gradient);
  virtual void _GetAction(const double* state,
                          double* action);
  void _mean_var_gradient_coeff(const double * feature, const double * action, double * mean_coeff, double * var_coeff);

};

#endif
