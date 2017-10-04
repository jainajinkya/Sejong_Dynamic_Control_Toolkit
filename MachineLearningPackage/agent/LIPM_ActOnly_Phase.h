#ifndef LIPM_ACTONLY_PHASE_SPACE
#define LIPM_ACTONLY_PHASE_SPACE

#include "LIPM_Act_Only_Learner.h"
#include <Utils/Spline/BSplineBasic.h>

class LIPM_ActOnly_Phase: public LIPM_Act_Only_Learner{
public:
  LIPM_ActOnly_Phase(LIPM_3D_Act_Only* pipm_env);
  virtual ~LIPM_ActOnly_Phase();

protected:
  double check_time_step_;
  // Test Each Action
  virtual bool _TestAction(const sejong::Vector & pivot_action,
                           const std::vector< std::vector<int > > & delta_list,
                           std::vector<double> & reward_list);

  void _build_phase_path();
  double _CalcDistanceFromDesired(const sejong::Vector & state);
  void _CheckCurrentPolicy(const sejong::Vector & pivot_action);

  double a_, b_, y_offset_;

  BS_Basic<1, 3, 0, 2, 2> spline_;
  double terminal_cost_;
};
#endif
