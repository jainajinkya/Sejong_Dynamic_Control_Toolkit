#ifndef LIPM_ACTONLY_TRAJECTORY
#define LIPM_ACTONLY_TRAJECTORY

#include "LIPM_Act_Only_Learner.h"

class LIPM_ActOnly_Traj: public LIPM_Act_Only_Learner{
public:
  LIPM_ActOnly_Traj(LIPM_3D_Act_Only* pipm_env);
  virtual ~LIPM_ActOnly_Traj();

protected:
  // Test Each Action
  virtual bool _TestAction(const sejong::Vector & pivot_action,
                           const std::vector< std::vector<int > > & delta_list,
                           std::vector<double> & reward_list);

};

#endif
