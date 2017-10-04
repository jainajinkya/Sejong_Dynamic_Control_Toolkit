#ifndef LIPM_ACTOR_CRITIC_TESTER
#define LIPM_ACTOR_CRITIC_TESTER

#include "LIPM_Tester.h"

class LIPM_ActorCritic_Tester: public LIPM_Tester{
 public:
  LIPM_ActorCritic_Tester();
  virtual ~LIPM_ActorCritic_Tester();
  virtual void Build_Test_Information(LIPM_3D* lipm, Agent* agent);
 protected:

};

#endif
