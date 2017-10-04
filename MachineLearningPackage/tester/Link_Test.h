#ifndef LINK_TEST_H
#define LINK_TEST_H

#include <environment/Link_Dyn_Sys.h>
#include <agent/Link_AC_RBF.h>



void Link_ActorCritic_Test(){
  double ini_state[DIM_LINK_STATE];
  ini_state[0] = 0.0;
  ini_state[1] = 0.0;

  Linkage_System* sys = new Linkage_System();
  Learner* learner = new Link_AC_RBF(sys);

  learner->DoLearning(ini_state);
}

#endif
