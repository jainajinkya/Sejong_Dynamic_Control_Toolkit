#ifndef LIPM_ACTOR_ONLY_TESTER
#define LIPM_ACTOR_ONLY_TESTER

#include "LIPM_Tester.h"

class LIPM_ActOnly_Tester: public LIPM_Tester{
 public:
  LIPM_ActOnly_Tester();
  virtual ~LIPM_ActOnly_Tester();
  virtual void Build_Test_Information(LIPM_3D* lipm, Agent* agent);
 protected:

};

#endif
