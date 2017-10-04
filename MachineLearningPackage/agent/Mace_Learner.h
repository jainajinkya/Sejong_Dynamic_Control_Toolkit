#ifndef MACE_LEARNER
#define MACE_LEARNER

#include "Agent.h"
#include <environment/Mace.h>

#define DIM_ACTION 2

enum ActionSet{
  Up = 0,
  Right = 1,
  Down = 2,
  Left = 3,
  Stay = 4,
  NUM_ACT
};

class Mace_learner: public Agent{
public:
  Mace_learner(Env_Mace* mace_env);
  virtual ~Mace_learner();

protected:
  void _DoTest(const sejong::Vector & ini_state);
  void _getAction(int idx, sejong::Vector & action);
  virtual double _FindMAX_Q(const sejong::Vector & state,
                            sejong::Vector & opt_action) = 0;


  Env_Mace* mace_env_;
};


#endif
