#ifndef LINKAGE_DYNAMIC_SYSTEM
#define LINKAGE_DYNAMIC_SYSTEM

#include "Dyn_System.h"

#define DIM_LINK_STATE 2
#define DIM_LINK_ACTION 1

#define POS_MAX 0.35
#define POS_MIN -0.35
#define POS_RES 0.1
#define NUM_POS_GRID 8

#define VEL_MAX 0.35
#define VEL_MIN -0.35
#define VEL_RES 0.1
#define NUM_VEL_GRID 8

class Linkage_System:public Dynamic_System{
 public:
  Linkage_System();
  virtual ~Linkage_System(){}

  virtual bool Transition(const double* state,
                          const double* action,
                          double & reward,
                          double* nx_state, bool b_print = false);
 protected:
  bool is_terminal(const double* state, const double* action);

  double dt_;
  double des_pos_;
  double grav_;
  double mass_;
};

#endif
