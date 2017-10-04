#ifndef LIPM_3D_SWITCHING_ACTOR_CRITIC_DYN_SYS_H
#define LIPM_3D_SWITCHING_ACTOR_CRITIC_DYN_SYS_H

#include "LIPM_3D_System.h"

#define AC_SW_DIM_STATE 4 //(x0, y0, xdot0, ydot0)
#define AC_SW_DIM_ACTION 2

#define X_SW_POS_MAX 0.0
#define X_SW_POS_MIN -0.25
#define X_SW_POS_RES 0.05
#define NUM_X_SW_POS_GRID 5

#define Y_SW_POS_MAX 0.25
#define Y_SW_POS_MIN 0.05
#define Y_SW_POS_RES 0.05
#define NUM_Y_SW_POS_GRID 5

#define X_SW_VEL_MAX 0.6
#define X_SW_VEL_MIN 0.1
#define X_SW_VEL_RES 0.1
#define NUM_X_SW_VEL_GRID 6

#define Y_SW_VEL_MAX -0.15
#define Y_SW_VEL_MIN -0.55
#define Y_SW_VEL_RES 0.1
#define NUM_Y_SW_VEL_GRID 5

class LIPM_AC_RBF_Switching_System : public LIPM_3D_System{
 public:
  LIPM_AC_RBF_Switching_System();
  virtual ~LIPM_AC_RBF_Switching_System();

  // Return True if nx state is Terminal
  virtual bool Transition(const double* state,
                          const double* action,
                          double & reward,
                          double* nx_state, bool b_print = false);

 protected:
  bool _is_terminal(double t_switch, double xp, double yp);
  double v_des_;
  double terminal_reward_;
};
#endif
