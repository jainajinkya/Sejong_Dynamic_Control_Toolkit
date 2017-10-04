#ifndef LIPM_3D_ACTOR_CRITIC_DYN_YP_SYS_H
#define LIPM_3D_ACTOR_CRITIC_DYN_YP_SYS_H

#include "LIPM_3D_System.h"

#define AC_DIM_STATE 3 //(y0, xdot0, ydot0)
#define AC_DIM_ACTION 3 // (xp, apex_vel, yp)

/* #define Y_POS_MAX 0.22 */
/* #define Y_POS_MIN 0.01 */
/* #define Y_POS_RES 0.005 */
/* #define NUM_Y_POS_GRID 43 */

/* #define X_VEL_MAX 0.70 */
/* #define X_VEL_MIN 0.31 */
/* #define X_VEL_RES 0.005 */
/* #define NUM_X_VEL_GRID 79 */

/* #define Y_VEL_MAX 0.2 */
/* #define Y_VEL_MIN -0.2 */
/* #define Y_VEL_RES 0.005 */
/* #define NUM_Y_VEL_GRID 81 */

/* #define RBF_SIGMA 0.005 */

/* #define Y_POS_MAX 0.2 */
/* #define Y_POS_MIN -0.1 */
/* #define Y_POS_RES 0.01 */
/* #define NUM_Y_POS_GRID 31 */

/* #define X_VEL_MAX 0.72 */
/* #define X_VEL_MIN 0.03 */
/* #define X_VEL_RES 0.01 */
/* #define NUM_X_VEL_GRID 70 */

/* #define Y_VEL_MAX 0.55 */
/* #define Y_VEL_MIN -0.55 */
/* #define Y_VEL_RES 0.01 */
/* #define NUM_Y_VEL_GRID 111 */

/* #define RBF_SIGMA 0.01 */

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

#define Y_POS_MAX 0.25
#define Y_POS_MIN -0.09
#define Y_POS_RES 0.02
#define NUM_Y_POS_GRID 18

#define X_VEL_MAX 0.51
#define X_VEL_MIN 0.03
#define X_VEL_RES 0.02
#define NUM_X_VEL_GRID 25

#define Y_VEL_MAX 0.35
#define Y_VEL_MIN -0.35
#define Y_VEL_RES 0.02
#define NUM_Y_VEL_GRID 36

#define RBF_SIGMA 0.02

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

/* #define Y_POS_MAX 0.20 */
/* #define Y_POS_MIN -0.10 */
/* #define Y_POS_RES 0.03 */
/* #define NUM_Y_POS_GRID 11 */

/* #define X_VEL_MAX 0.72 */
/* #define X_VEL_MIN 0.03 */
/* #define X_VEL_RES 0.03 */
/* #define NUM_X_VEL_GRID 24 */

/* #define Y_VEL_MAX 0.52 */
/* #define Y_VEL_MIN -0.5 */
/* #define Y_VEL_RES 0.03 */
/* #define NUM_Y_VEL_GRID 35 */

/* #define RBF_SIGMA 0.03 */

class LIPM_3D_AC_YP_System : public LIPM_3D_System{
public:
  LIPM_3D_AC_YP_System();
  virtual ~LIPM_3D_AC_YP_System();

  // Return True if nx state is Terminal
  virtual bool Transition(const double* state,
                          const double* action,
                          double & reward,
                          double* nx_state, bool b_print = false);
  void Get_Yp_SwitchingTime( double & yp, double & t_switch){
    yp = yp_;
    t_switch = t_switch_;
  }

 protected:
  double yp_;
  double t_switch_;
  bool _is_terminal(double t_switch, double xp, double yp);
  double v_des_;
  double terminal_reward_;
};

#endif
