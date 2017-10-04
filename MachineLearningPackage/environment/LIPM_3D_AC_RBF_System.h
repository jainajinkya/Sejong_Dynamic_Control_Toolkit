#ifndef LIPM_3D_ACTOR_CRITIC_DYN_SYS_H
#define LIPM_3D_ACTOR_CRITIC_DYN_SYS_H

#include "LIPM_3D_System.h"

#define AC_DIM_STATE 3 //(y0, xdot0, ydot0)
#define AC_DIM_ACTION 2

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

/* #define Y_POS_MAX 0.22 */
/* #define Y_POS_MIN 0.01 */
/* #define Y_POS_RES 0.01 */
/* #define NUM_Y_POS_GRID 22 */

/* #define X_VEL_MAX 0.70 */
/* #define X_VEL_MIN 0.15 */
/* #define X_VEL_RES 0.01 */
/* #define NUM_X_VEL_GRID 56 */

/* #define Y_VEL_MAX 0.25 */
/* #define Y_VEL_MIN -0.25 */
/* #define Y_VEL_RES 0.01 */
/* #define NUM_Y_VEL_GRID 51 */

/* #define RBF_SIGMA 0.01 */

/* #define Y_POS_MAX 0.23 */
/* #define Y_POS_MIN 0.01 */
/* #define Y_POS_RES 0.02 */
/* #define NUM_Y_POS_GRID 12 */

/* #define X_VEL_MAX 0.70 */
/* #define X_VEL_MIN 0.14 */
/* #define X_VEL_RES 0.02 */
/* #define NUM_X_VEL_GRID 29 */

/* #define Y_VEL_MAX 0.25 */
/* #define Y_VEL_MIN -0.25 */
/* #define Y_VEL_RES 0.02 */
/* #define NUM_Y_VEL_GRID 26 */

/* #define RBF_SIGMA 0.02 */

#define Y_POS_MAX 0.25
#define Y_POS_MIN 0.01
#define Y_POS_RES 0.03
#define NUM_Y_POS_GRID 9

#define X_VEL_MAX 0.45
#define X_VEL_MIN 0.03
#define X_VEL_RES 0.03
#define NUM_X_VEL_GRID 15

#define Y_VEL_MAX 0.24
#define Y_VEL_MIN -0.24
#define Y_VEL_RES 0.03
#define NUM_Y_VEL_GRID 17

#define RBF_SIGMA 0.03

class LIPM_AC_RBF_System : public LIPM_3D_System{
public:
  LIPM_AC_RBF_System();
  virtual ~LIPM_AC_RBF_System();

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
