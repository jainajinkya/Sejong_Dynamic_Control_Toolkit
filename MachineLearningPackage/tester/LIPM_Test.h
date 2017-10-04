#ifndef LIPM_TEST_H
#define LIPM_TEST_H

/* #include <environment/LIPM_3D_AC_RBF_System.h> */
/* #include <environment/LIPM_3D_Act_Only.h> */

/* #include <agent/LIPM_ActorCritic_RBF.h> */
#include <agent/LIPM_AC_YDOT_change.h>
/* #include <agent/LIPM_AC_YP_change.h> */
#include <agent/LIPM_AC_RBF_Switching.h>

#include <agent/LIPM_ActOnly_Traj.h>
#include <agent/LIPM_ActOnly_Phase.h>

#include "LIPM_Plot_System.h"
#include "PlottingManager.h"
#include <planner/PlannerTester.h>

/* #define DO_TEST */

sejong::Vector ini_state(4);

void _set_initial(){
  ini_state[0] = 0.0;
  ini_state[1] = 0.05;
  ini_state[2] = 0.1;
  ini_state[3] = 0.0;
}

void LIPM_ActorCritic_Test_Swiching(){
  // Actor Critic RBF
  Agent* agent = new LIPM_AC_RBF_Switching();

  sejong::Vector dummy_ini_state(4);
  agent->DoLearning(dummy_ini_state);
}

// Actor Critic
void LIPM_ActorCritic_Test(){
  _set_initial();

  /* // Actor Critic RBF */
  /* Agent* agent = new LIPM_ActorCritic_RBF(); */

  // Actor Critic YDOT
  Agent* agent = new LIPM_AC_YDOT_change();

  /* // Actor Critic YP */
  /* Agent* agent = new LIPM_AC_YP_change(); */

  sejong::Vector reduced_ini_state(3);
  for(int i(0); i<3; ++i){
    reduced_ini_state[i] = ini_state[i+1];
  }
  agent->DoLearning(reduced_ini_state);
}

// Act Only (Policy Gradient)
/* void LIPM_ActOnly_Test(){ */
/*   _set_initial(); */

/*   Environment* env = new LIPM_3D_Act_Only(); */
/*   // Agent* agent = new LIPM_ActOnly_Traj(dynamic_cast<LIPM_3D_Act_Only*>(env)); */
/*   Agent* agent = new LIPM_ActOnly_Phase(dynamic_cast<LIPM_3D_Act_Only*>(env)); */


/*   PlannerTester planner_test(ini_state, dynamic_cast<LIPM_3D_Act_Only*>(env) ); */
/*   planner_test.check(); */

/*   ////////////////////////////////////////////////// */
/*   // Do learning */
/*   agent->DoLearning(ini_state); */

/* #ifdef DO_TEST */
/*   Plotting_System* plot_sys = new LIPM_Plot_System( dynamic_cast<LIPM_3D*> (env), dynamic_cast<LIPM_Act_Only_Learner*>(agent)->GetLIPM_Tester() ); */

/*   PlottingManager* plot_manager = new PlottingManager(plot_sys); */

/*   double time(0.0); */
/*   while (true){ */
/*     plot_manager->SendData(time); */
/*     time += 0.02; */
/*     if(time > plot_sys->GetEndTime() ){ */
/*       break; */
/*     } */
/*   } */
/* #endif */
/* } */

#endif
