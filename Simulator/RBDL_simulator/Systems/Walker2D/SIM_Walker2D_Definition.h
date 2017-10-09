#ifndef DEFINITION_SIM_OPENCHAIN_3D
#define DEFINITION_SIM_OPENCHAIN_3D

#include <stdio.h>
#include <string>


#define NUM_LINES 3
#define NUM_VALUE_PYTHON NUM_LINES * 4 + 1

enum SJ_SIM_LinkID{
  LK_SIM_L1 = 0,
  LK_SIM_L2,
  LK_SIM_L3,
  NUM_SIM_LINK,
  LK_SIM_J2,
  LK_SIM_J3,
  LK_SIM_EE
};

#endif
