#ifndef DEFINITION_SIM_OPENCHAIN_2D
#define DEFINITION_SIM_OPENCHAIN_2D

#include <stdio.h>
#include <string>

#define NUM_LINES 2
#define NUM_VALUE_PYTHON NUM_LINES * 4 + 1

enum SJ_SIM_LinkID{
  LK_SIM_L1 = 0,
  LK_SIM_L2,
  NUM_SIM_LINK,
  LK_SIM_J2,
  LK_SIM_EE
};

#endif
