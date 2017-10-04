#include <iostream>
#include <stdio.h>
#include <Utils/wrap_eigen.hpp>

// Unit Test
#include "RBF_Test.h"

// Learning Test
#include "MaceTest.h"
#include "LIPM_Test.h"
#include "Link_Test.h"

int main(int argc, char ** argv){
  // Unit Test
  // Distribution_Test();
  // RBF_Test();

  printf("Learning Tester\n");
  // MaceTest();

  // LIPM_ActOnly_Test();
  LIPM_ActorCritic_Test();
  // LIPM_ActorCritic_Test_Swiching();

  // Link_ActorCritic_Test();

  return 0;
}
