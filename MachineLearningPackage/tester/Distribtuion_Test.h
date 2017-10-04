#ifndef DISTRIBUTION_TEST
#define DISTRIBUTION_TEST

#include <Utils/utilities.h>

// Distribution function test
void Distribution_Test(){
  double mean(0.8);
  double var(0.1);
  double min(0.4);
  double max(1.2);
  for(int i(0); i < 50000; ++i){
    double tmp = sejong::generator_truncated_white_noise(mean, var, min, max);
    sejong::saveValue(tmp, "truncated_normal_test");
  }
  exit(0);
}

#endif
