#include "iLQR.hpp"

iLQR::iLQR(){
  printf("[iLQR] Constructor Initialized\n");
}

iLQR::~iLQR(){
}

void iLQR::compute_ilqr(){
	l_cost(2.0);
	sejong::Vector x_vec_empty(10);
	x_vec_empty.setOnes();
	l_cost_vec(x_vec_empty);
}
