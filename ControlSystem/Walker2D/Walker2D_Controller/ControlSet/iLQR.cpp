#include "iLQR.hpp"

iLQR::iLQR(){
  printf("[iLQR] Constructor Initialized\n");
}

iLQR::~iLQR(){
}

void iLQR::compute_ilqr(){
	sejong::Vector x_vec_empty(10);
	x_vec_empty.setOnes();
	l_cost(x_vec_empty);
}
