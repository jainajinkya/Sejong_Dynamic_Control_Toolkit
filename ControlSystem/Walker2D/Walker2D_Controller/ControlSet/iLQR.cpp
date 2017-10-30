#include "iLQR.hpp"

iLQR::iLQR(){
  printf("[iLQR] Constructor Initialized\n");
}

iLQR::~iLQR(){
}

void iLQR::compute_ilqr(){
	sejong::Vector x_vec_empty(10);
	sejong::Vector u_vec_empty(10);
	sejong::Vector gamma(5);	
	x_vec_empty.setOnes();
	u_vec_empty.setOnes();
	gamma.setOnes();	
	l_cost(x_vec_empty, u_vec_empty);
	l_cost_final(x_vec_empty);

	sejong::pretty_print(gamma, std::cout, "gamma before wbc");
	get_WBC_command(x_vec_empty, u_vec_empty, gamma);
	sejong::pretty_print(gamma, std::cout, "gamma after wbc");	
}
