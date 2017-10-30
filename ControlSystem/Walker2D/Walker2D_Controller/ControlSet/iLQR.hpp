#ifndef WALKER_2D_ILQR_H
#define WALKER_2D_ILQR_H

#include <Utils/wrap_eigen.hpp>
#include <math.h>
#include <stdio.h>

class iLQR{
public:
  iLQR();
  virtual ~iLQR();
  //virtual void Initialization();

protected:
  // Functions
  // void _compute_ilqr(sejong::Vector & gamma);

  //double _l_cost(const sejong::Vector & x_state, const sejong::Vector & u_in);
  //double _l_final_cost(const sejong::Vector & x_state_final);  

  // Member Variables
  sejong::Vect3 vector_test;

};


#endif
