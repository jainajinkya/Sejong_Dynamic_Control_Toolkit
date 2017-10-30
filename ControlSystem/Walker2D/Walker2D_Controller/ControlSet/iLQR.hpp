#ifndef WALKER_2D_ILQR_H
#define WALKER_2D_ILQR_H

#include <math.h>
#include <stdio.h>
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
//#include <Utils/DataManager.hpp>
#include <functional>

class iLQR{
public:
  iLQR();
  ~iLQR();

  std::function<double(const sejong::Vector)> l_cost;  

  void compute_ilqr();

protected:

//  double (*_l_cost)(const sejong::Vector);   
  // Functions
  // void _compute_ilqr(sejong::Vector & gamma);

  //double _l_cost(const sejong::Vector & x_state, const sejong::Vector & u_in);
  //double _l_final_cost(const sejong::Vector & x_state_final);  

  // Member Variables
  sejong::Vect3 vector_test;

};


#endif

