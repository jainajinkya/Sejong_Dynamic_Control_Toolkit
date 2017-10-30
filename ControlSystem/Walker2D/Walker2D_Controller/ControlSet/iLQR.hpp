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
  // Output: double cost. Input: Vector x, Vector u
  std::function<double(const sejong::Vector&, const sejong::Vector&)> l_cost; 
  // Output: double cost. Input: Vector x_state
  std::function<double(const sejong::Vector&)> l_cost_final;     
  // Output: Vector, gamma. Input: x_state, u_state    
  std::function<void(const sejong::Vector&, const sejong::Vector&, sejong::Vector&)> get_WBC_command;

  void compute_ilqr();

protected:
  double lambda = 1.0; //  Regularization Parameter
  double lambda_min = 0.000001; 
  double dlambda = 1.0;
  double lambda_factor = 1.6; // Lambda Factor
  double z_min = 0.0;

//  double (*_l_cost)(const sejong::Vector);   
  // Functions
  // void _compute_ilqr(sejong::Vector & gamma);

  //double _l_cost(const sejong::Vector & x_state, const sejong::Vector & u_in);
  //double _l_final_cost(const sejong::Vector & x_state_final);  

  // Member Variables
  sejong::Vect3 vector_test;

};


#endif

