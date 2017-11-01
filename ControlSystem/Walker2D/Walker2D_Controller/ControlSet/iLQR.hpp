#ifndef GENERIC_ILQR_H
#define GENERIC_ILQR_H

#include <math.h>
#include <stdio.h>
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
//#include <Utils/DataManager.hpp>
#include <functional>
#include "Configuration.h"

class iLQR{
public:
  iLQR(int STATE_SIZE_in = NUM_QDOT + NUM_QDOT,
       int DIM_u_in = 4,
       int N_horizon_in = 10,
       int ilqr_iters_in = 15);
  ~iLQR();

  // Function Pointers ---------------------------------------------------------
  // Computes x_{t+1} = f(x_t, u_t)
  // Output Vector x_{t+1}. Input x_t, u_t 
  std::function<sejong::Vector(const sejong::Vector&, const sejong::Vector &)> f; 
  // Computes l(x, u) 
  // Output: double cost. Input: Vector x, Vector u
  std::function<double(const sejong::Vector&, const sejong::Vector&)> l_cost; 
  // Computes l_F(x)
  // Output: double cost. Input: Vector x_state
  std::function<double(const sejong::Vector&)> l_cost_final;     

  // Public Functions ---------------------------------------------------------
  void set_DIM_x(int DIM_x_in);
  void set_DIM_u(int DIM_u_in);
  void set_N_horizon(int N_horizon_in);
  void compute_ilqr(const sejong::Vector & x_state_start);

  // If cusom, finite difference will not be computed
  bool custom_l_x = false;
  bool custom_l_xx = false;
  bool custom_l_u = false;
  bool custom_l_uu = false;
  bool custom_l_ux = false;
  bool custom_f_x = false;
  bool custom_f_u = false;  
  bool custom_H_f_xx = false;    
  bool custom_H_f_xu = false;      
  bool custom_H_f_ux = false;    

protected:
  int STATE_SIZE;
  int DIM_u;
  int N_horizon; 
  int ilqr_iters;
  double finite_epsilon = 1e-6;
  double lambda = 1.0; //  Regularization Parameter
  double lambda_min = 0.000001; 
  double dlambda = 1.0;
  double lambda_factor = 1.6; // Lambda Factor
  double z_min = 0.0;

  std::vector<double> alpha_cand_pow = {0, -0.3, -0.6, -1.2, -1.5, -1.8, -2.1, -2.4, -2.7, -3.0};
  std::vector<double> alpha_cand;


  void   _initialize_X_U();
  void   _initialize_gradients_hessians();

  void   _initialize_U_sequence(std::vector<sejong::Vector> & U);

  void   _compute_X_sequence(const sejong::Vector & x_state_start,  
                             const std::vector<sejong::Vector> & U, 
                                   std::vector<sejong::Vector> & X);
  void   _compute_finite_differences();
  double _J_cost(const std::vector<sejong::Vector> & X, const std::vector<sejong::Vector> & U);

  // Member Variables
  std::vector<sejong::Vector> x_sequence;  
  std::vector<sejong::Vector> u_sequence;

  std::vector<sejong::Vector> l_x;  
  std::vector<sejong::Matrix> l_xx;  
  std::vector<sejong::Matrix> l_xu;    
  std::vector<sejong::Vector> l_u;   
  std::vector<sejong::Matrix> l_uu;   
  std::vector<sejong::Matrix> l_ux;

  std::vector<sejong::Matrix> f_x;
  std::vector<sejong::Matrix> f_u;

  std::vector<sejong::Vector> k_vec;
  std::vector<sejong::Matrix> K_vec; 

  std::vector< std::vector<sejong::Matrix> > H_f_xx; // sequence of N horizon elements. Each element has hessians H = (H(f1), H(f2), ... H(fn))
  std::vector< std::vector<sejong::Matrix> > H_f_xu;
  std::vector< std::vector<sejong::Matrix> > H_f_ux;    

  std::vector<sejong::Vector> V_x;
  std::vector<sejong::Matrix> V_xx;
};


#endif

