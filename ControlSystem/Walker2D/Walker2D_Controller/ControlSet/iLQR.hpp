#ifndef WALKER_2D_ILQR_H
#define WALKER_2D_ILQR_H

#include <math.h>
#include <stdio.h>
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
//#include <Utils/DataManager.hpp>
#include <functional>
#include "Configuration.h"

#define STATE_SIZE NUM_Q + NUM_QDOT
class WBC_iLQR{
public:
  WBC_iLQR();
  WBC_iLQR(int DIM_WBC_TASKS_in);  
  ~WBC_iLQR();
 
  // Output: double cost. Input: Vector x, Vector u
  std::function<double(const sejong::Vector&, const sejong::Vector&)> l_cost; 
  // Output: double cost. Input: Vector x_state
  std::function<double(const sejong::Vector&)> l_cost_final;     
  // Gets the torque computed by the WBC for a given u
  //    Output: Vector gamma. Input: x_state, u_state    
  std::function<void(const sejong::Vector&, const sejong::Vector&, sejong::Vector&)> get_WBC_command;

  void set_DIM_WBC_TASKS(int DIM_WBC_TASKS_in);
  void compute_ilqr();

  // If analytical, finite difference will not be computed
  bool analytical_l_x = false;
  bool analytical_l_xx = false;
  bool analytical_l_u = false;
  bool analytical_l_uu = false;
  bool analytical_l_ux = false;
  bool analytical_f_x = false;
  bool analytical_f_u = false;  
  bool analytical_H_f_xx = false;    
  bool analytical_H_f_xu = false;      
  bool analytical_H_f_ux = false;    

protected:
  int DIM_WBC_TASKS;
  int N_horizon = 10;
  double lambda = 1.0; //  Regularization Parameter
  double lambda_min = 0.000001; 
  double dlambda = 1.0;
  double lambda_factor = 1.6; // Lambda Factor
  double z_min = 0.0;

  void   _initialize_gradients_hessians();
  void   _compute_finite_differences();
  double _J_cost(const std::vector<sejong::Vector> & X, const std::vector<sejong::Vector> & U);

  // Member Variables
  std::vector<sejong::Vector> x_sequence;  
  std::vector<sejong::Vector> u_sequence;
  std::vector<sejong::Vector> gamma_sequence;  

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

