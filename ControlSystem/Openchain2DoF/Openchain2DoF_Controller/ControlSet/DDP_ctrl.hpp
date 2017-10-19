#ifndef OPENCHAIN_2DOF_DDP_CONTROLLER_H
#define OPENCHAIN_2DOF_DDP_CONTROLLER_H

#include <Openchain2DoF_Controller/OC2Controller.hpp>
//#include <Simulator/RBDL_simulator/Systems/Openchain2DoF/OC2_Sim_Model.hpp>

#include <Openchain2DoF_Model/OC2_Model.hpp>
#include <Openchain2DoF_Model/OC2_Dyn_Model.hpp>
#include <Openchain2DoF_Model/OC2_Kin_Model.hpp>

#define STATE_SIZE NUM_Q + NUM_QDOT
class DDP_ctrl: public OC2Controller{
public:
  DDP_ctrl();
  virtual ~DDP_ctrl();

  virtual void ComputeTorqueCommand(sejong::Vector & gamma);
  virtual void Initialization();

protected:
  // Functions
  void _zero_ctrl(sejong::Vector & gamma); 
  void _jpos_ctrl(sejong::Vector & gamma);
  void _ee_ctrl(sejong::Vector & gamma);
  void _mpc_ctrl(sejong::Vector & gamma);

  // Updates a model of the robot given x = [q, qdot]
  void _update_internal_model(const sejong::Vector & x_state);
  // Initializes U = {u1, u2, ..., uN}
  void _initialize_u_sequence(std::vector<sejong::Vector> & U); 
  // Initializes X = {x1, x2, ..., xN}
  void _initialize_x_sequence(const sejong::Vector & x_state_start, const std::vector<sejong::Vector> & U, std::vector<sejong::Vector> & X); 
  // gives torque for desired task accelerations
  void _getWBC_command(const sejong::Vector & x_state, const sejong::Vector & des_acc, sejong::Vector & gamma_int); 
   // Computes x_{t+1} = f(x, gamma(u)) for  a specified simulation rate
  void _internal_simulate_single_step(const sejong::Vector & x_state, const sejong::Vector & gamma_int, sejong::Vector & x_next_state);
  // x_{t+1} = f(x_t, u_t)
  sejong::Vector _x_tp1(const sejong::Vector & x_state, const sejong::Vector & u_in); 
  // Simulates U = {u1, u2, ..., uN} to get X = {x1, x2, ..., xN}
  void _internal_simulate_sequence(const std::vector<sejong::Vector> & U,  std::vector<sejong::Vector> & X); 
  // computes l(x,u) - A quadratic cost 
  double _l_cost(const sejong::Vector & x_state, const sejong::Vector & u_in);
  double _l_final_cost(const sejong::Vector & x_state_final);  
  
  void _get_finite_differences();

  void _compute_ilqr();


  double _J_cost(const std::vector<sejong::Vector> & X, const std::vector<sejong::Vector> & U);

/*  void  _simulate(const std::vector<sejong::Vector> & X, const std::vector<sejong::Vector> & U, double & cost);
  void  _simulate(const std::vector<sejong::Vector> & X, const std::vector<sejong::Vector> & U, std::vector<sejong::Vector> & X_new, double & cost);
  void _get_gradients_hessians();

  void _getB(const sejong::Vector & x_state, sejong::Matrix & B_out);
  void _getC(const sejong::Vector & x_state, sejong::Matrix & C_out);  
  void _getWBC_command(const sejong::Vector & x_state, const sejong::Vector & des_acc, const int index, sejong::Vector & gamma_int);
  void  _getUnew(const std::vector<sejong::Vector> & k, const std::vector<sejong::Matrix> & K);


  // calculates l_x, l_xx, l_u, l_uu, f_u
  void  _calc_analytical_gradients_hessians(); 
  // Finite difference calculate f_x
  void  _calculate_f_x(); */


  sejong::Vect3 ee_ini_;

  // Get Internal Model of the robot for DDP
  OC2Model* internal_model;
  sejong::Matrix A_int;
  sejong::Matrix Ainv_int;
  sejong::Vector grav_int;
  sejong::Vector coriolis_int;

  // DDP Variables
  double lambda_factor;

  sejong::Matrix Q_mat; // Position Cost matrix
  sejong::Matrix N_mat; // Acceleration Cost matrix
  sejong::Matrix R_mat; // Torque Cost matrix

  double eps_converge;
  std::vector<sejong::Matrix> B;
  std::vector<sejong::Matrix> C;
  std::vector<sejong::Vector> tau;


  int DIM_WBC_TASKS;
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

  std::vector<double> J_cost_tail;

  int N_horizon; // Control Horizon
  double mpc_time_step;
  double sim_rate;
  double finite_epsilon;

  sejong::Vector des_oper_goal;

  double time_sum;
  int count; // Time counter

  int ilqr_iters;

  // Data Save
  sejong::Vector des_pos_;
  sejong::Vector act_pos_;
  sejong::Vector act_vel_;
  sejong::Vector xddot_cmd_;
  
  int count_command_;
  double state_machine_time_;
  double start_time_;

  sejong::Vector jpos_ini_;
  sejong::Vector jpos_des_;

  SJLinkID link_id_;
  int initialization_count_;

};


#endif
