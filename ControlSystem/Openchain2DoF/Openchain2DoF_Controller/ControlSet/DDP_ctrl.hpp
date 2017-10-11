#ifndef OPENCHAIN_2DOF_DDP_CONTROLLER_H
#define OPENCHAIN_2DOF_DDP_CONTROLLER_H

#include <Openchain2DoF_Controller/OC2Controller.hpp>
//#include <Simulator/RBDL_simulator/Systems/Openchain2DoF/OC2_Sim_Model.hpp>

#include <Openchain2DoF_Model/OC2_Model.hpp>
#include <Openchain2DoF_Model/OC2_Dyn_Model.hpp>
#include <Openchain2DoF_Model/OC2_Kin_Model.hpp>

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
  void _initiailize_u_sequence(); 
  // Initializes X = {x1, x2, ..., xN}
  void _initiailize_x_sequence(const sejong::Vector x_state_start); 
  // gives torque for desired task accelerations
  void _getWBC_command(const sejong::Vector & x_state, const sejong::Vector & des_acc, sejong::Vector & gamma_int); 
   // Computes x_{t+1} = f(x, gamma(u)) for  a specified simulation rate
  void _internal_simulate_single_step(const sejong::Vector & x_state, 
                                      const sejong::Vector & gamma_int, 
                                      sejong::Vector & x_next_state);
  // Simulates U = {u1, u2, ..., uN} to get X = {x1, x2, ..., xN}
  void _internal_simulate_sequence(const std::vector<sejong::Vector> U,  std::vector<sejong::Vector> X); 
  // computes l(x,u) - A quadratic cost 
  void _l_cost(const sejong::Vector & x_state, const sejong::Vector & u_in, double & cost);
  void _l_running_cost(const sejong::Vector & x_state, const sejong::Vector & u_in, double & cost);
  void _l_final_cost(const sejong::Vector & x_state_final, double & cost);  
  void _gradient_finite_difference();
  void _hessian_finite_difference();  


  sejong::Vect3 ee_ini_;

  // Get Internal Model of the robot for DDP
  OC2Model* internal_model;
  sejong::Matrix A_int;
  sejong::Matrix Ainv_int;
  sejong::Vector grav_int;
  sejong::Vector coriolis_int;

  // DDP Variables
  int DIM_WBC_TASKS;
  std::vector<sejong::Vector> x_sequence;  
  std::vector<sejong::Vector> u_sequence;
  std::vector<sejong::Vector> gamma_sequence;  

  std::vector<sejong::Vector> l_x;  
  std::vector<sejong::Matrix> l_xx;  
  std::vector<sejong::Vector> l_u;   
  std::vector<sejong::Matrix> l_uu;   
  std::vector<sejong::Matrix> l_ux;

  std::vector<sejong::Matrix> f_x;
  std::vector<sejong::Matrix> f_u;

  std::vector<sejong::Vector> V_x;
  std::vector<sejong::Matrix> V_xx;

  double J_cost;
  std::vector<double> J_cost_tail;

  int N_horizon; // Control Horizon
  double mpc_time_step;
  double sim_rate;

  sejong::Vector des_oper_goal;


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
