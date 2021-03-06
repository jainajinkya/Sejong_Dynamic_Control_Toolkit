#ifndef WALKER_2D_DDP_CONTROLLER_H
#define WALKER_2D_DDP_CONTROLLER_H

#include <Walker2D_Controller/Walker2D_Controller.hpp>
#include <Walker2D_Model/Walker2D_Model.hpp>
#include <Walker2D_Model/Walker2D_Dyn_Model.hpp>
#include <Walker2D_Model/Walker2D_Kin_Model.hpp>

#include <Optimizer/Goldfarb/QuadProg++.hh>
#include "iLQR.hpp"

class DDP_ctrl: public Walker2D_Controller{
public:
  DDP_ctrl();
  virtual ~DDP_ctrl();

  virtual void ComputeTorqueCommand(sejong::Vector & gamma);
  virtual void Initialization();

  // Computes x_{t+1} = f(x_t, u_t)
  sejong::Vector f(const sejong::Vector & x, const sejong::Vector & u); 

  // Computes x_{t+1} given state and actuated torques f(x_t, gamma)
  sejong::Vector f_given_tact(const sejong::Vector & x, const sejong::Vector & gamma_int); 

  // Computes l(x, u)   
  double l_cost(const sejong::Vector &x, const sejong::Vector &u);  
  // Computes l_F(x)  
  double l_cost_final(const sejong::Vector &x_F);

  void l_x_analytical(const sejong::Vector &x, const sejong::Vector &u,  sejong::Vector & l_x);
  void l_x_final_analytical(const sejong::Vector &x, sejong::Vector & l_xF);  
  void l_xx_analytical(const sejong::Vector &x, const sejong::Vector &u, sejong::Matrix & l_xx);  
  void l_xx_final_analytical(const sejong::Vector &x, sejong::Matrix & l_xxF);  
  void l_u_analytical(const sejong::Vector &x, const sejong::Vector &u,  sejong::Vector & l_u);
  void l_uu_analytical(const sejong::Vector &x, const sejong::Vector &u, sejong::Matrix & l_uu);  
  void l_ux_analytical(const sejong::Vector &x, const sejong::Vector &u, sejong::Matrix & l_ux);
  void f_u_analytical(const sejong::Vector &x, const sejong::Vector &u,  sejong::Matrix & f_u);

  double custom_J_cost(const std::vector<sejong::Vector> & X, const std::vector<sejong::Vector> & U);

protected:
  int STATE_SIZE =  NUM_QDOT + NUM_QDOT;
  int DIM_u_SIZE =  4;
  const double NEAR_ZERO = std::sqrt(std::numeric_limits<double>::epsilon());
  double ddp_time_step = 1.0/100.0;//1.0/10000.0;//1.0/10000.0; //1.0/100.0; //

  // Functions
  void _DDP_ctrl(sejong::Vector & gamma);  
  void _jpos_ctrl(sejong::Vector & gamma);
  void _QP_ctrl(sejong::Vector & gamma);  
  sejong::Vect3 ee_ini_;

  void _update_internal_model(const sejong::Vector & x_state);
  void _get_B_c(const sejong::Vector & x_state, sejong::Matrix & B_out, sejong::Vector & c_out);   
  void _get_WBC_command(const sejong::Vector & x_state, 
                        const sejong::Vector & u_input, 
                        sejong::Vector & gamma_int); 

  void _prep_QP_U_f();  
  void _prep_QP_FR_sol(const sejong::Vector & x_state);  
  void _solveQP_for_FR(const sejong::Vector & x_state, 
                       sejong::Vector & Fr_result);

  void _prep_QP_xddot_sol(const sejong::Vector & x_state, const sejong::Vector & Fr);  
  void _solveQP_for_xddot(const sejong::Vector & x_state, const sejong::Vector & Fr,
                          sejong::Vector & xddot_result);

  iLQR* ilqr_;
  // Get Internal Model of the robot for DDP
  Walker2D_Model* internal_model;
  sejong::Matrix A_int;
  sejong::Matrix Ainv_int;
  sejong::Vector grav_int;
  sejong::Vector coriolis_int;

  sejong::Matrix Sv; // Virtual Joints Selection Matrix
  sejong::Matrix Sa; // Actuated Joints Selection Matrix

  std::vector<sejong::Vector> gamma_sequence;  
  // Prepare running and final costs
  sejong::Matrix P_run;   // EE Running State Cost  
  sejong::Matrix Q_run;   // Running State Cost
  sejong::Matrix Q_final; // Final State Cost  
  sejong::Matrix P_final;   // EE Final State Cost  
  sejong::Matrix N_run; // Acceleration Cost
  sejong::Matrix T_run; // Torque Cost. Will not be used as it's too expensive.

  sejong::Vector x_des_final; // desired final x_state

  sejong::Vector ee_des; // desired final x_state    


  // Quad Program
  sejong::Matrix tot_tau_mtx;
  sejong::Vector tot_tau_vec;  
  sejong::Vector Fr_result_;   
  int dim_opt;  // Number of Reaction Forces: Fr = [Fx1, Fz1, Fx2, Fz2]
  int dim_eq_cstr;  // Number of Equality Constraints.
  int dim_ieq_cstr;
  GolDIdnani::GVect<double> z_out;

  // Cost
  GolDIdnani::GMatr<double> G;
  GolDIdnani::GVect<double> g0;

  // Equality
  GolDIdnani::GMatr<double> CE;
  GolDIdnani::GVect<double> ce0;

  // Inequality
  GolDIdnani::GMatr<double> CI;
  GolDIdnani::GVect<double> ci0;

  // QP Const Matrices;
  sejong::Matrix U_f_;


  // QP constraint matrices and vectors
  double virt_eps_relax;
  sejong::Vector virt_relax; 
  sejong::Matrix U_f; // Friction Contact Matrix
  sejong::Vector tau_min; // Minimum Torque
  sejong::Vector tau_max; // Maximum Torque
  sejong::Vector Fr_max; //  Maximum Reaction Force



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
