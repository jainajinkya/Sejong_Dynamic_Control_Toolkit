#include "DDP_ctrl.hpp"
#include <math.h>
#include <stdio.h>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <chrono>

DDP_ctrl::DDP_ctrl(): OC2Controller(),
                          count_command_(0),
                          jpos_ini_(NUM_ACT_JOINT),
                          N_horizon(50),
                          DIM_WBC_TASKS(2),                          
                          des_oper_goal(2),
                          finite_epsilon(0.0001),
                          des_pos_(2),
                          act_pos_(2),
                          act_vel_(2)
{
  internal_model = OC2Model::GetOC2Model(); // Get Model of the robot for internal simulation
  x_sequence = std::vector<sejong::Vector>(N_horizon);
  u_sequence = std::vector<sejong::Vector>(N_horizon);  


  for (size_t i = 0; i < N_horizon; i++){
    sejong::Vector n_l_x(STATE_SIZE);
    sejong::Matrix n_l_xx(STATE_SIZE, STATE_SIZE);
    sejong::Matrix n_l_xu(STATE_SIZE, DIM_WBC_TASKS);  
    sejong::Vector n_l_u(DIM_WBC_TASKS);
    sejong::Matrix n_l_uu(DIM_WBC_TASKS, DIM_WBC_TASKS);    
    sejong::Matrix n_l_ux(DIM_WBC_TASKS, STATE_SIZE);      
    sejong::Matrix n_f_x(STATE_SIZE, STATE_SIZE);      
    sejong::Matrix n_f_u(STATE_SIZE, DIM_WBC_TASKS);

    sejong::Vector n_k_vec(DIM_WBC_TASKS);
    sejong::Matrix n_K_vec(DIM_WBC_TASKS, STATE_SIZE);    
    

    n_l_x.setZero();
    n_l_xx.setZero();
    n_l_xu.setZero();
    n_l_u.setZero();
    n_l_uu.setZero();
    n_l_ux.setZero();
    n_f_x.setZero();
    n_f_u.setZero();    

    l_x.push_back(n_l_x);
    l_xx.push_back(n_l_xx);
    l_xu.push_back(n_l_xu);
    l_u.push_back(n_l_u);
    l_uu.push_back(n_l_uu);
    l_ux.push_back(n_l_ux);
    f_x.push_back(n_f_x);                
    f_u.push_back(n_f_u);                

    k_vec.push_back(n_k_vec);
    K_vec.push_back(n_K_vec);        

    // Create empty Hessian H = H(f1), H(f2), ..., H(fn)
    std::vector<sejong::Matrix> H_f_kxx;
    std::vector<sejong::Matrix> H_f_kxu; 
    std::vector<sejong::Matrix> H_f_kux; 
    for (size_t j = 0; j < STATE_SIZE; j++){
      sejong::Matrix n_f_kxx(STATE_SIZE, STATE_SIZE);
      sejong::Matrix n_f_kxu(STATE_SIZE, DIM_WBC_TASKS);
      sejong::Matrix n_f_kux(DIM_WBC_TASKS, STATE_SIZE);      

      n_f_kxx.setZero();
      n_f_kxu.setZero();      
      
      H_f_kxx.push_back(n_f_kxx);
      H_f_kxu.push_back(n_f_kxu);  
      H_f_kux.push_back(n_f_kux);    
    }

    H_f_xx.push_back(H_f_kxx);     
    H_f_xu.push_back(H_f_kxu); 
    H_f_ux.push_back(H_f_kux);               
  }


  des_oper_goal[0] = 0.25; // Reaching Task x
  des_oper_goal[1] = 0.35; // Reaching Task y  
  ilqr_iters = 5;


  _initialize_u_sequence(u_sequence);

  J_cost_tail = std::vector<double>(N_horizon);

  mpc_time_step = 0.01;//SERVO_RATE; //SERVO_RATE/10; //0.001;
  sim_rate = SERVO_RATE/10;

  count = 0;
  time_sum = 0.0;

  printf("[DDP Controller] Start\n");
}

DDP_ctrl::~DDP_ctrl(){
}

void DDP_ctrl::Initialization(){
  for (int i(0); i < NUM_ACT_JOINT ; ++i){
    jpos_ini_[i] = sp_->Q_[i + NUM_VIRTUAL];
  }
  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_EE, ee_ini_);
  start_time_ = sp_->curr_time_;
  phase_ = 10;
}


void DDP_ctrl::ComputeTorqueCommand(sejong::Vector & gamma){
  _PreProcessing_Command();
  gamma.setZero();

  //_jpos_ctrl(gamma);
  //_ee_ctrl(gamma);
  //_zero_ctrl(gamma);  
  _mpc_ctrl(gamma);    
  ++count_command_;

  state_machine_time_ = sp_->curr_time_ - start_time_;
  _PostProcessing_Command(gamma);
}


void DDP_ctrl::_initialize_u_sequence(std::vector<sejong::Vector> & U){
  // Near zero initialization
  for(size_t i = 0; i < U.size(); i++){
    sejong::Vector u_vec(DIM_WBC_TASKS); // task acceleration vector
    for (size_t j = 0; j < DIM_WBC_TASKS; j++){
      u_vec[j] = 0.0;//0.001; // this can be random
    }
    U[i] = u_vec;
  }
}

// Use U = {u1, u2, ..., uN} to find X = {x1, x2, ..., xN}
void DDP_ctrl::_initialize_x_sequence(const sejong::Vector & x_state_start,  const std::vector<sejong::Vector> & U, std::vector<sejong::Vector> & X){
  X[0] = x_state_start;
  _internal_simulate_sequence(U, X);
}

void DDP_ctrl::_update_internal_model(const sejong::Vector & x_state){
  // Initialize States
  sejong::Vector q_int = x_state.head(NUM_Q);  
  sejong::Vector qdot_int = x_state.tail(NUM_QDOT);    

  // Update internal model
  internal_model->UpdateModel(q_int, qdot_int);
  internal_model->getMassInertia(A_int);
  internal_model->getInverseMassInertia(Ainv_int);
  internal_model->getGravity(grav_int);
  internal_model->getCoriolis(coriolis_int);  
}

sejong::Vector DDP_ctrl::_x_tp1(const sejong::Vector & x_state, const sejong::Vector & u_in){
  sejong::Vector x_next_state(STATE_SIZE);
  x_next_state.setZero();
  // Get gamma(u_in)
  sejong::Vector gamma_int(NUM_ACT_JOINT);
  gamma_int.setZero();
  _getWBC_command(x_state, u_in, gamma_int);

  // Simulate Next Step
  _internal_simulate_single_step(x_state, gamma_int, x_next_state);
  return x_next_state;
}

void DDP_ctrl::_internal_simulate_single_step(const sejong::Vector & x_state, 
                                              const sejong::Vector & gamma_int, 
                                              sejong::Vector & x_next_state){ // x_{t+1} = f(x, gamma(u))
  // Initialize States
  sejong::Vector q_int = x_state.head(NUM_Q);  
  sejong::Vector qdot_int = x_state.tail(NUM_QDOT);    

  _update_internal_model(x_state);
  // Get Joint Accelerations
  sejong::Vector qddot_int = Ainv_int*(gamma_int - coriolis_int - grav_int);

  // Perform Integration
  // Get q_{t+1} and qdot_{t+1}
  sejong::Vector qdot_int_next = qdot_int + qddot_int*mpc_time_step;
  sejong::Vector q_int_next = q_int + qdot_int*mpc_time_step ; 

  x_next_state.head(NUM_Q) = q_int_next;
  x_next_state.tail(NUM_QDOT) = qdot_int_next;  

}

// Initializes X = {x1, x2, ..., xN}
void DDP_ctrl::_internal_simulate_sequence(const std::vector<sejong::Vector> & U,  std::vector<sejong::Vector> & X){

  double mpc_interval = 0.0;
  for(size_t i = 1; i < N_horizon; i++){
     sejong::Vector gamma_int(NUM_ACT_JOINT);
    _getWBC_command(X[i-1], U[i-1], gamma_int);




    sejong::Vector x_state_tmp = X[i-1];
    sejong::Vector x_next_state_tmp(x_state_tmp.size());

    // Simulate constant application of torque 
    //while (mpc_interval <= mpc_time_step){
      //Apply the same gamma_int within the mpc time step
      _internal_simulate_single_step(x_state_tmp, gamma_int, x_next_state_tmp);
    //  x_state_tmp = x_next_state_tmp;
   //   mpc_interval += sim_rate;      
   // }

    X[i] = x_next_state_tmp; // Store X[i]    
    //mpc_interval = 0.0; // reset interval

    // Debug Output    
/*    std::cout << "X[" << i << "] = " << X[i][0] << " " 
                                     << X[i][1] << " "
                                     << X[i][2] << ""
                                     << X[i][3] << ""
                                     << std::endl;
*/    // sejong::pretty_print(X[i], std::cout, "X[i]");    

  }


} 

void DDP_ctrl::_get_finite_differences(){
  sejong::Vector h_step(STATE_SIZE); // Same size as state x
  sejong::Vector h2_step(STATE_SIZE); // Same size as state x  
  sejong::Vector k_step(DIM_WBC_TASKS);    // Same size as input u
  sejong::Vector k2_step(DIM_WBC_TASKS);    // Same size as input u  
  h_step.setZero();
  h2_step.setZero();  
  k_step.setZero();
  k2_step.setZero();


  for (size_t n = 0; n < N_horizon; n++){
    sejong::Vector x = x_sequence[n];
    sejong::Vector u = u_sequence[n];    

    // -----------------------------------
    // Calculate Finite Difference for l_x, l_xx, and l_xu 
    for (size_t i = 0; i < h_step.size(); i++){
      h_step[i] = finite_epsilon;
      // l_x
      l_x[n](i) = (_l_cost(x + h_step, u) - _l_cost(x - h_step, u) ) / (2.0*h_step[i]); 

      // l_xx
      for (size_t j = 0; j < h_step.size(); j++){
        h2_step[j] = finite_epsilon;
        l_xx[n](i,j) = (_l_cost(x + h_step + h2_step, u) - 
                     _l_cost(x + h_step - h2_step, u) - 
                     _l_cost(x - h_step + h2_step, u) + 
                     _l_cost(x - h_step - h2_step, u))/(4*h_step[i]*h2_step[j]);
        h2_step.setZero();        
      }

      // l_xu
      for (size_t j = 0; j < k_step.size(); j++){
        k_step[j] = finite_epsilon;        
        l_xu[n](i,j) = (_l_cost(x + h_step, u + k_step) - 
                     _l_cost(x + h_step, u - k_step) - 
                     _l_cost(x - h_step, u + k_step) + 
                     _l_cost(x - h_step, u - k_step))/(4*h_step[i]*k_step[j]);
        k_step.setZero();        
      }      

      h_step.setZero();
      //std::cout << "    i:" << i << " l_x[i] = "<< l_x[i] << std::endl;

    }

    // -----------------------------------
    // Calculate Finite Difference for l_u, l_uu, and l_ux 
    for (size_t i = 0; i < k_step.size(); i++){
      k_step[i] = finite_epsilon;    
      // Calculate l_u
      l_u[n](i) = (_l_cost(x, u + k_step) - _l_cost(x, u - k_step) ) / (2.0*k_step[i]);

      // Calculate l_uu
      for (size_t j = 0; j < k2_step.size(); j++){
        k2_step[j] = finite_epsilon;
        l_uu[n](i,j) = (_l_cost(x, u + k_step + k2_step) -
                  _l_cost(x, u + k_step - k2_step) -
                  _l_cost(x, u - k_step + k2_step) +
                  _l_cost(x, u - k_step - k2_step)) / (4*k_step[i]*k2_step[j]);
        k2_step.setZero();
      }

      // Calculate l_ux
      for (size_t j = 0; j < h2_step.size(); j++){
        h2_step[j] = finite_epsilon;
        l_ux[n](i,j) = (_l_cost(x + h2_step, u + k_step) -
                  _l_cost(x - h2_step, u + k_step) -
                  _l_cost(x + h2_step, u - k_step) +
                  _l_cost(x - h2_step, u - k_step)) / (4*k_step[i]*h2_step[j]);
        h2_step.setZero();
      }

      k_step.setZero();
    }

    // Calculate f_x
    h_step.setZero();
    for (size_t i = 0; i < STATE_SIZE; i++){
      h_step[i] = finite_epsilon;
      f_x[n].col(i) = (_x_tp1(x + h_step, u) - _x_tp1(x - h_step, u))/ (2.0*h_step[i]);

      // Calculate f_xx
      for (size_t j = 0; j < STATE_SIZE; j++){
        sejong::Vector f_kxx(STATE_SIZE); // partial derivative of f vector with respect to x_i,x_j
        f_kxx.setZero();
        h2_step[j] = finite_epsilon;
        f_kxx = (_x_tp1(x + h_step + h2_step, u) -
                 _x_tp1(x + h_step - h2_step, u) -
                 _x_tp1(x - h_step + h2_step, u) +
                 _x_tp1(x - h_step - h2_step, u)) / (4*h_step[i]*h2_step[j]);

        for (size_t k = 0; k < STATE_SIZE; k++){
          H_f_xx[n][k](i,j) = f_kxx[k]; // Store k element of f_kxx at k-th Hesisan's (i,j) element.
        }
        h2_step.setZero();
      }

      // Calculate f_xu
      for (size_t j = 0; j < DIM_WBC_TASKS; j++){
        sejong::Vector f_kxu(STATE_SIZE); // partial derivative of f vector with respect to x_i, u_j
        f_kxu.setZero();
        k2_step[j] = finite_epsilon;

        f_kxu = (_x_tp1(x + h_step, u + k2_step) -
                 _x_tp1(x + h_step, u - k2_step) -
                 _x_tp1(x - h_step, u + k2_step) +
                 _x_tp1(x - h_step, u - k2_step)) / (4*h_step[i]*k2_step[j]);

        for (size_t k = 0; k < STATE_SIZE; k++){
          H_f_xu[n][k](i,j) = f_kxu[k]; // Store k element of f_kxu at k-th Hesisan's (i,j) element.
        }

        k2_step.setZero();
      }


      h_step.setZero();
    }

    // Calculate f_u
    for (size_t i = 0; i < DIM_WBC_TASKS; i++){
      k_step[i] = finite_epsilon;
      f_u[n].col(i) = (_x_tp1(x, u + k_step) - _x_tp1(x, u - k_step))/ (2.0*k_step[i]);

      // Calculate f_ux
      for (size_t j = 0; j < STATE_SIZE; j++){
        sejong::Vector f_kux(STATE_SIZE); // partial derivative of f vector with respect to x_i, u_j
        f_kux.setZero();
        h2_step[j] = finite_epsilon;

        f_kux = (_x_tp1(x + h2_step, u + k_step) -
                 _x_tp1(x - h2_step, u + k_step) -
                 _x_tp1(x + h2_step, u - k_step) +
                 _x_tp1(x - h2_step, u - k_step)) / (4*k_step[i]*h2_step[j]);

        for (size_t k = 0; k < STATE_SIZE; k++){
          H_f_ux[n][k](i,j) = f_kux[k]; // Store k element of f_kux at k-th Hesisan's (i,j) element.
        }

        k2_step.setZero();
      }


      k_step.setZero();
    }


//    sejong::pretty_print(l_x[n], std::cout, "l_x");    
//    sejong::pretty_print(l_xx[n], std::cout, "l_xx");        
 //     sejong::pretty_print(l_u[n], std::cout, "l_u");        
//      sejong::pretty_print(l_uu[n], std::cout, "l_uu");              
//    sejong::pretty_print(l_xu[n], std::cout, "l_xu");
//    sejong::pretty_print(l_ux[n], std::cout, "l_ux");  
//    sejong::pretty_print(f_x[n], std::cout, "n_f_x");      
//    sejong::pretty_print(f_u[n], std::cout, "n_f_u");      
//     sejong::pretty_print(H_f_xx[n][3], std::cout, "H_f_3xx");      
/*     sejong::pretty_print(H_f_xu[n][3], std::cout, "H_f_3xu");          
     sejong::pretty_print(H_f_ux[n][3], std::cout, "H_f_3ux");          */


  }
//  l(x+h, u) - l(x-h, u) / 2h  
}

double DDP_ctrl::_J_cost(const std::vector<sejong::Vector> & X, const std::vector<sejong::Vector> & U){
  double J_cost = 0.0;
  for(size_t i = 0; i < N_horizon-1; i++){
    J_cost += _l_cost(X[i], U[i]);
  }
  J_cost += _l_final_cost(X[N_horizon-1]); 
  return J_cost;
}

double DDP_ctrl::_l_final_cost(const sejong::Vector & x_state_final){
  double cost = 0.0;

  sejong::Vect3 ee_pos;
  _update_internal_model(x_state_final);
  internal_model->getPosition(x_state_final.head(NUM_Q), SJLinkID::LK_EE, ee_pos);

  sejong::Vector cur_ee_pos(2);
  cur_ee_pos[0] = ee_pos[0];
  cur_ee_pos[1] = ee_pos[1];

  sejong::Matrix Q(cur_ee_pos.rows(), cur_ee_pos.rows()); // task cost

  Q.setZero();

  double cost_weight = 1.0;
  Q(0,0) = cost_weight;
  Q(1,1) = cost_weight;  


  sejong::Matrix cost_in_eigen;
  cost_in_eigen = (des_oper_goal - cur_ee_pos).transpose() * Q * (des_oper_goal - cur_ee_pos);

  cost = cost_in_eigen(0,0);
  //std::cout << "cost = " << cost << std::endl;
  return cost;  


}

double DDP_ctrl::_l_cost(const sejong::Vector & x_state, const sejong::Vector & u_in){
  sejong::Vect3 ee_pos;
  _update_internal_model(x_state);
  internal_model->getPosition(x_state.head(NUM_Q), SJLinkID::LK_EE, ee_pos);

  sejong::Vector cur_ee_pos(2);
  cur_ee_pos[0] = ee_pos[0];
  cur_ee_pos[1] = ee_pos[1];

  sejong::Matrix Q(cur_ee_pos.rows(), cur_ee_pos.rows()); // task cost
  sejong::Matrix R(NUM_ACT_JOINT, NUM_ACT_JOINT); // torque cost
  sejong::Vector gamma_int(NUM_ACT_JOINT);

  Q.setZero();
  R.setZero();
  gamma_int.setZero();
  // if gamma_int is notNan
  //_getWBC_command(x_state, u_in, gamma_int); // We can get torque for any given state and desired acceleration

  double cost_weight = 1.0;
  double acc_cost_weight = 0.001;  
  Q(0,0) = cost_weight;
  Q(1,1) = 3*cost_weight;  

  R(0,0) = cost_weight;
  R(1,1) = cost_weight;    

//  std::cout << "Size of des:" << des_oper_goal.rows() << std::endl;
//  std::cout << "Size of cur:" << ee_pos.rows() << std::endl;  


  sejong::Matrix cost_in_eigen;
  cost_in_eigen = (des_oper_goal - cur_ee_pos).transpose() * Q * (des_oper_goal - cur_ee_pos) + gamma_int.transpose() * R * gamma_int
                  + acc_cost_weight*(u_in.transpose()*u_in);

  double cost = cost_in_eigen(0,0);
  //std::cout << "cost = " << cost << std::endl;
  return cost;
}


// get the torque command given desired acceleration
void DDP_ctrl::_getWBC_command(const sejong::Vector & x_state, const sejong::Vector & des_acc, sejong::Vector & gamma_int){
  // Initialize States
  sejong::Vector q_int = x_state.head(NUM_Q);  
  sejong::Vector qdot_int = x_state.tail(NUM_QDOT);    

  _update_internal_model(x_state);

  // Commanded Task acceleration
  sejong::Vector xddot = des_acc;

  // Commanded Joint Task
  double kp(100.);
  double kd(10.);
  sejong::Vector jpos_cmd = kp * (jpos_ini_ - q_int ) + kd * (qdot_int);

  // For Debug
  sejong::Vect3 ee_pos;
  sejong::Vect3 ee_vel;

  internal_model->getPosition(q_int, SJLinkID::LK_EE, ee_pos);
  internal_model->getVelocity(qdot_int, sp_->Qdot_, SJLinkID::LK_EE, ee_vel);
  //sejong::pretty_print(ee_pos, std::cout, "x position");
  // 

  // Jacobian
  sejong::Matrix Jee, Jee_inv;
  sejong::Matrix Jtmp;
  internal_model->getFullJacobian(q_int, SJLinkID::LK_EE, Jtmp);
  Jee = Jtmp.block(0,0, 2, NUM_QDOT);
  _DynConsistent_Inverse(Jee, Jee_inv);

  sejong::Matrix Jee_dot;
  internal_model->getFullJacobianDot(q_int, qdot_int, SJLinkID::LK_EE, Jtmp);
  Jee_dot = Jtmp.block(0,0, 2, NUM_QDOT);

  
/*  // Debug. Find rank of Jee
  sejong::pretty_print(Jee, std::cout, "Jee:");
  Eigen::JacobiSVD<sejong::Matrix> svd(Jee, Eigen::ComputeThinU | Eigen::ComputeThinV);
  //std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;

  Eigen::FullPivLU<sejong::Matrix> lu(Jee);
  lu.setThreshold(1e-12);
  sejong::pretty_print(q_int, std::cout, "q");
  sejong::pretty_print(qdot_int, std::cout, "qdot");
  int jee_rank;
  jee_rank = lu.rank();
  std::cout << "rank(Jee) = " << lu.rank() << std::endl;

  if (jee_rank == 1){
    exit(0);
  }*/

  //
  

  // Joint task
  sejong::Matrix Nee = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - Jee_inv * Jee;
  sejong::Matrix Jqee_inv;
  _DynConsistent_Inverse(Nee, Jqee_inv);

  // Torque command
  sejong::Vector qddot = Jee_inv * (xddot - Jee_dot * qdot_int);
  qddot = qddot + Jqee_inv * (jpos_cmd - qddot);
  gamma_int = A_int * qddot + coriolis_int + grav_int;  

}


void DDP_ctrl::_compute_ilqr(){
  // Get starting state   // x = [q, qdot]
  sejong::Vector x_state_start(NUM_Q + NUM_QDOT);
  x_state_start.head(NUM_Q) = sp_->Q_; // Store current Q position to state
  x_state_start.tail(NUM_QDOT) = sp_->Qdot_; // Store current Qdot position

  // Initialize X = {x1, x2, ..., xN} using U = {u1, u2, ..., uN} 
  sejong::Vector x = x_state_start;

  double lambda = 1.0; //  Regularization Parameter
  for(size_t ii = 0; ii < ilqr_iters; ii++){
    // Step 1 Forward Pass -------------------------------------------------------------------------
    // Initialize X = {x1, x2, ..., xN} using U = {u1, u2, ..., uN} 
    _initialize_x_sequence(x, u_sequence, x_sequence);
    // Get Finite Difference: l_x, l_u, l_xx, l_xu, l_uu, f_x, f_u, f_xx, f_xu
    _get_finite_differences();

    // Step 2 Backward Pass -------------------------------------------------------------------------
    double V = _l_cost(x_sequence.back(), u_sequence.back()); // back() is equivalent to N_horizon-1
    sejong::Vector V_x = l_x.back();
    sejong::Matrix V_xx = l_xx.back();

    // Work backwards to solve for V, Q, k, and K
    for(int i = N_horizon-1; i >= 0; i--){
      // V' is the next state.
      sejong::Vector Q_x = l_x[i] + f_x[i].transpose()*V_x;
      sejong::Vector Q_u = l_u[i] + f_u[i].transpose()*V_x;
      sejong::Matrix Q_xx = l_xx[i] + f_x[i].transpose()*V_xx*f_x[i];
      sejong::Matrix Q_ux = l_ux[i] + f_u[i].transpose()*V_xx*f_x[i];
      sejong::Matrix Q_uu = l_uu[i] + f_u[i].transpose()*V_xx*f_u[i];


      sejong::Matrix Q_uu_reg = Q_uu;// + lambda * sejong::Matrix::Identity(DIM_WBC_TASKS, DIM_WBC_TASKS);

      Eigen::LLT<sejong::Matrix> lltOfQuu(Q_uu_reg);
      sejong::Matrix L = lltOfQuu.matrixL();

      // Compute inv(Q_uu) with Levenberg-Marquardt Heuristic
      Eigen::EigenSolver<sejong::Matrix> es(Q_uu);
      Eigen::VectorXcd v = es.eigenvectors().col(0);

      sejong::Matrix es_eigVecs = es.eigenvectors().real();
      sejong::Vector es_eigVals = es.eigenvalues().real();
      sejong::Vector es_eigVals_recp(es_eigVals.size());      

      for (size_t j = 0; j < es_eigVals.size(); j++){
        if (es_eigVals[j] < 0){ 
          es_eigVals_recp[j] = 0;
        }else{
          es_eigVals_recp[j] = es_eigVals[j];
        }
        es_eigVals_recp[j] = 1.0/(es_eigVals_recp[j] + lambda);
        // std::cout << "eig recp j" << j << " " << es_eigVals_recp[j] << std::endl;
      }
      sejong::Matrix Q_uu_inv = es_eigVecs * (es_eigVals_recp.asDiagonal()) * (es_eigVecs.transpose());



      // std::cout << "The eigenvalues of Q_uu are:" << std::endl << es_eigVals  << std::endl;
      // std::cout << "The matrix of eigenvectors, V, is:" << std::endl << es_eigVecs << std::endl;
      // std::cout << "regularized Q_uu inverse is:" << std::endl << Q_uu_inv << std::endl;
      std::cout << "real Q_uu inverse is:" << std::endl << Q_uu.inverse() << std::endl;      
      std::cout << "Cholesky Q_uu inverse is:" << std::endl << (L.inverse()).transpose()*(L.inverse()) << std::endl;            

      k_vec[i] = -Q_uu_inv*Q_u;
      K_vec[i] = -Q_uu_inv*Q_ux;

      V_x = Q_x - K_vec[i].transpose()*Q_uu*k_vec[i]; 
      V_xx = Q_xx - K_vec[i].transpose()* Q_uu * K_vec[i];

    }

    std::vector<sejong::Vector> x_new_sequence = x_sequence;//std::vector<sejong::Vector>(N_horizon);
    std::vector<sejong::Vector> u_new_sequence = u_sequence;//std::vector<sejong::Vector>(N_horizon);

    std::cout << "iter:" << ii << std::endl;
    // Update u sequence
    sejong::Vector x_new = x;
    for(int i = 0; i < N_horizon-1; i++){
      u_new_sequence[i] = u_sequence[i] + k_vec[i] + K_vec[i]*(x_new - x_sequence[i]);
      x_new = _x_tp1(x_new, u_new_sequence[i]);
//      std::cout << "  U[" << i << "]:" << std::endl << u_new_sequence[i] << std::endl;    
    }    
    _initialize_x_sequence(x, u_new_sequence, x_new_sequence);


//    std::cout << "  Old Sequence cost:" << _J_cost(x_sequence, u_sequence) << std::endl; // Cost of old Sequence
//    std::cout << "  New Sequence cost:" << _J_cost(x_new_sequence, u_new_sequence) << std::endl; // Cost of new Sequence
//    sejong::Vect3 ee_pos;
//      internal_model->getPosition(x_new_sequence.back().head(NUM_Q), SJLinkID::LK_EE, ee_pos);
//    std::cout << "  (X,Y) = (" << ee_pos[0] << "," << ee_pos[1] << ")" << std::endl; // Position


    x_sequence = x_new_sequence;
    u_sequence = u_new_sequence;
    x = x_sequence[0];
  }




}




void DDP_ctrl::_mpc_ctrl(sejong::Vector & gamma){
  gamma.setZero();  
  // Get starting state   // x = [q, qdot]
  sejong::Vector x_state_start(NUM_Q + NUM_QDOT);
  x_state_start.head(NUM_Q) = sp_->Q_; // Store current Q position to state
  x_state_start.tail(NUM_QDOT) = sp_->Qdot_; // Store current Qdot position

  // ---- START TIMER 
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  // -------------------------------------------------------------------------------------------
  // Computation Block
  _compute_ilqr();

  std::cout << "Hello?" << std::endl;  

  // End Computation Block
  // --------------------------------------------------------------------------------------------

  // ---- END TIMER
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
  time_sum += (time_span1.count()*1000.0);

//  std::cout << "Loop took " << time_span1.count()*1000.0 << "ms"<<std::endl;
  ++count;
  if (count % 100 == 99){
    double ave_time = time_sum/((double)count);
    std::cout << "MPC Loop took " << ave_time << "ms."<<std::endl;
    count = 0;
    time_sum = 0.;
  }
  // ---- END TIMER

  // Test WBC command given u_sequence[0]
  x_sequence[0] = x_state_start;
  _getWBC_command(x_sequence[0], u_sequence[0], gamma);

  // Test cost function
  double cost = _l_cost(x_sequence[0], u_sequence[0]);

  // Test simulate single step
  // sejong::Vector x_next_state(NUM_Q + NUM_QDOT);
  // _internal_simulate_single_step(x_sequence[0], gamma, x_next_state);

}


void DDP_ctrl::_zero_ctrl(sejong::Vector & gamma){
  gamma.setZero();
}

void DDP_ctrl::_jpos_ctrl(sejong::Vector & gamma){
 
  double kp(100.);
  double kd(10.);
  sejong::Vector qddot =
    kp*(jpos_ini_ - sp_->Q_.tail(NUM_ACT_JOINT)) +
    kd * ( - sp_->Qdot_.tail(NUM_ACT_JOINT));

  gamma = A_ * qddot + coriolis_ + grav_;
}

void DDP_ctrl::_ee_ctrl(sejong::Vector & gamma){

  sejong::Vect3 ee_des, ee_vel_des, ee_acc;

  ee_des = ee_ini_;
  ee_vel_des.setZero();
  ee_acc.setZero();
  // Vertical Up & Down
  double amp(0.1);
  double omega(2.*M_PI * 1.);
  ee_des[0] += amp * sin(omega * state_machine_time_);
  ee_vel_des[0] = amp * omega * cos(omega * state_machine_time_);
  ee_acc[0] = -amp * omega * omega* sin(omega * state_machine_time_);

  sejong::Vect3 ee_pos;
  sejong::Vect3 ee_vel;

  robot_model_->getPosition(sp_->Q_, SJLinkID::LK_EE, ee_pos);
  robot_model_->getVelocity(sp_->Q_, sp_->Qdot_, SJLinkID::LK_EE, ee_vel);

  // Commanded acceleration
  double kp(10.);
  double kd(1.0);
  sejong::Vector xddot(2);
  for(int i(0); i<2; ++i){ // (x, z)
    xddot[i] = ee_acc[i] + kp*(ee_des[i] - ee_pos[i]) + kd * (ee_vel_des[i] - ee_vel[i]);
  }

  sejong::Vector jpos_cmd = kp * (jpos_ini_ - sp_->Q_ ) + kd * (-sp_->Qdot_);

  // Jacobian
  sejong::Matrix Jee, Jee_inv;
  sejong::Matrix Jtmp;
  robot_model_->getFullJacobian(sp_->Q_, SJLinkID::LK_EE, Jtmp);
  Jee = Jtmp.block(0,0, 2, NUM_QDOT);
  _DynConsistent_Inverse(Jee, Jee_inv);

  sejong::Matrix Jee_dot;
  robot_model_->getFullJacobianDot(sp_->Q_, sp_->Qdot_, SJLinkID::LK_EE, Jtmp);
  Jee_dot = Jtmp.block(0,0, 2, NUM_QDOT);

  // Joint task
  sejong::Matrix Nee = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - Jee_inv * Jee;
  sejong::Matrix Jqee_inv;
  _DynConsistent_Inverse(Nee, Jqee_inv);

  // Torque command
  sejong::Vector qddot = Jee_inv * (xddot - Jee_dot * sp_->Qdot_);
  qddot = qddot + Jqee_inv * (jpos_cmd - qddot);
  gamma = A_ * qddot + coriolis_ + grav_;
}

