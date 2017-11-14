#include "iLQR.hpp"
#include <chrono>

//#define ITER_OUTPUT_PRINT


iLQR::iLQR(int STATE_SIZE_in, int DIM_u_in, int N_horizon_in, int ilqr_iters_in){
  // Set Parameters
  STATE_SIZE = STATE_SIZE_in;
  DIM_u = DIM_u_in;
  N_horizon = N_horizon_in;
  ilqr_iters = ilqr_iters_in;

  // Initialize Values
  _initialize_U_sequence(U_seq);
  _initialize_gradients_hessians();

  for (size_t i = 0; i < alpha_cand_pow.size(); i++){
    alpha_cand.push_back(pow(10.0, alpha_cand_pow[i]));
  }

/*  alpha_cand.push_back(1); // Start at 1.0
  double cand_start = -0.0001;
  double incr = 2;  
  while (cand_start > - 4.0){
    alpha_cand.push_back(pow(10.0, cand_start));
    cand_start *= incr;
  }*/
  //--------------------

  printf("[iLQR] Constructor Initialized\n");
  std::cout << "[iLQR] Parameters ----" << std::endl;
  std::cout << "         STATE_SIZE :" << STATE_SIZE << std::endl;    
  std::cout << "         DIM_u      :" << DIM_u << std::endl;    
  std::cout << "         N_horizon  :" << N_horizon << std::endl;
  std::cout << "         ilqr_iters :" << ilqr_iters << std::endl;
}

iLQR::~iLQR(){
}

void iLQR::set_DIM_x(int DIM_x_in){
  STATE_SIZE = DIM_x_in;
  printf("[iLQR] |x| = %i \n", STATE_SIZE);
  _initialize_gradients_hessians();  
}

void iLQR::set_DIM_u(int DIM_u_in){
	DIM_u = DIM_u_in;
  printf("[iLQR] |u| = %i \n", DIM_u);
  _initialize_gradients_hessians();  
}

void iLQR::set_N_horizon(int N_horizon_in){
  N_horizon = N_horizon_in;
  printf("[iLQR] |N_horizon| = %i \n", N_horizon);  
  _initialize_gradients_hessians();  
}

void iLQR::_initialize_U_sequence(std::vector<sejong::Vector> & U){
  std::vector<sejong::Vector> U_tmp;
  // Near zero initialization
  for(size_t i = 0; i < N_horizon-1; i++){
    sejong::Vector u_vec(DIM_u); // task acceleration vector
    for (size_t j = 0; j < DIM_u; j++){
      u_vec[j] = 0.0; // this can be random
    }
    U_tmp.push_back(u_vec);
  }
  U = U_tmp;
}


void iLQR::_initialize_gradients_hessians(){
  printf("[iLQR] Initializing Gradients and Hessians \n");  
  for (size_t i = 0; i < N_horizon; i++){
    sejong::Vector n_l_x(STATE_SIZE);
    sejong::Matrix n_l_xx(STATE_SIZE, STATE_SIZE);
    sejong::Matrix n_l_xu(STATE_SIZE, DIM_u);  
    sejong::Vector n_l_u(DIM_u);
    sejong::Matrix n_l_uu(DIM_u, DIM_u);    
    sejong::Matrix n_l_ux(DIM_u, STATE_SIZE);      
    sejong::Matrix n_f_x(STATE_SIZE, STATE_SIZE);      
    sejong::Matrix n_f_u(STATE_SIZE, DIM_u);

    sejong::Vector n_k_vec(DIM_u);
    sejong::Matrix n_K_vec(DIM_u, STATE_SIZE);    
    
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
      sejong::Matrix n_f_kxu(STATE_SIZE, DIM_u);
      sejong::Matrix n_f_kux(DIM_u, STATE_SIZE);      

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

}

void iLQR::_initialize_empty_X_sequence(std::vector<sejong::Vector> & X_seq_in){
  std::vector<sejong::Vector> X_tmp;
  for(size_t i = 0; i < N_horizon; i++){
    sejong::Vector zero_x(STATE_SIZE);
    zero_x.setZero();
    X_tmp.push_back(zero_x);
  }
  X_seq_in = X_tmp;
}

void  iLQR::_compute_X_sequence(const sejong::Vector & x_state_start,  
                                const std::vector<sejong::Vector> & U, 
                                   std::vector<sejong::Vector> & X){
  X[0] = x_state_start;
  for(size_t i = 1; i < N_horizon; i++){
    X[i] = f(X[i-1], U[i-1]);
  }
}

double iLQR::_J_cost(const std::vector<sejong::Vector> & X_seq_in, const std::vector<sejong::Vector> & U_seq_in){
  double J_cost = 0.0;
  for(size_t i = 0; i < N_horizon-1; i++){
    J_cost += l_cost(X_seq_in[i], U_seq_in[i]);
    //sejong::pretty_print(X[i], std::cout, "X[i]");
    //sejong::pretty_print(U[i], std::cout, "U[i]");    
  }
  J_cost += l_cost_final(X_seq_in[N_horizon-1]); 
  return J_cost;
}



void iLQR::compute_ilqr(const sejong::Vector & x_state_start,
                        sejong::Vector & u_out){
  // Initialize U_seq and prepare X_seq for Initialization
  sejong::Vector x_start = x_state_start; 
  std::vector<sejong::Vector> X_seq;
  _initialize_empty_X_sequence(X_seq);

  double old_Jcost = 0.0;
  double new_Jcost = 0.0;
  bool flag_change = true;

  // Initialize X_sequence given the current u_sequence
  _compute_X_sequence(x_start, U_seq, X_seq); 
  for(size_t ii = 0; ii < ilqr_iters; ii++){
    // Step 1: Compute Derivatives and cost along new traj =========================================
    if (flag_change){
      _compute_finite_differences(X_seq, U_seq);
      old_Jcost = _J_cost(X_seq, U_seq);
      flag_change = false;
    }

    // Step 2: Backwards Pass ======================================================================
    // Initialize Value Function with its terminal cost and derivatives 
    double V = l_cost_final(X_seq.back());
    sejong::Vector V_x = l_x.back(); 
    sejong::Matrix V_xx = l_xx.back();
    sejong::Vector dV(2); dV.setZero();
    double dcost = 0.0;
    double expected = 0.0;

    bool cholesky_failure = false;

    bool backward_pass_done = false;
    while (!backward_pass_done && !cholesky_failure){
      // Work backwards to solve for V, Q, k, and K
      for(int i = N_horizon-2; i >= 0; i--){
        sejong::Vector Q_x = l_x[i] + f_x[i].transpose()*V_x;
        sejong::Vector Q_u = l_u[i] + f_u[i].transpose()*V_x;
        sejong::Matrix Q_xx = l_xx[i] + f_x[i].transpose()*V_xx*f_x[i];
        sejong::Matrix Q_ux = l_ux[i] + f_u[i].transpose()* V_xx *f_x[i];
        sejong::Matrix Q_uu = l_uu[i] + f_u[i].transpose()* V_xx *f_u[i];


/*        sejong::Matrix Q_ux_reg = l_ux[i] + 
                                  f_u[i].transpose()* (V_xx + lambda *sejong::Matrix::Identity(STATE_SIZE, STATE_SIZE) ) *f_x[i];
        sejong::Matrix Q_uu_reg = l_uu[i] + 
                                  f_u[i].transpose()* (V_xx + lambda *sejong::Matrix::Identity(STATE_SIZE, STATE_SIZE) ) *f_u[i] +
                                  lambda*sejong::Matrix::Identity(DIM_u, DIM_u);*/
        
        sejong::Matrix Q_ux_reg = Q_ux; //l_ux[i] + f_u[i].transpose()* (V_xx + lambda *sejong::Matrix::Identity(STATE_SIZE, STATE_SIZE) ) *f_x[i];
        sejong::Matrix Q_uu_reg = Q_uu + lambda*sejong::Matrix::Identity(DIM_u, DIM_u);


        // Compute Cholesky Decomposition
        Eigen::LLT<sejong::Matrix> lltOfQuu(Q_uu_reg);
        sejong::Matrix L = lltOfQuu.matrixL();
        sejong::Matrix Q_uu_reg_inv = (L.inverse()).transpose()*(L.inverse());

        // If Cholesky Diverges perform scheduling on lambda
        if (isnan(Q_uu_reg_inv(0,0))){
          dlambda = std::max(dlambda * lambda_factor, lambda_factor);
          lambda  = std::max(lambda_min, dlambda * lambda_factor);
          //std::cout << "  Cholesky decomposition failed. Increasing Lambda" << std::endl;
          if (lambda > lambda_max){

            //std::cout << " ii" << ii << " Lambda cannot be increased anymore. iLQR cannot find a better solution" << std::endl;
            cholesky_failure = true;
            break;
          }
          continue; // keep trying for a successful Cholesky Decomposition
        }

        // Get k, K
        k_vec[i] = -Q_uu_reg_inv*Q_u;
        K_vec[i] = -Q_uu_reg_inv*Q_ux_reg;  

        dV[0] = dV[0] + k_vec[i].transpose()*Q_u;
        dV[1] = dV[1] + 0.5 * k_vec[i].transpose()*Q_uu*k_vec[i];      
        V_x = Q_x + K_vec[i].transpose()*Q_uu*k_vec[i] + K_vec[i].transpose()*Q_u + Q_ux.transpose()*k_vec[i];
        V_xx = Q_xx + K_vec[i].transpose()*Q_uu*K_vec[i] + K_vec[i].transpose()*Q_ux + Q_ux.transpose()*K_vec[i];      
        V_xx = 0.5*(V_xx + V_xx.transpose().eval());

        backward_pass_done = true;
      }
    }

    // Calculate max gradient of k
    sejong::Matrix g_vec_Mat(DIM_u, N_horizon-1);
    for(size_t i = 0; i < DIM_u; i++){
      for(size_t n = 0; n < N_horizon-1; n++){     
        g_vec_Mat(i, n) = std::abs(k_vec[n][i]) / (std::abs(U_seq[n][i]) + 1.0);
      }
    }
    sejong::Vector g_vec = g_vec_Mat.rowwise().maxCoeff();

    // Check for termination condition due to a small gradient
    // calculate gradient norm
    double g_norm = g_vec.sum();
    if ((g_norm < tolGrad) && (lambda < 1e-5)){
      dlambda = std::min(dlambda/lambda_factor, 1.0/lambda_factor);
      if (lambda > lambda_min){
        lambda = lambda*dlambda;      
      }else{
        lambda = 0.0;
      }
      //std::cout << "Success! Gradient Norm < tolGrad" << std::endl;
      break; // Exit iLQR loop
    }

   // Step 3: Line Search =====================================================================
   bool forward_pass_done = false;

   std::vector<sejong::Vector> X_seq_tmp;
   std::vector<sejong::Vector> U_seq_tmp;
   _initialize_empty_X_sequence(X_seq_tmp);
   _initialize_U_sequence(U_seq_tmp);

   if (backward_pass_done){    
     for(size_t j = 0; j < alpha_cand.size(); j++){
        double alpha = alpha_cand[j];

        // Compute Forward Pass ----------------------------
        sejong::Vector x_i = X_seq[0]; // Starting Condition
        X_seq_tmp[0] = x_i;
        for (size_t i = 0; i < N_horizon-1; i++){ 
          //sejong::pretty_print(x_i, std::cout, "x_i");  
          sejong::Vector u_i = U_seq[i] + alpha*k_vec[i] + K_vec[i]*(x_i - X_seq[i]);

          // Box u
          double max_task_acceleration = 100;
          for(size_t k = 0; k < DIM_u; k++){
            if (u_i[k] >= max_task_acceleration){
              u_i[k] = max_task_acceleration;
            }else if (u_i[k] <= -max_task_acceleration){
              u_i[k] = -max_task_acceleration;
            }
          }

          //sejong::pretty_print(u_i, std::cout, "u_i");              
          x_i = f(x_i, u_i); // x_{i+1}  

          if(isnan(x_i[0])){
            std::cout << "x_i became nan at forward pass" << std::endl;
            sejong::pretty_print(k_vec[i], std::cout, "k_vec[i]");  
            sejong::pretty_print(X_seq[i], std::cout, "X_seq[i]");                 
            sejong::pretty_print(K_vec[i], std::cout, "K_vec[i]");              
            sejong::pretty_print(U_seq[i], std::cout, "U_seq[i]");
            exit(1);
          }

          // Store to temporary X_seq_tmp and U_seq_tmp;
          X_seq_tmp[i+1] = x_i;
          U_seq_tmp[i] = u_i;
        }

        new_Jcost = _J_cost(X_seq_tmp, U_seq_tmp);
        dcost = old_Jcost - new_Jcost;
        expected = -alpha*(dV[0] + alpha*dV[1]);

        double z = - 1;

/*        std::cout << "  alpha:" << alpha << std::endl;
        std::cout << "  Old Cost: " << old_Jcost << "New Cost: " << new_Jcost << std::endl;
        std::cout << "  expected: " << expected << std::endl;        
        std::cout << "  dcost: " << dcost << std::endl;        
        std::cout << " " << std::endl;*/

        if (expected > 0){
          z = dcost / expected;        
        }else{
          //std::cout << "Non-Positive Expected Reduction. Should not occur" << std::endl;
        }
        if (z > z_min){
          forward_pass_done = true;
          break;
        }

/*        for (size_t i = 0; i < N_horizon; i++){ 
          std::cout << "i" << i << std::endl;
          sejong::pretty_print(X_seq[i], std::cout, "X_seq[i]");           
          sejong::pretty_print(X_seq_tmp[i], std::cout, "X_seq_tmp[i]"); 
        }
        for (size_t i = 0; i < N_horizon-1; i++){ 
          std::cout << "i" << i << std::endl;
          sejong::pretty_print(U_seq[i], std::cout, "U_seq[i]");           
          sejong::pretty_print(U_seq_tmp[i], std::cout, "U_seq_tmp[i]"); 
        }       */ 

      }
    } // Line Search Done  

    // Step 4: Accept or Discard Changes      
//    std::cout << "iteration, cost, reduction, expected, gradient, log10(lambda)" << std::endl;
    if (forward_pass_done){
      #ifdef ITER_OUTPUT_PRINT
        std::cout << "iteration: " << ii << "  cost:" << old_Jcost << "  reduction:" << dcost << "  expected" << expected << "  gradient:" << g_norm << "  log10(lambda)" << log10(lambda) << std::endl;     
      #endif

      // Decrease Lambda
      dlambda = std::min(dlambda/lambda_factor, 1.0/lambda_factor );
      if (lambda > lambda_min){
        lambda = lambda*dlambda;      
      }else{
        lambda = 0.0;
      }

      // Accept Changes
      U_seq = U_seq_tmp;
      X_seq = X_seq_tmp;      
      flag_change = 1.0;

      #ifdef ITER_OUTPUT_PRINT
        sejong::pretty_print(U_seq[0], std::cout, " U_seq[0]");
        sejong::pretty_print(X_seq.back(), std::cout, " xF");
      #endif

      if (dcost < tolFun){
        //std::cout << "Success! cost change < tolFun" << std::endl;
        break;
      }

    }else{ // No Cost Improvement
      // Increase Lambda
      dlambda = std::max(dlambda * lambda_factor, lambda_factor);
      lambda  = std::max(lambda_min, dlambda * lambda_factor);

      #ifdef ITER_OUTPUT_PRINT
        std::cout << "iteration: " << ii << "  NO_STEP " << "  reduction: " << dcost << "  expected: " << expected << "  gradient:" << g_norm << "  log10(lambda)" << log10(lambda) << std::endl;  
        sejong::pretty_print(U_seq[0], std::cout, "U_seq[0]");
        sejong::pretty_print(X_seq.back(), std::cout, " xF");
      #endif

      if (lambda > lambda_max){
        //std::cout << "EXIT: lambda > lambda_max" << std::endl;
        break;
      }
    } // Accept or Discard Changes


/*    if (ii > 2){
      exit(1);
    }*/
  }


  u_out = U_seq[0];



}












void iLQR::_compute_finite_differences(const std::vector<sejong::Vector> & X_seq_in, 
                                       const std::vector<sejong::Vector> & U_seq_in){
  sejong::Vector h_step(STATE_SIZE);  // Same size as state x
  sejong::Vector h2_step(STATE_SIZE); // Same size as state x  
  sejong::Vector k_step(DIM_u);       // Same size as input u
  sejong::Vector k2_step(DIM_u);      // Same size as input u  
  h_step.setZero();
  h2_step.setZero();  
  k_step.setZero();
  k2_step.setZero();

  //------------------------------------------------------------------------------------------------
  sejong::Vector x = X_seq_in[N_horizon-1];
  // Compute finite difference at the end

  if (custom_l_xF){  l_x_final_analytical(x, l_x[N_horizon-1]);   }
  if (custom_l_xxF){ l_xx_final_analytical(x, l_xx[N_horizon-1]); }  

  for (size_t i = 0; i < h_step.size(); i++){
    h_step[i] = finite_epsilon;
    if (!custom_l_xF){
      // l_xF
      l_x[N_horizon-1](i) = (l_cost_final(x + h_step) - l_cost_final(x - h_step) ) / (2.0*h_step[i]); 
      //std::cout << "Calculating l_xF" << std::endl;
    }

    if(!custom_l_xxF){
      // l_xxF
      for (size_t j = 0; j < h_step.size(); j++){
        h2_step[j] = finite_epsilon;
        l_xx[N_horizon-1](i,j) = (l_cost_final(x + h_step + h2_step) - 
                       l_cost_final(x + h_step - h2_step) - 
                       l_cost_final(x - h_step + h2_step) + 
                       l_cost_final(x - h_step - h2_step))/(4*h_step[i]*h2_step[j]);
      //std::cout << "Calculating l_xxF" << std::endl;
        h2_step.setZero();        
      }
    }else{
      break;
    }
    h_step.setZero();
  }
  //------------------------------------------------------------------------------------------------


  //------------------------------------------------------------------------------------------------
  // Compute finite difference of the sequence
  for (size_t n = 0; n < N_horizon - 1; n++){
    sejong::Vector x = X_seq_in[n];
    sejong::Vector u = U_seq_in[n];    

    if (custom_l_x)  { l_x_analytical(x, u,  l_x[n]);  }//std::cout << "computing l_x_analytical" << std::endl;   }
    if (custom_l_xx) { l_xx_analytical(x, u, l_xx[n]); }//std::cout << "computing l_xx_analytical" << std::endl;   }  
    if (custom_l_u)  { l_u_analytical(x, u,  l_u[n]);  }//std::cout << "computing l_u_analytical" << std::endl;   }
    if (custom_l_uu) { l_uu_analytical(x, u, l_uu[n]); }//std::cout << "computing l_uu_analytical" << std::endl;   }     
    if (custom_l_ux) { l_ux_analytical(x, u, l_ux[n]); }//std::cout << "computing l_ux_analytical" << std::endl;   }
    if (custom_f_u)  {  f_u_analytical(x, u,  f_u[n]); }//std::cout << "computing f_u_analytical" << std::endl;   }          
    if (custom_f_x)  {  f_x_analytical(x, u,  f_x[n]); }//std::cout << "computing f_x_analytical" << std::endl;   }              

    // Skip Finite Difference if l_x, l_xx, and l_xu or l_ux have analytical forms
    if (!(custom_l_x && custom_l_xx && (custom_l_xu || custom_l_ux))){
      // -----------------------------------
      // Calculate Finite Difference for l_x, l_xx, and l_xu 
      for (size_t i = 0; i < h_step.size(); i++){
        h_step[i] = finite_epsilon;
        // l_x
        if (!custom_l_x){ // Compute finite diff if analytical form is not available
         l_x[n](i) = (l_cost(x + h_step, u) - l_cost(x - h_step, u) ) / (2.0*h_step[i]); 
         //std::cout << "finite l_x" << std::endl;
        }

        if (!custom_l_xx){ // Compute finite diff if analytical form is not available
          // l_xx
          for (size_t j = 0; j < h_step.size(); j++){
            h2_step[j] = finite_epsilon;
            l_xx[n](i,j) = (l_cost(x + h_step + h2_step, u) - 
                           l_cost(x + h_step - h2_step, u) - 
                           l_cost(x - h_step + h2_step, u) + 
                           l_cost(x - h_step - h2_step, u))/(4*h_step[i]*h2_step[j]);         
            h2_step.setZero();        
          }
         //std::cout << "finite l_xx" << std::endl;
        }

        if (! (custom_l_xu || custom_l_ux) ){ // Compute finite diff if analytical form is not available   
          // l_xu
          for (size_t j = 0; j < k_step.size(); j++){
            k_step[j] = finite_epsilon;        
            l_xu[n](i,j) = (l_cost(x + h_step, u + k_step) - 
                         l_cost(x + h_step, u - k_step) - 
                         l_cost(x - h_step, u + k_step) + 
                         l_cost(x - h_step, u - k_step))/(4*h_step[i]*k_step[j]);
            k_step.setZero();        
          }
          //std::cout << "finite l_xu" << std::endl;
        } //END IF      

        h_step.setZero();
        //std::cout << "    i:" << i << " l_x[i] = "<< l_x[i] << std::endl;
      }
    }// END IF

    // Skip Finite Difference if l_x, l_xx, and l_xu or l_ux have analytical forms
    if (!(custom_l_u && custom_l_uu && (custom_l_xu || custom_l_ux))){
      // -----------------------------------
      // Calculate Finite Difference for l_u, l_uu, and l_ux 
      for (size_t i = 0; i < k_step.size(); i++){
        k_step[i] = finite_epsilon;    
        if (!custom_l_u){ // Compute finite diff if analytical form is not available        
          // Calculate l_u
          l_u[n](i) = (l_cost(x, u + k_step) - l_cost(x, u - k_step) ) / (2.0*k_step[i]);
          //std::cout << "finite l_u" << std::endl;
        }
  
        if (!custom_l_uu){ // Compute finite diff if analytical form is not available        
          // Calculate l_uu
          for (size_t j = 0; j < k2_step.size(); j++){
            k2_step[j] = finite_epsilon;
            l_uu[n](i,j) = (l_cost(x, u + k_step + k2_step) -
                      l_cost(x, u + k_step - k2_step) -
                      l_cost(x, u - k_step + k2_step) +
                      l_cost(x, u - k_step - k2_step)) / (4*k_step[i]*k2_step[j]);
            k2_step.setZero();
          }
          //std::cout << "finite l_uu" << std::endl;
        }

        if (! (custom_l_xu || custom_l_ux) ){ // Compute finite diff if analytical form is not available   
          // Calculate l_ux
          for (size_t j = 0; j < h2_step.size(); j++){
            h2_step[j] = finite_epsilon;
            l_ux[n](i,j) = (l_cost(x + h2_step, u + k_step) -
                      l_cost(x - h2_step, u + k_step) -
                      l_cost(x + h2_step, u - k_step) +
                      l_cost(x - h2_step, u - k_step)) / (4*k_step[i]*h2_step[j]);
            h2_step.setZero();
          }
          //std::cout << "finite l_ux" << std::endl;
        } // END IF

        k_step.setZero();
      }
    }// END IF

    h_step.setZero();

    if(!custom_f_x){
      // Calculate f_x
      for (size_t i = 0; i < STATE_SIZE; i++){
        h_step[i] = finite_epsilon;
        if(!custom_f_x){ // Compute finite diff if analytical form is not available   
          f_x[n].col(i) = (f(x + h_step, u) - f(x - h_step, u))/ (2.0*h_step[i]);
          //std::cout << "finite f_x" << std::endl;
        } // END IF

  // Skip Hessian calculation of f-------------------------------------------------------------------
  /*  // Calculate f_xx
        for (size_t j = 0; j < STATE_SIZE; j++){
          sejong::Vector f_kxx(STATE_SIZE); // partial derivative of f vector with respect to x_i,x_j
          f_kxx.setZero();
          h2_step[j] = finite_epsilon;
          f_kxx = (f(x + h_step + h2_step, u) -
                   f(x + h_step - h2_step, u) -
                   f(x - h_step + h2_step, u) +
                   f(x - h_step - h2_step, u)) / (4*h_step[i]*h2_step[j]);

          for (size_t k = 0; k < STATE_SIZE; k++){
            H_f_xx[n][k](i,j) = f_kxx[k]; // Store k element of f_kxx at k-th Hesisan's (i,j) element.
          }
          h2_step.setZero();
        }

        // Calculate f_xu
        for (size_t j = 0; j < DIM_u; j++){
          sejong::Vector f_kxu(STATE_SIZE); // partial derivative of f vector with respect to x_i, u_j
          f_kxu.setZero();
          k2_step[j] = finite_epsilon;

          f_kxu = (f(x + h_step, u + k2_step) -
                   f(x + h_step, u - k2_step) -
                   f(x - h_step, u + k2_step) +
                   f(x - h_step, u - k2_step)) / (4*h_step[i]*k2_step[j]);

          for (size_t k = 0; k < STATE_SIZE; k++){
            H_f_xu[n][k](i,j) = f_kxu[k]; // Store k element of f_kxu at k-th Hesisan's (i,j) element.
          }

          k2_step.setZero();
        }*/
  // Skip Hessian calculation of f-------------------------------------------------------------------

        h_step.setZero();
      }
    } // END IF



    if(!custom_f_u){
      // Calculate f_u
      for (size_t i = 0; i < DIM_u; i++){
        k_step[i] = finite_epsilon;
        if(!custom_f_u){ // Compute finite diff if analytical form is not available   
          f_u[n].col(i) = (f(x, u + k_step) - f(x, u - k_step))/ (2.0*k_step[i]);
          //std::cout << "finite f_u" << std::endl;          
        }// END IF

  // Skip Hessian calculation of f-------------------------------------------------------------------
  /*      // Calculate f_ux
        for (size_t j = 0; j < STATE_SIZE; j++){
          sejong::Vector f_kux(STATE_SIZE); // partial derivative of f vector with respect to x_i, u_j
          f_kux.setZero();
          h2_step[j] = finite_epsilon;

          f_kux = (f(x + h2_step, u + k_step) -
                   f(x - h2_step, u + k_step) -
                   f(x + h2_step, u - k_step) +
                   f(x - h2_step, u - k_step)) / (4*k_step[i]*h2_step[j]);

          for (size_t k = 0; k < STATE_SIZE; k++){
            H_f_ux[n][k](i,j) = f_kux[k]; // Store k element of f_kux at k-th Hesisan's (i,j) element.
          }
          k2_step.setZero();
        }*/
  // Skip Hessian calculation of f-------------------------------------------------------------------

        k_step.setZero();
      }
    }// END IF


//    sejong::pretty_print(l_x[n], std::cout, "l_x");    
//    sejong::pretty_print(l_xx[n], std::cout, "l_xx");        
//    sejong::pretty_print(l_u[n], std::cout, "l_u");        
//    sejong::pretty_print(l_uu[n], std::cout, "l_uu");              
//    sejong::pretty_print(l_xu[n], std::cout, "l_xu");
//    sejong::pretty_print(l_ux[n], std::cout, "l_ux");  
//    sejong::pretty_print(f_x[n], std::cout, "n_f_x");      
//    sejong::pretty_print(f_u[n], std::cout, "n_f_u");      
//     sejong::pretty_print(H_f_xx[n][3], std::cout, "H_f_3xx");      
//     sejong::pretty_print(H_f_xu[n][3], std::cout, "H_f_3xu");          
//     sejong::pretty_print(H_f_ux[n][3], std::cout, "H_f_3ux");          
  }

//    sejong::pretty_print(l_x[0], std::cout, "l_x");    
//      sejong::pretty_print(l_xx[0], std::cout, "l_xx");        
//    sejong::pretty_print(l_u[0], std::cout, "l_u");        
//    sejong::pretty_print(l_uu[0], std::cout, "l_uu");              
//    sejong::pretty_print(l_ux[0], std::cout, "l_ux");  
//    sejong::pretty_print(f_x[0], std::cout, "n_f_x");      
//    sejong::pretty_print(f_u[0], std::cout, "n_f_u");  


//  l(x+h, u) - l(x-h, u) / 2h  
}