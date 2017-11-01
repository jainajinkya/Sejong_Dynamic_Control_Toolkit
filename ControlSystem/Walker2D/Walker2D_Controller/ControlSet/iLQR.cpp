#include "iLQR.hpp"
#include <chrono>

iLQR::iLQR(int STATE_SIZE_in, int DIM_u_in, int N_horizon_in, int ilqr_iters_in){
  // Set Parameters
  STATE_SIZE = STATE_SIZE_in;
  DIM_u = DIM_u_in;
  N_horizon = N_horizon_in;
  ilqr_iters = ilqr_iters_in;

  // Initialize Values
  _initialize_X_U();
  _initialize_U_sequence(u_sequence);
  _initialize_gradients_hessians();

  for(size_t i = 0; i < alpha_cand_pow.size(); i++){
    alpha_cand.push_back(pow(10.0, alpha_cand_pow[i]));
  }
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

void iLQR::_initialize_X_U(){
  x_sequence = std::vector<sejong::Vector>(N_horizon);
  u_sequence = std::vector<sejong::Vector>(N_horizon - 1); 
}

void iLQR::_initialize_U_sequence(std::vector<sejong::Vector> & U){
  // Near zero initialization
  for(size_t i = 0; i < U.size(); i++){
    sejong::Vector u_vec(DIM_u); // task acceleration vector
    for (size_t j = 0; j < DIM_u; j++){
      u_vec[j] = 0.0; // this can be random
    }
    std::cout << i << std::endl;
    sejong::pretty_print(u_vec, std::cout, "u_vec");
    U[i] = u_vec;
  }
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

void  iLQR::_compute_X_sequence(const sejong::Vector & x_state_start,  
                                const std::vector<sejong::Vector> & U, 
                                   std::vector<sejong::Vector> & X){
  X[0] = x_state_start;
  for(size_t i = 1; i < N_horizon; i++){
    X[i] = f(X[i-1], U[i-1]);
  }
}

void iLQR::compute_ilqr(const sejong::Vector & x_state_start){
  std::cout << "[iLQR] Computing iLQR" << std::endl;

  bool compute_new_traj = true;

  _compute_X_sequence(x_state_start, u_sequence, x_sequence);
  _compute_finite_differences();

/*	sejong::Vector x_vec_empty(STATE_SIZE);
	sejong::Vector u_vec_empty(DIM_u);
	x_vec_empty.setOnes();
	u_vec_empty.setOnes();
	l_cost(x_vec_empty, u_vec_empty);
	l_cost_final(x_vec_empty);
*/
}

double iLQR::_J_cost(const std::vector<sejong::Vector> & X, const std::vector<sejong::Vector> & U){
  double J_cost = 0.0;
  for(size_t i = 0; i < N_horizon-1; i++){
    J_cost += l_cost(X[i], U[i]);
  }
  J_cost += l_cost_final(X[N_horizon-1]); 
  return J_cost;
}

void iLQR::_compute_finite_differences(){
  sejong::Vector h_step(STATE_SIZE);  // Same size as state x
  sejong::Vector h2_step(STATE_SIZE); // Same size as state x  
  sejong::Vector k_step(DIM_u);       // Same size as input u
  sejong::Vector k2_step(DIM_u);      // Same size as input u  
  h_step.setZero();
  h2_step.setZero();  
  k_step.setZero();
  k2_step.setZero();

  //------------------------------------------------------------------------------------------------
  sejong::Vector x = x_sequence[N_horizon-1];
  // Compute finite difference at the end

  if (custom_l_xF){  l_x_final_analytical(x, l_x[N_horizon-1]);   }
  if (custom_l_xxF){ l_xx_final_analytical(x, l_xx[N_horizon-1]); }  

  for (size_t i = 0; i < h_step.size(); i++){
    h_step[i] = finite_epsilon;
    if (!custom_l_xF){
      // l_xF
      l_x[N_horizon-1](i) = (l_cost_final(x + h_step) - l_cost_final(x - h_step) ) / (2.0*h_step[i]); 
      std::cout << "Calculating l_xF" << std::endl;
    }

    if(!custom_l_xxF){
      // l_xxF
      for (size_t j = 0; j < h_step.size(); j++){
        h2_step[j] = finite_epsilon;
        l_xx[N_horizon-1](i,j) = (l_cost_final(x + h_step + h2_step) - 
                       l_cost_final(x + h_step - h2_step) - 
                       l_cost_final(x - h_step + h2_step) + 
                       l_cost_final(x - h_step - h2_step))/(4*h_step[i]*h2_step[j]);
      std::cout << "Calculating l_xxF" << std::endl;
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
    sejong::Vector x = x_sequence[n];
    sejong::Vector u = u_sequence[n];    

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