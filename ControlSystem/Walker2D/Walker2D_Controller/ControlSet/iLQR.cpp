#include "iLQR.hpp"

iLQR::iLQR(int STATE_SIZE_in, int DIM_u_in, int N_horizon_in){
  // Set Parameters
  STATE_SIZE = STATE_SIZE_in;
  DIM_u = DIM_u_in;
  N_horizon = N_horizon_in;

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

void iLQR::compute_ilqr(){
	sejong::Vector x_vec_empty(STATE_SIZE);
	sejong::Vector u_vec_empty(DIM_u);
	x_vec_empty.setOnes();
	u_vec_empty.setOnes();
	l_cost(x_vec_empty, u_vec_empty);
	l_cost_final(x_vec_empty);
}

double iLQR::_J_cost(const std::vector<sejong::Vector> & X, const std::vector<sejong::Vector> & U){
  double J_cost = 0.0;
  for(size_t i = 0; i < N_horizon-1; i++){
    J_cost += l_cost(X[i], U[i]);
  }
  J_cost += l_cost_final(X[N_horizon-1]); 
  return J_cost;
}