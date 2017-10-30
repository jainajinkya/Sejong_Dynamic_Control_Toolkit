#include "iLQR.hpp"

WBC_iLQR::WBC_iLQR(){
  _initialize_gradients_hessians();
  printf("[WBC_iLQR] Constructor Initialized\n");
}

WBC_iLQR::WBC_iLQR(int DIM_WBC_TASKS_in){
  DIM_WBC_TASKS = DIM_WBC_TASKS_in;
  printf("[WBC_iLQR] Constructor Initialized\n");
}

WBC_iLQR::~WBC_iLQR(){
}

void WBC_iLQR::set_DIM_WBC_TASKS(int DIM_WBC_TASKS_in){
	DIM_WBC_TASKS = DIM_WBC_TASKS_in;
}


void WBC_iLQR::_initialize_gradients_hessians(){
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

}

void WBC_iLQR::compute_ilqr(){
	sejong::Vector x_vec_empty(10);
	sejong::Vector u_vec_empty(10);
	sejong::Vector gamma(5);	
	x_vec_empty.setOnes();
	u_vec_empty.setOnes();
	gamma.setOnes();	
	l_cost(x_vec_empty, u_vec_empty);
	l_cost_final(x_vec_empty);

	sejong::pretty_print(gamma, std::cout, "gamma before wbc");
	get_WBC_command(x_vec_empty, u_vec_empty, gamma);
	sejong::pretty_print(gamma, std::cout, "gamma after wbc");	
}

double WBC_iLQR::_J_cost(const std::vector<sejong::Vector> & X, const std::vector<sejong::Vector> & U){
  double J_cost = 0.0;
  for(size_t i = 0; i < N_horizon-1; i++){
    J_cost += l_cost(X[i], U[i]);
  }
  J_cost += l_cost_final(X[N_horizon-1]); 
  return J_cost;
}