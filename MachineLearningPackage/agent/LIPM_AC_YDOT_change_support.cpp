#include "LIPM_AC_YDOT_change.h"
#include <Utils/utilities.h>
#include <stdio.h>

LIPM_AC_YDOT_change::LIPM_AC_YDOT_change():
  LIPM_ActorCritic_Learner()
{
  lipm_env_ = new LIPM_3D_AC_YDOT_System();

  dim_state_ = AC_DIM_STATE;
  dim_action_ = AC_DIM_ACTION;
  dim_theta_ = DIM_THETA;
  dim_theta_half_ = DIM_THETA_HALF;
  num_value_feature_ = NUM_VALUE_FEATURE;
  num_action_feature_ = NUM_ACTION_FEATURE;

  w_ = new double[NUM_VALUE_FEATURE];
  w_eligi_ = new double[NUM_VALUE_FEATURE];

  theta_ = new double*[DIM_THETA];
  theta_eligi_ = new double*[DIM_THETA];
  policy_gradient_ = new double*[DIM_THETA];

  for(int i(0);i<DIM_THETA; ++i){
    theta_[i] = new double [NUM_ACTION_FEATURE];
    theta_eligi_[i] = new double [NUM_ACTION_FEATURE];

    sejong::SetArrayZero(theta_[i], NUM_ACTION_FEATURE);
    sejong::SetArrayZero(theta_eligi_[i], NUM_ACTION_FEATURE);

    policy_gradient_[i] = new double[NUM_ACTION_FEATURE];
  }
  for(int i(0);i<NUM_VALUE_FEATURE; ++i){
    w_[i] = 0.01 * rand()/(double)RAND_MAX;
  }

  for(int k(0);k<DIM_THETA; ++k){
    for(int i(0); i<NUM_VALUE_FEATURE; ++i){
      theta_[k][i] = 0.01 * rand()/(double)RAND_MAX;
    }
  }
  theta_[0][NUM_VALUE_FEATURE-1] = 0.3;
  theta_[1][NUM_VALUE_FEATURE-1] = 0.2;
  theta_[2][NUM_VALUE_FEATURE-1] = 0.0;

  theta_[3][NUM_VALUE_FEATURE-1] = 0.0;
  theta_[4][NUM_VALUE_FEATURE-1] = 0.0;
  theta_[5][NUM_VALUE_FEATURE-1] = 0.0;

  ///// Parameters
  gamma_ = 0.9;
  alpha_ = 0.00003;
  beta_ =  0.00003;
  lambda_w_ = 0.88;
  lambda_theta_ = 0.88;

  I_ = 1.0;

  ///// action offset
  action_offset_[0] = 0.0;
  action_offset_[1] = 0.0;

  ///// set min & max
  // p_x
  action_min_[0] = 0.1;
  action_max_[0] = 0.5;
  // apex velocity
  // action_min_[1] = x_vel_min + 0.03;
  // action_max_[1] = x_vel_max - 0.03;
  action_min_[1] = X_VEL_MIN + 0.0;
  action_max_[1] = 0.37;
  // action_max_[1] = X_VEL_MAX - 0.24;

  // APEX VELOCITY
  action_min_[2] = -0.25;
  action_max_[2] = 0.25;

  ///// RBF SETTING
  double ** mean  = new double*[NUM_RBF_FEATURE];
  for(int i(0);i<NUM_RBF_FEATURE; ++i){
    mean[i] = new double[AC_DIM_STATE];
  }

  for (int k(0); k<NUM_Y_POS_GRID; ++k){
    for(int i(0); i<NUM_X_VEL_GRID; ++i){
      for(int j(0); j<NUM_Y_VEL_GRID; ++j){
        mean[NUM_X_VEL_GRID * NUM_Y_VEL_GRID*k + NUM_Y_VEL_GRID * i + j][0] = Y_POS_MIN + Y_POS_RES*k;
        mean[NUM_X_VEL_GRID * NUM_Y_VEL_GRID*k + NUM_Y_VEL_GRID * i + j][1] = X_VEL_MIN + X_VEL_RES*i;
        mean[NUM_X_VEL_GRID * NUM_Y_VEL_GRID*k + NUM_Y_VEL_GRID * i  + j][2] = Y_VEL_MIN + Y_VEL_RES*j;
      }
    }
  }
  // PRINT MEAN LIST
  // FOR(INT I(0); I<NUM_RBF_FEATURE; ++I){
  //   SEJONG::PRETTY_PRINT(MEAN[I], "MEAN LIST", AC_DIM_STATE);
  // } EXIT(0);

  double* sigma = new double[NUM_RBF_FEATURE];
  for(int i(0); i<NUM_RBF_FEATURE; ++i){
    sigma[i] = RBF_SIGMA;
  }
  // set mean & sigma
  rbf_generic_.setMeanSigma(mean, sigma);

  for(int i(0); i<NUM_RBF_FEATURE; ++i){
    delete [] mean[i];
  }
  delete [] mean;
  delete [] sigma;
}


void LIPM_AC_YDOT_change::_SetTestStateList(std::vector<sejong::Vector> & ini_state_list){
  sejong::Vector test_state(3);
  sejong::Vector normal_state(3);

  // original
  normal_state[0] = 0.056;
  normal_state[1] = 0.2;
  normal_state[2] = 0.0;
  ini_state_list.push_back(normal_state);

  int count_impulse(0);
  int count_angle(0);

  double omega = sqrt(9.81/ lipm_env_->GetHeight());

  for(int i(0); i< 21; ++i){
    // double impulse(50.); // 60 N* sec (600 N / 0.1 sec)
    // double angle = (90. - 10. * i)/180. * M_PI;


    double impulse(50.); // 50 N* sec (600 N / 0.1 sec)
    impulse -= 20 * (count_impulse%3);
    double angle = (90. - 30. * (count_angle%7))/180.*M_PI;

    if(count_impulse %3 ==0){
      ++count_angle;
    }
    ++count_impulse;

    double body_mass(136.); // 136 kg
    double dt(0.1); // 0.1 sec
    double acc = impulse / body_mass / dt;



    double acc_x = acc * cos (angle);
    double acc_y = acc * sin (angle);
    // if(acc_x< -1.){
    //   acc_x = -1.;
    // }
    printf("acc: %f, %f\n", acc_x, acc_y);
    // X
    double Ax (0.5 * (1./(omega * omega) * acc_x + 1./omega * normal_state[1]));
    double Bx (0.5 * (1./(omega * omega) * acc_x - 1./omega * normal_state[1]));
    // Y
    double Ay (0.5 * ( normal_state[0] + 1./(omega * omega) * acc_y + 1./omega * normal_state[2]));
    double By (0.5 * ( normal_state[0] + 1./(omega * omega) * acc_y - 1./omega * normal_state[2]));


    double vel_change_x =  omega* (Ax * exp(omega * dt) - Bx * exp(-omega * dt));
    double vel_change_y =  omega* (Ay * exp(omega * dt) - By * exp(-omega * dt));

    double pos_change_x = Ax * exp(omega * dt) + Bx * exp(-omega * dt) - 1./(omega * omega) * acc_x;
    double pos_change_y = Ay * exp(omega * dt) + By * exp(-omega * dt) - 1./(omega * omega) * acc_y;

    double disturbed_pos[2];
    double disturbed_vel[2];
    // Pos
    disturbed_pos[0] = pos_change_x;
    disturbed_pos[1] = pos_change_y;
    // Vel
    disturbed_vel[0] = vel_change_x;
    disturbed_vel[1] = vel_change_y;

    // printf("disturbed state: %f, %f, %f, %f\n",
    //        disturbed_pos[0], disturbed_pos[1],
    //        disturbed_vel[0], disturbed_vel[1]);

    _FindZeroCoM_X(disturbed_pos, disturbed_vel);
    // printf("disturbed state come back: %f, %f, %f, %f\n",
    //        disturbed_pos[0], disturbed_pos[1],
    //        disturbed_vel[0], disturbed_vel[1]);
    // printf("\n");

    // Check
    if(fabs(disturbed_pos[0]) > 0.000001){
      printf("disturbed x is not zero: %f, %f, %f, %f\n",
             disturbed_pos[0], disturbed_pos[1],
             disturbed_vel[0], disturbed_vel[1]);
      exit(0);
    }

    test_state[0] = disturbed_pos[1];
    test_state[1] = disturbed_vel[0];
    test_state[2] = disturbed_vel[1];
    ini_state_list.push_back(test_state);
  }

  for(int i(0); i<ini_state_list.size(); ++i){
    sejong::pretty_print(ini_state_list[i], std::cout, "ini_state");
    printf("\n");
  }
}


void LIPM_AC_YDOT_change::_Sampling_Initial_State(double * ini_state){
  int sample_type(1);

  if (sample_type == 0){
    double h = lipm_env_->GetHeight();
    double slope = sqrt(9.81/h);
    // y, xdot, ydot
    ini_state[1] = X_VEL_MIN+(X_VEL_MAX - X_VEL_MIN)*rand()/(double)RAND_MAX;
    do {
      ini_state[0] = Y_POS_MIN+(Y_POS_MAX - Y_POS_MIN)*rand()/(double)RAND_MAX;
      ini_state[2] = Y_VEL_MIN+(Y_VEL_MAX - Y_VEL_MIN)*rand()/(double)RAND_MAX;
      // sejong::pretty_print(ini_state, "initial state", AC_DIM_STATE);
    }
    // while( !((ini_state[2] < slope * ini_state[0]) && (ini_state[2] > -slope * ini_state[0])) );
    while( false );

  }
  else if (sample_type == 1){
    sejong::Vector test_state(3);
    sejong::Vector normal_state(3);

    // original
    normal_state[0] = 0.056;
    normal_state[1] = 0.2;
    normal_state[2] = 0.0;

    // static int count(0);
    // ++count;
    // int trial = count%19;
    // double impulse(50.); // 60 N* sec (600 N / 0.1 sec)
    // double body_mass(136.); // 136 kg
    // double dt(0.1); // 0.1 sec
    // double acc = impulse / body_mass / dt;
    // double omega = sqrt(9.81/ lipm_env_->GetHeight());
    // double angle = (90.0 - 10. * trial)/180. * M_PI;

    // double acc_x = acc * cos (angle);
    // double acc_y = acc * sin (angle);

    static int count_impulse(0);
    static int count_angle(0);

    double impulse(50.); // 50 N* sec (600 N / 0.1 sec)
    impulse -= 20 * (count_impulse%3);
    double angle = (90. - 30. * (count_angle%7) )/180. * M_PI;

    if(count_impulse %3 ==0){
      ++count_angle;
    }
    ++count_impulse;

    double body_mass(136.); // 136 kg
    double dt(0.1); // 0.1 sec
    double acc = impulse / body_mass / dt;
    double omega = sqrt(9.81/ lipm_env_->GetHeight());


    // double angle = 20./180. * M_PI * trial;


    double acc_x = acc * cos (angle);
    double acc_y = acc * sin (angle);


    // X
    double Ax (0.5 * (1./(omega * omega) * acc_x + 1./omega * normal_state[1]));
    double Bx (0.5 * (1./(omega * omega) * acc_x - 1./omega * normal_state[1]));
    // Y
    double Ay (0.5 * ( normal_state[0] + 1./(omega * omega) * acc_y + 1./omega * normal_state[2]));
    double By (0.5 * ( normal_state[0] + 1./(omega * omega) * acc_y - 1./omega * normal_state[2]));

    double vel_change_x =  omega* (Ax * exp(omega * dt) - Bx * exp(-omega * dt));
    double vel_change_y =  omega* (Ay * exp(omega * dt) - By * exp(-omega * dt));
  
    double pos_change_x = Ax * exp(omega * dt) + Bx * exp(-omega * dt) - 1./(omega * omega) * acc_x;
    double pos_change_y = Ay * exp(omega * dt) + By * exp(-omega * dt) - 1./(omega * omega) * acc_y;
  
    double disturbed_pos[2];
    double disturbed_vel[2];
    // Pos
    disturbed_pos[0] = pos_change_x;
    disturbed_pos[1] = pos_change_y;
    // Vel
    disturbed_vel[0] = vel_change_x;
    disturbed_vel[1] = vel_change_y;
    _FindZeroCoM_X(disturbed_pos, disturbed_vel);

    test_state[0] = disturbed_pos[1];
    test_state[1] = disturbed_vel[0];
    test_state[2] = disturbed_vel[1];
    // printf("test_state: %f, %f, %f\n", test_state[0], test_state[1], test_state[2]);
    // push left
    for (int i(0); i < 3; ++i){
      ini_state[i] = test_state[i] - 2. * RBF_SIGMA + 4. * RBF_SIGMA * rand()/(double)RAND_MAX;
    }
  }

  // push forward
  // ini_state[0] = 0.05815 + 0.02 * rand()/(double)RAND_MAX;
  // ini_state[1] = 0.6295 + 0.02 * rand()/(double)RAND_MAX;
  // ini_state[2] = -0.09615 + 0.02 * rand()/(double)RAND_MAX;

}
