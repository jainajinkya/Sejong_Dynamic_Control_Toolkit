#ifndef RBF_TEST_H
#define RBF_TEST_H

#include <features/RadialBasisFunction.h>
#include <Utils/utilities.h>

#define NUM_LEARNING_PT 1000
#define INPUT_DIM 2
#define NUM_GRID 3
#define NUM_NODE NUM_GRID * NUM_GRID

void RBF_Test(){
  double **  mean_list = new double*[NUM_NODE];
  for(int i(0); i<NUM_NODE; ++i){   mean_list[i] = new double[INPUT_DIM]; }

  double weight[NUM_NODE];
  double min(-0.6);
  double res(0.6);
  double sig_list[NUM_NODE];

  // Mean & Sigma Setting
  for(int i(0); i<NUM_GRID; ++i){
    for(int j(0); j<NUM_GRID; ++j){
      mean_list[NUM_GRID*i + j][0] = min + res*i;
      mean_list[NUM_GRID*i + j][1] = min + res*j;

      weight[NUM_GRID*i + j] = rand()/RAND_MAX;
      sig_list[NUM_GRID*i + j] = 0.5;
    }
  }
  // Make RBF
  RadialBasisFunction<INPUT_DIM, NUM_NODE> rbf;
  rbf.setMeanSigma(mean_list, sig_list);
  rbf.updateWeight(weight);

  // Learning Data
  double max = min + res* (NUM_GRID-1);
  double input_data[NUM_LEARNING_PT][INPUT_DIM];
  double output_data[NUM_LEARNING_PT];
  for(int i(0); i< NUM_LEARNING_PT; ++i){
    for(int j(0); j<INPUT_DIM; ++j){
      input_data[i][j] = min + (max - min) * rand()/RAND_MAX;
    }
    // smooth surface
    /* output_data[i] = sin(3. * input_data[i][0]) */
    /*                  + cos(3. * input_data[i][1]) */
    /*                   + sejong::generator_white_noise(0.0, 0.05); */

    // Parabola
    output_data[i] =  input_data[i][0] * input_data[i][0]
      + input_data[i][1] * input_data[i][1]
      + sejong::generator_white_noise(0.0, 0.01);

    sejong::saveVector(input_data[i], "input_data", INPUT_DIM);
    sejong::saveValue(output_data[i], "output_data");
  }

  // Learning
  int num_learning(50000);
  double output(0.);
  double error(0.);
  double weight_grad[NUM_NODE];
  double grad[NUM_NODE];
  double alpha(0.01);
  const double* curr_weight;
  double pre_error(1.e5);
  int increasing(0);
  for(int i(0); i<num_learning; ++i){
    for(int cc(0);cc<NUM_NODE; ++cc){
      weight_grad[cc] = 0.;
    }

    for (int j(0); j<NUM_LEARNING_PT; ++j){
      output = rbf.getOutput(input_data[j]);
      error = output_data[j] - output;
      rbf.getGradient(input_data[j], grad);

      for(int k(0); k<NUM_NODE; ++k)
        weight_grad[k] += (- 2./NUM_LEARNING_PT * error * grad[k] );

      // printf("error: %f\n", error);
    }
    curr_weight = rbf.getWeight();
    // sejong::pretty_print(curr_weight, "curr weight", NUM_NODE);
    // sejong::pretty_print(weight_grad, "weight_grad", NUM_NODE);

    for(int j(0);j<NUM_NODE; ++j){
      weight[j] = curr_weight[j] - weight_grad[j] * alpha;
    }
    rbf.updateWeight(weight);
    printf("%d th: error - %f\n", i, fabs(error));

    if(pre_error < fabs(error)){
      printf("increasing\n");
      ++increasing;
      /* if(increasing > 50){ break; } */
    }else{ increasing = 0; }

    // if(fabs(pre_error - fabs(error)) <0.000001){
    //   break;
    // }
    pre_error = fabs(error);
  }

  // Test
  double test_res(0.05);
  int num_test_grid = round( (max - min)/test_res +1);
  double test_input[INPUT_DIM];
  for(int i(0);i<num_test_grid; ++i){
    for(int j(0);j<num_test_grid; ++j){
      test_input[0] = min + test_res * i;
      test_input[1] = min + test_res * j;
      sejong::saveValue(rbf.getOutput(test_input), "test_output");
    }
  }
  exit(0);
}

#endif
