#ifndef RADIAL_BASIS_FUNCTION_H
#define RADIAL_BASIS_FUNCTION_H

#include <math.h>

template <int DIM, int NUM_NODE>
class RadialBasisFunction{
 public:
  RadialBasisFunction(){}
  ~RadialBasisFunction(){}

  void setMeanSigma(double ** mean, const double * sig){
    for(int i(0); i<NUM_NODE; ++i){
      for(int j(0); j<DIM; ++j){
        mean_list_[i][j] = mean[i][j];
      }
      sigma_list_[i] = sig[i];
    }
  }
  void updateWeight(const double* weight){
    for(int i(0); i<NUM_NODE; ++i)  weight_[i] = weight[i];
  }

  const double* getWeight(){ return weight_; }

  double getOutput(const double * x){
    double sum(0.);
    for(int i(0); i<NUM_NODE; ++i){
      sum += weight_[i] * _exp_node(x, i);
    }
    return sum;
  }
  void  getGradient(const double * x, double* grad){
    for(int i(0); i<NUM_NODE; ++i){
      grad[i] = _exp_node(x, i);
    }
  }

 protected:
  double weight_[NUM_NODE];
  double mean_list_[NUM_NODE][DIM];
  double sigma_list_[NUM_NODE];

  double _exp_node(const double* x, int idx){
    double sqr_sum(0.);
    for (int i(0); i<DIM; ++i){
      sqr_sum += (x[i] - mean_list_[idx][i]) * (x[i] - mean_list_[idx][i]);
    }
    return exp(-sqr_sum/(2*sigma_list_[idx]*sigma_list_[idx]));
  }
};

#endif
