#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <Utils/wrap_eigen.hpp>

class Environment{
public:
  Environment(){}
  virtual ~Environment(){}

  virtual bool Transition(const sejong::Vector & state,
                          const sejong::Vector & action,
                          double & reward,
                          sejong::Vector & nx_state) = 0;
};
#endif
