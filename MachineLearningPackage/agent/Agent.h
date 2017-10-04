#ifndef AGENT_H
#define AGENT_H

#include <Utils/wrap_eigen.hpp>

class Agent{
public:
  Agent(){}
  virtual ~Agent(){}

  virtual bool DoLearning(const sejong::Vector& ini_state)=0;

};

#endif
