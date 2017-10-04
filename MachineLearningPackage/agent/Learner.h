#ifndef LEARNER_H
#define LEARNER_H

class Learner{
 public:
  Learner(){}
  virtual ~Learner(){}

  virtual bool DoLearning(const double * ini_state) = 0;
};

#endif
