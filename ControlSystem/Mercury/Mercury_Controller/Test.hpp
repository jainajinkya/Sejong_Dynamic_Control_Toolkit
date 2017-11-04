#ifndef TEST_H
#define TEST_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

class Controller;

class Test{
public:
  Test();
  virtual ~Test();

  virtual void TestInitialization() = 0;
  void getTorqueInput(sejong::Vector & gamma);

protected:
  virtual int _NextPhase(const int & phase) = 0;

  bool b_first_visit_;
  int phase_;
  std::vector<Controller*> state_list_;
};


#endif
