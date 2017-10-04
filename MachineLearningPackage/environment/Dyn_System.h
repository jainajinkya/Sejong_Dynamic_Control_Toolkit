#ifndef DYNAMIC_SYSTEM
#define DYNAMIC_SYSTEM

class Dynamic_System{
 public:
  Dynamic_System(){}
  virtual ~Dynamic_System(){}

  virtual bool Transition(const double* state,
                          const double* action,
                          double & reward,
                          double* nx_state, bool b_print = false) = 0;
};

#endif
