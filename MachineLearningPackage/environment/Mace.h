#ifndef MACE_H
#define MACE_H

#include "Environment.h"

#include <vector>
#include <map>

#define DIM_STATE 2

class Env_Mace: public Environment{
public:
  Env_Mace();
  virtual ~Env_Mace();

  virtual bool Transition(const sejong::Vector & state,
                          const sejong::Vector & action,
                          double & reward,
                          sejong::Vector & nx_state);

  void getTerminalPlace(std::vector< std::vector<int> > & terminal_loc_list);
  
protected:
  double life_cost_;
  std::vector<sejong::Vector> obstacle_list_;
  std::map<std::vector<int>, double> terminal_list_;

  void _Small_Mace_Setup();
  void _Midium_Mace_Setup();
};
#endif
