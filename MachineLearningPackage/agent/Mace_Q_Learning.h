#ifndef MACE_Q_LEARNING
#define MACE_Q_LEARNING

#include <map>
#include "Mace_Learner.h"

class Mace_Q_learning: public Mace_learner{
public:
  Mace_Q_learning(Env_Mace* mace_env);
  virtual ~Mace_Q_learning();

  virtual bool DoLearning(const sejong::Vector & ini_state);

protected:
  double alpha_;
  double gamma_;

  double _FindMAX_Q(const sejong::Vector & state,
                    sejong::Vector & opt_action);
  void _getAugmentedStateAction(const sejong::Vector & state,
                                const sejong::Vector & action,
                                std::vector<double> & aug_state_action);

  std::map<std::vector<double>, double> q_value_;
};

#endif
