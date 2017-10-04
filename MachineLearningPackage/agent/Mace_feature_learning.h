#ifndef MACE_FEATURE_LEARNING
#define MACE_FEATURE_LEARNING

#include <map>
#include "Mace_Learner.h"

class Mace_feature_learning: public Mace_learner{
public:
  Mace_feature_learning(Env_Mace* mace_env);
  virtual ~Mace_feature_learning();

  virtual bool DoLearning(const sejong::Vector & ini_state);

protected:
  sejong::Vector ini_state_;
  sejong::Vector eligibility_;
  double alpha_;
  double gamma_;
  double lambda_;
  double epsilon_;
  
  int num_feature_;

  double _FindMAX_Q(const sejong::Vector & state,
                    sejong::Vector & opt_action);

  double _getQValue(const sejong::Vector & state,
                    const sejong::Vector & action);

  void _getFeatureValue(const sejong::Vector & state,
                          const sejong::Vector & action,
                          sejong::Vector & feature_value);
  sejong::Vector weight_;

  int count_;
  std::vector< std::vector<int> > terminal_loc_list_;
};

#endif
