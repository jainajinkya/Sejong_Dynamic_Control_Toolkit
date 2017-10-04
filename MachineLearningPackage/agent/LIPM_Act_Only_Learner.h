#ifndef LIPM_ACT_ONLY_LEARNER
#define LIPM_ACT_ONLY_LEARNER

#include "Agent.h"

class LIPM_3D_Act_Only;
class LIPM_Tester;

// Actor Only
class LIPM_Act_Only_Learner: public Agent{
public:
  LIPM_Act_Only_Learner(LIPM_3D_Act_Only* pipm_env);
  virtual ~LIPM_Act_Only_Learner();

  virtual bool DoLearning(const sejong::Vector & ini_state);

  void GetPolicy(sejong::Vector & policy) { policy = best_policy_; }
  void GetInitialState(sejong::Vector & state){ state = ini_state_; }
  double GetHeight();
  const LIPM_Tester * GetLIPM_Tester(){ return lipm_tester_; }
protected:
  sejong::Vector ini_state_;

  void _ShortenState(const sejong::Vector & full_state,
                     sejong::Vector & shorten_state);
  void _ExtendState(const sejong::Vector & shorten_state,
                    sejong::Vector & full_state);

  int _rand();
  bool _GenerateDeltaList(std::vector< std::vector<int> > & delta_list);

  // Test Each Action
  virtual bool _TestAction(const sejong::Vector & pivot_action,
                           const std::vector< std::vector<int > > & delta_list,
                           std::vector<double> & reward_list) = 0;
  // Gradient
  bool _FindGradient(const std::vector<std::vector<int> > & delta_list,
                     const std::vector<double> & reward_list,
                     sejong::Vector & grad);

  void _PrintDeltaList();
  void _PrintRewardList();

  LIPM_3D_Act_Only* lipm_env_;
  double etha_;
  // (t_switch, xp, yp)
  std::vector<double >  epsilon_;

  int step_horizon_;
  int dim_act_;

  int num_allocation_;
  int dim_allocation_;

  std::vector< std::vector<int> > delta_list_;
  std::vector< double > reward_list_;
  // (t_switch, xp, yp) X (step_horizon)
  sejong::Vector policy_;

  double best_reward_;
  sejong::Vector best_policy_;
  LIPM_Tester* lipm_tester_;
};

#endif
