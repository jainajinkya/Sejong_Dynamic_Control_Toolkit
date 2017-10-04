#ifndef Valkyrie_Model
#define Valkyrie_Model

#include "srDyn/srSpace.h"

#include "classifier.h"
#include <urdf/model.h>
#include <urdf/urdf_parser.h>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <string>
#include <stack>

#include <srConfiguration.h>

#define SIM_NUM_RJOINT	32
#define SIM_NUM_PJOINT 0

#define SIM_NUM_PASSIVE_P   3  //R+P = num of passive
#define SIM_NUM_PASSIVE_R   3  //R+P = num of passive

#define SIM_NUM_PASSIVE     SIM_NUM_PASSIVE_P + SIM_NUM_PASSIVE_R
#define SIM_NUM_JOINT       SIM_NUM_RJOINT + SIM_NUM_PJOINT

////////////////////////////
#define SIM_NUM_LINK	86 + 6 // Link + Passive
#define SIM_NUM_WJOINT	53

typedef boost::shared_ptr<urdf::Link> LinkPtr;
typedef boost::shared_ptr<urdf::Joint> JointPtr;
typedef boost::shared_ptr<urdf::ModelInterface> ModelPtr;
typedef vector<LinkPtr> URDFLinkVector;
typedef vector<JointPtr> URDFJointVector;
typedef map<string, LinkPtr> URDFLinkMap;
typedef map<string, JointPtr> URDFJointMap;

class SR_Valkyrie: public srSystem
{
public:
  SR_Valkyrie(const Vec3 & location, srSystem::BASELINKTYPE base_link_type, srJoint::ACTTYPE joint_type);
  virtual ~SR_Valkyrie(){}
  void SetConfiguration(const std::vector<double>& _conf);

public:
  srRevoluteJoint      m_Rjoint[100];
  srPrismaticJoint     m_VPjoint[6];
  srRevoluteJoint      m_VRjoint[6];
  srWeldJoint		     m_Wjoint[100];
  srRevoluteState     *Full_Joint_State_[90];
  srLink		m_Link[96];
  srLink      m_VLink[6];
  srCollision		m_Collision[SIM_NUM_LINK];

  std::map<string, int> ACT_JointIdxMap;
  std::map<string, int> LinkIdxMap;
  std::map<string,JointParam> JointparamMap;
  std::map<string,LinkParam> LinksparamMap;

  //From RBDL-parser
  vector<string> joint_names;
  vector<string> link_names;
  URDFLinkMap link_map;
  URDFJointMap joint_map;
  int RJidx;
  int WJidx;
  int num_act_joint_;

private:
  void _SetControllableJoint();
  void _AssembleModel(const Vec3 & location, BASELINKTYPE base_link_type, srJoint::ACTTYPE joint_type);
  void _SetInertia();
  void _SetCollision();
  void _SetLinkShape();
  void _SetLinkShape_Cylinder();
  void _SetPassiveJoint(srJoint::ACTTYPE joint_type);
  void _SetInitialConf();
  void _SetJointLinkIdx();
  void _SetJointLimit();
  void _SetJointFrames(const int J_idx,const int P_idx, const int C_idx);
  void _SetLinkFrames(const int L_idx, const string MeshModel);
  void _readfromURDF();
  void _ParsingURDF();
  void _printURDF();
  void _setJoints_Param(const int Jidx_);
  void _setLinks_Param(const int L_idx_);
  void _setLinkIdxforJointIdx();
  double cpball_r;
private:
  vector<LinkParam>  m_LinkSet;
  vector<JointParam> m_JointSet;
};


#endif
