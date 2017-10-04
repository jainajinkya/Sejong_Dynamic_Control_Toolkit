#include "valkyrie.h"
#include <time.h>
#include <iostream>
#include <stdio.h>
#include <srConfiguration.h>
#include "Val_Model_define.h"
#include <Utils/utilities.h>

SR_Valkyrie::SR_Valkyrie(const  Vec3 & location, BASELINKTYPE base_link_type, srJoint::ACTTYPE joint_type):RJidx(0), WJidx(0), num_act_joint_(0), cpball_r(0.015) {

    // Parsing Information
    _ParsingURDF();
    _setLinkIdxforJointIdx();
    // Build Robot
    _SetControllableJoint();
    _SetInitialConf();
    _AssembleModel(location, base_link_type, joint_type);
    printf("[Valkyrie] END of Valkyrie assemble\n");

}

void SR_Valkyrie::_SetControllableJoint(){
    URDFJointMap::iterator Jointidxiter;

    for(int i(0); i<joint_names.size(); ++i){
        Jointidxiter = joint_map.find(joint_names[i]);

        if(Jointidxiter->second->type != urdf::Joint::FIXED){
            ACT_JointIdxMap.insert( make_pair(joint_names[i], num_act_joint_));
            // printf("%i th Actuated Joint: %s\n", num_act_joint_, joint_names[i].c_str());
            ++num_act_joint_;
        }
        std::cout << num_act_joint_ << std::endl;
    }
}

void SR_Valkyrie::_setLinkIdxforJointIdx()
{

    JointPtr urdf_joint = joint_map[joint_names[0]];
    string rootLink=urdf_joint->parent_link_name;
    link_names.push_back(urdf_joint->parent_link_name);

    LinkIdxMap.insert(make_pair(urdf_joint->parent_link_name,0));
    int j=1;

    for(int i(0);i<joint_names.size();i++)
    {
        link_names.push_back(joint_map[joint_names[i]]->child_link_name);
        LinkIdxMap.insert(make_pair(joint_map[joint_names[i]]->child_link_name,i+1));

    }

}

void SR_Valkyrie::_setJoints_Param(const int Jidx)
{

    URDFJointMap::iterator Jointidxiter = joint_map.find(joint_names[Jidx]);

    URDFLinkMap::iterator p_Linkidxiter = link_map.find(Jointidxiter->second->parent_link_name);
    URDFLinkMap::iterator c_Linkidxiter = link_map.find(Jointidxiter->second->child_link_name);

    Vec3 p_Link_offset;
    Vec3 c_Link_offset;

    if(p_Linkidxiter->second->inertial)
        p_Link_offset=Vec3(p_Linkidxiter->second->inertial->origin.position.x,p_Linkidxiter->second->inertial->origin.position.y,p_Linkidxiter->second->inertial->origin.position.z);
    if(c_Linkidxiter->second->inertial)
        c_Link_offset=Vec3(c_Linkidxiter->second->inertial->origin.position.x,c_Linkidxiter->second->inertial->origin.position.y,c_Linkidxiter->second->inertial->origin.position.z);

    Vec3 joint_rpy;
    Jointidxiter->second->parent_to_joint_origin_transform.rotation.getRPY (joint_rpy[0], joint_rpy[1], joint_rpy[2]);
    Vec3 JointOff_pos(Jointidxiter->second->parent_to_joint_origin_transform.position.x,
            Jointidxiter->second->parent_to_joint_origin_transform.position.y,
            Jointidxiter->second->parent_to_joint_origin_transform.position.z);

    Vec3 JointOff_ori(joint_rpy[2],joint_rpy[1],joint_rpy[0]);
    Vec3 axis010=Vec3(0,0,-SR_PI_HALF);
    Vec3 axis100=Vec3(0,SR_PI_HALF,0);

    for(int i(0);i<3;i++)
        JointOff_pos[i]= (JointOff_pos[i] - p_Link_offset[i]);


    if(Jointidxiter->second->axis.x){
        m_Rjoint[RJidx].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
        map<string,int>::iterator Linkidxiter=LinkIdxMap.find(p_Linkidxiter->first);
        m_Rjoint[RJidx].SetParentLink(&m_Link[Linkidxiter->second]);
        Linkidxiter=LinkIdxMap.find(c_Linkidxiter->second->name);
        m_Rjoint[RJidx].SetChildLink(&m_Link[Linkidxiter->second]);
        m_Rjoint[RJidx].SetParentLinkFrame(EulerZYX(axis100,JointOff_pos));
        m_Rjoint[RJidx].SetChildLinkFrame(EulerZYX(axis100,-c_Link_offset));
        RJidx++;
    }

    if(Jointidxiter->second->axis.y){
        m_Rjoint[RJidx].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
        map<string,int>::iterator Linkidxiter=LinkIdxMap.find(p_Linkidxiter->first);
        m_Rjoint[RJidx].SetParentLink(&m_Link[Linkidxiter->second]);
        Linkidxiter=LinkIdxMap.find(c_Linkidxiter->second->name);
        m_Rjoint[RJidx].SetChildLink(&m_Link[Linkidxiter->second]);
        m_Rjoint[RJidx].SetParentLinkFrame(EulerZYX(axis010,JointOff_pos));
        m_Rjoint[RJidx].SetChildLinkFrame(EulerZYX(axis010,-c_Link_offset));
        RJidx++;
    }

    if(Jointidxiter->second->axis.z){
        m_Rjoint[RJidx].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
        map<string,int>::iterator Linkidxiter=LinkIdxMap.find(p_Linkidxiter->first);
        m_Rjoint[RJidx].SetParentLink(&m_Link[Linkidxiter->second]);
        Linkidxiter=LinkIdxMap.find(c_Linkidxiter->second->name);
        m_Rjoint[RJidx].SetChildLink(&m_Link[Linkidxiter->second]);
        m_Rjoint[RJidx].SetParentLinkFrame( EulerZYX(JointOff_ori, JointOff_pos) );
        m_Rjoint[RJidx].SetChildLinkFrame(EulerZYX(Vec3(),-c_Link_offset));
        RJidx++;
    }

    if(Jointidxiter->second->type == urdf::Joint::FIXED){
        m_Wjoint[WJidx].GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
        map<string,int>::iterator Linkidxiter=LinkIdxMap.find(p_Linkidxiter->first);
        m_Wjoint[WJidx].SetParentLink(&m_Link[Linkidxiter->second]);
        Linkidxiter=LinkIdxMap.find(c_Linkidxiter->second->name);
        m_Wjoint[WJidx].SetChildLink(&m_Link[Linkidxiter->second]);
        m_Wjoint[WJidx].SetParentLinkFrame(EulerZYX(JointOff_ori,JointOff_pos));
        m_Wjoint[WJidx].SetChildLinkFrame(EulerZYX(Vec3(),-c_Link_offset));
        WJidx++;
    }
}


void SR_Valkyrie::_setLinks_Param(const int L_idx)
{
    URDFLinkMap::iterator Linkidxiter = link_map.find(link_names[L_idx]);

    string ds3D=".3ds";
    string tok1="/";
    string tok2=".";
    string* first_str = new string[32];
    string* second_str= new string[32];
    Vec3 Inertiaoffset_;
    Vec3 link_visual_rpy;
    Vec3 link_visual_xyz;

    if(Linkidxiter->second->inertial){
        Inertiaoffset_=Vec3(Linkidxiter->second->inertial->origin.position.x,Linkidxiter->second->inertial->origin.position.y,Linkidxiter->second->inertial->origin.position.z);
    }

    if(Linkidxiter->second->visual!=0){

        link_visual_xyz[0]=Linkidxiter->second->visual->origin.position.x;
        link_visual_xyz[1]=Linkidxiter->second->visual->origin.position.y;
        link_visual_xyz[2]=Linkidxiter->second->visual->origin.position.z;
        Linkidxiter->second->visual->origin.rotation.getRPY (link_visual_rpy[2], link_visual_rpy[1], link_visual_rpy[0]);

        if((Linkidxiter->second->visual_array[0]->geometry->type)==urdf::Geometry::MESH){
            boost::shared_ptr<urdf::Mesh>mesh=boost::dynamic_pointer_cast<urdf::Mesh>(Linkidxiter->second->visual_array[0]->geometry);
            sejong::split_string(first_str,mesh->filename,tok1);
            sejong::split_string(second_str,first_str[5],tok2);

            string ModelFileName_=ModelPath"meshes/" + second_str[0] + ds3D;
            const char* modelnamepath = ModelFileName_.c_str();
            m_Link[L_idx].GetGeomInfo().SetShape(srGeometryInfo::TDS);
            m_Link[L_idx].GetGeomInfo().SetFileName(modelnamepath);
        }

        if((Linkidxiter->second->visual_array[0]->geometry->type)==urdf::Geometry::BOX){
            boost::shared_ptr<urdf::Box>box=boost::dynamic_pointer_cast<urdf::Box>(Linkidxiter->second->visual_array[0]->geometry);
            m_Link[L_idx].GetGeomInfo().SetShape(srGeometryInfo::BOX);
            m_Link[L_idx].GetGeomInfo().SetDimension(box->dim.x,box->dim.y,box->dim.z);
        }

        if((Linkidxiter->second->visual_array[0]->geometry->type)==urdf::Geometry::CYLINDER){
            boost::shared_ptr<urdf::Cylinder>cylinder=boost::dynamic_pointer_cast<urdf::Cylinder>(Linkidxiter->second->visual_array[0]->geometry);
            m_Link[L_idx].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
            m_Link[L_idx].GetGeomInfo().SetDimension(cylinder->radius,cylinder->length,0);
        }

        if((Linkidxiter->second->visual_array[0]->geometry->type)==urdf::Geometry::SPHERE){
            boost::shared_ptr<urdf::Sphere>sphere=boost::dynamic_pointer_cast<urdf::Sphere>(Linkidxiter->second->visual_array[0]->geometry);
            m_Link[L_idx].GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
            m_Link[L_idx].GetGeomInfo().SetDimension(sphere->radius,0,0);
        }
    }
    else{
        m_Link[L_idx].GetGeomInfo().SetShape(srGeometryInfo::BOX);
        m_Link[L_idx].GetGeomInfo().SetDimension(Vec3(0.00001,0.00001,0.00001));
        // m_Link[L_idx].GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
        // m_Link[L_idx].GetGeomInfo().SetDimension(cpball_r,0,0);
    }

    if(Linkidxiter->second->inertial!=0){
        m_Link[L_idx].GetGeomInfo().SetLocalFrame(EulerZYX(link_visual_rpy+Vec3(0.0, 0.0, SR_PI_HALF),link_visual_xyz-Inertiaoffset_ ));
    }
    else
        m_Link[L_idx].GetGeomInfo().SetLocalFrame(EulerZYX(link_visual_rpy+Vec3(0.0, 0.0, SR_PI_HALF),link_visual_xyz));
}


void SR_Valkyrie::_AssembleModel(const Vec3 & location, BASELINKTYPE base_link_type, srJoint::ACTTYPE joint_type)
{
    _SetLinkShape();
    _SetPassiveJoint(joint_type);
    _SetInertia();
    _SetCollision();
    _SetJointLimit();
    //////////////////////////////////////////////////////////////////////////
    ///////  Collision Setting
    //////////////////////////////////////////////////////////////////////////

    //	srJoint::ACTTYPE joint_type(srJoint::TORQUE);
    for(int i(0); i< num_act_joint_; ++i){
        m_Rjoint[i].SetActType(srJoint::TORQUE);
    }

    m_VLink[SR_ValkyrieID::SIM_Base].SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), location));

    this->SetBaseLink(&m_VLink[SR_ValkyrieID::SIM_Base]);
    this->SetBaseLinkType(base_link_type);
    this->SetSelfCollision(true);
    cout<<"Assemble Model End"<<endl;
}

void SR_Valkyrie::_SetCollision(){
    m_Collision[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
    // m_Collision[0].GetGeomInfo().SetDimension(0.3, 0.17, 0.01);
    m_Collision[0].GetGeomInfo().SetDimension(0.27, 0.16, 0.09);

    m_Collision[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
    // m_Collision[1].GetGeomInfo().SetDimension(0.3, 0.17, 0.01);
    m_Collision[1].GetGeomInfo().SetDimension(0.27, 0.16, 0.09);

    // for(int i(2);i<10;++i){
    //   m_Collision[i].GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
    //   m_Collision[i].GetGeomInfo().SetDimension(cpball_r,0,0);
    // }

    m_Link[LinkIdxMap.find("rightFoot")->second].AddCollision(&m_Collision[0]);
    m_Link[LinkIdxMap.find("leftFoot")->second].AddCollision(&m_Collision[1]);
    // m_Link[LinkIdxMap.find("leftFootOutFront")->second].AddCollision(&m_Collision[2]);
    // m_Link[LinkIdxMap.find("leftFootOutBack")->second].AddCollision(&m_Collision[3]);
    // m_Link[LinkIdxMap.find("leftFootInBack")->second].AddCollision(&m_Collision[4]);
    // m_Link[LinkIdxMap.find("leftFootInFront")->second].AddCollision(&m_Collision[5]);
    // m_Link[LinkIdxMap.find("rightFootOutFront")->second].AddCollision(&m_Collision[6]);
    // m_Link[LinkIdxMap.find("rightFootOutBack")->second].AddCollision(&m_Collision[7]);
    // m_Link[LinkIdxMap.find("rightFootInBack")->second].AddCollision(&m_Collision[8]);
    // m_Link[LinkIdxMap.find("rightFootInFront")->second].AddCollision(&m_Collision[9]);


    // double fric(0.15);
    double fric(0.8);

    m_Link[LinkIdxMap.find("rightFoot")->second].SetFriction(fric);
    m_Link[LinkIdxMap.find("leftFoot")->second].SetFriction(fric);
    // m_Link[LinkIdxMap.find("leftFootOutFront")->second].SetFriction(fric);
    // m_Link[LinkIdxMap.find("leftFootOutBack")->second].SetFriction(fric);
    // m_Link[LinkIdxMap.find("leftFootInBack")->second].SetFriction(fric);
    // m_Link[LinkIdxMap.find("leftFootInFront")->second].SetFriction(fric);
    // m_Link[LinkIdxMap.find("rightFootOutFront")->second].SetFriction(fric);
    // m_Link[LinkIdxMap.find("rightFootOutBack")->second].SetFriction(fric);
    // m_Link[LinkIdxMap.find("rightFootInBack")->second].SetFriction(fric);
    // m_Link[LinkIdxMap.find("rightFootInFront")->second].SetFriction(fric);

    double damp(0.01);
    m_Link[LinkIdxMap.find("rightFoot")->second].SetDamping(damp);
    m_Link[LinkIdxMap.find("leftFoot")->second].SetDamping(damp);
    // m_Link[LinkIdxMap.find("leftFootOutFront")->second].SetDamping(damp);
    // m_Link[LinkIdxMap.find("leftFootOutBack")->second].SetDamping(damp);
    // m_Link[LinkIdxMap.find("leftFootInBack")->second].SetDamping(damp);
    // m_Link[LinkIdxMap.find("leftFootInFront")->second].SetDamping(damp);
    // m_Link[LinkIdxMap.find("rightFootOutFront")->second].SetDamping(damp);
    // m_Link[LinkIdxMap.find("rightFootOutBack")->second].SetDamping(damp);
    // m_Link[LinkIdxMap.find("rightFootInBack")->second].SetDamping(damp);
    // m_Link[LinkIdxMap.find("rightFootInFront")->second].SetDamping(damp);

    double restit(0.0);
    m_Link[LinkIdxMap.find("rightFoot")->second].SetRestitution(restit);
    m_Link[LinkIdxMap.find("leftFoot")->second].SetRestitution(restit);
    // m_Link[LinkIdxMap.find("leftFootOutFront")->second].SetRestitution(restit);
    // m_Link[LinkIdxMap.find("leftFootOutBack")->second].SetRestitution(restit);
    // m_Link[LinkIdxMap.find("leftFootInBack")->second].SetRestitution(restit);
    // m_Link[LinkIdxMap.find("leftFootInFront")->second].SetRestitution(restit);
    // m_Link[LinkIdxMap.find("rightFootOutFront")->second].SetRestitution(restit);
    // m_Link[LinkIdxMap.find("rightFootOutBack")->second].SetRestitution(restit);
    // m_Link[LinkIdxMap.find("rightFootInBack")->second].SetRestitution(restit);
    // m_Link[LinkIdxMap.find("rightFootInFront")->second].SetRestitution(restit);
}

void SR_Valkyrie::_SetLinkShape(){
    for(int i(0);i<joint_names.size();i++)
    {
        _setJoints_Param(i);
        _setLinks_Param(i);
    }
    _setLinks_Param(joint_names.size());
}

void SR_Valkyrie::SetConfiguration( const std::vector<double>& _conf )
{

    KIN_UpdateFrame_All_The_Entity();
}

void SR_Valkyrie::_SetInitialConf()
{
    //case 0 : just stand
    //case 1 : left leg up
    //case 2 : lower stand
    //case 3: walking ready
    // int pose(1);
    int pose(3);

    m_VPjoint[SR_ValkyrieID::VIRTUAL_X].m_State.m_rValue[0] = 0.0;
    m_VPjoint[SR_ValkyrieID::VIRTUAL_Y].m_State.m_rValue[0] = 0.0;
    m_VPjoint[SR_ValkyrieID::VIRTUAL_Z].m_State.m_rValue[0] = 1.135;// + 0.3;
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rx].m_State.m_rValue[0] = 0.0;
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Ry].m_State.m_rValue[0] = 0.0;
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rz].m_State.m_rValue[0] = 0.0;

    m_Rjoint[ACT_JointIdxMap.find("leftHipPitch"  )->second].m_State.m_rValue[0] = -0.3;
    m_Rjoint[ACT_JointIdxMap.find("leftHipPitch"  )->second].m_State.m_rValue[0] = -0.3;
    m_Rjoint[ACT_JointIdxMap.find("rightHipPitch" )->second].m_State.m_rValue[0] = -0.3;
    m_Rjoint[ACT_JointIdxMap.find("leftKneePitch" )->second].m_State.m_rValue[0] = 0.6;
    m_Rjoint[ACT_JointIdxMap.find("rightKneePitch")->second].m_State.m_rValue[0] = 0.6;
    m_Rjoint[ACT_JointIdxMap.find("leftAnklePitch")->second].m_State.m_rValue[0] = -0.3;
    m_Rjoint[ACT_JointIdxMap.find("rightAnklePitch")->second].m_State.m_rValue[0] = -0.3;

    //m_Rjoint[ACT_JointIdxMap.find("rightShoulderPitch")->second].m_State.m_rValue[0] = -SR_PI_HALF;
    m_Rjoint[ACT_JointIdxMap.find("rightShoulderPitch")->second].m_State.m_rValue[0] = 0.2;
    // m_Rjoint[ACT_JointIdxMap.find("rightShoulderRoll" )->second].m_State.m_rValue[0] = 0.9;
    m_Rjoint[ACT_JointIdxMap.find("rightShoulderRoll" )->second].m_State.m_rValue[0] = 1.1;
    // m_Rjoint[ACT_JointIdxMap.find("rightElbowPitch"   )->second].m_State.m_rValue[0] = 1.3; //
    m_Rjoint[ACT_JointIdxMap.find("rightElbowPitch"   )->second].m_State.m_rValue[0] = 0.4;
    m_Rjoint[ACT_JointIdxMap.find("rightForearmYaw" )->second].m_State.m_rValue[0] = 1.5;
    //m_Rjoint[ACT_JointIdxMap.find("leftShoulderPitch" )->second].m_State.m_rValue[0] = 1.07;
    m_Rjoint[ACT_JointIdxMap.find("leftShoulderPitch" )->second].m_State.m_rValue[0] = -0.2;
    // m_Rjoint[ACT_JointIdxMap.find("leftShoulderRoll"  )->second].m_State.m_rValue[0] = -0.9;
    m_Rjoint[ACT_JointIdxMap.find("leftShoulderRoll"  )->second].m_State.m_rValue[0] = -1.1;
    // m_Rjoint[ACT_JointIdxMap.find("leftElbowPitch"    )->second].m_State.m_rValue[0] = -1.3;
    m_Rjoint[ACT_JointIdxMap.find("leftElbowPitch"    )->second].m_State.m_rValue[0] = -0.4;
    m_Rjoint[ACT_JointIdxMap.find("leftForearmYaw" )->second].m_State.m_rValue[0] = 1.5;

    switch(pose){
        case 0:

            break;

        case 1:
            m_VPjoint[SR_ValkyrieID::VIRTUAL_Z].m_State.m_rValue[0] = 1.131;

            m_Rjoint[ACT_JointIdxMap.find("rightHipRoll" )->second].m_State.m_rValue[0] = 0.17;
            m_Rjoint[ACT_JointIdxMap.find("rightAnkleRoll" )->second].m_State.m_rValue[0] = -0.17;
            m_Rjoint[ACT_JointIdxMap.find("leftKneePitch" )->second].m_State.m_rValue[0] = 1.0;
            m_Rjoint[ACT_JointIdxMap.find("leftHipPitch" )->second].m_State.m_rValue[0] = -1.0;

            break;

        case 2:
            m_VPjoint[SR_ValkyrieID::VIRTUAL_X].m_State.m_rValue[0] =  -0.0183;
            m_VPjoint[SR_ValkyrieID::VIRTUAL_Z].m_State.m_rValue[0] =  1.079;

            m_Rjoint[ACT_JointIdxMap.find("leftHipYaw" )->second].m_State.m_rValue[0] =  0.001277;
            m_Rjoint[ACT_JointIdxMap.find("leftHipRoll" )->second].m_State.m_rValue[0] =  0.000066;
            m_Rjoint[ACT_JointIdxMap.find("leftHipPitch" )->second].m_State.m_rValue[0] =  -0.421068;
            m_Rjoint[ACT_JointIdxMap.find("leftKneePitch" )->second].m_State.m_rValue[0] = 0.915849;
            m_Rjoint[ACT_JointIdxMap.find("leftAnklePitch" )->second].m_State.m_rValue[0] =  -0.470756;
            m_Rjoint[ACT_JointIdxMap.find("leftAnkleRoll" )->second].m_State.m_rValue[0] =  0.000312;

            m_Rjoint[ACT_JointIdxMap.find("rightHipYaw" )->second].m_State.m_rValue[0] =  0.003021;
            m_Rjoint[ACT_JointIdxMap.find("rightHipRoll" )->second].m_State.m_rValue[0] =  -0.000109;
            m_Rjoint[ACT_JointIdxMap.find("rightHipPitch" )->second].m_State.m_rValue[0] =  -0.421119;
            m_Rjoint[ACT_JointIdxMap.find("rightKneePitch" )->second].m_State.m_rValue[0] = 0.917231;
            m_Rjoint[ACT_JointIdxMap.find("rightAnklePitch" )->second].m_State.m_rValue[0] =  -0.467243;
            m_Rjoint[ACT_JointIdxMap.find("rightAnkleRoll" )->second].m_State.m_rValue[0] =  0.000072;
            break;

        case 3:
            // -0.032720   0.004087   1.050418
            // -0.002063  -0.012110  -0.375930   1.027675  -0.663891   0.008284
            //  0.017352  -0.058997  -0.755560   0.650955   0.106764   0.055034

            m_VPjoint[SR_ValkyrieID::VIRTUAL_X].m_State.m_rValue[0] =  -0.032720;
            m_VPjoint[SR_ValkyrieID::VIRTUAL_Z].m_State.m_rValue[0] =  1.050418;

            m_Rjoint[ACT_JointIdxMap.find("leftHipYaw" )->second].m_State.m_rValue[0] =  -0.002063;
            m_Rjoint[ACT_JointIdxMap.find("leftHipRoll" )->second].m_State.m_rValue[0] =  -0.012110;
            m_Rjoint[ACT_JointIdxMap.find("leftHipPitch" )->second].m_State.m_rValue[0] =  -0.375930;
            m_Rjoint[ACT_JointIdxMap.find("leftKneePitch" )->second].m_State.m_rValue[0] = 1.027675;
            m_Rjoint[ACT_JointIdxMap.find("leftAnklePitch" )->second].m_State.m_rValue[0] =  -0.663891;
            m_Rjoint[ACT_JointIdxMap.find("leftAnkleRoll" )->second].m_State.m_rValue[0] =  0.008284;

            m_Rjoint[ACT_JointIdxMap.find("rightHipYaw" )->second].m_State.m_rValue[0] =  0.017352;
            m_Rjoint[ACT_JointIdxMap.find("rightHipRoll" )->second].m_State.m_rValue[0] =  -0.058997;
            m_Rjoint[ACT_JointIdxMap.find("rightHipPitch" )->second].m_State.m_rValue[0] =  -0.755560;
            m_Rjoint[ACT_JointIdxMap.find("rightKneePitch" )->second].m_State.m_rValue[0] = 0.650955;
            m_Rjoint[ACT_JointIdxMap.find("rightAnklePitch" )->second].m_State.m_rValue[0] =  0.106764;
            m_Rjoint[ACT_JointIdxMap.find("rightAnkleRoll" )->second].m_State.m_rValue[0] =  0.055034;
            break;

    }
    KIN_UpdateFrame_All_The_Entity();
}


void SR_Valkyrie::_SetInertia(){
    Inertia dummy = Inertia(0.0);
    m_VLink[SR_ValkyrieID::SIM_Base].SetInertia(dummy);
    m_VLink[SR_ValkyrieID::SIM_L_Y].SetInertia(dummy);
    m_VLink[SR_ValkyrieID::SIM_L_Z].SetInertia(dummy);
    m_VLink[SR_ValkyrieID::SIM_L_Rz].SetInertia(dummy);
    m_VLink[SR_ValkyrieID::SIM_L_Ry].SetInertia(dummy);
    m_VLink[SR_ValkyrieID::SIM_L_Rx].SetInertia(dummy);

    // for(int i(0); i<link_names.size(); ++i){
    //     printf("%i th link name: %s\n", i, link_names[i].c_str());
    // }

    for (int i(0); i<link_names.size(); i++){
        URDFLinkMap::iterator iter = link_map.find(link_names[i]);
        if(iter->second->inertial){
            Inertia inertia = Inertia(iter->second->inertial->ixx,iter->second->inertial->iyy,iter->second->inertial->izz,iter->second->inertial->ixy,iter->second->inertial->iyz,iter->second->inertial->ixz);
            Vec3  offset =  Vec3(iter->second->inertial->origin.position.x,
                    iter->second->inertial->origin.position.y,
                    iter->second->inertial->origin.position.z);
            inertia.SetMass(iter->second->inertial->mass);
            m_Link[i].SetInertia(inertia);
        }
        else{
            Inertia inertia = Inertia(0,0,0,0,0,0);
            m_Link[i].SetInertia(inertia);
        }
    }
}

void SR_Valkyrie::_SetPassiveJoint(srJoint::ACTTYPE joint_type){

    float Link_R = (rand()%100)*0.008f;
    float Link_G = (rand()%100)*0.011f;
    float Link_B = (rand()%100)*0.012f;
    double passive_radius(0.001);
    double passive_length(0.001);

    //Passive Joint (PRISMATIC)
    m_VPjoint[SR_ValkyrieID::VIRTUAL_X].SetParentLink(&m_VLink[SR_ValkyrieID::SIM_Base]);
    m_VPjoint[SR_ValkyrieID::VIRTUAL_X].SetChildLink( &m_VLink[SR_ValkyrieID::SIM_L_Y]);
    m_VPjoint[SR_ValkyrieID::VIRTUAL_X].GetGeomInfo().SetColor(Link_R, Link_G, 0.0);
    m_VPjoint[SR_ValkyrieID::VIRTUAL_X].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    m_VPjoint[SR_ValkyrieID::VIRTUAL_X].GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);

    m_VPjoint[SR_ValkyrieID::VIRTUAL_X].SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));
    m_VPjoint[SR_ValkyrieID::VIRTUAL_X].SetChildLinkFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));

    m_VPjoint[SR_ValkyrieID::VIRTUAL_Y].SetParentLink(&m_VLink[SR_ValkyrieID::SIM_L_Y]);
    m_VPjoint[SR_ValkyrieID::VIRTUAL_Y].SetChildLink( &m_VLink[SR_ValkyrieID::SIM_L_Z]);
    m_VPjoint[SR_ValkyrieID::VIRTUAL_Y].GetGeomInfo().SetColor(0.0, Link_G, Link_B);
    m_VPjoint[SR_ValkyrieID::VIRTUAL_Y].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    m_VPjoint[SR_ValkyrieID::VIRTUAL_Y].GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);

    m_VPjoint[SR_ValkyrieID::VIRTUAL_Y].SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));
    m_VPjoint[SR_ValkyrieID::VIRTUAL_Y].SetChildLinkFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));

    m_VPjoint[SR_ValkyrieID::VIRTUAL_Z].SetParentLink(&m_VLink[SR_ValkyrieID::SIM_L_Z]);
    m_VPjoint[SR_ValkyrieID::VIRTUAL_Z].SetChildLink( &m_VLink[SR_ValkyrieID::SIM_L_Rz]);
    m_VPjoint[SR_ValkyrieID::VIRTUAL_Z].GetGeomInfo().SetColor(Link_R, 0.0, Link_B);
    m_VPjoint[SR_ValkyrieID::VIRTUAL_Z].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    m_VPjoint[SR_ValkyrieID::VIRTUAL_Z].GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
    m_VPjoint[SR_ValkyrieID::VIRTUAL_Z].SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));
    m_VPjoint[SR_ValkyrieID::VIRTUAL_Z].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));

    //Passive Joint (ROTATION)
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rz].SetParentLink(&m_VLink[SR_ValkyrieID::SIM_L_Rz]);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rz].SetChildLink( &m_VLink[SR_ValkyrieID::SIM_L_Ry]);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rz].GetGeomInfo().SetColor(Link_R, 0.0, 0.0);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rz].GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rz].GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rz].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),
                Vec3(0.0, 0.0, 0.0)));
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rz].SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));

    m_VRjoint[SR_ValkyrieID::VIRTUAL_Ry].SetParentLink(&m_VLink[SR_ValkyrieID::SIM_L_Ry]);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Ry].SetChildLink( &m_VLink[SR_ValkyrieID::SIM_L_Rx]);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Ry].GetGeomInfo().SetColor(0.0, Link_G, 0.0);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Ry].GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Ry].GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Ry].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Ry].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));

    //Pelvis -- virtual
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rx].SetParentLink(&m_VLink[SR_ValkyrieID::SIM_L_Rx]);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rx].SetChildLink( &m_Link[0]);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rx].GetGeomInfo().SetColor(0.0, 0.0, Link_B);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rx].GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rx].GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rx].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rx].SetChildLinkFrame(EulerZYX(Vec3(0.0,-SR_PI_HALF,SR_PI),-Vec3(-0.00532, -0.003512, 0.)));
    //m_VRjoint[SR_ValkyrieID::VIRTUAL_Rx].SetChildLinkFrame(EulerZYX(Vec3(0.0,-SR_PI_HALF,SR_PI),-Vec3(-0.00532, -0.003512, cpball_r-0.01)));

    for (int i(0); i<SIM_NUM_PASSIVE; ++i){
        m_VLink[i + SR_ValkyrieID::SIM_Base].GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
        m_VLink[i + SR_ValkyrieID::SIM_Base].GetGeomInfo().SetDimension(0.001, 0.001, 0);
    }

    ////////////// Actuation Type (Torque.. Default) ////////
    m_VPjoint[SR_ValkyrieID::VIRTUAL_X].SetActType(joint_type);
    m_VPjoint[SR_ValkyrieID::VIRTUAL_Y].SetActType(joint_type);
    m_VPjoint[SR_ValkyrieID::VIRTUAL_Z].SetActType(joint_type);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rx].SetActType(joint_type);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Ry].SetActType(joint_type);
    m_VRjoint[SR_ValkyrieID::VIRTUAL_Rz].SetActType(joint_type);
}

void SR_Valkyrie::_ParsingURDF(){
    string filename(ModelPath"urdf/r5_urdf_rbdl.urdf");
    // string filename(ModelPath"urdf/r5_urdf.urdf");

    ifstream  model_file(filename.c_str());

    if (!model_file) {
        cerr << "Error opening file '" << filename << "'." << endl;
        abort();
    }
    // reserve memory for the contents of the file
    string model_xml_string;
    model_file.seekg(0, std::ios::end);
    model_xml_string.reserve(model_file.tellg());
    model_file.seekg(0, std::ios::beg);
    model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());

    model_file.close();
    boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF (model_xml_string);
    boost::shared_ptr<urdf::Link> urdf_root_link;

    link_map = urdf_model->links_;
    joint_map = urdf_model->joints_;
    // add the root body
    const boost::shared_ptr<const urdf::Link>& root = urdf_model->getRoot();
    stack<LinkPtr > link_stack;
    stack<int> joint_index_stack;
    //Firstly put root link
    link_stack.push (link_map[(urdf_model->getRoot()->name)]);

    if (link_stack.top()->child_joints.size() > 0) {
        joint_index_stack.push(0);
    }
    while (link_stack.size() > 0) {
        LinkPtr cur_link = link_stack.top();
        unsigned int joint_idx = joint_index_stack.top();
        if (joint_idx < cur_link->child_joints.size()) {
            JointPtr cur_joint = cur_link->child_joints[joint_idx];
            // increment joint index
            joint_index_stack.pop();
            joint_index_stack.push (joint_idx + 1);
            link_stack.push (link_map[cur_joint->child_link_name]);
            joint_index_stack.push(0);
            joint_names.push_back(cur_joint->name);
        }
        else {
            link_stack.pop();
            joint_index_stack.pop();
        }
    }
    //Link add;
    URDFLinkMap::iterator iter = link_map.begin();
    URDFJointMap::iterator j_iter = joint_map.begin();
    Vec3 root_inertial_rpy;

    /////LINK INFORMATION
    int linkiters=0;
    Vec3 link_rpy;

    string ds3D=".3ds";
    string tok1="/";
    string tok2=".";
    string* first_str = new string[32];
    string* second_str= new string[32];

    for(; iter != link_map.end(); ++iter){
        linkiters++;
        if(iter->second->inertial){
        }

        if(iter->second->collision){
        }

        if(iter->second->visual){

            if((iter->second->visual_array[0]->geometry->type)==urdf::Geometry::MESH){
                boost::shared_ptr<urdf::Mesh>mesh=boost::dynamic_pointer_cast<urdf::Mesh>(iter->second->visual_array[0]->geometry);
                sejong::split_string(first_str,mesh->filename,tok1);
                sejong::split_string(second_str,first_str[5],tok2);

                string ModelFileName_=ModelPath"meshes/"+second_str[0]+ds3D;
                const char* modelnamepath = ModelFileName_.c_str();
            }

            if((iter->second->visual_array[0]->geometry->type)==urdf::Geometry::BOX){
                boost::shared_ptr<urdf::Box>box=boost::dynamic_pointer_cast<urdf::Box>(iter->second->visual_array[0]->geometry);
            }

            if((iter->second->visual_array[0]->geometry->type)==urdf::Geometry::CYLINDER){
                boost::shared_ptr<urdf::Cylinder>cylinder=boost::dynamic_pointer_cast<urdf::Cylinder>(iter->second->visual_array[0]->geometry);
            }

            if((iter->second->visual_array[0]->geometry->type)==urdf::Geometry::SPHERE){
                boost::shared_ptr<urdf::Sphere>sphere=boost::dynamic_pointer_cast<urdf::Sphere>(iter->second->visual_array[0]->geometry);
            }
        }
    }
}

void SR_Valkyrie::_SetJointLimit(){
    // m_Rjoint[ACT_JointIdxMap.find("leftShoulderRoll" )->second].SetPositionLimit(-80, -20);
    // m_Rjoint[ACT_JointIdxMap.find("rightShoulderRoll" )->second].SetPositionLimit(20, 80);

    // m_Rjoint[ACT_JointIdxMap.find("leftElbowPitch" )->second].SetPositionLimit(-50., 50.);
    // m_Rjoint[ACT_JointIdxMap.find("rightElbowPitch" )->second].SetPositionLimit(-50., 50.);


  m_Rjoint[ACT_JointIdxMap.find("leftShoulderRoll" )->second].SetPositionLimit(-70, -10);
  m_Rjoint[ACT_JointIdxMap.find("rightShoulderRoll" )->second].SetPositionLimit(10, 70);

    m_Rjoint[ACT_JointIdxMap.find("leftElbowPitch" )->second].SetPositionLimit(-60., 0.);
    m_Rjoint[ACT_JointIdxMap.find("rightElbowPitch" )->second].SetPositionLimit(-0., 60.);

    m_Rjoint[ACT_JointIdxMap.find("leftShoulderPitch" )->second].SetPositionLimit(-45, 75);
    m_Rjoint[ACT_JointIdxMap.find("rightShoulderPitch" )->second].SetPositionLimit(-75, 45);

    m_Rjoint[ACT_JointIdxMap.find("torsoYaw" )->second].SetPositionLimit(-140., 140.);


    for(int i(0);i <6; ++i){
        m_VRjoint[i].MakePositionLimit(false);
        m_VPjoint[i].MakePositionLimit(false);
    }
}
