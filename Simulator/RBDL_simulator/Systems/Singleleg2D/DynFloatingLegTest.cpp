#include <plotter/DynFloatingLegTest.hpp>
#include <utils/ParameterFetcher.hpp>
#include <ctrl/DynTestControl.hpp>
#include <model/RobotModel.hpp>
#include <utils/pseudo_inverse.hpp>

#if CTRL_MODEL == FLOATING_LEG

DynFloatingLegTest::DynFloatingLegTest():DynCtrlTester(){
  ros::NodeHandle nh("~");
  std::string controller_name;

  ParameterFetcher::getReqParam(nh,"controller/name",controller_name);

  if(controller_name.compare("DemoControl_3D") == 0){
    ROS_INFO("DemoControl initialized.");
    // m_ctrl = new DemoControl_3D(nh);
  } else if (controller_name.compare("DynTestControl") == 0){
    // m_ctrl = new TestControl(nh);
    m_ctrl = new DynTestControl(nh);

  } else {
    ROS_ERROR("Controller name, '%s', not recognized. ", controller_name.c_str());
    ros::shutdown();
    exit(1);
  }
}

void DynFloatingLegTest::Initialization(){
  m_jpos_ini.setZero();
  m_jvel_ini.setZero();
  m_jeff_ini.setZero();

  m_jpos_ini[0] = -0.9; 
  m_jpos_ini[1] = M_PI/2. - 0.3;
  m_jpos_ini[2] = -m_jpos_ini[0] - m_jpos_ini[1] - M_PI/2.;

  m_q.tail(NUM_ACT_JOINT) = m_jpos_ini;
  m_qdot.setZero();
  m_q[0] = 0.;
  m_q[1] = 1.0;

  // m_qdot[1] = 5.0;
  // m_qdot[0] = -1.;
  RobotModel* model = RobotModel::getRobotModel();
  model->update(m_q, m_qdot);
}

void DynFloatingLegTest::_ExtraProcess(){
}

void DynFloatingLegTest::_DataPrint(){
  bool b_plot = false;

  if(b_plot){
    RobotModel* model = RobotModel::getRobotModel();

    printf("q:\n");
    std::cout<<m_q<<std::endl;

    printf("qdot:\n");
    std::cout<<m_qdot<<std::endl;

    VectorQ cori, grav;
    Eigen::Matrix<double, NUM_Q, NUM_Q> A;
    model->getCoriolis(cori);
    model->getGravity(grav);
    model->getInertia(A);

    std::cout<<"grav:\n"<<grav<<std::endl;
    std::cout<<"cori:\n"<<cori<<std::endl;
    std::cout<<"Inertia:\n"<<A<<std::endl;
  }
}
void DynFloatingLegTest::_ComputeQdot(const VectorA & jpos_cmd,
                                      const VectorA & jvel_cmd,
                                      const VectorA & jeff_cmd,
                                      VectorQ & qdot){
  // Update
  RobotModel* model = RobotModel::getRobotModel();

  Eigen::Matrix<double, NUM_Q, NUM_Q> Ainv;
  VectorQ grav, cori;
  model->getInertiaInverse(Ainv);
  model->getGravity(grav);
  model->getCoriolis(cori);

  VectorQ tau, qddot;
  tau.setZero();
  tau.tail(NUM_ACT_JOINT) = jeff_cmd;
  
  qddot = Ainv * (tau - grav - cori);
  qdot = m_qdot + qddot * m_sim_rate;

  bool toe_contact(false);
  bool heel_contact(false);

  MatrixJ Jtmp;
  Eigen::Vector3d pos, vel;

  // Toe
  model->getPos(LinkID::LK_TOE, pos);
  model->getVel(LinkID::LK_TOE, vel);

  if(pos[1] < 0.001 && vel[1] < 0.){
    toe_contact = true;
  }
  // Heel
  model->getPos(LinkID::LK_HEEL, pos);
  model->getVel(LinkID::LK_HEEL, vel);

  if(pos[1] < 0.001 && vel[1] < 0.){
    heel_contact = true;
  }


  Eigen::MatrixXd Jc;

  if ( toe_contact && heel_contact){
    Jc = Eigen::MatrixXd(4, NUM_Q);

    model->getJacobian(LinkID::LK_TOE, Jtmp);
    Jc.block(0, 0, 2, NUM_Q) = Jtmp.block(0, 0, 2, NUM_Q);

    model->getJacobian(LinkID::LK_HEEL, Jtmp);
    Jc.block(2, 0, 2, NUM_Q) = Jtmp.block(0, 0, 2, NUM_Q);

  } else if (toe_contact){
    model->getJacobian(LinkID::LK_TOE, Jtmp);
    Jc = Jtmp.block(0, 0, 2, NUM_Q);

  } else if (heel_contact){
    model->getJacobian(LinkID::LK_HEEL, Jtmp);
    Jc = Jtmp.block(0, 0, 2, NUM_Q);
  } else {
    return ;
  }

  Eigen::Matrix<double, NUM_Q, NUM_Q> I;
  I.setIdentity();

  Eigen::MatrixXd Lambda_inv (Jc * Ainv * Jc.transpose());
  Eigen::MatrixXd Lambda;

  sejong::pseudoInverse(Lambda_inv, 0.0001, Lambda);

  qdot = (I - Ainv * Jc.transpose() * Lambda * Jc) * qdot;
}

#endif

