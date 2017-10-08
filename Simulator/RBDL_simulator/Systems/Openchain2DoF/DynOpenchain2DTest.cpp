#include <plotter/DynOpenchain2DTest.hpp>
#include <utils/ParameterFetcher.hpp>
#include <ctrl/DynOSCControl_2DoF.hpp>
#include <ctrl/DCControl.hpp>
#include <model/RobotModel.hpp>
#include <utils/pseudo_inverse.hpp>

#if CTRL_MODEL == OPENCHAIN_2D

DynOpenchain2DTest::DynOpenchain2DTest():DynCtrlTester(){
  // Sparse simulation rate
  m_frequency_data_send = 500;
  //m_ratio_SERVO_sim_rate = 10;
  m_ratio_SERVO_sim_rate = 5;
  m_usleep_time = 100;
  m_sim_rate = (SERVO_RATE/(double)m_ratio_SERVO_sim_rate);

  ros::NodeHandle nh("~");
  std::string controller_name;

  ParameterFetcher::getReqParam(nh,"controller/name",controller_name);

  if(controller_name.compare("DemoControl_3D") == 0){
    ROS_INFO("DemoControl initialized.");
    // m_ctrl = new DemoControl_3D(nh);
  } else if (controller_name.compare("DCControl") == 0) {
    ROS_INFO("DCControl initialized.");
    m_ctrl = new DCControl(nh);
  } else if (controller_name.compare("DynOSCControl_2DoF") == 0){
    m_ctrl = new DynOSCControl_2DoF(nh);
  } else {
    ROS_ERROR("Controller name, '%s', not recognized. ", controller_name.c_str());
    ros::shutdown();
    exit(1);
  }

  //TEST
  m_rot_inertia[0] = 1.;
  m_rot_inertia[1] = 1.;
}

void DynOpenchain2DTest::Initialization(){
  m_jpos_ini.setZero();
  m_jvel_ini.setZero();
  m_jeff_ini.setZero();

  m_jpos_ini[0] = M_PI/2. + 0.7;
  m_jpos_ini[1] = 2*M_PI - 1.6;

  m_q = m_jpos_ini;
  m_qdot = m_jvel_ini;

  RobotModel* model = RobotModel::getRobotModel();
  model->update(m_q, m_qdot);
}

void DynOpenchain2DTest::_ExtraProcess(){
}

void DynOpenchain2DTest::_DataPrint(){
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

void DynOpenchain2DTest::_ComputeQdot(const VectorA & jpos_cmd,
                                      const VectorA & jvel_cmd,
                                      const VectorA & jeff_cmd,
                                      VectorQ & qdot){
  // Update
  RobotModel* model = RobotModel::getRobotModel();
  Eigen::Matrix<double, NUM_Q, NUM_Q> Ainv;
  VectorQ grav, cori;
  Eigen::Matrix<double, NUM_Q, NUM_Q> A;
  model->getInertia(A);
  model->getInertiaInverse(Ainv);
  model->getGravity(grav);
  model->getCoriolis(cori);

  Eigen::Matrix<double, NUM_Q, NUM_Q> A_rotor;
  A_rotor = A;
  A_rotor(0,0) += m_rot_inertia[0];
  A_rotor(1,1) += m_rot_inertia[1];

  //VectorQ qddot = Ainv * (cmd - grav - cori) ;
  // VectorQ qddot = A_rotor.inverse() * (cmd - grav - cori) ;
  VectorQ tau = jeff_cmd + 60.0 * (jpos_cmd - m_jpos) + 20.*(jvel_cmd - m_jvel);
  // VectorQ tau = jeff_cmd;
  m_jeff =tau;
  VectorQ qddot = A_rotor.inverse() * (tau - grav - cori) ;

  qdot = m_qdot + qddot * m_sim_rate;
}


#endif

