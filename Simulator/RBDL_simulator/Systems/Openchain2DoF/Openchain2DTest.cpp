#include <plotter/Openchain2DTest.hpp>
#include <ctrl/TestControl.hpp>
#include <model/RobotModel.hpp>
#include <ctrl/DemoControl.hpp>
#include <utils/ParameterFetcher.hpp>

#if CTRL_MODEL == OPENCHAIN_2D

Openchain2DTest::Openchain2DTest():CtrlTester(){
  ros::NodeHandle nh("~");
  std::string controller_name;

  ParameterFetcher::getReqParam(nh,"controller/name",controller_name);

  if(controller_name.compare("DemoControl") == 0){
    ROS_INFO("DemoControl initialized.");
    m_ctrl = new DemoControl(nh);
  } else if (controller_name.compare("TestControl") == 0){
    m_ctrl = new TestControl(nh);
  }else {
    ROS_ERROR("Controller name, '%s', not recognized. ", controller_name.c_str());
    ros::shutdown();
    exit(1);
  }

}

void Openchain2DTest::Initialization(){
  m_jpos_ini.setZero();
  m_jvel_ini.setZero();
  m_jeff_ini.setZero();

  m_jpos_ini[0] = M_PI/2. + 1.3;
  m_jpos_ini[1] = M_PI + 0.4;
}

void Openchain2DTest::_ExtraProcess(){
}


void Openchain2DTest::_DataPrint(){
  bool b_plot = false;

  if(b_plot){
    RobotModel* model = RobotModel::getRobotModel();

    printf("q:\n");
    std::cout<<m_jpos<<std::endl;

    printf("qdot:\n");
    std::cout<<m_jvel<<std::endl;

    VectorQ cori, grav;
    Eigen::Matrix<double, NUM_Q, NUM_Q> A;
    model->getCoriolis(cori);
    model->getGravity(grav);
    model->getInertia(A);

    std::cout<<"grav:\n"<<grav<<std::endl;
    std::cout<<"cori:\n"<<cori<<std::endl;
    std::cout<<"Inertia:\n"<<A<<std::endl;

    printf("jpos cmd: %f, %f\n", m_jpos_cmd[0], m_jpos_cmd[1]);
    printf("jvel cmd: %f, %f\n", m_jvel_cmd[0], m_jvel_cmd[1]);
    printf("jeff_cmd: %f, %f\n", m_jeff_cmd[0], m_jeff_cmd[1]);
  }
}

#endif // CTRL_MODEL == OPENCHAIN_2D
