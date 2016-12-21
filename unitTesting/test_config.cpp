
/*--------STD-------------*/
#include <sstream>

/*-----------BOOST TEST SUITE-------------*/
#define BOOST_TEST_MODULE sot_dynamic_config
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/output_test_stream.hpp>

/*-----------SOT DYNAMIC ------------*/
#include <sot-dynamic-pinocchio/dynamic-pinocchio.h>
#include <sot/core/debug.hh>

/*-----------DYNAMIC GRAPH ------------*/
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/exception-abstract.hh>


/*-----------PINOCCHIO-------------*/
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>


using namespace dynamicgraph::sot;


#define MATRIX_BOOST_REQUIRE_CLOSE(N, M, LEFT, RIGHT, TOLERANCE)	\
  for (unsigned i = 0; i < N; ++i)					\
    for (unsigned j = 0; j < M; ++j)					\
      BOOST_REQUIRE_CLOSE(LEFT (i, j), RIGHT (i, j), TOLERANCE)

/* ----- TEST SIGNAL CLASS -----*/

BOOST_AUTO_TEST_CASE (config)
{

  /*-----------------------CONSTRUCTOR-----------------------------------------*/
  Dynamic dynamic_("sot_dynamic_test");
 
  /*------------------------CONFIG-------------------------------------------*/
  //Create Empty Robot
  dynamic_.createRobot();
  BOOST_CHECK_EQUAL(dynamic_.m_model.nbody,1);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.m_model.names[0].c_str(),"universe"),0);

  //Parse urdf file
  dynamic_.setUrdfFile("urdf/two_link.urdf");
  dynamic_.parseUrdfFile();
  dynamic_.displayModel();  
  BOOST_CHECK_EQUAL(dynamic_.m_model.nbody,3);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.m_model.names[0].c_str(),"universe"),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.m_model.names[1].c_str(),"JOINT1"),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.m_model.names[2].c_str(),"JOINT2"),0);

  //CreateJoint and AddBody
  se3::SE3 joint_3_pos(Eigen::Matrix3d::Identity(),
		       (Eigen::Vector3d() << 1,0,0).finished());
  dynamic_.createJoint("JOINT3","JointModelRZ",joint_3_pos.toHomogeneousMatrix());
  dynamic_.addBody("CHILD2","JOINT3","CHILD3");
  BOOST_CHECK_EQUAL(dynamic_.m_model.nbody,4);
  BOOST_CHECK_EQUAL(dynamic_.m_model.nq,3);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.m_model.names[3].c_str(),"JOINT3"),0);

  //Setter and Getter
  dynamic_.setMass("CHILD3",10);
  BOOST_CHECK_EQUAL(dynamic_.m_model.inertias[3].mass(),10);

  Eigen::Vector3d vx; vx<<0.5,0,0;
  dynamic_.setLocalCenterOfMass("CHILD3", vx);
  MATRIX_BOOST_REQUIRE_CLOSE(3, 1, dynamic_.m_model.inertias[3].lever(), vx, 0.0001);
  
  dynamic_.setInertiaMatrix("CHILD3",Eigen::Matrix3d::Identity());
  MATRIX_BOOST_REQUIRE_CLOSE(3, 3, dynamic_.m_model.inertias[3].inertia(), 
			     Eigen::Matrix3d::Identity(), 0.0001);

  dynamicgraph::Vector result_vec;
  dynamic_.setDofBounds("JOINT3",0,0,1.57);
  dynamic_.getLowerPositionLimits(result_vec,0);
  MATRIX_BOOST_REQUIRE_CLOSE(3,1,result_vec,Eigen::Vector3d::Zero(), 0.0001);

  dynamic_.getUpperPositionLimits(result_vec,0);
  MATRIX_BOOST_REQUIRE_CLOSE(3,1,result_vec,
			     (Eigen::Vector3d() << 3.14,3.14,1.57).finished(), 0.0001);

  dynamic_.setLowerPositionLimit("JOINT3",(Eigen::Matrix<double,1,1>() << 0.2).finished());
  dynamic_.getLowerPositionLimits(result_vec,0);
  MATRIX_BOOST_REQUIRE_CLOSE(3,1,result_vec,
			     (Eigen::Vector3d() << 0,0,0.2).finished(), 0.0001);

  dynamic_.setLowerPositionLimit("JOINT3",0.12);
  dynamic_.getLowerPositionLimits(result_vec,0);
  MATRIX_BOOST_REQUIRE_CLOSE(3,1,result_vec,
			     (Eigen::Vector3d() << 0,0,0.12).finished(), 0.0001);

  dynamic_.setUpperPositionLimit("JOINT3",2.12);
  dynamic_.getUpperPositionLimits(result_vec,0);
  MATRIX_BOOST_REQUIRE_CLOSE(3,1,result_vec,
			     (Eigen::Vector3d() << 3.14,3.14,2.12).finished(), 0.0001);

  dynamic_.setMaxVelocityLimit("JOINT3",9.12);
  dynamic_.getUpperVelocityLimits(result_vec,0);
  MATRIX_BOOST_REQUIRE_CLOSE(3,1,result_vec,
			     (Eigen::Vector3d() << 10,10,9.12).finished(), 0.0001);

  dynamic_.setMaxEffortLimit("JOINT3",9.12);
  dynamic_.getMaxEffortLimits(result_vec,0);
  MATRIX_BOOST_REQUIRE_CLOSE(3,1,result_vec,
			     (Eigen::Vector3d() << 12,12,9.12).finished(), 0.0001);
}


