
/*--------STD-------------*/
#include <sstream>

/*-----------BOOST TEST SUITE-------------*/
#define BOOST_TEST_MODULE sot_dynamic_constructor
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/output_test_stream.hpp>

/*-----------SOT DYNAMIC ------------*/
#include <sot-dynamic/dynamic.h>
#include <sot/core/debug.hh>

/*-----------DYNAMIC GRAPH ------------*/
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/exception-abstract.hh>


/*-----------PINOCCHIO-------------*/
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/parser/urdf.hpp>


using namespace dynamicgraph::sot;

/* ----- TEST SIGNAL CLASS -----*/

BOOST_AUTO_TEST_CASE (constructor)
{
  /*-----------------------CONSTRUCTOR-----------------------------------------*/
  Dynamic dynamic_("sot_dynamic_test");
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.jointPositionSIN.getName().c_str(),"sotDynamic(sot_dynamic_test)::input(vector)::position"),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.freeFlyerPositionSIN.getName().c_str(),"sotDynamic(sot_dynamic_test)::input(vector)::ffposition"),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.jointVelocitySIN.getName().c_str(),"sotDynamic(sot_dynamic_test)::input(vector)::velocity"),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.freeFlyerVelocitySIN.getName().c_str(),"sotDynamic(sot_dynamic_test)::input(vector)::ffvelocity"),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.jointAccelerationSIN.getName().c_str(),"sotDynamic(sot_dynamic_test)::input(vector)::acceleration"),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.freeFlyerAccelerationSIN.getName().c_str(),"sotDynamic(sot_dynamic_test)::input(vector)::ffacceleration"),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.newtonEulerSINTERN.getName().c_str(),"sotDynamic(sot_dynamic_test)::intern(dummy)::newtoneuler" ),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.zmpSOUT.getName().c_str(),"sotDynamic(sot_dynamic_test)::output(vector)::zmp" ),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.JcomSOUT.getName().c_str(),"sotDynamic(sot_dynamic_test)::output(matrix)::Jcom" ),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.comSOUT.getName().c_str(),"sotDynamic(sot_dynamic_test)::output(vector)::com" ),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.inertiaSOUT.getName().c_str(),"sotDynamic(sot_dynamic_test)::output(matrix)::inertia" ),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.footHeightSOUT.getName().c_str(),"sotDynamic(sot_dynamic_test)::output(double)::footHeight" ),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.upperJlSOUT.getName().c_str(),"sotDynamic(sot_dynamic_test)::output(vector)::upperJl" ),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.lowerJlSOUT.getName().c_str(),"sotDynamic(sot_dynamic_test)::output(vector)::lowerJl" ),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.upperVlSOUT.getName().c_str(),"sotDynamic(sot_dynamic_test)::output(vector)::upperVl" ),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.upperTlSOUT.getName().c_str(),"sotDynamic(sot_dynamic_test)::output(vector)::upperTl" ),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.inertiaRotorSOUT.getName().c_str(),"sotDynamic(sot_dynamic_test)::output(matrix)::inertiaRotor" ),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.MomentaSOUT.getName().c_str(),"sotDynamic(sot_dynamic_test)::output(vector)::momenta" ),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.AngularMomentumSOUT.getName().c_str(),"sotDynamic(sot_dynamic_test)::output(vector)::angularmomentum" ),0);
  BOOST_CHECK_EQUAL(std::strcmp(dynamic_.dynamicDriftSOUT.getName().c_str(),"sotDynamic(sot_dynamic_test)::output(vector)::dynamicDrift" ),0);
}
