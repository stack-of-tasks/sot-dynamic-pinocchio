/*
 * Copyright 2019,  CNRS
 * Author: Olivier Stasse
 *
 * Please check LICENSE.txt for licensing
 *
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#define ENABLE_RT_LOG

#include "sot-dynamic-pinocchio/state-integrator.h"
#include <sot/core/debug.hh>
using namespace std;

#include <dynamic-graph/factory.h>
#include <dynamic-graph/real-time-logger.h>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/matrix-geometry.hh>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

#define DBGFILE "/tmp/state_integrator.txt"

#if 0
#define RESETDEBUG5() { std::ofstream DebugFile;  \
    DebugFile.open(DBGFILE,std::ofstream::out);   \
    DebugFile.close();}
#define ODEBUG5FULL(x) { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::app);   \
    DebugFile << __FILE__ << ":"      \
        << __FUNCTION__ << "(#"     \
        << __LINE__ << "):" << x << std::endl;  \
    DebugFile.close();}
#define ODEBUG5(x) { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::app); \
    DebugFile << x << std::endl;    \
    DebugFile.close();}

#else
// Void the macro
#define RESETDEBUG5()
#define ODEBUG5FULL(x)
#define ODEBUG5(x)
#endif

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(StateIntegrator, "StateIntegrator");
//const std::string StateIntegrator::CLASS_NAME = "StateIntegrator";
const double StateIntegrator::TIMESTEP_DEFAULT = 0.001;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

StateIntegrator::StateIntegrator( const std::string& n )
  : Entity(n)
  , timestep_(TIMESTEP_DEFAULT)
  , position_(6)
  , ffPose_(6)
  , sanityCheck_(true)
  , controlSIN( NULL, "StateIntegrator(" + n + ")::input(double)::control" )
  , stateSOUT_(boost::bind(&StateIntegrator::getPosition,this,_1,_2),
    controlSIN,
    "StateIntegrator(" + n + ")::output(vector)::state" )
  , velocitySOUT_(boost::bind(&StateIntegrator::getVelocity,this,_1,_2),
    controlSIN,
    "StateIntegrator(" + n + ")::output(vector)::velocity"  )
  , freeFlyerPositionOdomSOUT_(boost::bind(&StateIntegrator::getFreeFlyerPosition,this,_1,_2),
    controlSIN,
    "StateIntegrator(" + n + ")::output(vector)::freeFlyerPositionOdom")
  , freeFlyerVelocitySOUT_(boost::bind(&StateIntegrator::getFreeFlyerVelocity,this,_1,_2),
    controlSIN,
    "StateIntegrator(" + n + ")::output(vector)::freeFlyerVelocity")
  , debug_mode_(5)
  , last_integration_(-1)
{
  signalRegistration( controlSIN
                      << stateSOUT_
                      << velocitySOUT_
                      << freeFlyerPositionOdomSOUT_
                      << freeFlyerVelocitySOUT_);
  position_.fill(.0);

  velocity_.resize(position_.size());
  velocity_.setZero();

  ffPose_.fill(.0);

  ffVel_.resize(ffPose_.size());
  ffVel_.setZero();

  /* --- Commands --- */
  {
    std::string docstring;

    docstring =
      "\n"
      "    Set integration timestep value\n"
      "\n";
    addCommand("init",
               new command::Setter<StateIntegrator, double>(*this, &StateIntegrator::init, docstring));

    docstring =
      "\n"
      "    Set state vector value\n"
      "\n";
    addCommand("set",
               new command::Setter<StateIntegrator, Vector>(*this, &StateIntegrator::setState, docstring));

    docstring =
      "\n"
      "    Set velocity vector value\n"
      "\n";
    addCommand("setVelocity",
               new command::Setter<StateIntegrator, Vector>(*this, &StateIntegrator::setVelocity, docstring));

    docstring =
      "\n"
      "    Set control type vector value (for each joint).\n"
      "    Vector of types (int): qVEL:0 | qACC:1 | ffVEL:2 | ffACC:3\n"
      "\n";
    addCommand("setControlType",
               new command::Setter<StateIntegrator, Vector>(*this, &StateIntegrator::setControlTypeInt, docstring));
    
    void(StateIntegrator::*setRootPtr)(const Matrix&) = &StateIntegrator::setRoot;
    docstring = command::docCommandVoid1("Set the root position.",
                                         "matrix homogeneous");
    addCommand("setRoot", command::makeCommandVoid1(*this, setRootPtr, docstring));

    docstring =
      "\n"
      "    Enable/Disable sanity checks\n"
      "\n";
    addCommand("setSanityCheck",
               new command::Setter<StateIntegrator, bool>
               (*this, &StateIntegrator::setSanityCheck, docstring));
  }
}

StateIntegrator::~StateIntegrator( ) {
  return;
}

void StateIntegrator::init(const double& step){
  timestep_ = step;
}

void StateIntegrator::integrateRollPitchYaw(Vector& state, const Vector& control, double dt) 
{
  using Eigen::AngleAxisd;
  using Eigen::Vector3d;
  using Eigen::QuaternionMapd;

  typedef pinocchio::SpecialEuclideanOperationTpl<3, double> SE3;
  Eigen::Matrix<double, 7, 1> qin, qout;
  qin.head<3>() = state.head<3>();

  QuaternionMapd quat (qin.tail<4>().data());
  quat = AngleAxisd(state(5), Vector3d::UnitZ())
         * AngleAxisd(state(4), Vector3d::UnitY())
         * AngleAxisd(state(3), Vector3d::UnitX());

  SE3().integrate (qin, control*dt, qout);

  // Update freeflyer state (pose|vel)
  state.head<3>() = qout.head<3>();
  state.segment<3>(3) = QuaternionMapd(qout.tail<4>().data()).toRotationMatrix().eulerAngles(0, 1, 2);
}

const MatrixHomogeneous& StateIntegrator::freeFlyerPose() 
{
  using Eigen::AngleAxisd;
  using Eigen::Vector3d;
  using Eigen::Quaterniond;

  freeFlyerPose_.translation() = ffPose_.head<3>();
  Quaterniond quat;
  quat = AngleAxisd(ffPose_(5), Vector3d::UnitZ())
         * AngleAxisd(ffPose_(4), Vector3d::UnitY())
         * AngleAxisd(ffPose_(3), Vector3d::UnitX());
  freeFlyerPose_.linear() = quat.normalized().toRotationMatrix();

  return freeFlyerPose_;
}

void StateIntegrator::setControlType(const StringVector& controlTypeVector) 
{
  controlTypeVector_ = controlTypeVector;
}

void StateIntegrator::setControlTypeInt(const Vector& controlTypeVector) 
{
  std::string type;
  controlTypeVector_.resize(controlTypeVector.size());

  for (unsigned int i=0; i<controlTypeVector.size(); i++){
    try 
    {
      type = SoTControlType_s[int(controlTypeVector[i])];
    }
    catch (...)
    {
      dgRTLOG () << "StateIntegrator::setControlTypeInt: The controlType at position "
                 << i << " is not valid: " << controlTypeVector[i] << "\n"
                 << "Expected values are (int): qVEL = 0 | qACC = 1 | ffVEL = 2 | ffACC = 3"
                 << '\n';
    }
    controlTypeVector_[i] = type;
  }
}

void StateIntegrator::setURDFModel(const std::string &aURDFModel) 
{
  pinocchio::urdf::buildModelFromXML(aURDFModel, model_, false);
  position_.resize(model_.nq);
  velocity_.resize(model_.nv);
  acceleration_.resize(model_.nv);
}

void StateIntegrator::setState( const Vector& st ) 
{
  if (st.size() == model_.nq)
  {
    position_ = st;
  }
  else 
  {
    position_ = st.tail(model_.nq);
    int size = int(st.size() - model_.nq);
    ffPose_ = st.head(size);
  }  
}

void StateIntegrator::setVelocity( const Vector& vel ) 
{
  if (vel.size() == model_.nv)
  {
    velocity_ = vel;
  }
  else 
  {
    velocity_ = vel.tail(model_.nv);
    int size = int(vel.size() - model_.nq);
    ffVel_ = vel.head(size);
  }
}

void StateIntegrator::setRoot( const Matrix & root ) 
{
  Eigen::Matrix4d _matrix4d(root);
  MatrixHomogeneous _root(_matrix4d);
  setRoot( _root );
}

void StateIntegrator::setRoot( const MatrixHomogeneous & worldMwaist ) 
{
  freeFlyerPose_ = worldMwaist;
  ffPose_.head<3>() = worldMwaist.translation();
  ffPose_.segment<3>(3) = worldMwaist.linear().eulerAngles(0,1,2);
}

void StateIntegrator::setSanityCheck(const bool & enableCheck) 
{
  sanityCheck_ = enableCheck;
}

int StateIntegrator::getControlType(const std::string &strCtrlType, SoTControlType &aCtrlType) 
{
  for (int j = 0; j < 4; j++) 
  {
    if (strCtrlType==SoTControlType_s[j])
    {
      aCtrlType = (SoTControlType)j;
      return 0;
    }
  }
  if (debug_mode_ > 1)
  {
    std::cerr << "Control type not allowed/recognized: "
              << strCtrlType 
              << "\n Authorized control types: "
              << "qVEL | qACC | ffVEL | ffACC"
              << std::endl;
  }
  return 1;
}

void StateIntegrator::integrate(int t, const double & dt) 
{
  controlSIN(t);
  const Vector & controlIN = controlSIN.accessCopy();

  if (controlTypeVector_.size() == 0){
    dgRTLOG () << "StateIntegrator::integrate: The controlType vector cannot be empty" << '\n';
    return;
  }

  if (sanityCheck_ && controlIN.hasNaN()) 
  {
    dgRTLOG () << "StateIntegrator::integrate: Control has NaN values: " << '\n'
               << controlIN.transpose() << '\n';
    return;
  }

  // Integration 
  unsigned int indexControlVector = 0, indexComputedVector = 0;
  for (unsigned int indexTypeVector = 0; indexTypeVector<controlTypeVector_.size(); indexTypeVector++) 
  {
    SoTControlType type;    
    if (getControlType(controlTypeVector_[indexTypeVector], type) > 0) 
    {
      if (debug_mode_ > 1)
      {
        std::cerr << "No control type for joint at position " << indexTypeVector
                  << " in the controlType vector"
                  << std::endl;
      }
      break;
    }

    // Control of the freeflyer in acceleration
    if (type == ffACC) 
    {
      // Set controlFreeFlyer from control (size 6 because derivative of twist)
      Vector controlFreeFlyer = controlIN.segment<6>(indexControlVector);
      // Integrate once to obtain velocity -> update ffVel_ 
      integrateRollPitchYaw(ffVel_, controlFreeFlyer, dt);
      indexControlVector = indexControlVector + 6;
    }
    // Control of the freeflyer in velocity    
    else if (type == ffVEL)
    {
      // Set ffVel_ (twist) from control for the integration in position 
      ffVel_ = controlIN.segment<6>(indexControlVector);
      indexControlVector = indexControlVector + 6;
    }

    // Control of a joint in acceleration    
    else if (type == qACC) 
    {
      // Set acceleration from control and integrate to find velocity.
      acceleration_[indexComputedVector] = controlIN[indexControlVector];
      velocity_[indexComputedVector] = velocity_[indexComputedVector] + acceleration_[indexComputedVector] * (0.5)*dt;
      indexControlVector++;
    }
    // Control of a joint in velocity    
    else if (type == qVEL) 
    {
      // Set velocity from control.
      acceleration_[indexComputedVector] = 0;
      velocity_[indexComputedVector] = controlIN[indexControlVector];
      indexControlVector++;
    }

    // Velocity integration
    if (type == ffVEL || type == ffACC){
      // Integrate freeflyer velocity to obtain position -> update ffPose_ from ffVel_
      integrateRollPitchYaw(ffPose_, ffVel_, dt);
    }
    else if (type == qVEL || type == qACC){
      // Velocity integration of the joint to get position
      position_[indexComputedVector] = position_[indexComputedVector] + velocity_[indexComputedVector] * dt;
      indexComputedVector++;
    }
  }  
}


/* --- DISPLAY ------------------------------------------------------------ */

void StateIntegrator::display ( std::ostream& os ) const
{
  os << name << ": " << position_ << endl; 
}


Vector& StateIntegrator::getPosition(Vector &controlOut, const int& t) 
{
  ODEBUG5FULL("start");
  sotDEBUGIN(25) ;

  // Integrate control
  if (last_integration_ != t){
    integrate(t, timestep_);
    last_integration_ = t;
  }
  sotDEBUG (25) << "position_ = " << position_ << std::endl;

  ODEBUG5FULL("position_ = " << position_);

  controlOut = position_;

  return controlOut;

  ODEBUG5FULL("end");
  sotDEBUGOUT(25) ;
}

Vector& StateIntegrator::getVelocity(Vector &controlOut, const int& t) 
{
  ODEBUG5FULL("start");
  sotDEBUGIN(25) ;

  // Integrate control
  if (last_integration_ != t){
    integrate(t, timestep_);
    last_integration_ = t;
  }
  sotDEBUG (25) << "velocity_ = " << velocity_ << std::endl;

  ODEBUG5FULL("velocity_ = " << velocity_);

  controlOut = velocity_;

  return controlOut;

  ODEBUG5FULL("end");
  sotDEBUGOUT(25) ;
}

Vector& StateIntegrator::getFreeFlyerPosition(Vector &ffPose, const int& t) 
{
  ODEBUG5FULL("start");
  sotDEBUGIN(25);

  // Integrate control
  if (last_integration_ != t){
    integrate(t, timestep_);
    last_integration_ = t;
  }
  sotDEBUG (25) << "ffPose_ = " << ffPose_ << std::endl;

  ODEBUG5FULL("ffPose_ = " << ffPose_);

  ffPose = ffPose_;

  return ffPose;

  ODEBUG5FULL("end");
  sotDEBUGOUT(25) ;
}

Vector& StateIntegrator::getFreeFlyerVelocity(Vector &ffVel, const int& t) 
{
  ODEBUG5FULL("start");
  sotDEBUGIN(25);

  // Integrate control
  if (last_integration_ != t){
    integrate(t, timestep_);
    last_integration_ = t;
  }
  sotDEBUG (25) << "ffVel_ = " << ffVel_ << std::endl;

  ODEBUG5FULL("ffVel_ = " << ffVel_);

  ffVel = ffVel_;

  return ffVel;

  ODEBUG5FULL("end");
  sotDEBUGOUT(25);
}