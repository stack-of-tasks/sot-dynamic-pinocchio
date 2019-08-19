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

const std::string StateIntegrator::CLASS_NAME = "StateIntegrator";
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
  , controlTypeSIN(NULL, "StateIntegrator(" + n + ")::input(double)::controlType" ) 
  , stateSOUT_( "StateIntegrator(" + n + ")::output(vector)::state" )
  , velocitySOUT_("StateIntegrator(" + n + ")::output(vector)::velocity"  )
  , freeFlyerPositionOdomSOUT_("StateIntegrator(" + n + ")::output(vector)::freeFlyerPositionOdom")
  , freeFlyerVelocitySOUT_("StateIntegrator(" + n + ")::output(vector)::freeFlyerVelocity")
  , debug_mode_(5)
{
  signalRegistration( controlSIN
                      << controlTypeSIN
                      << stateSOUT_
                      << velocitySOUT_
                      << freeFlyerPositionOdomSOUT_
                      << freeFlyerVelocitySOUT_);
  position_.fill(.0);
  stateSOUT_.setConstant( position_ );

  velocity_.resize(position_.size());
  velocity_.setZero();
  velocitySOUT_.setConstant( velocity_ );

  ffPose_.fill(.0);
  freeFlyerPositionOdomSOUT_.setConstant( ffPose_ );

  ffVel_.resize(ffPose_.size());
  ffVel_.setZero();
  freeFlyerVelocitySOUT_.setConstant( ffVel_ );

  /* --- Commands --- */
  {
    std::string docstring;
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

void StateIntegrator::integrateRollPitchYaw(Vector& state, Signal<Vector, int>& signal, const Vector& control, double dt) 
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

  // Update freeflyer pose/vel and signal
  state.head<3>() = qout.head<3>();
  state.segment<3>(3) = QuaternionMapd(qout.tail<4>().data()).toRotationMatrix().eulerAngles(0, 1, 2);
  signal.setConstant(state);
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
  else if (st.size() == model_.nq+6)
  {
    position_ = st.tail(model_.nq);
    ffPose_ = st.head<6>();
    freeFlyerPositionOdomSOUT_.setConstant( ffPose_);
  }  
  stateSOUT_ .setConstant( position_ );
}

void StateIntegrator::setVelocity( const Vector& vel ) 
{
  if (vel.size() == model_.nv)
  {
    velocity_ = vel;
  }
  else if (vel.size() == model_.nv+6)
  {
    velocity_ = vel.tail(model_.nv);
    ffVel_ = vel.head<6>();
    freeFlyerVelocitySOUT_.setConstant( ffVel_ );
  }
  velocitySOUT_ .setConstant( velocity_ );
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
  freeFlyerPositionOdomSOUT_.setConstant( ffPose_);
}

void StateIntegrator::setSanityCheck(const bool & enableCheck) 
{
  sanityCheck_ = enableCheck;
}

int StateIntegrator::getControlType(const int &ctrlType, ControlType &aCtrlType) 
{
  for (int j = 0; j < 2; j++) 
  {
    if (ctrlType == j)
    {
      aCtrlType = (ControlType)j;
      return 0;
    }
  }
  if (debug_mode_ > 1)
  {
    std::cerr << "Control type not allowed/recognized: "
              << ctrlType 
              << "\n Authorized control types: "
              << "VELOCITY = 0 | ACCELERATION = 1"
              << std::endl;
  }
  return 1;
}

void StateIntegrator::integrate( const double & dt ) 
{
  const Vector & controlIN = controlSIN.accessCopy();
  const Vector & controlTypeIN = controlTypeSIN.accessCopy();

  if (controlIN.size() != controlTypeIN.size())
  {
    if (debug_mode_ > 1)
    {
      std::cerr << "Signals controlSIN and controlTypeSIN must have the same size: "
                << "\n controlIN.size(): " << controlIN.size() 
                << "\n controlTypeIN.size(): " << controlTypeIN.size()
                << std::endl;
    }
    return;
  }

  Vector controlFreeFlyer(6), controlFreeFlyerType(6);
  Vector control(model_.nv), controlType(model_.nv);
  bool hasFreeFlyer = false;

  if (sanityCheck_ && controlIN.hasNaN()) 
  {
    dgRTLOG () << "StateIntegrator::integrate: Control has NaN values: " << '\n'
               << controlIN.transpose() << '\n';
    return;
  }
  // Check size to separate freeFlyer
  if (controlIN.size() == model_.nv+6)
  {
    controlFreeFlyer = controlIN.head<6>();
    control = controlIN.tail(model_.nv);
    controlFreeFlyerType = controlTypeIN.head<6>();
    controlType = controlTypeIN.tail(model_.nv);
    hasFreeFlyer = true;
  }
  else
  {
    control = controlIN; 
    controlType = controlTypeIN;
  }

  // Integration of the joints
  for (int i=0;i<control.size(); i++) 
  {
    ControlType type;    
    if (getControlType(int(controlType[i]), type) > 0) 
    {
      if (debug_mode_ > 1)
      {
        std::cerr << "No control type for joint at position " << i
                  << " in the controlSIN vector"
                  << std::endl;
      }
      break;
    }
    // Set acceleration from control and integrates to find velocity.
    if (type == ACCELERATION) 
    {
      acceleration_[i] = control[i];
      velocity_[i] = velocity_[i] + acceleration_[i] * (0.5)*dt;
    }
    // Set velocity from control.
    else if (type == VELOCITY) 
    {
      acceleration_[i] = 0;
      velocity_[i] = control[i];
    }
    // Velocity integration to get position
    position_[i] = position_[i] + velocity_[i] * dt;
  }  

  stateSOUT_ .setConstant( position_ );
  velocitySOUT_ .setConstant( velocity_ );

  // Freeflyer integration 
  if (hasFreeFlyer) 
  {
    ControlType ffType;    
    if (getControlType(int(controlFreeFlyerType[0]), ffType) > 0) 
    {
      if (debug_mode_ > 1)
      {
        std::cerr << "No valid control type for freeflyer in the controlTypeSIN signal"
                  << std::endl;
      }
      return;
    }
    if (ffType == ACCELERATION) 
    {
      // Integrate once to obtain velocity -> update ffVel_ and signal freeFlyerVelocitySOUT_
      integrateRollPitchYaw(ffVel_, freeFlyerVelocitySOUT_, controlFreeFlyer, dt);
    }
    else if (ffType == VELOCITY)
    {
      // Set ffVel_ from control for the integration in position and update freeFlyerVelocitySOUT_ 
      ffVel_ = controlFreeFlyer;
      freeFlyerVelocitySOUT_.setConstant(ffVel_);
    }
    // Integrate to obtain position -> update ffPose_ and signal freeFlyerPositionOdomSOUT_
    integrateRollPitchYaw(ffPose_, freeFlyerPositionOdomSOUT_, ffVel_, dt);
  }
}


/* --- DISPLAY ------------------------------------------------------------ */

void StateIntegrator::display ( std::ostream& os ) const
{
  os << name << ": " << position_ << endl; 
}


void StateIntegrator::getControl(map<string, dgsot::ControlValues> &controlOut) 
{
  ODEBUG5FULL("start");
  sotDEBUGIN(25) ;

  // Integrate control
  integrate(timestep_);
  sotDEBUG (25) << "position_ = " << position_ << std::endl;

  ODEBUG5FULL("position_ = " << position_);

  vector<double> lcontrolOut;
  lcontrolOut.resize(position_.size());
  Eigen::VectorXd::Map(&lcontrolOut[0], position_.size()) = position_;

  controlOut["control"].setValues(lcontrolOut);

  ODEBUG5FULL("end");
  sotDEBUGOUT(25) ;
}

