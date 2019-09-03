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
  , controlTypeFF_("")
  , ffPose_(6)
  , sanityCheck_(true)
  , controlSIN( NULL, "StateIntegrator(" + n + ")::input(double)::control" )
  , freeFlyerSIN( NULL, "StateIntegrator(" + n + ")::input(double)::freeFlyer" )
  , stateSOUT_(boost::bind(&StateIntegrator::getPosition, this, _1, _2),
               controlSIN,
               "StateIntegrator(" + n + ")::output(vector)::state" )
  , velocitySOUT_(boost::bind(&StateIntegrator::getVelocity, this, _1, _2),
                  controlSIN,
                  "StateIntegrator(" + n + ")::output(vector)::velocity"  )
  , freeFlyerPositionEulerSOUT_(boost::bind(&StateIntegrator::getFreeFlyerPositionEuler, this, _1, _2),
                                freeFlyerSIN,
                                "StateIntegrator(" + n + ")::output(vector)::freeFlyerPositionEuler")
  , freeFlyerPositionQuatSOUT_(boost::bind(&StateIntegrator::getFreeFlyerPositionQuat, this, _1, _2),
                               freeFlyerSIN,
                               "StateIntegrator(" + n + ")::output(vector)::freeFlyerPositionQuat")
  , freeFlyerVelocitySOUT_(boost::bind(&StateIntegrator::getFreeFlyerVelocity, this, _1, _2),
                           freeFlyerSIN,
                           "StateIntegrator(" + n + ")::output(vector)::freeFlyerVelocity")
  , debug_mode_(5)
  , last_integration_(-1)
  , last_integration_FF_(-1) {
  signalRegistration( controlSIN
                      << freeFlyerSIN
                      << stateSOUT_
                      << velocitySOUT_
                      << freeFlyerPositionEulerSOUT_
                      << freeFlyerPositionQuatSOUT_
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
    addCommand("setState",
               new command::Setter<StateIntegrator, Vector>(*this, &StateIntegrator::setState, docstring));

    docstring =
      "\n"
      "    Set velocity vector value\n"
      "\n";
    addCommand("setVelocity",
               new command::Setter<StateIntegrator, Vector>(*this, &StateIntegrator::setVelocity, docstring));

    docstring =
      "\n"
      "    Set Freeflyer state vector value\n"
      "\n";
    addCommand("setStateFreeflyer",
               new command::Setter<StateIntegrator, Vector>(*this, &StateIntegrator::setStateFreeflyer, docstring));

    docstring =
      "\n"
      "    Set Freeflyer velocity vector value\n"
      "\n";
    addCommand("setVelocityFreeflyer",
               new command::Setter<StateIntegrator, Vector>(*this, &StateIntegrator::setVelocityFreeflyer, docstring));

    docstring =
      "\n"
      "    Set control type vector value (for each joint).\n"
      "    Vector of types (int): qVEL:0 | qACC:1 \n"
      "\n";
    addCommand("setControlType",
               new command::Setter<StateIntegrator, Vector>(*this, &StateIntegrator::setControlTypeInt, docstring));
    docstring =
      "\n"
      "    Set control type value (for freeflyer).\n"
      "    type (string): ffVEL | ffACC\n"
      "\n";
    addCommand("setControlTypeFreeFlyer",
               new command::Setter<StateIntegrator, std::string>(*this, &StateIntegrator::setControlTypeFreeFlyer, docstring));

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

void StateIntegrator::init(const double& step) {
  timestep_ = step;
}

void StateIntegrator::integrateRollPitchYaw(Vector& state, const Vector& control, double dt) {
  using Eigen::AngleAxisd;
  using Eigen::Vector3d;
  using Eigen::Matrix3d;
  using Eigen::QuaternionMapd;

  typedef pinocchio::SpecialEuclideanOperationTpl<3, double> SE3;
  Eigen::Matrix<double, 7, 1> qin, qout;
  qin.head<3>() = state.head<3>();

  QuaternionMapd quat (qin.tail<4>().data());
  quat = AngleAxisd(state(5), Vector3d::UnitZ())
         * AngleAxisd(state(4), Vector3d::UnitY())
         * AngleAxisd(state(3), Vector3d::UnitX());

  SE3().integrate (qin, control * dt, qout);

  // Create the Euler angles in good range : [-pi:pi]x[-pi/2:pi/2]x[-pi:pi]
  Matrix3d rotationMatrix = QuaternionMapd(qout.tail<4>().data()).normalized().toRotationMatrix();
  double m = sqrt(pow(rotationMatrix(2, 1), 2.0) + pow(rotationMatrix(2, 2), 2.0));
  double p = atan2(-rotationMatrix(2, 0), m);
  double r, y;
  if (abs(abs(p) - M_PI / 2) < 0.001) {
    r = 0;
    y = -atan2(rotationMatrix(0, 1), rotationMatrix(1, 1));
  } else {
    y = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));  // alpha
    r = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2)); // gamma
  }
  // Update freeflyer state (pose|vel)
  state.head<3>() = qout.head<3>();
  state.segment<3>(3) << r, p, y;
}

const MatrixHomogeneous& StateIntegrator::freeFlyerPose() {
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

void StateIntegrator::setControlType(const StringVector& controlTypeVector) {
  controlTypeVector_ = controlTypeVector;
}

void StateIntegrator::setControlTypeFreeFlyer(const std::string& controlTypeFF) {
  controlTypeFF_ = controlTypeFF;
}

void StateIntegrator::setControlTypeInt(const Vector& controlTypeVector) {
  std::string type;
  controlTypeVector_.resize(controlTypeVector.size());

  for (unsigned int i = 0; i < controlTypeVector.size(); i++) {
    try {
      type = SoTControlType_s[int(controlTypeVector[i])];
    } catch (...) {
      dgRTLOG () << "StateIntegrator::setControlTypeInt: The controlType at position "
                 << i << " is not valid: " << controlTypeVector[i] << "\n"
                 << "Expected values are (int): qVEL = 0 | qACC = 1 | ffVEL = 2 | ffACC = 3"
                 << '\n';
    }
    controlTypeVector_[i] = type;
  }
}

void StateIntegrator::setURDFModel(const std::string &aURDFModel) {
  pinocchio::urdf::buildModelFromXML(aURDFModel, model_, false);
  position_.resize(model_.nq);
  velocity_.resize(model_.nv);
  acceleration_.resize(model_.nv);
}

void StateIntegrator::setState( const Vector& st ) {
  if (st.size() == model_.nq) {
    position_ = st;
  } else {
    std::cerr << "Asked size of State vector: " << st.size()
              << " not the expected one: " << model_.nq << std::endl;
  }
}

void StateIntegrator::setVelocity( const Vector& vel ) {
  if (vel.size() == model_.nv) {
    velocity_ = vel;
  } else {
    std::cerr << "Asked size of Velocity vector: " << vel.size()
              << " not the expected one: " << model_.nv << std::endl;
  }
}

void StateIntegrator::setStateFreeflyer( const Vector& st ) {
  if (st.size() == 6 || st.size() == 7) {
    ffPose_ = st;
  } else {
    std::cerr << "Asked size of State Freeflyer vector: " << st.size()
              << " not the expected one: 6 (euler angle) or 7 (quaternion)" << std::endl;
  }
}

void StateIntegrator::setVelocityFreeflyer( const Vector& vel ) {
  if (vel.size() == 6) {
    ffVel_ = vel;
  } else {
    std::cerr << "Asked size of velocity Freeflyer vector: " << vel.size()
              << " not the expected one: 6 (twist)" << std::endl;
  }
}

void StateIntegrator::setRoot( const Matrix & root ) {
  Eigen::Matrix4d _matrix4d(root);
  MatrixHomogeneous _root(_matrix4d);
  setRoot( _root );
}

void StateIntegrator::setRoot( const MatrixHomogeneous & worldMwaist ) {
  freeFlyerPose_ = worldMwaist;
  ffPose_.head<3>() = worldMwaist.translation();
  ffPose_.segment<3>(3) = worldMwaist.linear().eulerAngles(0, 1, 2);
}

void StateIntegrator::setSanityCheck(const bool & enableCheck) {
  sanityCheck_ = enableCheck;
}

int StateIntegrator::getControlType(const std::string &strCtrlType, SoTControlType &aCtrlType) {
  for (int j = 0; j < 4; j++) {
    if (strCtrlType == SoTControlType_s[j]) {
      aCtrlType = (SoTControlType)j;
      return 0;
    }
  }
  if (debug_mode_ > 1) {
    std::cerr << "Control type not allowed/recognized: "
              << strCtrlType
              << "\n Authorized control types: "
              << "qVEL | qACC | ffVEL | ffACC"
              << std::endl;
  }
  return 1;
}

void StateIntegrator::integrateControl(int t, const double & dt) {
  controlSIN(t);
  const Vector & controlIN = controlSIN.accessCopy();

  if (controlTypeVector_.size() == 0) {
    dgRTLOG () << "StateIntegrator::integrate: The controlType vector cannot be empty" << '\n';
    return;
  }

  if (sanityCheck_ && controlIN.hasNaN()) {
    dgRTLOG () << "StateIntegrator::integrate: Control has NaN values: " << '\n'
               << controlIN.transpose() << '\n';
    return;
  }
  // Integration joints
  for (unsigned int j = 0; j < controlTypeVector_.size(); j++) {
    SoTControlType type;
    if (getControlType(controlTypeVector_[j], type) > 0) {
      if (debug_mode_ > 1) {
        std::cerr << "No control type for joint at position " << j
                  << " in the controlType vector"
                  << std::endl;
      }
      break;
    }
    // Control of a joint in acceleration
    if (type == qACC) {
      // Set acceleration from control and integrate to find velocity.
      acceleration_[j] = controlIN[j];
      velocity_[j] = velocity_[j] + acceleration_[j] * (0.5) * dt;
    }
    // Control of a joint in velocity
    else if (type == qVEL) {
      // Set velocity from control.
      acceleration_[j] = 0;
      velocity_[j] = controlIN[j];
    }

    // Velocity integration of the joint to get position
    position_[j] = position_[j] + velocity_[j] * dt;
  }
}

void StateIntegrator::integrateFreeFlyer(int t, const double & dt) {
  freeFlyerSIN(t);
  const Vector & freeFlyerIN = freeFlyerSIN.accessCopy();

  if (controlTypeFF_ == "") {
    dgRTLOG () << "StateIntegrator::integrate: The controlType of the freeflyer cannot be empty" << '\n';
    return;
  }

  if (sanityCheck_ && freeFlyerIN.hasNaN()) {
    dgRTLOG () << "StateIntegrator::integrate: Control of the FreeFlyer has NaN values: " << '\n'
               << freeFlyerIN.transpose() << '\n';
    return;
  }

  SoTControlType typeFF;
  if (getControlType(controlTypeFF_, typeFF) > 0) {
    if (debug_mode_ > 1) {
      std::cerr << "No control type for the Freeflyer " << std::endl;
    }
    return;
  }
  // Control of the freeflyer in acceleration
  if (typeFF == ffACC) {
    // Integrate once to obtain velocity -> update ffVel_
    integrateRollPitchYaw(ffVel_, freeFlyerIN, dt);
  }
  // Control of the freeflyer in velocity
  else if (typeFF == ffVEL) {
    // Set ffVel_ (twist) from control for the integration in position
    ffVel_ = freeFlyerIN;
  }
  // Integrate freeflyer velocity to obtain position -> update ffPose_ from ffVel_
  integrateRollPitchYaw(ffPose_, ffVel_, dt);
}

/* --- DISPLAY ------------------------------------------------------------ */

void StateIntegrator::display ( std::ostream& os ) const {
  os << name << ": " << position_ << endl;
}


Vector& StateIntegrator::getPosition(Vector &controlOut, const int& t) {
  ODEBUG5FULL("start");
  sotDEBUGIN(25) ;

  // Integrate control
  if (last_integration_ != t) {
    integrateControl(t, timestep_);
    last_integration_ = t;
  }
  sotDEBUG (25) << "position_ = " << position_ << std::endl;

  ODEBUG5FULL("position_ = " << position_);

  controlOut = position_;

  return controlOut;

  ODEBUG5FULL("end");
  sotDEBUGOUT(25) ;
}

Vector& StateIntegrator::getVelocity(Vector &controlOut, const int& t) {
  ODEBUG5FULL("start");
  sotDEBUGIN(25) ;

  // Integrate control
  if (last_integration_ != t) {
    integrateControl(t, timestep_);
    last_integration_ = t;
  }
  sotDEBUG (25) << "velocity_ = " << velocity_ << std::endl;

  ODEBUG5FULL("velocity_ = " << velocity_);

  controlOut = velocity_;

  return controlOut;

  ODEBUG5FULL("end");
  sotDEBUGOUT(25) ;
}

Vector& StateIntegrator::getFreeFlyerPositionEuler(Vector &ffPose, const int& t) {
  ODEBUG5FULL("start");
  sotDEBUGIN(25);

  // Integrate control
  if (last_integration_FF_ != t) {
    integrateFreeFlyer(t, timestep_);
    last_integration_FF_ = t;
  }
  sotDEBUG (25) << "ffPose_ = " << ffPose_ << std::endl;

  ODEBUG5FULL("ffPose_ = " << ffPose_);

  ffPose = ffPose_;

  return ffPose;

  ODEBUG5FULL("end");
  sotDEBUGOUT(25) ;
}

Vector& StateIntegrator::getFreeFlyerPositionQuat(Vector &ffPose, const int& t) {
  using Eigen::Quaterniond;
  using Eigen::AngleAxisd;
  using Eigen::Vector3d;
  ODEBUG5FULL("start");
  sotDEBUGIN(25);

  ffPose.resize(7);
  // Integrate control
  if (last_integration_FF_ != t) {
    integrateFreeFlyer(t, timestep_);
    last_integration_FF_ = t;
  }

  ffPose.head<3>() = ffPose_.head<3>();
  Quaterniond quat;
  quat = AngleAxisd(ffPose_(5), Vector3d::UnitZ())
         * AngleAxisd(ffPose_(4), Vector3d::UnitY())
         * AngleAxisd(ffPose_(3), Vector3d::UnitX());
  ffPose.segment<4>(3) = quat.normalized().coeffs();

  sotDEBUG (25) << "ffPose = " << ffPose << std::endl;

  ODEBUG5FULL("ffPose = " << ffPose);

  return ffPose;

  ODEBUG5FULL("end");
  sotDEBUGOUT(25) ;
}

Vector& StateIntegrator::getFreeFlyerVelocity(Vector &ffVel, const int& t) {
  ODEBUG5FULL("start");
  sotDEBUGIN(25);

  // Integrate control
  if (last_integration_FF_ != t) {
    integrateFreeFlyer(t, timestep_);
    last_integration_FF_ = t;
  }
  sotDEBUG (25) << "ffVel_ = " << ffVel_ << std::endl;

  ODEBUG5FULL("ffVel_ = " << ffVel_);

  ffVel = ffVel_;

  return ffVel;

  ODEBUG5FULL("end");
  sotDEBUGOUT(25);
}