/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_DYNAMIC_PINOCCHIO_H__
#define __SOT_DYNAMIC_PINOCCHIO_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* STD */
#include <map>
#include <memory>
#include <string>

/* pinocchio */

#include <pinocchio/fwd.hpp>

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

#include <sot/core/exception-dynamic.hh>
#include <sot/core/flags.hh>
#include <sot/core/matrix-geometry.hh>
/* Matrix */
#include <dynamic-graph/linear-algebra.h>

#include <sot/dynamic-pinocchio/deprecated.hh>

/* PINOCCHIO */
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/macros.hpp>
#include <pinocchio/multibody/model.hpp>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(dynamic_EXPORTS)
#define SOTDYNAMIC_EXPORT __declspec(dllexport)
#else
#define SOTDYNAMIC_EXPORT __declspec(dllimport)
#endif
#else
#define SOTDYNAMIC_EXPORT
#endif

namespace dynamicgraph {
namespace sot {
namespace dg = dynamicgraph;

namespace command {
class SetFile;
class CreateOpPoint;
}  // namespace command
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/*! @ingroup signals
  \brief This class provides an inverse dynamic model of the robot.
  More precisely it wraps the newton euler algorithm implemented
  by the dynamicsJRLJapan library to make it accessible in the stack of tasks.
  The robot is described by a VRML file.
*/
class SOTDYNAMIC_EXPORT DynamicPinocchio : public dg::Entity {
  friend class sot::command::SetFile;
  friend class sot::command::CreateOpPoint;
  //  friend class sot::command::InitializeRobot;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DYNAMIC_GRAPH_ENTITY_DECL();

  /*  --- MODEL ATRIBUTES --- */
  pinocchio::Model* m_model;
  std::unique_ptr<pinocchio::Data> m_data;

  /*  --- MODEL ATRIBUTES --- */

 public:
  /* --- SIGNAL ACTIVATION --- */
  dg::SignalTimeDependent<dg::Matrix, sigtime_t>& createEndeffJacobianSignal(
      const std::string& signame, const std::string&,
      const bool isLocal = true);
  dg::SignalTimeDependent<dg::Matrix, sigtime_t>& createJacobianSignal(
      const std::string& signame, const std::string&);
  void destroyJacobianSignal(const std::string& signame);

  dg::SignalTimeDependent<MatrixHomogeneous, sigtime_t>& createPositionSignal(
      const std::string&, const std::string&);
  void destroyPositionSignal(const std::string& signame);

  dg::SignalTimeDependent<dg::Vector, sigtime_t>& createVelocitySignal(
      const std::string&, const std::string&);
  void destroyVelocitySignal(const std::string& signame);

  dg::SignalTimeDependent<dg::Vector, sigtime_t>& createAccelerationSignal(
      const std::string&, const std::string&);
  void destroyAccelerationSignal(const std::string& signame);

  /*! @} */
  std::list<dg::SignalBase<sigtime_t>*> genericSignalRefs;

 public:
  /* --- SIGNAL --- */
  typedef int Dummy;
  dg::SignalPtr<dg::Vector, sigtime_t> jointPositionSIN;
  dg::SignalPtr<dg::Vector, sigtime_t> freeFlyerPositionSIN;
  dg::SignalPtr<dg::Vector, sigtime_t> jointVelocitySIN;
  dg::SignalPtr<dg::Vector, sigtime_t> freeFlyerVelocitySIN;
  dg::SignalPtr<dg::Vector, sigtime_t> jointAccelerationSIN;
  dg::SignalPtr<dg::Vector, sigtime_t> freeFlyerAccelerationSIN;

  dg::SignalTimeDependent<dg::Vector, sigtime_t> pinocchioPosSINTERN;
  dg::SignalTimeDependent<dg::Vector, sigtime_t> pinocchioVelSINTERN;
  dg::SignalTimeDependent<dg::Vector, sigtime_t> pinocchioAccSINTERN;

  dg::SignalTimeDependent<Dummy, sigtime_t> newtonEulerSINTERN;
  dg::SignalTimeDependent<Dummy, sigtime_t> jacobiansSINTERN;
  dg::SignalTimeDependent<Dummy, sigtime_t> forwardKinematicsSINTERN;
  dg::SignalTimeDependent<Dummy, sigtime_t> ccrbaSINTERN;

  int& computeNewtonEuler(int& dummy, const sigtime_t& time);
  int& computeForwardKinematics(int& dummy, const sigtime_t& time);
  int& computeCcrba(int& dummy, const sigtime_t& time);
  int& computeJacobians(int& dummy, const sigtime_t& time);

  dg::SignalTimeDependent<dg::Vector, sigtime_t> zmpSOUT;
  dg::SignalTimeDependent<dg::Matrix, sigtime_t> JcomSOUT;
  dg::SignalTimeDependent<dg::Vector, sigtime_t> comSOUT;
  dg::SignalTimeDependent<dg::Matrix, sigtime_t> inertiaSOUT;

  dg::SignalTimeDependent<dg::Matrix, sigtime_t>& jacobiansSOUT(
      const std::string& name);
  dg::SignalTimeDependent<MatrixHomogeneous, sigtime_t>& positionsSOUT(
      const std::string& name);
  dg::SignalTimeDependent<dg::Vector, sigtime_t>& velocitiesSOUT(
      const std::string& name);
  dg::SignalTimeDependent<dg::Vector, sigtime_t>& accelerationsSOUT(
      const std::string& name);

  dg::SignalTimeDependent<double, sigtime_t> footHeightSOUT;
  dg::SignalTimeDependent<dg::Vector, sigtime_t> upperJlSOUT;
  dg::SignalTimeDependent<dg::Vector, sigtime_t> lowerJlSOUT;
  dg::SignalTimeDependent<dg::Vector, sigtime_t> upperVlSOUT;
  dg::SignalTimeDependent<dg::Vector, sigtime_t> upperTlSOUT;

  dg::Signal<dg::Vector, sigtime_t> inertiaRotorSOUT;
  dg::Signal<dg::Vector, sigtime_t> gearRatioSOUT;
  dg::SignalTimeDependent<dg::Matrix, sigtime_t> inertiaRealSOUT;
  dg::SignalTimeDependent<dg::Vector, sigtime_t> MomentaSOUT;
  dg::SignalTimeDependent<dg::Vector, sigtime_t> AngularMomentumSOUT;
  dg::SignalTimeDependent<dg::Vector, sigtime_t> dynamicDriftSOUT;

 public:
  /* --- CONSTRUCTOR --- */
  DynamicPinocchio(const std::string& name);
  virtual ~DynamicPinocchio(void);

  /* --- MODEL CREATION --- */

  void displayModel() const {
    assert(m_model);
    std::cout << (*m_model) << std::endl;
  };

  void setModel(pinocchio::Model*);

  void createData();

  pinocchio::Model* getModel() { return m_model; };

  pinocchio::Data* getData() { return m_data.get(); };

  /* --- GETTERS --- */

  /// \brief Get joint position lower limits
  ///
  /// \param[out] result vector
  /// \return result vector
  dg::Vector& getLowerPositionLimits(dg::Vector& res, const sigtime_t&) const;

  /// \brief Get joint position upper limits
  ///
  /// \param[out] result vector
  /// \return result vector
  dg::Vector& getUpperPositionLimits(dg::Vector& res, const sigtime_t&) const;

  /// \brief Get joint velocity upper limits
  ///
  /// \param[out] result vector
  /// \return result vector
  dg::Vector& getUpperVelocityLimits(dg::Vector& res, const sigtime_t&) const;

  /// \brief Get joint effort upper limits
  ///
  /// \param[out] result vector
  /// \return result vector
  dg::Vector& getMaxEffortLimits(dg::Vector& res, const sigtime_t&) const;

  //  dg::Vector& getAnklePositionInFootFrame() const;

 protected:
  dg::Matrix& computeGenericJacobian(const bool isFrame, const int jointId,
                                     dg::Matrix& res, const sigtime_t& time);
  dg::Matrix& computeGenericEndeffJacobian(const bool isFrame,
                                           const bool isLocal,
                                           const int jointId, dg::Matrix& res,
                                           const sigtime_t& time);
  MatrixHomogeneous& computeGenericPosition(const bool isFrame,
                                            const int jointId,
                                            MatrixHomogeneous& res,
                                            const sigtime_t& time);
  dg::Vector& computeGenericVelocity(const int jointId, dg::Vector& res,
                                     const sigtime_t& time);
  dg::Vector& computeGenericAcceleration(const int jointId, dg::Vector& res,
                                         const sigtime_t& time);

  dg::Vector& computeZmp(dg::Vector& res, const sigtime_t& time);
  dg::Vector& computeMomenta(dg::Vector& res, const sigtime_t& time);
  dg::Vector& computeAngularMomentum(dg::Vector& res, const sigtime_t& time);
  dg::Matrix& computeJcom(dg::Matrix& res, const sigtime_t& time);
  dg::Vector& computeCom(dg::Vector& res, const sigtime_t& time);
  dg::Matrix& computeInertia(dg::Matrix& res, const sigtime_t& time);
  dg::Matrix& computeInertiaReal(dg::Matrix& res, const sigtime_t& time);
  double& computeFootHeight(double& res, const sigtime_t& time);

  dg::Vector& computeTorqueDrift(dg::Vector& res, const sigtime_t& time);

 public: /* --- PARAMS --- */
  void cmd_createOpPointSignals(const std::string& sig, const std::string& j);
  void cmd_createJacobianWorldSignal(const std::string& sig,
                                     const std::string& j);
  void cmd_createJacobianEndEffectorSignal(const std::string& sig,
                                           const std::string& j);
  void cmd_createJacobianEndEffectorWorldSignal(const std::string& sig,
                                                const std::string& j);
  void cmd_createPositionSignal(const std::string& sig, const std::string& j);
  void cmd_createVelocitySignal(const std::string& sig, const std::string& j);
  void cmd_createAccelerationSignal(const std::string& sig,
                                    const std::string& j);

 private:
  /// \brief map of joints in construction.
  /// map: jointName -> (jointType,jointPosition (in parent frame), function_ptr
  /// to pinocchio Joint)
  dg::Vector& getPinocchioPos(dg::Vector& q, const sigtime_t& time);
  dg::Vector& getPinocchioVel(dg::Vector& v, const sigtime_t& time);
  dg::Vector& getPinocchioAcc(dg::Vector& a, const sigtime_t& time);

  //\brief Index list for the first dof of (spherical joints)/ (spherical part
  // of free-flyer joint).
  std::vector<int> sphericalJoints;
};

// std::ostream& operator<<(std::ostream& os, const CjrlJoint& r);
} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_DYNAMIC_PINOCCHIO_H__
