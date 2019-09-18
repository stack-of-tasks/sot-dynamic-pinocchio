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
#include <string>
#include <map>

/* SOT */
#include <pinocchio/fwd.hpp>
#include <sot/core/flags.hh>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/core/exception-dynamic.hh>
#include <sot/core/matrix-geometry.hh>
/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* PINOCCHIO */
#include <pinocchio/macros.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (dynamic_EXPORTS)
#    define SOTDYNAMIC_EXPORT __declspec(dllexport)
#  else
#    define SOTDYNAMIC_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTDYNAMIC_EXPORT
#endif


namespace dynamicgraph {
  namespace sot {
    namespace dg = dynamicgraph;

    namespace command {
      class SetFile;
      class CreateOpPoint;
    }
    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */



    /*! @ingroup signals
      \brief This class provides an inverse dynamic model of the robot.
      More precisely it wraps the newton euler algorithm implemented
      by the dynamicsJRLJapan library to make it accessible in the stack of tasks.
      The robot is described by a VRML file.
    */
class SOTDYNAMIC_EXPORT DynamicPinocchio
 :public dg::Entity {
  friend class sot::command::SetFile;
  friend class sot::command::CreateOpPoint;
  //  friend class sot::command::InitializeRobot;

    public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DYNAMIC_GRAPH_ENTITY_DECL();

  /*  --- MODEL ATRIBUTES --- */
  pinocchio::Model*  m_model;
  pinocchio::Data*   m_data;

  /*  --- MODEL ATRIBUTES --- */

    public:
  /* --- SIGNAL ACTIVATION --- */
  dg::SignalTimeDependent< dg::Matrix,int >&
    createEndeffJacobianSignal( const std::string& signame, const std::string&, const bool isLocal = true);
  dg::SignalTimeDependent< dg::Matrix,int >&
    createJacobianSignal( const std::string& signame, const std::string& );
  void destroyJacobianSignal( const std::string& signame );

  dg::SignalTimeDependent< MatrixHomogeneous,int >&
    createPositionSignal( const std::string&,const std::string& );
  void destroyPositionSignal( const std::string& signame );

  dg::SignalTimeDependent< dg::Vector,int >&
    createVelocitySignal( const std::string&,const std::string& );
  void destroyVelocitySignal( const std::string& signame );

  dg::SignalTimeDependent< dg::Vector,int >&
    createAccelerationSignal( const std::string&, const std::string& );
  void destroyAccelerationSignal( const std::string& signame );

  /*! @} */
  std::list< dg::SignalBase<int>*  > genericSignalRefs;



    public:
  /* --- SIGNAL --- */
  typedef int Dummy;
  dg::SignalPtr<dg::Vector,int> jointPositionSIN;
  dg::SignalPtr<dg::Vector,int> freeFlyerPositionSIN;
  dg::SignalPtr<dg::Vector,int> jointVelocitySIN;
  dg::SignalPtr<dg::Vector,int> freeFlyerVelocitySIN;
  dg::SignalPtr<dg::Vector,int> jointAccelerationSIN;
  dg::SignalPtr<dg::Vector,int> freeFlyerAccelerationSIN;

  dg::SignalTimeDependent<dg::Vector,int> pinocchioPosSINTERN;
  dg::SignalTimeDependent<dg::Vector,int> pinocchioVelSINTERN;
  dg::SignalTimeDependent<dg::Vector,int> pinocchioAccSINTERN;

  dg::SignalTimeDependent<Dummy,int> newtonEulerSINTERN;
  dg::SignalTimeDependent<Dummy,int> jacobiansSINTERN;
  dg::SignalTimeDependent<Dummy,int> forwardKinematicsSINTERN;
  dg::SignalTimeDependent<Dummy,int> ccrbaSINTERN;

  int& computeNewtonEuler(int& dummy,const int& time );
  int& computeForwardKinematics(int& dummy,const int& time );
  int& computeCcrba( int& dummy,const int& time );
  int& computeJacobians( int& dummy,const int& time );

  dg::SignalTimeDependent<dg::Vector,int> zmpSOUT;
  dg::SignalTimeDependent<dg::Matrix,int> JcomSOUT;
  dg::SignalTimeDependent<dg::Vector,int> comSOUT;
  dg::SignalTimeDependent<dg::Matrix,int> inertiaSOUT;

  dg::SignalTimeDependent<dg::Matrix,int>& jacobiansSOUT( const std::string& name );
  dg::SignalTimeDependent<MatrixHomogeneous,int>& positionsSOUT( const std::string& name );
  dg::SignalTimeDependent<dg::Vector,int>& velocitiesSOUT( const std::string& name );
  dg::SignalTimeDependent<dg::Vector,int>& accelerationsSOUT( const std::string& name );

  dg::SignalTimeDependent<double,int> footHeightSOUT;
  dg::SignalTimeDependent<dg::Vector,int> upperJlSOUT;
  dg::SignalTimeDependent<dg::Vector,int> lowerJlSOUT;
  dg::SignalTimeDependent<dg::Vector,int> upperVlSOUT;
  dg::SignalTimeDependent<dg::Vector,int> upperTlSOUT;

  dg::Signal<dg::Vector,int> inertiaRotorSOUT;
  dg::Signal<dg::Vector,int> gearRatioSOUT;
  dg::SignalTimeDependent<dg::Matrix,int> inertiaRealSOUT;
  dg::SignalTimeDependent<dg::Vector,int> MomentaSOUT;
  dg::SignalTimeDependent<dg::Vector,int> AngularMomentumSOUT;
  dg::SignalTimeDependent<dg::Vector,int> dynamicDriftSOUT;


    public:
  /* --- CONSTRUCTOR --- */
  DynamicPinocchio( const std::string& name);
  virtual ~DynamicPinocchio( void );


  /* --- MODEL CREATION --- */


  void displayModel() const
  { assert(m_model); std::cout<<(*m_model)<<std::endl; };

  void setModel(pinocchio::Model*);

  void setData(pinocchio::Data*);

  /* --- GETTERS --- */

  /// \brief Get joint position lower limits
  ///
  /// \param[out] result vector
  /// \return result vector
  dg::Vector& getLowerPositionLimits(dg::Vector& res,const int&) const ;

  /// \brief Get joint position upper limits
  ///
  /// \param[out] result vector
  /// \return result vector
  dg::Vector& getUpperPositionLimits(dg::Vector& res,const int&) const;

  /// \brief Get joint velocity upper limits
  ///
  /// \param[out] result vector
  /// \return result vector
  dg::Vector& getUpperVelocityLimits(dg::Vector& res,const int&) const;

  /// \brief Get joint effort upper limits
  ///
  /// \param[out] result vector
  /// \return result vector
  dg::Vector& getMaxEffortLimits(dg::Vector& res,const int&) const;


  //  dg::Vector& getAnklePositionInFootFrame() const;

    protected:
  dg::Matrix& computeGenericJacobian(const bool isFrame,
				     const int jointId,
				     dg::Matrix& res,const int& time );
  dg::Matrix& computeGenericEndeffJacobian(const bool isFrame, const bool isLocal,
					   const int jointId,
					   dg::Matrix& res,const int& time );
  MatrixHomogeneous& computeGenericPosition(const bool isFrame,
					    const int jointId,
					    MatrixHomogeneous& res,const int& time );
  dg::Vector& computeGenericVelocity(const int jointId,dg::Vector& res,const int& time );
  dg::Vector& computeGenericAcceleration(const int jointId,dg::Vector& res,const int& time );

  dg::Vector& computeZmp( dg::Vector& res,const int& time );
  dg::Vector& computeMomenta( dg::Vector &res,const  int& time);
  dg::Vector& computeAngularMomentum( dg::Vector &res,const  int& time);
  dg::Matrix& computeJcom( dg::Matrix& res,const int& time );
  dg::Vector& computeCom( dg::Vector& res,const int& time );
  dg::Matrix& computeInertia( dg::Matrix& res,const int& time );
  dg::Matrix& computeInertiaReal( dg::Matrix& res,const int& time );
  double& computeFootHeight( double& res,const int& time );

  dg::Vector& computeTorqueDrift( dg::Vector& res,const int& time );

 public: /* --- PARAMS --- */
  void cmd_createOpPointSignals           ( const std::string& sig, const std::string& j );
  void cmd_createJacobianWorldSignal      ( const std::string& sig, const std::string& j );
  void cmd_createJacobianEndEffectorSignal( const std::string& sig, const std::string& j );
  void cmd_createJacobianEndEffectorWorldSignal( const std::string& sig, const std::string& j );
  void cmd_createPositionSignal           ( const std::string& sig, const std::string& j );
  void cmd_createVelocitySignal           ( const std::string& sig, const std::string& j );
  void cmd_createAccelerationSignal       ( const std::string& sig, const std::string& j );

 private:
  /// \brief map of joints in construction.
  /// map: jointName -> (jointType,jointPosition (in parent frame), function_ptr to pinocchio Joint)
  dg::Vector& getPinocchioPos(dg::Vector& q,const int& time);
  dg::Vector& getPinocchioVel(dg::Vector& v, const int& time);
  dg::Vector& getPinocchioAcc(dg::Vector& a, const int&time);

  //\brief Index list for the first dof of (spherical joints)/ (spherical part of free-flyer joint).
  std::vector<int> sphericalJoints;

};

  // std::ostream& operator<<(std::ostream& os, const CjrlJoint& r);
} /* namespace sot */} /* namespace dynamicgraph */



#endif // #ifndef __SOT_DYNAMIC_PINOCCHIO_H__
