/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      Dynamic.h
 * Project:   SOT
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef __SOT_DYNAMIC_H__
#define __SOT_DYNAMIC_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>
/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* JRL dynamic */
#include <robotDynamics/jrlHumanoidDynamicRobot.h>
#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>
namespace djj = dynamicsJRLJapan;

/* SOT */
#include <sot-core/flags.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot-core/exception-dynamic.h>
#include <sot-core/matrix-homogeneous.h>

/* STD */
#include <string>

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


namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/*! @ingroup signals
  \brief This class provides an inverse dynamic model of the robot.
  More precisely it wraps the newton euler algorithm implemented
  by the dynamicsJRLJapan library to make it accessible in the stack of tasks.
  The robot is described by a VRML file.
*/
  
class SOTDYNAMIC_EXPORT Dynamic
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;

 protected:
 public:

  /*! \name Fields to access dynamicsJRLJapan Library 
    @{
   */

  /*! \brief Abstract pointer on the structure.
    Ultimately we should be able to use only the abstract
    interface and do not care about the implementation.
  */
  CjrlHumanoidDynamicRobot* m_HDR;


  int debugInertia;

  /*! \brief Fields to access the humanoid model 
   @{ */

  /*! \brief Directory where the VRML humanoid model is stored */
  std::string vrmlDirectory;
  /*! \brief Name of the root's robot model file */
  std::string vrmlMainFile;
  /*! \brief Name of the name specifying which end-effector is
    the head, the feet and so on... */
  std::string xmlSpecificityFile;
  /*! \brief Name of the name specifying which end-effector is
    the head, the feet and so on... */
  std::string xmlRankFile;
  /*! @} */

  /*! @} */
  bool init;
  std::list< dg::SignalBase<int>*  > genericSignalRefs;

 public: /* --- CONSTRUCTION --- */

  Dynamic( const std::string& name, bool build=true );
  virtual ~Dynamic( void );

 public: /* --- MODEL CREATION --- */
 
  virtual void buildModel( void );

  void setVrmlDirectory( const std::string& filename );
  void setVrmlMainFile( const std::string& filename );
  void setXmlSpecificityFile( const std::string& filename );
  void setXmlRankFile( const std::string& filename );
  void parseConfigFiles( void );

 public: /* --- SIGNAL ACTIVATION --- */
  dg::SignalTimeDependent< ml::Matrix,int > &
    createEndeffJacobianSignal( const std::string& signame,
				   const unsigned int & bodyRank );
  dg::SignalTimeDependent< ml::Matrix,int > &
    createJacobianSignal( const std::string& signame,
			  const unsigned int & bodyRank );
  void destroyJacobianSignal( const std::string& signame );
  dg::SignalTimeDependent< MatrixHomogeneous,int >&
    createPositionSignal( const std::string& signame,
			  const unsigned int & bodyRank );
  void destroyPositionSignal( const std::string& signame );
  dg::SignalTimeDependent< ml::Vector,int >&
      createVelocitySignal( const std::string& signame,
  			     const unsigned int & bodyRank );
    void destroyVelocitySignal( const std::string& signame );
  dg::SignalTimeDependent< ml::Vector,int >&
    createAccelerationSignal( const std::string& signame,
			     const unsigned int & bodyRank );
  void destroyAccelerationSignal( const std::string& signame );

  bool zmpActivation( void ) { std::string Property("ComputeZMP");
    std::string Value; m_HDR->getProperty(Property,Value); return (Value=="true");}
  void zmpActivation( const bool& b ) { std::string Property("ComputeZMP");
    std::string Value; if (b) Value="true"; else Value="false"; m_HDR->setProperty(Property,Value); }
  bool comActivation( void ) { std::string Property("ComputeCoM");
    std::string Value; m_HDR->getProperty(Property,Value); return (Value=="true"); }
  void comActivation( const bool& b ) { std::string Property("ComputeCoM");
    std::string Value; if (b) Value="true"; else Value="false"; m_HDR->setProperty(Property,Value); } 

 public: /* --- SIGNAL --- */

  dg::SignalPtr<ml::Vector,int> jointPositionSIN;
  dg::SignalPtr<ml::Vector,int> freeFlyerPositionSIN;
  dg::SignalPtr<ml::Vector,int> jointVelocitySIN;
  dg::SignalPtr<ml::Vector,int> freeFlyerVelocitySIN;
  dg::SignalPtr<ml::Vector,int> jointAccelerationSIN;
  dg::SignalPtr<ml::Vector,int> freeFlyerAccelerationSIN;

  // protected:
 public:
  typedef int Dummy;
  dg::SignalTimeDependent<Dummy,int> firstSINTERN;
  dg::SignalTimeDependent<Dummy,int> newtonEulerSINTERN;

  int& computeNewtonEuler( int& dummy,int time );
  int& initNewtonEuler( int& dummy,int time );

 public:
  dg::SignalTimeDependent<ml::Vector,int> zmpSOUT;
  dg::SignalTimeDependent<ml::Matrix,int> JcomSOUT;
  dg::SignalTimeDependent<ml::Vector,int> comSOUT;
  dg::SignalTimeDependent<ml::Matrix,int> inertiaSOUT;

  dg::SignalTimeDependent<ml::Matrix,int>& jacobiansSOUT( const std::string& name );
  dg::SignalTimeDependent<MatrixHomogeneous,int>& positionsSOUT( const std::string& name );
  dg::SignalTimeDependent<ml::Vector,int>& velocitiesSOUT( const std::string& name );
  dg::SignalTimeDependent<ml::Vector,int>& accelerationsSOUT( const std::string& name );

  dg::SignalTimeDependent<double,int> footHeightSOUT;
  dg::SignalTimeDependent<ml::Vector,int> upperJlSOUT;
  dg::SignalTimeDependent<ml::Vector,int> lowerJlSOUT;

  dg::Signal<ml::Vector,int> inertiaRotorSOUT;
  dg::Signal<ml::Vector,int> gearRatioSOUT;
  dg::SignalTimeDependent<ml::Matrix,int> inertiaRealSOUT;
  dg::SignalTimeDependent<ml::Vector,int> MomentaSOUT;
  dg::SignalTimeDependent<ml::Vector,int> AngularMomentumSOUT;

 protected:
  ml::Vector& computeZmp( ml::Vector& res,int time );
  ml::Vector& computeMomenta( ml::Vector &res, int time);
  ml::Vector& computeAngularMomentum( ml::Vector &res, int time);
  ml::Matrix& computeJcom( ml::Matrix& res,int time );
  ml::Vector& computeCom( ml::Vector& res,int time );
  ml::Matrix& computeInertia( ml::Matrix& res,int time );
  ml::Matrix& computeInertiaReal( ml::Matrix& res,int time );
  double& computeFootHeight( double& res,int time );
  
  ml::Matrix& computeGenericJacobian( CjrlJoint* j,ml::Matrix& res,int time );
  ml::Matrix& computeGenericEndeffJacobian( CjrlJoint* j,ml::Matrix& res,int time );
  MatrixHomogeneous& computeGenericPosition( CjrlJoint* j,MatrixHomogeneous& res,int time );
  ml::Vector& computeGenericVelocity( CjrlJoint* j,ml::Vector& res,int time );
  ml::Vector& computeGenericAcceleration( CjrlJoint* j,ml::Vector& res,int time );

  ml::Vector& getUpperJointLimits( ml::Vector& res,const int& time );
  ml::Vector& getLowerJointLimits( ml::Vector& res,const int& time );


 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );
    

};


} // namespace sot



#endif // #ifndef __SOT_DYNAMIC_H__

