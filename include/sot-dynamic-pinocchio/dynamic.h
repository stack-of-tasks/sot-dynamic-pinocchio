#ifndef __SOT_DYNAMIC_PINOCCHIO_H
#define __SOT_DYNAMIC_PINOCCHIO_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* STD */
#include <string>
#include <map>

/* Matrix */
#include <jrl/mal/boost.hh>
#include "jrl/mal/matrixabstractlayer.hh"
namespace ml = maal::boost;
#include <Eigen/Core>


/* SOT kinematics*/
#include <sot/core/flags.hh>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/core/exception-dynamic.hh>
#include <sot/core/matrix-homogeneous.hh>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/parser/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>



using namespace std;
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

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

    namespace commande {
        class CreateOpPoint;
    }

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTDYNAMIC_EXPORT Dynamic:public dg::Entity
{
    friend class sot::commande::CreateOpPoint;

    DYNAMIC_GRAPH_ENTITY_DECL();
    std::list< dg::SignalBase<int>*  > genericSignalRefs;
public: /* --- CONSTRUCTION --- */

    Dynamic( const std::string& name, bool build=true );
    virtual ~Dynamic( void );

public: /* --- ACCESSORS --- */
    void setUrdfPath( const std::string& path );
public:/* --- CONVERTION --- */
    Eigen::VectorXd getPinocchioPos(int time);//get q from JointPos and freeFlyerPos signals
    Eigen::VectorXd getPinocchioVel(int time);
    Eigen::VectorXd getPinocchioAcc(int time);

public:/*  --- ATRIBUTES --- */
    se3::Model  m_model;
    se3::Data*   m_data;
    std::string m_urdfPath;

    ml::Vector testRNEA(const ml::Vector& maalQ,const ml::Vector& maalV,const ml::Vector& maalA);


public: /* --- SIGNAL ACTIVATION --- */
 //CAUTION: j as int type, temporary
    dg::SignalTimeDependent< ml::Matrix,int > & createEndeffJacobianSignal( const std::string& signame, int jointId );
    dg::SignalTimeDependent< ml::Matrix,int > & createJacobianSignal      ( const std::string& signame, int jointId );
    void destroyJacobianSignal                                            ( const std::string& signame );
    dg::SignalTimeDependent< MatrixHomogeneous,int >&createPositionSignal ( const std::string& signame, int jointId );
    void destroyPositionSignal                                            ( const std::string& signame );
    dg::SignalTimeDependent< ml::Vector,int >&     createVelocitySignal   ( const std::string& signame,  int jointId );
    void destroyVelocitySignal                                            ( const std::string& signame );
    dg::SignalTimeDependent< ml::Vector,int >&   createAccelerationSignal ( const std::string& signame, int jointId );
    void destroyAccelerationSignal                                        ( const std::string& signame );



public: /* --- SIGNAL --- */
    dg::SignalPtr<ml::Vector,int> jointPositionSIN;
    dg::SignalPtr<ml::Vector,int> freeFlyerPositionSIN;
    dg::SignalPtr<ml::Vector,int> jointVelocitySIN;
    dg::SignalPtr<ml::Vector,int> freeFlyerVelocitySIN;
    dg::SignalPtr<ml::Vector,int> jointAccelerationSIN;
    dg::SignalPtr<ml::Vector,int> freeFlyerAccelerationSIN;



public:
 typedef int Dummy;
 dg::SignalTimeDependent<Dummy,int> newtonEulerSINTERN;

 int& computeNewtonEuler( int& dummy,int time );

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
 dg::SignalTimeDependent<ml::Vector,int> upperVlSOUT;
 dg::SignalTimeDependent<ml::Vector,int> upperTlSOUT;

 dg::Signal<ml::Vector,int> inertiaRotorSOUT;
 dg::Signal<ml::Vector,int> gearRatioSOUT;
 dg::SignalTimeDependent<ml::Matrix,int> inertiaRealSOUT;
 dg::SignalTimeDependent<ml::Vector,int> MomentaSOUT;
 dg::SignalTimeDependent<ml::Vector,int> AngularMomentumSOUT;
 dg::SignalTimeDependent<ml::Vector,int> dynamicDriftSOUT;

 // public for tests //
 ml::Matrix& computeInertia( ml::Matrix& res,int time );
 ml::Vector& computeZmp( ml::Vector& res,int time );

protected: /* --- METHODS --- */


 ml::Vector& computeMomenta( ml::Vector &res, int time);
 ml::Vector& computeAngularMomentum( ml::Vector &res, int time);
 ml::Matrix& computeInertiaReal( ml::Matrix& res,int time );
 double& computeFootHeight( double& res,int time );

 //CAUTION: j as int type, temporary
public:
 ml::Matrix& computeGenericJacobian( int jointId,ml::Matrix& res,int time );
 ml::Matrix& computeGenericEndeffJacobian( int jointId,ml::Matrix& res,int time );
 MatrixHomogeneous& computeGenericPosition( int jointId,MatrixHomogeneous& res,int time );
 ml::Vector& computeGenericVelocity( int j,ml::Vector& res,int time );
 ml::Vector& computeGenericAcceleration( int j,ml::Vector& res,int time );

 ml::Vector& computeCom( ml::Vector& res,int time );
 ml::Matrix& computeJcom( ml::Matrix& res,int time );

 ml::Vector& getUpperJointLimits( ml::Vector& res,const int& time );
 ml::Vector& getLowerJointLimits( ml::Vector& res,const int& time );
 ml::Vector& getUpperVelocityLimits( ml::Vector& res,const int& time );
 ml::Vector& getUpperTorqueLimits( ml::Vector& res,const int& time );

 ml::Vector& computeTorqueDrift( ml::Vector& res,const int& time );

public: /* --- PARAMS --- */
 void cmd_createOpPointSignals(const std::string& sig,const std::string& j);
 void cmd_createJacobianWorldSignal( const std::string& sig,const std::string& j );
 void cmd_createJacobianEndEffectorSignal( const std::string& sig,const std::string& j );
 void cmd_createPositionSignal( const std::string& sig,const std::string& j );

};
} /* namespace sot */} /* namespace dynamicgraph */
#endif // #ifndef __SOT_DYNAMIC_PINOCCHIO_H
