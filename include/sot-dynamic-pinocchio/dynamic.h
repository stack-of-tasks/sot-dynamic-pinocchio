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

namespace command {
class CreateOpPoint;
}

/* --------------------------------------------------------------------- */
/* --- Structure ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

typedef struct NewJoints NewJoints;
struct NewJoints{
    const std::string JointName;
    const std::string JointType;
    const ml::Matrix Position;

    NewJoints():JointName(""),JointType(""),Position(ml::Matrix(6,6)){}
    NewJoints(const std::string& name, const std::string& type,const ml::Matrix& position):JointName(name),JointType(type),Position(position){}
};

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTDYNAMIC_EXPORT Dynamic:public dg::Entity
{
    friend class sot::command::CreateOpPoint;

    static void initMap();

public:
    bool init;
    int debuginertia;

    DYNAMIC_GRAPH_ENTITY_DECL();
    std::list< dg::SignalBase<int>*  > genericSignalRefs;
public: /* --- CONSTRUCTION --- */

    Dynamic( const std::string& name, bool build=true );
    virtual ~Dynamic( void );

public: /* --- ACCESSORS --- */
    void setUrdfPath( const std::string& path );
public:/* --- CONVERTION --- */
    Eigen::VectorXd getPinocchioPos(int time);
    Eigen::VectorXd getPinocchioVel(int time);
    Eigen::VectorXd getPinocchioAcc(int time);

public:/*  --- ATRIBUTES --- */
    se3::Model  m_model;
    se3::Data*   m_data;
    std::string m_urdfPath;

    ml::Vector testRNEA(const ml::Vector& maalQ,const ml::Vector& maalV,const ml::Vector& maalA);


public: /* --- SIGNAL ACTIVATION --- */
    dg::SignalTimeDependent< ml::Matrix,int > & createEndeffJacobianSignal( const std::string& signame, int jointId );
    dg::SignalTimeDependent< ml::Matrix,int > & createJacobianSignal      ( const std::string& signame, int jointId );
    void destroyJacobianSignal                                            ( const std::string& signame );
    dg::SignalTimeDependent< MatrixHomogeneous,int >&createPositionSignal ( const std::string& signame, int jointId );
    void destroyPositionSignal                                            ( const std::string& signame );
    dg::SignalTimeDependent< ml::Vector,int >&     createVelocitySignal   ( const std::string& signame,  int jointId );
    void destroyVelocitySignal                                            ( const std::string& signame );
    dg::SignalTimeDependent< ml::Vector,int >&   createAccelerationSignal ( const std::string& signame, int jointId );
    void destroyAccelerationSignal                                        ( const std::string& signame );

    // getProperty and setProperty never implement in cjrl >> useless
    bool zmpActivation( void ) {
        std::string Property("ComputeZMP");
        std::string Value;
        //m_HDR->getProperty(Property,Value);
        return (Value=="true");
    }
    void zmpActivation( const bool& b ) {
        std::string Property("ComputeZMP");
        std::string Value;
        if (b) Value="true";
        else Value="false";
        //m_HDR->setProperty(Property,Value);
    }
    bool comActivation( void ) {
        std::string Property("ComputeCoM");
        std::string Value;
        //m_HDR->getProperty(Property,Value);
        return (Value=="true");
    }
    void comActivation( const bool& b ) {
        std::string Property("ComputeCoM");
        std::string Value;
        if (b) Value="true";
        else Value="false";
        //m_HDR->setProperty(Property,Value);
    }

public: /* --- SIGNAL --- */
    typedef int Dummy;
    dg::SignalPtr<ml::Vector,int> jointPositionSIN;
    dg::SignalPtr<ml::Vector,int> freeFlyerPositionSIN;
    dg::SignalPtr<ml::Vector,int> jointVelocitySIN;
    dg::SignalPtr<ml::Vector,int> freeFlyerVelocitySIN;
    dg::SignalPtr<ml::Vector,int> jointAccelerationSIN;
    dg::SignalPtr<ml::Vector,int> freeFlyerAccelerationSIN;

    dg::SignalTimeDependent<Dummy,int> newtonEulerSINTERN;
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

protected: /* --- METHODS --- */

    ml::Vector& computeMomenta( ml::Vector &res, int time);
    ml::Vector& computeAngularMomentum( ml::Vector &res, int time);
    double& computeFootHeight( double& res,int time );

public:
    int& computeNewtonEuler( int& dummy,int time );
    ml::Matrix& computeGenericJacobian( int jointId,ml::Matrix& res,int time );
    ml::Matrix& computeGenericEndeffJacobian( int jointId,ml::Matrix& res,int time );
    MatrixHomogeneous& computeGenericPosition( int jointId,MatrixHomogeneous& res,int time );
    ml::Vector& computeGenericVelocity( int jointId,ml::Vector& res,int time );
    ml::Vector& computeGenericAcceleration( int jointId,ml::Vector& res,int time );
    ml::Matrix& computeInertiaReal( ml::Matrix& res,int time );
    ml::Matrix& computeInertia( ml::Matrix& res,int time );
    ml::Vector& computeZmp( ml::Vector& res,int time );

    ml::Vector& computeCom( ml::Vector& res,int time );
    ml::Matrix& computeJcom( ml::Matrix& res,int time );

    ml::Vector& getUpperJointLimits( ml::Vector& res,const int& time );
    ml::Vector& getLowerJointLimits( ml::Vector& res,const int& time );
    ml::Vector& getUpperVelocityLimits( ml::Vector& res,const int& time );
    ml::Vector& getUpperTorqueLimits( ml::Vector& res,const int& time );

    ml::Vector& computeTorqueDrift( ml::Vector& res,const int& time );

public: /* --- PARAMS --- */
    virtual void commandLine( const std::string& cmdLine,
                              std::istringstream& cmdArgs,
                              std::ostream& os );
    void cmd_createOpPointSignals(const std::string& sig,const std::string& j);
    void cmd_createJacobianWorldSignal( const std::string& sig,const std::string& j );
    void cmd_createJacobianEndEffectorSignal( const std::string& sig,const std::string& j );
    void cmd_createPositionSignal( const std::string& sig,const std::string& j );
    void createRobot();
    void createJoint(const std::string& inJointName,const std::string& inJointType, const ml::Matrix& inPosition);
    void setRootJoint(const std::string& inJointName);
    void addJoint(const std::string& inParentName,const std::string& inChildName);
    void setDofBounds(const std::string& inJointName,double inMinValue, double inMaxValue);
    void setMass(const std::string& inJointName, double inMass);
    void setLocalCenterOfMass(const std::string& inJointName, ml::Vector inCom);
    void setInertiaMatrix(const std::string& inJointName, ml::Matrix inMatrix);
};
} /* namespace sot */} /* namespace dynamicgraph */
#endif // #ifndef __SOT_DYNAMIC_PINOCCHIO_H
