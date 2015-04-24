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


/* SOT */
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

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTDYNAMIC_EXPORT Dynamic:public dg::Entity
{
    DYNAMIC_GRAPH_ENTITY_DECL();
    std::list< dg::SignalBase<int>*  > genericSignalRefs;
public: /* --- CONSTRUCTION --- */

    Dynamic( const std::string& name, bool build=true );
    virtual ~Dynamic( void );

public: /* --- ACCESSORS --- */
    void setUrdfPath( const std::string& path );

public:/*  --- ATRIBUTES --- */
    se3::Model  m_model;
    se3::Data*   m_data;
    std::string m_urdfPath;

    ml::Vector testRNEA(const ml::Vector& maalQ,const ml::Vector& maalV,const ml::Vector& maalA);

public: /* --- SIGNAL --- */
 dg::SignalPtr<ml::Vector,int> jointPositionSIN;
 dg::SignalPtr<ml::Vector,int> freeFlyerPositionSIN;
 dg::SignalPtr<ml::Vector,int> jointVelocitySIN;
 dg::SignalPtr<ml::Vector,int> freeFlyerVelocitySIN;
 dg::SignalPtr<ml::Vector,int> jointAccelerationSIN;
 dg::SignalPtr<ml::Vector,int> freeFlyerAccelerationSIN;

protected: /* --- METHODS --- */

 ml::Vector& computeZmp( ml::Vector& res,int time );
 ml::Vector& computeMomenta( ml::Vector &res, int time);
 ml::Vector& computeAngularMomentum( ml::Vector &res, int time);
 ml::Matrix& computeJcom( ml::Matrix& res,int time );
 ml::Vector& computeCom( ml::Vector& res,int time );
 ml::Matrix& computeInertia( ml::Matrix& res,int time );
 ml::Matrix& computeInertiaReal( ml::Matrix& res,int time );
 double& computeFootHeight( double& res,int time );

 ml::Vector& getUpperJointLimits( ml::Vector& res,const int& time );
 ml::Vector& getLowerJointLimits( ml::Vector& res,const int& time );

 ml::Vector& getUpperVelocityLimits( ml::Vector& res,const int& time );
 ml::Vector& getLowerVelocityLimits( ml::Vector& res,const int& time );

 ml::Vector& getUpperTorqueLimits( ml::Vector& res,const int& time );
 ml::Vector& getLowerTorqueLimits( ml::Vector& res,const int& time );

 ml::Vector& computeTorqueDrift( ml::Vector& res,const int& time );

};
} /* namespace sot */} /* namespace dynamicgraph */
#endif // #ifndef __SOT_DYNAMIC_PINOCCHIO_H
