#ifndef __SOT_DYNAMIC_H__
#define __SOT_DYNAMIC_H__

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


/* SOT */
#include <sot/core/flags.hh>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/core/exception-dynamic.hh>
#include <sot/core/matrix-homogeneous.hh>
#include <pinocchio/multibody/model.hpp>
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

/*! @ingroup signals
  \brief This class provides an inverse dynamic model of the robot.
  More precisely it wraps the newton euler algorithm implemented
  by the pinocchio library to make it accessible in the stack of tasks.
  The robot is described by a URDF file.
*/

class SOTDYNAMIC_EXPORT Dynamic
:public dg::Entity
{


public:
    DYNAMIC_GRAPH_ENTITY_DECL();

public: /* --- CONSTRUCTION --- */
    Dynamic( const std::string& name, bool build=true );
    virtual ~Dynamic( void );
public: /* --- MODEL CREATION --- */
    void setUrdfPath( const std::string& path );
    se3::Model m_model;
public: /* --- SIGNAL ACTIVATION --- */

public: /* --- SIGNAL --- */

public: /* --- PARAMS --- */


};
} /* namespace sot */} /* namespace dynamicgraph */
#endif // #ifndef __SOT_DYNAMIC_H__
