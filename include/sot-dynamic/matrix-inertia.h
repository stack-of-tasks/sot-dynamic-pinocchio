#ifndef __SOT_SOTMATRIXINERTIA_H__
#define __SOT_SOTMATRIXINERTIA_H__

#include <ostream>

#include "MatrixAbstractLayer/boost.h"
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"

#include <sot-core/matrix-twist.h>
#include <sot-core/matrix-force.h>
#include <sot-core/matrix-rotation.h>
#include <sot-core/matrix-homogeneous.h>

namespace dynamicsJRLJapan
{
  class HumanoidDynamicMultiBody;
}

class CjrlHumanoidDynamicRobot;
class CjrlJoint;

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (matrix_inertia_EXPORTS)
#    define SOTMATRIXINERTIA_EXPORT __declspec(dllexport)
#  else  
#    define SOTMATRIXINERTIA_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTMATRIXINERTIA_EXPORT
#endif


namespace sot {
namespace dg = dynamicgraph;

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
class SOTMATRIXINERTIA_EXPORT MatrixInertia
{
public:

 private:
  MatrixInertia( void ) {}

  void initParents( void );
  void initDofTable( void );

 public:
  MatrixInertia( CjrlHumanoidDynamicRobot* aHDR );
  ~MatrixInertia( void );
  void init( CjrlHumanoidDynamicRobot* aHDR );

public:

  void update( void );
  void computeInertiaMatrix();
  void getInertiaMatrix(double* A);
  const maal::boost::Matrix& getInertiaMatrix( void );
  size_t getDoF() { return joints_.size(); }

private:

  CjrlHumanoidDynamicRobot*                            aHDR_;
  dynamicsJRLJapan::HumanoidDynamicMultiBody*          aHDMB_;
  std::vector<CjrlJoint*>                              joints_;
  std::vector<int>                                     parentIndex_;
 
  std::vector< ml::Matrix >  Ic;
  std::vector< ml::Vector >      phi;
  std::vector< sotMatrixTwist >  iVpi;
  std::vector< MatrixForce >  iVpiT;
  ml::Matrix inertia_;


};

} // namespace sot

#endif // __SOT_SOTMATRIXINERTIA_H__
