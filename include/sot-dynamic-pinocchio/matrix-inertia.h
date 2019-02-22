/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_SOTMATRIXINERTIA_H__
#define __SOT_SOTMATRIXINERTIA_H__

#include <ostream>

#include "dynamic-graph/linear-algebra.h"



#include <sot/core/matrix-geometry.hh>


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


namespace dynamicgraph { namespace sot {
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

  std::vector< dynamicgraph::Matrix >  Ic;
  std::vector< dynamicgraph::Vector >      phi;
  std::vector< MatrixTwist >  iVpi;
  std::vector< MatrixForce >  iVpiT;
  dynamicgraph::Matrix inertia_;


};

} /* namespace sot */} /* namespace dynamicgraph */

#endif // __SOT_SOTMATRIXINERTIA_H__
