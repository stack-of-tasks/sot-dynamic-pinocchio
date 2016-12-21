/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-dynamic-pinocchio.
 * sot-dynamic-pinocchio is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-dynamic-pinocchio is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-dynamic-pinocchio.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <sot-dynamic-pinocchio/integrator-force-exact.h>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <sot/core/exception-dynamic.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(IntegratorForceExact,"IntegratorForceExact");

IntegratorForceExact::
IntegratorForceExact( const std::string & name ) 
  :IntegratorForce(name)
{
  sotDEBUGIN(5);
  
  velocitySOUT.
    setFunction( boost::bind(&IntegratorForceExact::computeVelocityExact,
			     this,_1,_2));
  velocitySOUT.removeDependency( velocityDerivativeSOUT );

  sotDEBUGOUT(5);
}


IntegratorForceExact::
~IntegratorForceExact( void )
{
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
  return;
}

/* --- EIGEN VALUE ---------------------------------------------------------- */
/* --- EIGEN VALUE ---------------------------------------------------------- */
/* --- EIGEN VALUE ---------------------------------------------------------- */


extern "C"
{
  void dgeev_( const char* jobvl, const char* jobvr, const int* n, double* a,
	       const int* lda, double* wr, double* wi, double* vl, const int* ldvl,
	       double* vr, const int* ldvr, double* work, const int* lwork, int* info );
}


int geev(Matrix &a,
	 Eigen::VectorXcd &w,
	 Matrix &vl2,
	 Matrix &vr2
	 )
{
  char jobvl='V';
  char jobvr='V';
  int const n = (int)a.rows();
  
  Vector wr(n);
  Vector wi(n);
  double* vl_real = MRAWDATA(vl2);
  const int ldvl = (int)vl2.rows();
  double* vr_real = MRAWDATA(vr2);
  const int ldvr = (int)vr2.rows();
  
  // workspace query
  int lwork = -1;
  double work_temp;
  int result=0;
  dgeev_(&jobvl, &jobvr, &n,
	 MRAWDATA(a), &n, 
	 MRAWDATA(wr), MRAWDATA(wi), 
	 vl_real, &ldvl, vr_real, &ldvr,
	 &work_temp, &lwork, &result);
  if (result != 0)
    return result;
  
  lwork = (int) work_temp;
  Vector work(lwork);
  dgeev_(&jobvl, &jobvr, &n,
	 MRAWDATA(a), &n, 
	 MRAWDATA(wr), MRAWDATA(wi), 
	 vl_real, &ldvl, vr_real, &ldvr,
	 MRAWDATA(work), &lwork,
	 &result);

  for (int i = 0; i < n; i++)
    w[i] = std::complex<double>(wr[i], wi[i]);
  
  return result;
}

static void eigenDecomp( const dynamicgraph::Matrix& M,
			 dynamicgraph::Matrix& P,
			 dynamicgraph::Vector& eig )
{
  long int SIZE = M.cols();
  Matrix Y(M);
  Eigen::VectorXcd evals(SIZE);
  Matrix vl(SIZE,SIZE);
  Matrix vr(SIZE,SIZE);
  
  //  const int errCode = lapack::geev(Y, evals, &vl, &vr, lapack::optimal_workspace());
  const int errCode = geev(Y,evals,vl,vr);
  if( errCode<0 )
    { SOT_THROW ExceptionDynamic( ExceptionDynamic::INTEGRATION,
				     "Invalid argument to geev","" ); }
  else if( errCode>0 )
    { SOT_THROW ExceptionDynamic( ExceptionDynamic::INTEGRATION,
				     "No convergence for given matrix",""); }
  
  P.resize(SIZE,SIZE);   eig.resize(SIZE);
  for( unsigned int i=0;i<SIZE;++i )
    {
      for( unsigned int j=0;j<SIZE;++j ){ P(i,j)=vr(i,j); }
      eig(i)=evals(i).real();
      if( fabsf(static_cast<float>(evals(i).imag()))>1e-5 ) 
	{ 
	  SOT_THROW ExceptionDynamic( ExceptionDynamic::INTEGRATION,
					 "Error imaginary part not null. ",
					 "(at position %d: %f)",i,evals(i).imag() );
	}
    }
  return ;
}

static void expMatrix( const dynamicgraph::Matrix& MiB,
		       dynamicgraph::Matrix& Mexp )
{
  long int SIZE = MiB.cols();

  dynamicgraph::Matrix Pmib(MiB.cols(),MiB.cols());
  dynamicgraph::Vector eig_mib(MiB.cols());
  eigenDecomp( MiB,Pmib,eig_mib );
  sotDEBUG(45) << "V = " << Pmib;
  sotDEBUG(45) << "d = " << eig_mib;

  dynamicgraph::Matrix Pinv(SIZE,SIZE); Pinv = Pmib.inverse();
  sotDEBUG(45) << "Vinv = " << Pinv;

  Mexp.resize(SIZE,SIZE);
  for( unsigned int i=0;i<SIZE;++i )
    for( unsigned int j=0;j<SIZE;++j )
      Pmib(i,j)*= exp( eig_mib(j) );
  Mexp = Pmib*Pinv;
  
  sotDEBUG(45) << "expMiB = " << Mexp;
  return ;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */

/* The derivative of the signal is such that: M v_dot + B v = f. We deduce:
 * v_dot =  M^-1 (f - Bv)
 * Using Exact method: 
 * dv = exp( M^-1.B.t) ( v0-B^-1.f ) + B^-1.f
 */

dynamicgraph::Vector& IntegratorForceExact::
computeVelocityExact( dynamicgraph::Vector& res,
		      const int& time )
{
  sotDEBUGIN(15);

  const dynamicgraph::Vector & force = forceSIN( time );
  const dynamicgraph::Matrix & massInverse = massInverseSIN( time );
  const dynamicgraph::Matrix & friction = frictionSIN( time );
  long int nf = force.size(), nv = friction.cols();
  res.resize(nv); res.setZero();

  if(! velocityPrecSIN )
    { 
      dynamicgraph::Vector zero( nv ); zero.fill(0);
      velocityPrecSIN = zero;
    } 
  const dynamicgraph::Vector & vel = velocityPrecSIN( time );
  double & dt = this->IntegratorForce::timeStep; // this is &

  sotDEBUG(15) << "force = " << force;
  sotDEBUG(15) << "vel = " << vel;
  sotDEBUG(25) << "Mi = " << massInverse;
  sotDEBUG(25) << "B = " << friction;
  sotDEBUG(25) << "dt = " << dt << std::endl;

  dynamicgraph::Matrix MiB( nv,nv );
  MiB = massInverse*friction;
  sotDEBUG(25) << "MiB = " << MiB;
  
  dynamicgraph::Matrix MiBexp( nv,nv );
  MiB*=-dt; expMatrix(MiB,MiBexp);
  sotDEBUG(25) << "expMiB = " << MiBexp;

  dynamicgraph::Matrix Binv( nv,nv ); Binv = friction.inverse();
  dynamicgraph::Vector Bif( nf ); Bif = Binv*force;
  sotDEBUG(25) << "Binv = " << Binv;
  sotDEBUG(25) << "Bif = " << Bif;
  
  dynamicgraph::Vector v0_bif = vel;
  v0_bif -= Bif;
  sotDEBUG(25) << "Kst = " << v0_bif;
  
  res.resize( MiBexp.rows() );
  res = MiBexp*v0_bif;

  res += Bif;
  velocityPrecSIN = res ;
  sotDEBUG(25) << "vfin = " << res;
 

  sotDEBUGOUT(15);
  return res;
}


/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
// void IntegratorForceExact::
// commandLine( const std::string& cmdLine,
// 	     std::istringstream& cmdArgs,
// 	     std::ostream& os )
// {
//   sotDEBUG(25) << "Cmd " << cmdLine <<std::endl;

//   if( cmdLine == "help" )
//     {
//       os << "IntegratorForceExact: " << std::endl;
//     }
//   else { IntegratorForce::commandLine( cmdLine,cmdArgs,os); }
// }

