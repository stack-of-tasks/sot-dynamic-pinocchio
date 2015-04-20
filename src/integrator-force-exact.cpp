/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-dynamic.
 * sot-dynamic is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-dynamic is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-dynamic.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <sot-dynamic/integrator-force-exact.h>
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
namespace ublas = boost::numeric::ublas;


extern "C"
{
  void dgeev_( const char* jobvl, const char* jobvr, const int* n, double* a,
	       const int* lda, double* wr, double* wi, double* vl, const int* ldvl,
	       double* vr, const int* ldvr, double* work, const int* lwork, int* info );
}


int geev(::boost::numeric::ublas::matrix<double, ::boost::numeric::ublas::column_major> &a,
	 ublas::vector< std::complex<double> > &w,
	 ::boost::numeric::ublas::matrix<double,::boost::numeric::ublas::column_major> &vl2,
	 ::boost::numeric::ublas::matrix<double,::boost::numeric::ublas::column_major> &vr2
	 )
{
  char jobvl='V';
  char jobvr='V';
  int const n = a.size1();
  
  ::boost::numeric::ublas::vector<double> wr(n);
  ::boost::numeric::ublas::vector<double> wi(n);
  double* vl_real = MRAWDATA(vl2);
  const int ldvl =vl2.size1();
  double* vr_real = MRAWDATA(vr2);
  const int ldvr = vr2.size1();
  
  // workspace query
  int lwork = -1;
  double work_temp;
  int result=0;
  dgeev_(&jobvl, &jobvr, &n,
	 MRAWDATA(a), &n, 
	 VRAWDATA(wr), VRAWDATA(wi), 
	 vl_real, &ldvl, vr_real, &ldvr,
	 &work_temp, &lwork, &result);
  if (result != 0)
    return result;
  
  lwork = (int) work_temp;
  ::boost::numeric::ublas::vector<double> work(lwork);
  dgeev_(&jobvl, &jobvr, &n,
	 MRAWDATA(a), &n, 
	 VRAWDATA(wr), VRAWDATA(wi), 
	 vl_real, &ldvl, vr_real, &ldvr,
	 VRAWDATA(work), &lwork,
	 &result);

  for (int i = 0; i < n; i++)
    w[i] = std::complex<double>(wr[i], wi[i]);
  
  return result;
}

static void eigenDecomp( const ml::Matrix& M,
			 ml::Matrix& P,
			 ml::Vector& eig )
{
  unsigned int SIZE = M.nbCols();
  ublas::matrix<double, ublas::column_major> Y=M.matrix;
  ublas::vector< std::complex<double> > evals(SIZE);
  ublas::matrix<double, ublas::column_major> vl(SIZE,SIZE);
  ublas::matrix<double, ublas::column_major> vr(SIZE,SIZE);
  
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

static void expMatrix( const ml::Matrix& MiB,
		       ml::Matrix& Mexp )
{
  unsigned int SIZE = MiB.nbCols();

  ml::Matrix Pmib(MiB.nbCols(),MiB.nbCols());
  ml::Vector eig_mib(MiB.nbCols());
  eigenDecomp( MiB,Pmib,eig_mib );
  sotDEBUG(45) << "V = " << Pmib;
  sotDEBUG(45) << "d = " << eig_mib;

  ml::Matrix Pinv(SIZE,SIZE); Pmib.inverse(Pinv);
  sotDEBUG(45) << "Vinv = " << Pinv;

  Mexp.resize(SIZE,SIZE);
  for( unsigned int i=0;i<SIZE;++i )
    for( unsigned int j=0;j<SIZE;++j )
      Pmib(i,j)*= exp( eig_mib(j) );
  Pmib.multiply(Pinv,Mexp);
  
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

ml::Vector& IntegratorForceExact::
computeVelocityExact( ml::Vector& res,
		      const int& time )
{
  sotDEBUGIN(15);

  const ml::Vector & force = forceSIN( time );
  const ml::Matrix & massInverse = massInverseSIN( time );
  const ml::Matrix & friction = frictionSIN( time );
  unsigned int nf = force.size(), nv = friction.nbCols();
  res.resize(nv); res.fill(0);

  if(! velocityPrecSIN )
    { 
      ml::Vector zero( nv ); zero.fill(0);
      velocityPrecSIN = zero;
    } 
  const ml::Vector & vel = velocityPrecSIN( time );
  double & dt = this->IntegratorForce::timeStep; // this is &

  sotDEBUG(15) << "force = " << force;
  sotDEBUG(15) << "vel = " << vel;
  sotDEBUG(25) << "Mi = " << massInverse;
  sotDEBUG(25) << "B = " << friction;
  sotDEBUG(25) << "dt = " << dt << std::endl;

  ml::Matrix MiB( nv,nv );
  massInverse.multiply(friction,MiB);
  sotDEBUG(25) << "MiB = " << MiB;
  
  ml::Matrix MiBexp( nv,nv );
  MiB*=-dt; expMatrix(MiB,MiBexp);
  sotDEBUG(25) << "expMiB = " << MiBexp;

  ml::Matrix Binv( nv,nv ); friction.inverse(Binv);
  ml::Vector Bif( nf ); Binv.multiply( force,Bif );
  sotDEBUG(25) << "Binv = " << Binv;
  sotDEBUG(25) << "Bif = " << Bif;
  
  ml::Vector v0_bif = vel;
  v0_bif -= Bif;
  sotDEBUG(25) << "Kst = " << v0_bif;
  
  res.resize( MiBexp.nbRows() );
  MiBexp.multiply( v0_bif,res );

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

