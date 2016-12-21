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

#include <fstream>
#include <vector>
#include <map>

#include <sot-dynamic-pinocchio/matrix-inertia.h>
//#include <jrl/dynamics/Joint.h>
//#include <jrl/dynamics/HumanoidDynamicMultiBody.h>
#include <abstract-robot-dynamics/robot-dynamics-object-constructor.hh>

#include <sot/core/debug.hh>

using namespace dynamicsJRLJapan;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

static matrix3d skewSymmetric(const vector3d& v)
{
  matrix3d res;

  res(0, 0) = 0.0;
  res(1, 1) = 0.0;
  res(2, 2) = 0.0;

  res(0, 1) = -v[2];
  res(0, 2) = v[1];

  res(1, 0) = v[2];
  res(1, 2) = -v[0];

  res(2, 0) = -v[1];
  res(2, 1) = v[0];

  return res;
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */


MatrixInertia::
MatrixInertia( CjrlHumanoidDynamicRobot* aHDR )
  :aHDR_( aHDR )
  ,aHDMB_( 0x0 )
{
  sotDEBUGIN(25);
  if( aHDR!=NULL ) init( aHDR );
  sotDEBUGOUT(25);
}

void MatrixInertia::
init( CjrlHumanoidDynamicRobot* aHDR )
{
  sotDEBUGIN(25);
  aHDR_ = aHDR;
  aHDMB_ = dynamic_cast<dynamicsJRLJapan::HumanoidDynamicMultiBody*>(aHDR_);

  /* STEP 2: get the joints and resize internal vectors according to
   * number of joints. */
  joints_ = aHDMB_->jointVector();
  sotDEBUG(25) << "Joints:" << joints_.size() << endl;

  parentIndex_.resize(joints_.size());
  inertia_.resize(joints_.size() + 5, joints_.size() + 5);
  phi.resize( joints_.size() );
  iVpi.resize( joints_.size() );
  iVpiT.resize( joints_.size() );
  Ic.resize( joints_.size() );

  /* STEP 3: create the index of parents. */
  initParents();

  /* STEP 4: initialize phi (dof table) for each joint. */
  initDofTable();
  sotDEBUGOUT(25);
}

void MatrixInertia::
initParents( void )
{
  sotDEBUGIN(25);
  std::map<CjrlJoint*, int> m;
  for(size_t i = 0; i < joints_.size(); ++i)
    m[joints_[i]] = i;

  sotDEBUG(15) << "Parent Indexes:" << std::endl;
  for(size_t i = 0; i < joints_.size(); ++i)
    {
      if(joints_[i]->parentJoint() == 0x0)
	{
	  parentIndex_[i] = -1;
	  sotDEBUG(15) << "parent of\t" << i << "\t(" 
		       << static_cast<Joint*>(joints_[i])->getName() 
		       << "):\t" << -1 << std::endl;
	}
      else
	{
	  parentIndex_[i] = m[joints_[i]->parentJoint()];
	  sotDEBUG(15) << "parent of\t" << i << ":\t(" 
		       << static_cast<Joint*>(joints_[i])->getName() 
		       << "):\t" << m[joints_[i]->parentJoint()]
		       << "\t(" << static_cast<Joint*>
	    (joints_[m[joints_[i]->parentJoint()]])->getName() 
		       << ")" << std::endl;
	}
    }
  sotDEBUGOUT(25);
}

void MatrixInertia::
initDofTable( void )
{
  sotDEBUGIN(25);
  for(size_t i = 0; i < joints_.size(); ++i)
    {
      Joint* j = static_cast<Joint*>(joints_[i]);
      vector3d z =  j->axe();
      dynamicgraph::Vector & phi_i = phi[i];
      phi_i.resize(6);      phi_i.fill(0.);
      for( int n=0;n<3;++n ) phi_i(n+3) = z(n);
      sotDEBUG(25) << phi_i <<endl;
    }
  sotDEBUGOUT(25);
}

MatrixInertia::~MatrixInertia()
{}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
void MatrixInertia::update( void )
{
  sotDEBUGIN(25);
  const vectorN  & currentConf = aHDMB_->currentConfiguration();
  sotDEBUG(45) << "q = [ " << currentConf << endl;

  sotDEBUG(15) << "Spatial transforms:" << std::endl;
  const size_t SIZE = joints_.size();
  for(size_t i = 0; i < SIZE; ++i)
    {
      sotDEBUG(25) << "Joint " << i << ": rank = " 
		   << joints_[i]->rankInConfiguration() <<endl;
      Joint* j = static_cast<Joint*>(joints_[i]);

      /* iRpi: Rotation from joint i to parent of i. */
      MatrixRotation piRi; 
      {
	/* phi_[1] = phi_(1:3) is the rotation axis of the joint. */
	matrix3d Ri; j->getStaticRotation(Ri);
	vector3d axeJ = Ri*j->axe();
	const double & q = currentConf(joints_[i]->rankInConfiguration());
	sotDEBUG(45) << "q"<<i<<" = " <<q <<endl;
	matrix3d piRi_tmp;
	j->RodriguesRotation(axeJ,q,piRi_tmp);
	for( unsigned int loopi=0;loopi<3;++loopi )
	  for( unsigned int loopj=0;loopj<3;++loopj ) 
	    piRi( loopi,loopj ) = piRi_tmp(loopi,loopj);
      }
      dynamicgraph::Vector piTi(3); 
      {
	vector3d piTi_tmp; 
	j->getStaticTranslation( piTi_tmp );
	for( unsigned int loopi=0;loopi<3;++loopi ) piTi(loopi) = piTi_tmp(loopi);
      }
      /* Twist matrix iXpi: transfo of the velocities between i en pi (parent i). 
       * iXpi = [   iRpi     Skew( iRpi.t )*iRpi  ]
       *        [     0        iRpi               ]  */
      MatrixHomogeneous piMi; piMi.buildFrom( piRi,piTi );
      MatrixHomogeneous iMpi; piMi.inverse(iMpi);
      sotDEBUG(45) << "piMi = " <<piMi <<endl;
      sotDEBUG(45) << "iMpi = " <<iMpi <<endl;

      iVpi[i].buildFrom( iMpi );
      iVpi[i].transpose( iVpiT[i] );
      sotDEBUG(25) << "iVpi" << i << " = " <<iVpi[i] <<endl;
    }
  sotDEBUGOUT(25);
}

void MatrixInertia::computeInertiaMatrix()
{
  sotDEBUGIN(25);
  inertia_.fill(0.0);

  const size_t SIZE = joints_.size();

  /* Compute the local 6D inertia matrices. */
  for( size_t i = 0;i<SIZE;++i )
    {
      /* Position of the mass in the joint frame. */
      vector3d com = joints_[i]->linkedBody()->localCenterOfMass();
      matrix3d Sc = skewSymmetric(com);
      /* Inertia of the link. */
      matrix3d Icm = joints_[i]->linkedBody()->inertiaMatrix();
      
      double m = joints_[i]->linkedBody()->mass();

      
      /* Ic_ is the inertia 6D matrix of the joint in the joint frame. 
       *   Ic = [  Ai+mSc.Sc'   mSc   ]
       *        [     mSc'      mId   ]   
       */
      sotDEBUG(45) << "com"<<i<<" = [ " << com <<"]"<<endl;
      sotDEBUG(45) << "Sc"<<i<<" = [ " << Sc<<"]"<<endl;
      sotDEBUG(45) << "Icm"<<i<<" = [ " << Icm<<"]"<<endl;
      matrix3d Sct = Sc.Transpose();
      matrix3d Irr = Sc*Sct;
      dynamicgraph::Matrix & Ici = Ic[i];  Ici.resize(6,6);
      for( unsigned int loopi=0;loopi<3;++loopi )
	for( unsigned int loopj=0;loopj<3;++loopj )
	  {
	    /*TT*/if( loopi==loopj ) Ici( loopi,loopj ) = m; 
	    else Ici( loopi,loopj ) = 0.;
	    /*TR*/Ici( loopi,loopj+3 ) = m*Sct( loopi,loopj );
	    /*RT*/Ici( loopi+3,loopj ) = m*Sc( loopi,loopj );
	    /*RR*/Ici( loopi+3,loopj+3 ) = m*Irr( loopi,loopj )+ Icm( loopi,loopj );
	  }
      
      sotDEBUG(25) << "Ic" << i << " = " << Ici;
    }
  
  dynamicgraph::Vector Fi(6);
  dynamicgraph::Matrix iVpiT_Ici(6,6);
  dynamicgraph::Matrix iVpiT_Ici_iVpi(6,6);

  for( int i=SIZE-1;i>=1;--i ) 
    {
      const unsigned int iRank = joints_[i]->rankInConfiguration();

      dynamicgraph::Matrix & Ici = Ic[i]; 
      sotMatrixTwist & iVpii = iVpi[i]; 
      MatrixForce & iVpiiT = iVpiT[i]; 
      dynamicgraph::Vector & phii = phi[i];
      /* F = Ic_i . phi_i */
      Fi = Ici*phii;  
      /* H_ii = phi_i' . F */
      /*DEBUGinertia_(i + 5, i + 5) = phii.scalarProduct(Fi);*/
   
 
      inertia_(iRank,iRank) = phii.scalarProduct(Fi);
      sotDEBUG(30) << "phi"<<i<<" = " << phii;
      sotDEBUG(35) << "Fi"<<i<<" =  " << Fi << endl;
      sotDEBUG(25) << "IcA"<<i<<" = " << Ici << endl;
      sotDEBUG(45) << "Joint " << i << " in " << iRank <<endl;

      /* Ic_pi = Ic_pi + iXpi' Ic_i iXpi */
      iVpiiT.multiply(Ici,iVpiT_Ici);
      iVpiT_Ici.multiply( iVpii,iVpiT_Ici_iVpi );

      sotDEBUG(45) << "Icpi"<<parentIndex_[i]<<" = "  << Ic[parentIndex_[i]]  ;
      Ic[ parentIndex_[i] ] += iVpiT_Ici_iVpi;

      sotDEBUG(45) << "Icpi"<<parentIndex_[i]<<"_"<<i<<" = "  << Ic[parentIndex_[i]]  ;
      sotDEBUG(45) << "Vpi"<<i<<" = "  << iVpii ;

      size_t j = i;
      while(parentIndex_[j] != 0)
	{
	  /* F = jXpj' . F */
	  Fi = iVpiT[j]*Fi;
	  /* j = pj */
	  j = parentIndex_[j];
	  /* Hij = Hji = F' phi_j */
	  inertia_(iRank,joints_[j]->rankInConfiguration()) 
	    = inertia_(joints_[j]->rankInConfiguration(),iRank) 
	    = Fi.scalarProduct( phi[j]);
	  sotDEBUG(35) << "Fi =  " << Fi << endl;
	  sotDEBUG(35) << "FiXphi =  " <<  inertia_(j + 5, i + 5) << endl;
  	}

      /* When parentIndex_[j] == 0: FREE FLYER. */
      Fi = iVpiT[j]*Fi;
      for(size_t k = 0; k < 6; ++k)
	{
	  inertia_(iRank, k) = inertia_(k,iRank) = Fi(k);
	}
    }

  /* --- FREE FLYER = Ic0 --- */
  const dynamicgraph::Matrix & Ic0 = Ic[0];
  for(size_t i = 0; i < 6; ++i)
    for(size_t j = 0; j < 6; ++j)
      {inertia_(i, j) = Ic0(i,j);}


  sotDEBUGOUT(25);
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */


void MatrixInertia::
getInertiaMatrix(double* A)
{
  for(size_t i = 0; i < (joints_.size() + 5); ++i)
    for(size_t j = 0; j < (joints_.size() + 5); ++j)
      A[i * (joints_.size() + 5) + j] = inertia_(i, j);
}

const maal::boost::Matrix& MatrixInertia::
getInertiaMatrix( void )
{ return inertia_; }
