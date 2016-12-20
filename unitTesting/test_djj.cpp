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

#include <string>
#include <cstdio>
#include <jrl/mal/matrixabstractlayer.hh>
#include "jrl/dynamics/dynamicsfactory.hh"
using namespace std;
using namespace dynamicsJRLJapan;

/* --- DISPLAY TREE --------------------------------------------------------- */
void RecursiveDisplayOfJoints(const CjrlJoint *aJoint)
{
  if( aJoint==0 )return;
   int NbChildren = aJoint->countChildJoints();
  cout << " rank : " << aJoint->rankInConfiguration() << endl;

  for(int i=0;i<NbChildren;i++)
    { RecursiveDisplayOfJoints(aJoint->childJoint(i));   }
}


void DisplayDynamicRobotInformation(CjrlDynamicRobot *aDynamicRobot)
{
  std::vector<CjrlJoint *> aVec = aDynamicRobot->jointVector();
  int r = aVec.size();
  cout << "Number of joints :" << r << endl;
  for(int i=0;i<r;i++)
    {
      cout << aVec[i]->rankInConfiguration()<< endl;
    }


}

void DisplayMatrix(MAL_MATRIX(&aJ,double))
{
  for(unsigned int i=0;i<6;i++)
    {
      for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(aJ);j++)
	{
	  if (aJ(i,j)==0.0)
	    printf("0 ");
	  else
	    printf("%10.5f ",aJ(i,j));
	}
      printf("\n");
    }

}

/* --- DISPLAY MASS PROPERTIES OF A CHAIN --- */
void GoDownTree(const CjrlJoint * startJoint)
{
  cout << "Mass-inertie property of joint ranked :"
       << startJoint->rankInConfiguration() << endl;
  cout << "Mass of the body: "
       << startJoint->linkedBody()->mass() << endl;
  cout << "llimit: "
       << startJoint->lowerBound(0)*180/M_PI
       << " ulimit: " << startJoint->upperBound(0)*180/M_PI << endl;
  cout << startJoint->currentTransformation() << endl;

  if (startJoint->countChildJoints()!=0)
    {
      const CjrlJoint * childJoint = startJoint->childJoint(0);
      GoDownTree(childJoint);
    }
}

/* --- MAIN ----------------------------------------------------------------- */
/* --- MAIN ----------------------------------------------------------------- */
/* --- MAIN ----------------------------------------------------------------- */
int main(int argc, char *argv[])
{
  if (argc!=4)
    {
      cerr << " This program takes 3 arguments: " << endl;
      cerr << "./TestHumanoidDynamicRobot PATH_TO_VRML_FILE "
	   << "VRML_FILE_NAME PATH_TO_SPECIFICITIES_XML" << endl;
      exit(0);
    }

  string aPath=argv[1];
  string aName=argv[2];

//   DynamicMultiBody * aDMB
//     = new DynamicMultiBody();
//   aDMB->parserVRML(aPath,aName,"");
//   HumanoidDynamicMultiBody *aHDMB
//     = new HumanoidDynamicMultiBody(aDMB,aSpecificitiesFileName);

  /* ------------------------------------------------------------------------ */
  dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
  CjrlHumanoidDynamicRobot * aHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();


  // DynamicMultiBody * aDMB
//     = (DynamicMultiBody *) aHDMB->getDynamicMultiBody();
  string SpecificitiesFile=argv[3];
  string RankFile=argv[3];
  // Parsing the file.
  string RobotFileName = aPath + aName;
  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR,RobotFileName,RankFile,SpecificitiesFile);

  cout << "-> Finished the initialization"<< endl;
  /* ------------------------------------------------------------------------ */

  // Display tree of the joints.
  CjrlJoint* rootJoint = aHDR->rootJoint();
  RecursiveDisplayOfJoints(rootJoint);

  // Test the computation of the jacobian.
  double dInitPos[40] = {
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // right arm
    15.0,  10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // left arm

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  };

  int NbOfDofs = aHDR->numberDof();
  cout << "NbOfDofs :" << NbOfDofs << endl;

  /* Set current conf to dInitPos. */
  MAL_VECTOR_DIM(aCurrentConf,double,NbOfDofs);
  for( int i=0;i<((NbOfDofs<46)?NbOfDofs:46);++i )
    if( i<6 ) aCurrentConf[i] = 0.0;
    else aCurrentConf[i] = dInitPos[i-6]*M_PI/180.0;
  aHDR->currentConfiguration(aCurrentConf);

  /* Set current velocity to 0. */
  MAL_VECTOR_DIM(aCurrentVel,double,NbOfDofs);
  for(int i=0;i<NbOfDofs;i++) aCurrentVel[i] = 0.0;
  aHDR->currentVelocity(aCurrentVel);

  /* Compute ZMP and CoM */
  MAL_S3_VECTOR(ZMPval,double);
  string Property("ComputeZMP");
  string Value("true");
  aHDR->setProperty(Property,Value);
  aHDR->computeForwardKinematics();
  ZMPval = aHDR->zeroMomentumPoint();
  cout << "First value of ZMP : " << ZMPval <<endl;
  cout << "Should be equal to the CoM (on x-y): "
       << aHDR->positionCenterOfMass() << endl;

  /* Get Rhand joint. */

  cout << "****************************" << endl;
  cout << "Rank of the left hand "<< endl;
  cout << aHDR->leftWrist()->rankInConfiguration() << endl;

  vector<CjrlJoint *> aVec = aHDR->jointVector();
  CjrlJoint * aJoint = aVec[22];
  aJoint->computeJacobianJointWrtConfig();
  MAL_MATRIX(aJ,double);
  aJ = aJoint->jacobianJointWrtConfig();
  DisplayMatrix(aJ);

  /* Get Waist joint. */
  cout << "****************************" << endl;
  rootJoint->computeJacobianJointWrtConfig();
  aJ = rootJoint->jacobianJointWrtConfig();
  cout << "Rank of Root: " << rootJoint->rankInConfiguration() << endl;
  aJoint = aHDR->waist();

  /* Get CoM jacobian. */
  cout << "****************************" << endl;
  matrixNxP jacobian;
  aHDR->getJacobianCenterOfMass(*aHDR->rootJoint(), jacobian);
  cout << "Value of the CoM's Jacobian:" << endl
       << jacobian << endl;

  /* Display the mass property of the leg. */
  cout << "****************************" << endl;
  GoDownTree(aHDR->rootJoint());
  cout << "Mass of the robot " << aHDR->mass() << endl;
  cout << "Force " << aHDR->mass()*9.81 << endl;

  cout << "****************************" << endl;
  MAL_VECTOR_FILL(aCurrentVel,0.0);
  MAL_VECTOR_DIM(aCurrentAcc,double,NbOfDofs);
  MAL_VECTOR_FILL(aCurrentAcc,0.0);

  // This is mandatory for this implementation of computeForwardKinematics
  // to compute the derivative of the momentum.
  string Properties[4] = {"TimeStep","ComputeAcceleration",
			  "ComputeBackwardDynamics","ComputeZMP"};
  string Values[4] = {"0.005","false","false","true"};
  for(unsigned int i=0;i<4;i++)
    aHDR->setProperty(Properties[i],Values[i]);

  for(int i=0;i<4;i++)
    {
      aHDR->currentVelocity(aCurrentVel);
      aHDR->currentAcceleration(aCurrentAcc);
      aHDR->computeForwardKinematics();
      ZMPval = aHDR->zeroMomentumPoint();
      cout << i << "-th value of ZMP : " << ZMPval <<endl;
      cout << "Should be equal to the CoM: " << aHDR->positionCenterOfMass() << endl;
    }


  // The End!
  delete aHDR;

  return true;
}
