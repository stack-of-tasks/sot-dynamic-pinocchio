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

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <iostream>
#include <fstream>

/* JRL dynamic */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* JRL dynamic */

#include <jrl/dynamics/dynamicsfactory.hh>
namespace djj = dynamicsJRLJapan;

#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>
#include <abstract-robot-dynamics/robot-dynamics-object-constructor.hh>

using namespace std;

int main(int argc, char * argv[])
{
  if (argc!=6)
    {
      cerr << " This program takes 4 arguments: " << endl;
      cerr << "./TestHumanoidDynamicRobot PATH_TO_VRML_FILE VRML_FILE_NAME "<< endl;
      cerr << " PATH_TO_SPECIFICITIES_XML PATH PATH_TO_MAP_JOINT_2_RANK INITIAL_CONFIGURATION_FILE" << endl;
      exit(0);
    }	

  string aSpecificitiesFileName = argv[3];
  string aPath=argv[1];
  string aName=argv[2];
  string aMapFromJointToRank=argv[4];

  dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
  CjrlHumanoidDynamicRobot * aHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();

  string RobotFileName = aPath + aName;
  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR,RobotFileName,aMapFromJointToRank,aSpecificitiesFileName);


  CjrlHumanoidDynamicRobot * aHDR2 = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
  //  cout << "aHDMB2 Finished the initialization"<< endl;
  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR2,RobotFileName,aMapFromJointToRank,aSpecificitiesFileName);

  std::ifstream ReferenceStateFile;
  ReferenceStateFile.open("teleop-rstate.log");
  if (!ReferenceStateFile.is_open())
    {
      cout << "Could not open teleop-rstate.log" << endl;
      exit(0);
    }

  std::ifstream ActualStateFile;
  ActualStateFile.open("teleop-astate.log");
  if (!ActualStateFile.is_open())
    {
      cout << "Could not open teleop-state.log" << endl;
      exit(0);
    }

  std::ofstream FileActualRHPos,FileRefRHPos,FileRefLHPos;

  MAL_VECTOR_DIM(m_ReferenceStateConf,double,46);
  MAL_VECTOR_DIM(m_ReferenceStateConfPrev,double,46);
  MAL_VECTOR_DIM(m_ReferenceStateSpeed,double,46);
  MAL_VECTOR_DIM(m_ReferenceStateSpeedPrev,double,46);
  MAL_VECTOR_DIM(m_ReferenceStateAcc,double,46);
  MAL_VECTOR_DIM(m_ActualStateConf,double,46);
  MAL_VECTOR_DIM(m_ActualStateConfPrev,double,46);
  MAL_VECTOR_DIM(m_ActualStateSpeed,double,46);
  MAL_VECTOR_DIM(m_ActualStateSpeedPrev,double,46);
  MAL_VECTOR_DIM(m_ActualStateAcc,double,46);

  MAL_VECTOR_DIM(m_ReferenceStateData,double,100);
  MAL_VECTOR_DIM(m_ActualStateData,double,131);
  
  unsigned int NbIterations=0;

  const CjrlJoint *ActualLeftFoot, *ActualRightFoot, *ActualRightHand;
  const CjrlJoint *ReferenceLeftFoot, *ReferenceRightFoot, *ReferenceRightHand ,* ReferenceLeftHand;

  ReferenceRightFoot = aHDR->rightFoot()->associatedAnkle();
  ReferenceLeftFoot = aHDR->leftFoot()->associatedAnkle();
  ReferenceRightHand = aHDR->rightWrist();
  ReferenceLeftHand = aHDR->leftWrist();

  ActualRightFoot = aHDR2->rightFoot()->associatedAnkle();
  ActualLeftFoot = aHDR2->leftFoot()->associatedAnkle();
  ActualRightHand = aHDR2->rightWrist();
  
  matrix4d ReferenceSupportFootPosition;
  matrix4d ReferenceRightHandPosition;
  matrix4d ReferenceLeftHandPosition;

  matrix4d ActualSupportFootPosition;
  matrix4d ActualRightHandPosition;

  FileActualRHPos.open("ActualRHPos.dat");
  FileRefRHPos.open("RefRHPos.dat");
  FileRefLHPos.open("RefLHPos.dat");

  string aProperty("ComputeZMP");
  string aValue("true");
  aHDR->setProperty(aProperty,aValue);
  aHDR2->setProperty(aProperty,aValue);
  while(!ActualStateFile.eof())
    {
      for(unsigned int i=0;i<100;i++)
	ReferenceStateFile >> m_ReferenceStateData[i];
     
      for(unsigned int i=0;i<40;i++)
	m_ReferenceStateConf[i+6] = m_ReferenceStateData[i];
	
      
      if (NbIterations>0)
	{
	  for(unsigned int i=0;i<46;i++)
	    m_ReferenceStateSpeed[i] = (m_ReferenceStateConf[i] - m_ReferenceStateConfPrev[i])/0.005;
	}
      else
	{
	  for(unsigned int i=0;i<46;i++)
	    m_ReferenceStateSpeed[i] = 0.0;
	}
      
      //      cout << "ReferenceStateConf: " << m_ReferenceStateConf << endl;
      /* Update the current configuration vector */
      aHDR->currentConfiguration(m_ReferenceStateConf);

      /* Update the current velocity vector */
      aHDR->currentVelocity(m_ReferenceStateSpeed);

      for(unsigned int i=0;i<46;i++)
	{
	  m_ReferenceStateConfPrev[i] = m_ReferenceStateConf[i];
	}

      aHDR->computeForwardKinematics();

      ReferenceRightHandPosition = ReferenceRightHand->currentTransformation();
      ReferenceLeftHandPosition = ReferenceLeftHand->currentTransformation();
      ReferenceSupportFootPosition = ReferenceRightFoot->currentTransformation();

      FileRefRHPos << MAL_S4x4_MATRIX_ACCESS_I_J(ReferenceRightHandPosition,0,3) << " ";
      FileRefRHPos << MAL_S4x4_MATRIX_ACCESS_I_J(ReferenceRightHandPosition,1,3) << " " ;
      FileRefRHPos << MAL_S4x4_MATRIX_ACCESS_I_J(ReferenceRightHandPosition,2,3) << endl;
      

      FileRefLHPos << MAL_S4x4_MATRIX_ACCESS_I_J(ReferenceLeftHandPosition,0,3) << " ";
      FileRefLHPos << MAL_S4x4_MATRIX_ACCESS_I_J(ReferenceLeftHandPosition,1,3) << " " ;
      FileRefLHPos << MAL_S4x4_MATRIX_ACCESS_I_J(ReferenceLeftHandPosition,2,3) << endl;

      /*
      cout << ReferenceRightHandPosition(0,3) << " "
	   << ReferenceRightHandPosition(1,3) << " "
	   << ReferenceRightHandPosition(2,3) << endl;

      cout << ReferenceSupportFootPosition(0,3) << " "
	   << ReferenceSupportFootPosition(1,3) << " "
	   << ReferenceSupportFootPosition(2,3) << endl;
      */
      // Actual state.
      for(unsigned int i=0;i<131;i++)
	{
	  ActualStateFile >> m_ActualStateData[i];
	}
      for(unsigned int i=0;i<40;i++)
	m_ActualStateConf[i+6] = m_ActualStateData[i];
	
      
      if (NbIterations>0)
	{
	  for(unsigned int i=6;i<46;i++)
	    m_ActualStateSpeed[i] = (m_ActualStateConf[i] - m_ActualStateConfPrev[i])/0.005;
	}
      else
	{
	  for(unsigned int i=0;i<46;i++)
	    m_ActualStateSpeed[i] = 0.0;
	}
      /* Update the current configuration vector */
      aHDR2->currentConfiguration(m_ActualStateConf);

      /* Update the current velocity vector */
      aHDR2->currentVelocity(m_ActualStateSpeed);

      for(unsigned int i=0;i<46;i++)
	{
	  m_ActualStateConfPrev[i] = m_ActualStateConf[i];
	}
      
      aHDR2->computeForwardKinematics();

      ActualRightHandPosition = ActualRightHand->currentTransformation();

      FileActualRHPos << MAL_S4x4_MATRIX_ACCESS_I_J(ActualRightHandPosition,0,3) << " ";
      FileActualRHPos << MAL_S4x4_MATRIX_ACCESS_I_J(ActualRightHandPosition,1,3) << " " ;
      FileActualRHPos << MAL_S4x4_MATRIX_ACCESS_I_J(ActualRightHandPosition,2,3) << endl;

      NbIterations++;
    }
  ActualStateFile.close();
  ReferenceStateFile.close();
  FileActualRHPos.close();
  FileRefRHPos.close();
  FileRefLHPos.close();
  
}
