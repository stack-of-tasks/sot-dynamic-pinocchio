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

#ifndef __SOT_DYNAMIC_H__
#define __SOT_DYNAMIC_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* STD */
#include <string>
#include <map>
/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/*Python API*/

/* SOT */
#include <sot/core/flags.hh>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/core/exception-dynamic.hh>
#include <sot/core/matrix-geometry.hh>

/* PINOCCHIO */
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/joint/joint-variant.hpp>
#include <pinocchio/multibody/parser/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/operational-frames.hpp>

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


namespace dynamicgraph {   namespace sot {
    namespace dg = dynamicgraph;
    
    namespace command {
      class SetFile;
      class CreateOpPoint;
    }
    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */
    
    
    
    /*! @ingroup signals
      \brief This class provides an inverse dynamic model of the robot.
      More precisely it wraps the newton euler algorithm implemented
      by the dynamicsJRLJapan library to make it accessible in the stack of tasks.
      The robot is described by a VRML file.
    */
class SOTDYNAMIC_EXPORT Dynamic
 :public dg::Entity {
  friend class sot::command::SetFile;
  friend class sot::command::CreateOpPoint;
  //  friend class sot::command::InitializeRobot;
  
    public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DYNAMIC_GRAPH_ENTITY_DECL();

  /*  --- MODEL ATRIBUTES --- */
  se3::Model  m_model;
  se3::Data*   m_data;
  std::string  m_urdfPath;

  /*  --- MODEL ATRIBUTES --- */

    public:
  /* --- SIGNAL ACTIVATION --- */
  dg::SignalTimeDependent< dg::Matrix,int >&
    createEndeffJacobianSignal( const std::string& signame, const std::string& );
  dg::SignalTimeDependent< dg::Matrix,int >&
    createJacobianSignal( const std::string& signame, const std::string& );
  void destroyJacobianSignal( const std::string& signame );
  
  dg::SignalTimeDependent< MatrixHomogeneous,int >&
    createPositionSignal( const std::string&,const std::string& );
  void destroyPositionSignal( const std::string& signame );
  
  dg::SignalTimeDependent< dg::Vector,int >&
    createVelocitySignal( const std::string&,const std::string& );
  void destroyVelocitySignal( const std::string& signame );
  
  dg::SignalTimeDependent< dg::Vector,int >&
    createAccelerationSignal( const std::string&, const std::string& );
  void destroyAccelerationSignal( const std::string& signame );

  /*! @} */
  bool init;
  std::list< dg::SignalBase<int>*  > genericSignalRefs;



    public:
  /* --- SIGNAL --- */
  typedef int Dummy;
  dg::SignalPtr<dg::Vector,int> jointPositionSIN;
  dg::SignalPtr<dg::Vector,int> freeFlyerPositionSIN;
  dg::SignalPtr<dg::Vector,int> jointVelocitySIN;
  dg::SignalPtr<dg::Vector,int> freeFlyerVelocitySIN;
  dg::SignalPtr<dg::Vector,int> jointAccelerationSIN;
  dg::SignalPtr<dg::Vector,int> freeFlyerAccelerationSIN;
  
  dg::SignalTimeDependent<Dummy,int> newtonEulerSINTERN;
  
  int& computeNewtonEuler( int& dummy,int time );
  
  dg::SignalTimeDependent<dg::Vector,int> zmpSOUT;
  dg::SignalTimeDependent<dg::Matrix,int> JcomSOUT;
  dg::SignalTimeDependent<dg::Vector,int> comSOUT;
  dg::SignalTimeDependent<dg::Matrix,int> inertiaSOUT;
  
  dg::SignalTimeDependent<dg::Matrix,int>& jacobiansSOUT( const std::string& name );
  dg::SignalTimeDependent<MatrixHomogeneous,int>& positionsSOUT( const std::string& name );
  dg::SignalTimeDependent<dg::Vector,int>& velocitiesSOUT( const std::string& name );
  dg::SignalTimeDependent<dg::Vector,int>& accelerationsSOUT( const std::string& name );
  
  dg::SignalTimeDependent<double,int> footHeightSOUT;
  dg::SignalTimeDependent<dg::Vector,int> upperJlSOUT;
  dg::SignalTimeDependent<dg::Vector,int> lowerJlSOUT;
  dg::SignalTimeDependent<dg::Vector,int> upperVlSOUT;
  dg::SignalTimeDependent<dg::Vector,int> upperTlSOUT;

  dg::Signal<dg::Vector,int> inertiaRotorSOUT;
  dg::Signal<dg::Vector,int> gearRatioSOUT;
  dg::SignalTimeDependent<dg::Matrix,int> inertiaRealSOUT;
  dg::SignalTimeDependent<dg::Vector,int> MomentaSOUT;
  dg::SignalTimeDependent<dg::Vector,int> AngularMomentumSOUT;
  dg::SignalTimeDependent<dg::Vector,int> dynamicDriftSOUT;


    public:
  /* --- CONSTRUCTOR --- */
  Dynamic( const std::string& name);
  virtual ~Dynamic( void );


  /* --- MODEL CREATION --- */

  /// \brief sets a urdf filepath to create robot model. Call parseUrdfFile to parse
  /// \param path file location.
  ///
  void setUrdfFile( const std::string& path );


  /// \brief parses a urdf filepath to create robot model. Call setUrdfFile to give path
  /// \param none.
  ///
  /// \note creates a pinocchio model. needs urdfdom libraries to parse.
  void parseUrdfFile(void);


  /// \brief Create an empty device
  void createRobot();
  void displayModel() const 
  { std::cout<<m_model<<std::endl; };

  /// \brief create a joint
  /// \param inJointName name of the joint,
  /// \param inJointType type of joint in ["freeflyer","rotation","translation","anchor"],
  /// \param inPosition position of the joint (4x4 matrix).
  ///
  /// \note joints are stored in a map with names as keys for retrieval by other
  void createJoint(const std::string& inJointName,
		   const std::string& inJointType,
		   const dg::Matrix& inPosition);
  
  /// \brief Add a child body.
  /// \param inParentName name of the body to which a child is added.
  /// \param inJointName name of the joint added.
  /// \param inChildName name of the body added.
  /// \param mass mass of the child body. default 0.
  /// \param lever com position of the body. default zero vector.
  /// \param inertia3 matrix of the body. default zero matrix.
  void addBody(const std::string& inParentName,
	       const std::string& inJointName,
	       const std::string& inChildName,
	       const double mass,
	       const dg::Vector& lever,
	       const dg::Matrix& inertia3);
  

  /// \brief Add a child body.
  /// \param inParentName name of the body to which a child is added.
  /// \param inJointName name of the joint added.
  /// \param inChildName name of the body added.
  /// \note default mass=0, default inertia=Zero Matrix, default com=Zero Vector
  void addBody(const std::string& inParentName,
	       const std::string& inJointName,
	       const std::string& inChildName);
  

  /// \brief Set mass of a body
  ///
  /// \param inBodyName name of the body whose properties are being set,
  /// \param mass mass of the body. default 0.
  void setMass(const std::string& inBodyName,
	       const double mass = 0);


  /// \brief Set COM of the body in local frame
  ///
  /// \param inBodyName name of the body whose properties are being set,
  /// \param local COM vector
  void setLocalCenterOfMass(const std::string& inBodyName,
			    const dg::Vector& lever);

  /// \brief Set Inertia Matrix of the body
  ///
  /// \param inBodyName name of the body whose properties are being set,
  /// \param Inertia matrix
  void setInertiaMatrix(const std::string& inBodyName,
			const dg::Matrix& inertia3);

  /// \brief Set Inertia properties of a body
  ///
  /// \param inBodyName name of the body whose properties are being set,
  /// \param mass mass of the body. default 0.
  /// \param lever com position of the body. default zero vector,
  /// \param inertia3 inertia matrix of the body. default zero matrix.
  void setInertiaProperties(const std::string& inBodyName,
			    const double mass,
			    const dg::Vector& lever,
			    const dg::Matrix& inertia3);
  
    
  /// \brief Set the bounds of a joint degree of freedom
  /// \param the name of the joint
  /// \param  non-negative integer: the dof id in the joint
  /// \param  the minimal value of the dof
  /// \param: the maximal value of the dof
  void setDofBounds(const std::string& inJointName,
		    const unsigned int inDofId,
		    const double inMinValue, double inMaxValue);
    


  /// \brief Set lower bound of joint position
  ///
  /// \param inJointName name of the joint,
  /// \param vector containing lower limit bounds for all dofs of joint, or a double containing limits for a revolute joint.
  void setLowerPositionLimit(const std::string& inJointName,
			     const dg::Vector& lowPos);
  
  void setLowerPositionLimit(const std::string& inJointName,
			     const double lowPos);
  
  /// \brief Set upper bound of joint position
  ///
  /// \param inJointName name of the joint,
  /// \param upPos vector containing upper limit bounds for all dofs of joint, or a double containing limits for a revolute joint.
  void setUpperPositionLimit(const std::string& inJointName,
			     const dg::Vector& upPos);
  void setUpperPositionLimit(const std::string& inJointName,
			     const double upPos);

  /// \brief Set upper bound of joint velocities
  ///
  /// \param inJointName name of the joint,
  /// \param maxVel vector containing upper limit bounds for all dofs of joint, or a double containing limits for a revolute joint.
  void setMaxVelocityLimit(const std::string& inJointName,
			   const dg::Vector& maxVel);
  void setMaxVelocityLimit(const std::string& inJointName,
			   const double maxVel);


  /// \brief Set upper bound of joint effort
  ///
  /// \param inJointName name of the joint,
  /// \param maxEffort vector containing upper limit bounds for all dofs of joint, or a double containing limits for a revolute joint.
  void setMaxEffortLimit(const std::string& inJointName,
			 const dg::Vector& maxEffort);
  void setMaxEffortLimit(const std::string& inJointName,
			 const double maxEffort);
  
  /* --- GETTERS --- */

  /// \brief Get joint position lower limits
  ///
  /// \param[out] result vector
  /// \return result vector
  dg::Vector& getLowerPositionLimits(dg::Vector& res,const int&);

  /// \brief Get joint position upper limits
  ///
  /// \param[out] result vector
  /// \return result vector
  dg::Vector& getUpperPositionLimits(dg::Vector& res,const int&);

  /// \brief Get joint velocity upper limits
  ///
  /// \param[out] result vector
  /// \return result vector
  dg::Vector& getUpperVelocityLimits(dg::Vector& res,const int&);

  /// \brief Get joint effort upper limits
  ///
  /// \param[out] result vector
  /// \return result vector
  dg::Vector& getMaxEffortLimits(dg::Vector& res,const int&);


  //  dg::Vector& getAnklePositionInFootFrame() const;

    protected:
  dg::Matrix& computeGenericJacobian(bool isFrame,
				     int jointId,
				     dg::Matrix& res,int time );
  dg::Matrix& computeGenericEndeffJacobian(bool isFrame,
					   int jointId,
					   dg::Matrix& res,int time );
  MatrixHomogeneous& computeGenericPosition(bool isFrame, 
					    int jointId,
					    MatrixHomogeneous& res,int time );
  dg::Vector& computeGenericVelocity(int jointId,dg::Vector& res,int time );
  dg::Vector& computeGenericAcceleration(int jointId,dg::Vector& res,int time );

  dg::Vector& computeZmp( dg::Vector& res,int time );
  dg::Vector& computeMomenta( dg::Vector &res, int time);
  dg::Vector& computeAngularMomentum( dg::Vector &res, int time);
  dg::Matrix& computeJcom( dg::Matrix& res,int time );
  dg::Vector& computeCom( dg::Vector& res,int time );
  dg::Matrix& computeInertia( dg::Matrix& res,int time );
  dg::Matrix& computeInertiaReal( dg::Matrix& res,int time );
  double& computeFootHeight( double& res,int time );

  dg::Vector& computeTorqueDrift( dg::Vector& res,const int& time );

 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );
  void cmd_createOpPointSignals           ( const std::string& sig,const std::string& j );
  void cmd_createJacobianWorldSignal      ( const std::string& sig,const std::string& j );
  void cmd_createJacobianEndEffectorSignal( const std::string& sig,const std::string& j );
  void cmd_createPositionSignal           ( const std::string& sig,const std::string& j );

 private:
  /// \brief map of joints in construction.
  /// map: jointName -> (jointType,jointPosition (in parent frame), function_ptr to pinocchio Joint) 
  dg::Vector getPinocchioPos(int);
  dg::Vector getPinocchioVel(int);
  dg::Vector getPinocchioAcc(int);

  typedef std::pair<std::string,Eigen::Matrix4d> JointDetails;
  std::map<std::string, JointDetails> jointMap_;
  std::vector<std::string> jointTypes;


  //std::map<std::string,std::string> specificitiesMap;

};

  // std::ostream& operator<<(std::ostream& os, const CjrlJoint& r);
} /* namespace sot */} /* namespace dynamicgraph */



#endif // #ifndef __SOT_DYNAMIC_H__
