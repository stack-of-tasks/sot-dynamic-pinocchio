#include <sot/core/debug.hh>
#include <sot-dynamic-pinocchio/dynamic.h>

#include <boost/version.hpp>
//#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <jrl/mal/matrixabstractlayer.hh>

#include <dynamic-graph/all-commands.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

const std::string dynamicgraph::sot::Dynamic::CLASS_NAME = "DynamicLib";

using namespace std;


Dynamic::Dynamic( const std::string & name, bool build ):Entity(name)
  ,m_data(NULL)
  ,jointPositionSIN         (NULL,"sotDynamic("+name+")::input(vector)::position")
  ,freeFlyerPositionSIN     (NULL,"sotDynamic("+name+")::input(vector)::ffposition")
  ,jointVelocitySIN         (NULL,"sotDynamic("+name+")::input(vector)::velocity")
  ,freeFlyerVelocitySIN     (NULL,"sotDynamic("+name+")::input(vector)::ffvelocity")
  ,jointAccelerationSIN     (NULL,"sotDynamic("+name+")::input(vector)::acceleration")
  ,freeFlyerAccelerationSIN (NULL,"sotDynamic("+name+")::input(vector)::ffacceleration")
  ,newtonEulerSINTERN( boost::bind(&Dynamic::computeNewtonEuler,this,_1,_2),
                       sotNOSIGNAL,
                       "sotDynamic("+name+")::intern(dummy)::newtoneuleur" )

  ,zmpSOUT( boost::bind(&Dynamic::computeZmp,this,_1,_2),
            newtonEulerSINTERN,
            "sotDynamic("+name+")::output(vector)::zmp" )
  ,JcomSOUT( boost::bind(&Dynamic::computeJcom,this,_1,_2),
             newtonEulerSINTERN,
             "sotDynamic("+name+")::output(matrix)::Jcom" )
  ,comSOUT( boost::bind(&Dynamic::computeCom,this,_1,_2),
            newtonEulerSINTERN,
            "sotDynamic("+name+")::output(vector)::com" )
  ,inertiaSOUT( boost::bind(&Dynamic::computeInertia,this,_1,_2),
                newtonEulerSINTERN,
                "sotDynamic("+name+")::output(matrix)::inertia" )
  ,footHeightSOUT( boost::bind(&Dynamic::computeFootHeight,this,_1,_2),
                   newtonEulerSINTERN,
                   "sotDynamic("+name+")::output(double)::footHeight" )

  ,upperJlSOUT( boost::bind(&Dynamic::getUpperJointLimits,this,_1,_2),
                sotNOSIGNAL,
                "sotDynamic("+name+")::output(vector)::upperJl" )

  ,lowerJlSOUT( boost::bind(&Dynamic::getLowerJointLimits,this,_1,_2),
                sotNOSIGNAL,
                "sotDynamic("+name+")::output(vector)::lowerJl" )

  ,upperVlSOUT( boost::bind(&Dynamic::getUpperVelocityLimits,this,_1,_2),
                sotNOSIGNAL,
                "sotDynamic("+name+")::output(vector)::upperVl" )

  ,lowerVlSOUT( boost::bind(&Dynamic::getLowerVelocityLimits,this,_1,_2),
                sotNOSIGNAL,
                "sotDynamic("+name+")::output(vector)::lowerVl" )

  ,upperTlSOUT( boost::bind(&Dynamic::getUpperTorqueLimits,this,_1,_2),
                sotNOSIGNAL,
                "sotDynamic("+name+")::output(vector)::upperTl" )

  ,lowerTlSOUT( boost::bind(&Dynamic::getLowerTorqueLimits,this,_1,_2),
                sotNOSIGNAL,
                "sotDynamic("+name+")::output(vector)::lowerTl" )

  ,inertiaRotorSOUT( "sotDynamic("+name+")::output(matrix)::inertiaRotor" )
  ,gearRatioSOUT( "sotDynamic("+name+")::output(matrix)::gearRatio" )
  ,inertiaRealSOUT( boost::bind(&Dynamic::computeInertiaReal,this,_1,_2),
                    inertiaSOUT << gearRatioSOUT << inertiaRotorSOUT,
                    "sotDynamic("+name+")::output(matrix)::inertiaReal" )
  ,MomentaSOUT( boost::bind(&Dynamic::computeMomenta,this,_1,_2),
                newtonEulerSINTERN,
                "sotDynamic("+name+")::output(vector)::momenta" )
  ,AngularMomentumSOUT( boost::bind(&Dynamic::computeAngularMomentum,this,_1,_2),
                        newtonEulerSINTERN,
                        "sotDynamic("+name+")::output(vector)::angularmomentum" )
  ,dynamicDriftSOUT( boost::bind(&Dynamic::computeTorqueDrift,this,_1,_2),
                     newtonEulerSINTERN,
                     "sotDynamic("+name+")::output(vector)::dynamicDrift" )
{
    sotDEBUGIN(5);



    signalRegistration(jointPositionSIN);
    signalRegistration(freeFlyerPositionSIN);
    signalRegistration(jointVelocitySIN);
    signalRegistration(freeFlyerVelocitySIN);
    signalRegistration(jointAccelerationSIN);
    signalRegistration(freeFlyerAccelerationSIN);
    signalRegistration(zmpSOUT);
    signalRegistration(comSOUT);
    signalRegistration(JcomSOUT);
    signalRegistration(footHeightSOUT);
    signalRegistration(upperJlSOUT);
    signalRegistration(lowerJlSOUT);
    signalRegistration(upperVlSOUT);
    signalRegistration(lowerVlSOUT);
    signalRegistration(upperTlSOUT);
    signalRegistration(lowerTlSOUT);
    signalRegistration(inertiaSOUT);
    signalRegistration(inertiaRealSOUT);
    signalRegistration(inertiaRotorSOUT);
    signalRegistration(gearRatioSOUT);
    signalRegistration( MomentaSOUT);
    signalRegistration(AngularMomentumSOUT);
    signalRegistration(dynamicDriftSOUT);

    //
    // Commande
    //
    // #### Work in progress
    using namespace ::dynamicgraph::command;
    std::string docstring;

    // CreateOpPoint
    docstring =
            "    \n"
            "    Create an operational point attached to a robot joint local frame.\n"
            "    \n"
            "      Input: \n"
            "        - a string: name of the operational point,\n"
            "        - a string: name the joint, among (gaze, left-ankle, right ankle\n"
            "          , left-wrist, right-wrist, waist, chest).\n"
            "\n";
    addCommand("CreateOpPoint",
               makeCommandVoid2(*this,&Dynamic::cmd_createOpPointSignals,
                                docstring));



    // #### End Work in progress
}

Dynamic::~Dynamic( void )
{
    if (this->m_data) delete this->m_data;
    for(  std::list< SignalBase<int>* >::iterator iter = genericSignalRefs.begin();
          iter != genericSignalRefs.end();
          ++iter )
    {
        SignalBase<int>* sigPtr = *iter;
        delete sigPtr;
    }
    return;
}

void Dynamic::setUrdfPath( const std::string& path )
{
    this->m_model = se3::urdf::buildModel(path, true);
    this->m_urdfPath = path;
    if (this->m_data) delete this->m_data;
    this->m_data = new se3::Data(m_model);
}

/* --- CONVERTION ---------------------------------------------------- */
/* --- CONVERTION ---------------------------------------------------- */
/* --- CONVERTION ---------------------------------------------------- */

static Eigen::VectorXd maalToEigenVectorXd(const ml::Vector& inVector)
{
    Eigen::VectorXd vector(inVector.size());
    for (unsigned int r=0; r<inVector.size(); r++)
        vector(r) = inVector(r);
    return vector;
}

static Eigen::MatrixXd maalToEigenMatrixXd(const ml::Matrix& inMatrix)
{
    Eigen::MatrixXd matrix(inMatrix.nbRows(),inMatrix.nbCols());
    for (unsigned int r=0; r<inMatrix.nbRows(); r++)
        for (unsigned int c=0; c<inMatrix.nbCols(); c++)
            matrix(r,c) = inMatrix(r,c);
    return matrix;
}

static ml::Vector eigenVectorXdToMaal(const Eigen::VectorXd& inVector)
{
    ml::Vector vector(inVector.size());
    for (unsigned int r=0; r<inVector.size(); r++)
        vector(r) = inVector(r);
    return vector;
}

static ml::Matrix eigenMatrixXdToMaal(const Eigen::MatrixXd& inMatrix)
{
    ml::Matrix matrix(inMatrix.rows(),inMatrix.cols());
    for (unsigned int r=0; r<inMatrix.rows(); r++)
        for (unsigned int c=0; c<inMatrix.cols(); c++)
            matrix(r,c) = inMatrix(r,c);
    return matrix;
}

static void eigenVector3dToMaal( const Eigen::Vector3d& source,
                ml::Vector & res )
{
  sotDEBUG(5) << source <<endl;
  res(0) = source[0];
  res(1) = source[1];
  res(2) = source[2];
}

Eigen::VectorXd Dynamic::getPinocchioPos(int time)
{
    const Eigen::VectorXd qJoints=maalToEigenVectorXd(jointPositionSIN.access(time));
    const Eigen::VectorXd qFF=maalToEigenVectorXd(freeFlyerPositionSIN.access(time));
    Eigen::VectorXd q(qJoints.size() + 7);// assert qFF.size() = 6?
    urdf::Rotation rot;
    rot.setFromRPY(qFF(3),qFF(4),qFF(5));
    double x,y,z,w;
    double &refx = x;
    double &refy = y;
    double &refz = z;
    double &refw = w;
    rot.getQuaternion(refx,refy,refz,refw);

    q << qFF(0),qFF(1),qFF(2),x,y,z,w,qJoints;// assert q.size()==m_model.nq?
    return q;
}

Eigen::VectorXd Dynamic::getPinocchioVel(int time)
{
    const Eigen::VectorXd vJoints=maalToEigenVectorXd(jointVelocitySIN.access(time));
    const Eigen::VectorXd vFF=maalToEigenVectorXd(freeFlyerVelocitySIN.access(time));
    Eigen::VectorXd v(vJoints.size() + vFF.size());

    v << vFF,vJoints;// assert q.size()==m_model.nq?
    return v;
}

Eigen::VectorXd Dynamic::getPinocchioAcc(int time)
{
    const Eigen::VectorXd aJoints=maalToEigenVectorXd(jointAccelerationSIN.access(time));
    const Eigen::VectorXd aFF=maalToEigenVectorXd(freeFlyerAccelerationSIN.access(time));
    Eigen::VectorXd a(aJoints.size() + aFF.size());

    a << aFF,aJoints;// assert q.size()==m_model.nq?
    return a;
}

/* --- SIGNAL ACTIVATION ---------------------------------------------------- */
/* --- SIGNAL ACTIVATION ---------------------------------------------------- */
/* --- SIGNAL ACTIVATION ---------------------------------------------------- */
dg::SignalTimeDependent< ml::Matrix,int > & Dynamic::
createEndeffJacobianSignal( const std::string& signame, int jointId )
{
    //TODO: implement here
    dg::SignalTimeDependent<ml::Matrix,int> res;
    return res;
}

dg::SignalTimeDependent< ml::Matrix,int > & Dynamic::
createJacobianSigna( const std::string& signame, int jointId )
{
    //TODO: implement here
    dg::SignalTimeDependent<ml::Matrix,int> res;
    return res;
}

void Dynamic::
destroyJacobianSignal( const std::string& signame )
{
    //TODO: implement here
}

dg::SignalTimeDependent< MatrixHomogeneous,int >& Dynamic::
createPositionSignal ( const std::string& signame, int jointId )
{
    //TODO: implement here
    dg::SignalTimeDependent<MatrixHomogeneous,int> res;
    return res;
}

void Dynamic::
destroyPositionSignal( const std::string& signame )
{
    //TODO: implement here
}

dg::SignalTimeDependent< ml::Vector,int >& Dynamic::
createVelocitySignal( const std::string& signame,  int jointId )
{
    //TODO: implement here
    dg::SignalTimeDependent<ml::Vector,int> res;
    return res;
}

void Dynamic::
destroyVelocitySignal( const std::string& signame )
{
    //TODO: implement here
}

dg::SignalTimeDependent< ml::Vector,int >& Dynamic::
createAccelerationSignal( const std::string& signame, int jointId )
{
    //TODO: implement here
    dg::SignalTimeDependent<ml::Vector,int> res;
    return res;
}

void Dynamic::
destroyAccelerationSignal( const std::string& signame )
{
    //TODO: implement here
}



/* --- COMPUTE -------------------------------------------------------------- */
/* --- COMPUTE -------------------------------------------------------------- */
/* --- COMPUTE -------------------------------------------------------------- */


ml::Matrix& Dynamic::computeGenericJacobian( int jointId,ml::Matrix& res,int time )
{
    // Work done
    sotDEBUGIN(25);
    newtonEulerSINTERN(time);

    se3::jacobian(this->m_model,*this->m_data,this->getPinocchioPos(time),jointId);
    res.initFromMotherLib(eigenMatrixXdToMaal(m_data->J).accessToMotherLib());

    sotDEBUGOUT(25);

    return res;
}

ml::Matrix& Dynamic::computeGenericEndeffJacobian( int aJoint,ml::Matrix& res,int time )
{
    //TODO: implement here
    return res;
}

MatrixHomogeneous& Dynamic::computeGenericPosition( int aJoint,MatrixHomogeneous& res,int time )
{
    //TODO: implement here
    return res;
}

ml::Vector& Dynamic::computeGenericVelocity( int j,ml::Vector& res,int time )
{
    //TODO: implement here
    return res;
}

ml::Vector& Dynamic::computeGenericAcceleration( int j,ml::Vector& res,int time )
{
    //TODO: implement here
    return res;
}

ml::Vector& Dynamic::computeZmp( ml::Vector& res,int time )
{
    //TODO: implement here
    return res;
}

ml::Vector& Dynamic::computeMomenta( ml::Vector &res, int time)
{
    //TODO: implement here
    return res;
}

ml::Vector& Dynamic::computeAngularMomentum( ml::Vector &res, int time)
{
    //TODO: implement here
    return res;
}

ml::Matrix& Dynamic::computeJcom( ml::Matrix& res,int time )
{
    //Work done
    sotDEBUGIN(25);
    newtonEulerSINTERN(time);

    res.initFromMotherLib(eigenMatrixXdToMaal(m_data->Jcom).accessToMotherLib());
    sotDEBUGOUT(25);
    return res;
}

ml::Vector& Dynamic::computeCom( ml::Vector& res,int time )
{
    //Work in progress
    sotDEBUGIN(25);
    newtonEulerSINTERN(time);
    res.resize(3);
    eigenVector3dToMaal(this->m_data->com[0],res);
    sotDEBUGOUT(25);
    return res;
}

ml::Matrix& Dynamic::computeInertia( ml::Matrix& res,int time )
{
    //TODO: implement here
    return res;
}

ml::Matrix& Dynamic::computeInertiaReal( ml::Matrix& res,int time )
{
    //TODO: implement here
    return res;
}

double&     Dynamic::computeFootHeight( double& res,int time )
{
    //TODO: implement here
    return res;
}



/* --- SIGNAL --------------------------------------------------------------- */
/* --- SIGNAL --------------------------------------------------------------- */
/* --- SIGNAL --------------------------------------------------------------- */
dg::SignalTimeDependent<ml::Matrix,int>& Dynamic::jacobiansSOUT( const std::string& name )
{
    //TODO: implement here
    dg::SignalTimeDependent<ml::Matrix,int> res;
    return res;
}

dg::SignalTimeDependent<MatrixHomogeneous,int>& Dynamic::positionsSOUT( const std::string& name )
{
    //TODO: implement here
    dg::SignalTimeDependent<MatrixHomogeneous,int> res;
    return res;
}

dg::SignalTimeDependent<ml::Vector,int>& Dynamic::velocitiesSOUT( const std::string& name )
{
    //TODO: implement here
    dg::SignalTimeDependent<ml::Vector,int> res;
    return res;
}

dg::SignalTimeDependent<ml::Vector,int>& Dynamic::accelerationsSOUT( const std::string& name )
{
    //TODO: implement here
    dg::SignalTimeDependent<ml::Vector,int> res;
    return res;
}

int& Dynamic::computeNewtonEuler( int& dummy,int time )
{
    const Eigen::VectorXd q=getPinocchioPos(time);
    const Eigen::VectorXd v=getPinocchioVel(time);
    const Eigen::VectorXd a=getPinocchioAcc(time);
    se3::rnea(m_model,*m_data,q,v,a);
    return dummy;
}

ml::Vector& Dynamic::getUpperJointLimits( ml::Vector& res,const int& time )
{
    //TODO: implement here
    return res;
}

ml::Vector& Dynamic::getLowerJointLimits( ml::Vector& res,const int& time )
{
    //TODO: implement here
    return res;
}

ml::Vector& Dynamic::getUpperVelocityLimits( ml::Vector& res,const int& time )
{
    //TODO: implement here
    return res;
}

ml::Vector& Dynamic::getLowerVelocityLimits( ml::Vector& res,const int& time )
{
    //TODO: implement here
    return res;
}

ml::Vector& Dynamic::getUpperTorqueLimits( ml::Vector& res,const int& time )
{
    //TODO: implement here
    return res;
}

ml::Vector& Dynamic::getLowerTorqueLimits( ml::Vector& res,const int& time )
{
    //TODO: implement here
    return res;
}

ml::Vector& Dynamic::computeTorqueDrift( ml::Vector& res,const int& time )
{
    //TODO: implement here
    return res;
}

/* --- COMMANDS ------------------------------------------------------------- */
/* --- COMMANDS ------------------------------------------------------------- */
/* --- COMMANDS ------------------------------------------------------------- */

void Dynamic::cmd_createOpPointSignals( const std::string& opPointName,
                                        const std::string& jointName )
{
    if(!this->m_model.existBodyName(jointName))
    {
        throw runtime_error ("Robot has no joint corresponding to " + jointName);
    }
    int jointId = this->m_model.getBodyId(jointName);
    createEndeffJacobianSignal(std::string("J")+opPointName,jointId);
    createPositionSignal(opPointName,jointId);
}
