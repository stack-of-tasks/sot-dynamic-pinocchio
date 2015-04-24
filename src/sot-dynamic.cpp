#include <sot/core/debug.hh>
#include <sot-dynamic-pinocchio/dynamic.h>

//#include <boost/version.hpp>
//#include <boost/filesystem.hpp>
//#include <boost/format.hpp>

#include <jrl/mal/matrixabstractlayer.hh>

#include <dynamic-graph/all-commands.h>



using namespace dynamicgraph::sot;
using namespace dynamicgraph;

const std::string dynamicgraph::sot::Dynamic::CLASS_NAME = "DynamicLib";

using namespace std;

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

Dynamic::Dynamic( const std::string & name, bool build ):Entity(name)
  ,m_data(NULL)
  ,jointPositionSIN         (NULL,"sotDynamic("+name+")::input(vector)::position")
  ,freeFlyerPositionSIN     (NULL,"sotDynamic("+name+")::input(vector)::ffposition")
  ,jointVelocitySIN         (NULL,"sotDynamic("+name+")::input(vector)::velocity")
  ,freeFlyerVelocitySIN     (NULL,"sotDynamic("+name+")::input(vector)::ffvelocity")
  ,jointAccelerationSIN     (NULL,"sotDynamic("+name+")::input(vector)::acceleration")
  ,freeFlyerAccelerationSIN (NULL,"sotDynamic("+name+")::input(vector)::ffacceleration")
{
    signalRegistration(jointPositionSIN);
    signalRegistration(freeFlyerPositionSIN);
    signalRegistration(jointVelocitySIN);
    signalRegistration(freeFlyerVelocitySIN);
    signalRegistration(jointAccelerationSIN);
    signalRegistration(freeFlyerAccelerationSIN);
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
    this->m_model = se3::urdf::buildModel(path);
    this->m_urdfPath = path;
    if (this->m_data) delete this->m_data;
    this->m_data = new se3::Data(m_model);

    /*
    self.modelFileName = filename
    self.model = se3.buildModelFromUrdf(filename,True)
    self.data = self.model.createData()
    self.v0 = utils.zero(self.nv)
    self.q0 = utils.zero(self.nq)*/
}
/* --- COMPUTE -------------------------------------------------------------- */
/* --- COMPUTE -------------------------------------------------------------- */
/* --- COMPUTE -------------------------------------------------------------- */
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
    //TODO: implement here
    return res;
}
ml::Vector& Dynamic::computeCom( ml::Vector& res,int time )
{
    //TODO: implement here
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
ml::Vector  Dynamic::testRNEA(const ml::Vector& maalQ,const ml::Vector& maalV,const ml::Vector& maalA)
{
    Eigen::VectorXd q=maalToEigenVectorXd(maalQ);
    Eigen::VectorXd v=maalToEigenVectorXd(maalV);
    Eigen::VectorXd a=maalToEigenVectorXd(maalA);
    return eigenVectorXdToMaal(se3::rnea(m_model,*m_data,q,v,a));
}
