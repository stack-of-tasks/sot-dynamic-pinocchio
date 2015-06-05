#define EIGEN_INITIALIZE_MATRICES_BY_NAN

#include <sot-dynamic-pinocchio/dynamic.h>
#include <sot/core/debug.hh>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/parser/urdf.hpp>

#include <dynamic-graph/all-signals.h>
#include <jrl/mal/boost.hh>
#include <sot/core/vector-utheta.hh>
#include <sot/core/exception-abstract.hh>



#include <iostream>
#include <sstream>



using namespace std;
using namespace dynamicgraph;
using namespace dynamicgraph::sot;

/* ----- TEST SIGNAL CLASS -----*/

class TestSignal
{
public:
  ml::Vector& initVect( ml::Vector& res, int size );

};
ml::Vector& TestSignal::initVect(ml::Vector& res, int size)
{
    res.resize(size);
    res.fill(0);
    cout <<"Vect 0 size :" << size << endl;
    return res;
}

TestSignal testSign;

/* ----- END TEST SIGNAL CLASS -----*/

int main(int argc, char * argv[])
{
    cout<<"tests sot-dynamic-pinocchio"<<endl;
    if (argc!=2)
    {
        cerr << "Wrong argument number: expect 1 got " << argc-1 <<endl;
        cerr << "Usage:" << endl;
        cerr << "./" << argv[0] << " PATH_TO_URDF_FILE" << endl;
        cerr << "\tPATH_TO_URDF_FILE : Path to the URDF model ftimedataile describing the robot. "<< endl;
        return 1;
    }
    cout<< "Test parsing " << argv[1] << " ..."<<endl;
    Dynamic * dyn = new Dynamic("tot");
    dyn->setUrdfPath( argv[1]);

    ml::Vector vectPos;
    ml::Vector vectVel;
    ml::Vector vectAcc;
    ml::Vector vectFreePos;
    ml::Vector vectFreeVel;
    ml::Vector vectFreeAcc;

    int nq = dyn->m_model.nq;
    int nv = dyn->m_model.nv;

    SignalTimeDependent<ml::Vector, int> sigPosOUT(sotNOSIGNAL,"sigPosOUT");
    SignalTimeDependent<ml::Vector, int> sigVelOUT(sotNOSIGNAL,"sigVelOUT");
    SignalTimeDependent<ml::Vector, int> sigAccOUT(sotNOSIGNAL,"sigAccOUT");
    SignalTimeDependent<ml::Vector, int> sigFreePosOUT(sotNOSIGNAL,"sigFreePosOUT");
    SignalTimeDependent<ml::Vector, int> sigFreeVelOUT(sotNOSIGNAL,"sigFreeVelOUT");
    SignalTimeDependent<ml::Vector, int> sigFreeAccOUT(sotNOSIGNAL,"sigFreeAccOUT");

    sigPosOUT.setFunction(boost::bind(&TestSignal::initVect,&testSign,_1,nq) );
    sigVelOUT.setFunction(boost::bind(&TestSignal::initVect,&testSign,_1,nv) );
    sigAccOUT.setFunction(boost::bind(&TestSignal::initVect,&testSign,_1,nv) );
    sigFreePosOUT.setFunction(boost::bind(&TestSignal::initVect,&testSign,_1,nq) );
    sigFreeVelOUT.setFunction(boost::bind(&TestSignal::initVect,&testSign,_1,nv) );
    sigFreeAccOUT.setFunction(boost::bind(&TestSignal::initVect,&testSign,_1,nv) );
    try{
        dyn->jointPositionSIN.plug(&sigPosOUT);
        dyn->jointVelocitySIN.plug(&sigVelOUT);
        dyn->jointAccelerationSIN.plug(&sigAccOUT);
        dyn->freeFlyerPositionSIN.plug(&sigFreePosOUT);
        dyn->freeFlyerVelocitySIN.plug(&sigFreeVelOUT);
        dyn->freeFlyerAccelerationSIN.plug(&sigFreeAccOUT);
    }

    catch(sot::ExceptionAbstract& e ) { cout << e << endl; exit(1); }

    sigPosOUT.access(1); sigPosOUT.setReady();
    sigVelOUT.access(2); sigVelOUT.setReady();
    sigAccOUT.access(3); sigAccOUT.setReady();
    sigFreePosOUT.access(1); sigFreePosOUT.setReady();
    sigFreeVelOUT.access(2); sigFreeVelOUT.setReady();
    sigFreeAccOUT.access(3); sigFreeAccOUT.setReady();

    int dummy(0);

    /*  -----  get body -----   */
//    cout << "Get body guard" << endl;
    string name_body("CHEST");
    int Id;
    if(dyn->m_model.existBodyName(name_body))
    {
        Id = dyn->m_model.getBodyId(name_body);
    }
    else{
        cout << "ERROR : Any body with this name!"<< endl;
        return -1;
    }

    /* ----- computeGenericPosition ----- */

    cout << endl << "/* --- Test computegenericPosition --- */" << endl;
    MatrixHomogeneous matmat;
    dyn->computeGenericPosition(Id,matmat,1);
    cout << "Display matmat" << endl;
    cout <<"Matrix homogeneous : "<< matmat << endl;
//    dyn->computeNewtonEuler(dummy,2);
//    dyn->computeGenericPosition(Id,matmat,1);
//    cout <<"Matrix homogeneous : "<< matmat << endl;

    /* ----- computeGenericVelocity ----- */

    cout << endl << "/* --- Test computeGenericVelocity --- */" << endl;
    ml::Vector restestVit;
    dyn->computeGenericVelocity(Id,restestVit,1);
    cout <<"restestVit vector : "<< restestVit << endl;


    /* ----- computeGenericAcceleration ----- */

    cout << endl << "/* --- Test computeGenericAcceleration --- */" << endl;
    ml::Vector restestAcc;
    dyn->computeGenericAcceleration(Id,restestAcc,1);
    cout <<"restestAcc vector : "<< restestAcc << endl;

    /* ----- computeInertia ----- */
    cout << endl << "/* --- Test computeInertia --- */" << endl;
    ml::Matrix restestInertia;
    dyn->computeInertia(restestInertia,1);
    cout <<"restestInertia Matrix : "<< restestInertia << endl;

    /* ----- computeZMP ----- */
    cout << endl << "/* --- Test computeZMP --- */" << endl;
    ml::Vector restestZMP;
    dyn->computeZmp(restestZMP,1);
    cout <<"restestZMP Vector : "<< restestZMP << endl;

    delete dyn;
    return 0;
}
