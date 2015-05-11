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
static int appel;
/* ----- DUMMY CLASS -----*/


template< class Res=double >
class DummyClass
{

public:
    DummyClass( void ) : res(),timedata(0) {}

    Res& fun( Res& res,int t)
    {
        appel++;  timedata=t;

        cout << "DUMMY : " << appel <<" " << timedata << endl;

        sotDEBUG(5) << "Inside " << typeid(Res).name() <<endl;
        for( list< SignalTimeDependent<double,int>* >::iterator it=inputsig.begin();
             it!=inputsig.end();++it )
        { sotDEBUG(5) << *(*it) << endl; (*it)->access(timedata);}
        for( list< SignalTimeDependent<ml::Vector,int>* >::iterator it=inputsigV.begin();
             it!=inputsigV.end();++it )
        { sotDEBUG(5) << *(*it) << endl; (*it)->access(timedata);}

        return res=(*this)();
    }

    list< SignalTimeDependent<double,int>* > inputsig;
    list< SignalTimeDependent<ml::Vector,int>* > inputsigV;

    void add( SignalTimeDependent<double,int>& sig ){ inputsig.push_back(&sig); }
    void add( SignalTimeDependent<ml::Vector,int>& sig ){ inputsigV.push_back(&sig); }

    Res operator() ( void );

    Res res;

    int timedata;

};

template< class Res >
Res DummyClass<Res>::operator() (void)
{ return this->res; }

template<>
double DummyClass<double>::operator() (void)
{
    res=appel*timedata; return res;
}
template<>
ml::Vector DummyClass<ml::Vector>::operator() (void)
{
    if(appel==1){
        res.resize(35);
        res.fill(appel*timedata);
    }else{
        res.resize(35);
        //res.fill(0);
        res.fill(appel*timedata);
    }
    cout << "res : " << res <<endl;
    return res;

}

template<>
VectorUTheta DummyClass<VectorUTheta>::operator() (void)
{
    res.fill(12.6); return res;
}

void funtest( ml::Vector& /*v*/ ){ }

/* ----- END DUMMY CLASS -----*/

ml::Vector& setVector(ml::Vector& vect){
    vect.resize(3);
    vect.fill(42);
    return vect;
}

class Dummy2Class
{

public:
    ml::Vector& fun( ml::Vector& res,double j )
    { res.resize(6); res.fill(j); return res; }
};

ml::Vector data(6);
ml::Vector data2(35);
Signal<ml::Vector,double> sig("sigtest");
Dummy2Class dummy;

ml::Vector& fun( ml::Vector& res,double /*j*/ ) { return res=data; }

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
    DummyClass<ml::Vector> vectDummyPos;
    DummyClass<ml::Vector> vectDummyVel;
    DummyClass<ml::Vector> vectDummyAcc;
    DummyClass<ml::Vector> vectDummyFreePos;
    DummyClass<ml::Vector> vectDummyFreeVel;
    DummyClass<ml::Vector> vectDummyFreeAcc;

    SignalTimeDependent<ml::Vector, int> sigPosOUT(sotNOSIGNAL,"sigPosOUT");
    SignalTimeDependent<ml::Vector, int> sigVelOUT(sotNOSIGNAL,"sigVelOUT");
    SignalTimeDependent<ml::Vector, int> sigAccOUT(sotNOSIGNAL,"sigAccOUT");
    SignalTimeDependent<ml::Vector, int> sigFreePosOUT(sotNOSIGNAL,"sigFreePosOUT");
    SignalTimeDependent<ml::Vector, int> sigFreeVelOUT(sotNOSIGNAL,"sigFreeVelOUT");
    SignalTimeDependent<ml::Vector, int> sigFreeAccOUT(sotNOSIGNAL,"sigFreeAccOUT");

    //this is an example using DummyClass or Dummy2Class (with or without template)
    sigPosOUT.setFunction(boost::bind(&DummyClass<ml::Vector>::fun,vectDummyPos,_1,_2) );
    sigVelOUT.setFunction(boost::bind(&DummyClass<ml::Vector>::fun,vectDummyVel,_1,_2) );//class with template
    sigAccOUT.setFunction(boost::bind(&DummyClass<ml::Vector>::fun,vectDummyAcc,_1,_2) );
    sigFreePosOUT.setFunction(boost::bind(&Dummy2Class::fun,&dummy,_1,_2) );//class without template
    sigFreeVelOUT.setFunction(boost::bind(&Dummy2Class::fun,&dummy,_1,_2) );
    sigFreeAccOUT.setFunction(boost::bind(&Dummy2Class::fun,&dummy,_1,_2) );
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

    /* ----- ComputegenericPosition ----- */

    cout << "Test computeGenericPosition :" << endl;
    MatrixHomogeneous matmat;
    dyn->computeGenericPosition(Id,matmat,1);
    cout << "Display matmat" << endl;
    cout <<"Matrix homogeneous : "<< matmat << endl;
    dyn->computeNewtonEuler(dummy,2);
    dyn->computeGenericPosition(Id,matmat,1);
    cout <<"Matrix homogeneous : "<< matmat << endl;



    delete dyn;
    return 0;
}
