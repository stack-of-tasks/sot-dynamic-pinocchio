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

/* ----- DUMMY CLASS -----*/


template< class Res=double >
class DummyClass
{

public:
    DummyClass( void ) : res(),appel(0),timedata(0) {}

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
    int appel;
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
    res.resize(3);
    res.fill(appel*timedata); return res;
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

int main(int argc, char * argv[])
{
    cout<<"tests sot-dynamic-pinocchio"<<endl;
    if (argc!=2)
    {
        cerr << "Wrong argument number: expect 1 got " << argc-1 <<endl;
        cerr << "Usage:" << endl;
        cerr << "./" << argv[0] << " PATH_TO_URDF_FILE" << endl;
        cerr << "\tPATH_TO_URDF_FILE : Path to the URDF model file describing the robot. "<< endl;
        return 1;
    }
    cout<< "Test parsing " << argv[1] << " ..."<<endl;
    Dynamic * dyn = new Dynamic("tot");
    dyn->setUrdfPath( argv[1]);
    DummyClass<ml::Vector> vect;

    SignalTimeDependent<ml::Vector, int> sigPosOUT(sotNOSIGNAL,"sigPosOUT");
    SignalTimeDependent<ml::Vector, int> sigVelOUT(sotNOSIGNAL,"sigVelOUT");
    SignalTimeDependent<ml::Vector, int> sigAccOUT(sotNOSIGNAL,"sigAccOUT");
    ml::Vector vectPos(6);
    ml::Vector vectVel;
    ml::Vector vectAcc;


    //cout << set;
    sigPosOUT.setFunction(boost::bind(&DummyClass<ml::Vector>::fun,vect,_1,_2) );
    try{
        dyn->jointPositionSIN.plug(&sigPosOUT);
    }
    catch(sot::ExceptionAbstract& e ) { cout << e << endl; exit(1); }

    sigPosOUT.access(1);
    sigPosOUT.setReady();
    cout << "time : " <<sigPosOUT.getTime()<< endl;
    dyn->jointPositionSIN.access(2);

    for(int i=1;i<10;++i){
        cout << "Joint position : " << dyn->jointPositionSIN.access(i) << endl;
        //sigPosOUT.access(i+1);
        if (i%2==0)
          sigPosOUT.setReady();
       //DummyClass<ml::Vector>::fun(vectPos,i);
        cout << "timeOUT: " <<sigPosOUT.getTime()<< endl;
        cout << "timeIN : " <<dyn->jointPositionSIN.getTime()<< endl;
    }

    delete dyn;
    return 0;
}
