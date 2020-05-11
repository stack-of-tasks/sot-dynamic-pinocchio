/*
 * Copyright 2012,
 * Florent Lamiraux
 *
 * CNRS
 *
 */

#include <vector>
#include <sstream>

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/core/matrix-geometry.hh>

namespace sot {
namespace dynamic {
using dynamicgraph::Entity;
using dynamicgraph::SignalPtr;
using dynamicgraph::SignalTimeDependent;
using dynamicgraph::Vector;
using dynamicgraph::sot::MatrixHomogeneous;

class ZmpFromForces : public Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  static const unsigned int footNumber = 2;
  ZmpFromForces(const std::string& name) : Entity(name), zmpSOUT_(CLASS_NAME + "::output(Vector)::zmp") {
    zmpSOUT_.setFunction(boost::bind(&ZmpFromForces::computeZmp, this, _1, _2));
    signalRegistration(zmpSOUT_);
    for (unsigned int i = 0; i < footNumber; i++) {
      std::ostringstream forceName, positionName;
      forceName << CLASS_NAME << "::input(vector6)::force_" << i;
      positionName << CLASS_NAME << "::input(MatrixHomo)::sensorPosition_" << i;
      forcesSIN_[i] = new SignalPtr<Vector, int>(0, forceName.str());
      sensorPositionsSIN_[i] = new SignalPtr<MatrixHomogeneous, int>(0, positionName.str());
      signalRegistration(*forcesSIN_[i]);
      signalRegistration(*sensorPositionsSIN_[i]);
      zmpSOUT_.addDependency(*forcesSIN_[i]);
      zmpSOUT_.addDependency(*sensorPositionsSIN_[i]);
    }
  }
  virtual std::string getDocString() const {
    std::string docstring =
        "Compute ZMP from force sensor measures and positions\n"
        "\n"
        "  Takes 4 signals as input:\n"
        "    - force_0: wrench measured by force sensor 0 as a 6 dimensional vector\n"
        "    - force_0: wrench measured by force sensor 1 as a 6 dimensional vector\n"
        "    - sensorPosition_0: position of force sensor 0\n"
        "    - sensorPosition_1: position of force sensor 1\n"
        "  \n"
        "  compute the Zero Momentum Point of the contact forces as measured by the \n"
        "  input signals under the asumptions that the contact points between the\n"
        "  robot and the environment are located in the same horizontal plane.\n";
    return docstring;
  }

 private:
  Vector& computeZmp(Vector& zmp, int time) {
    double fnormal = 0;
    double sumZmpx = 0;
    double sumZmpy = 0;
    double sumZmpz = 0;
    zmp.resize(3);

    for (unsigned int i = 0; i < footNumber; ++i) {
      const Vector& f = forcesSIN_[i]->access(time);
      // Check that force is of dimension 6
      if (f.size() != 6) {
        zmp.fill(0.);
        return zmp;
      }
      const MatrixHomogeneous& M = sensorPositionsSIN_[i]->access(time);
      double fx = M(0, 0) * f(0) + M(0, 1) * f(1) + M(0, 2) * f(2);
      double fy = M(1, 0) * f(0) + M(1, 1) * f(1) + M(1, 2) * f(2);
      double fz = M(2, 0) * f(0) + M(2, 1) * f(1) + M(2, 2) * f(2);

      if (fz > 0) {
        double Mx = M(0, 0) * f(3) + M(0, 1) * f(4) + M(0, 2) * f(5) + M(1, 3) * fz - M(2, 3) * fy;
        double My = M(1, 0) * f(3) + M(1, 1) * f(4) + M(1, 2) * f(5) + M(2, 3) * fx - M(0, 3) * fz;
        fnormal += fz;
        sumZmpx -= My;
        sumZmpy += Mx;
        sumZmpz += fz * M(2, 3);
      }
    }
    if (fnormal != 0) {
      zmp(0) = sumZmpx / fnormal;
      zmp(1) = sumZmpy / fnormal;
      zmp(2) = sumZmpz / fnormal;
    } else {
      zmp.fill(0.);
    }
    return zmp;
  }
  // Force as measured by force sensor on the foot
  SignalPtr<Vector, int>* forcesSIN_[footNumber];
  SignalPtr<MatrixHomogeneous, int>* sensorPositionsSIN_[footNumber];
  SignalTimeDependent<Vector, int> zmpSOUT_;
};  // class ZmpFromForces
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ZmpFromForces, "ZmpFromForces");
}  // namespace dynamic
}  // namespace sot
