from dynamic import Dynamic
from angle_estimator import AngleEstimator
from zmp_from_forces import ZmpFromForces

DynamicOld = Dynamic

class Dynamic (DynamicOld):

    def setData(self, pinocchio_data):
        dynamic.wrap.set_pinocchio_data(self.obj,pinocchio_data)
        return
        
    def setModel(self, pinocchio_model):
        dynamic.wrap.set_pinocchio_model(self.obj,pinocchio_model)
        return
