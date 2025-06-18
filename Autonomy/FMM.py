import numpy as np
from Global.simdata import Quad_target, FW_target, Target_data_struct
from Autonomy.guidance import FW_guidance


class FMM:
    def __init__(self):
        self.navigator = Guidance()
        self.output : Target_data_struct = Target_data_struct()

    def Autonavigation(self):

        return self.output
