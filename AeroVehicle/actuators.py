from Global.utils import linear_scale
from Global.simdata import ActuatorOutputs

class Actuator_model:
    def __init__(self, min_thrust, max_thrust, min_deflection, max_deflection):
        self.min_thrust = min_thrust
        self.max_thrust = max_thrust
        self.min_deflection = min_deflection
        self.max_deflection = max_deflection
        self.output: ActuatorOutputs = ActuatorOutputs()

    def run(self, control_input:ActuatorOutputs)-> ActuatorOutputs:
        # Scale motor thrusts
        self.output.quad.motor1 = linear_scale(control_input.quad.motor1, 1000, 2000, self.min_thrust, self.max_thrust)
        self.output.quad.motor2 = linear_scale(control_input.quad.motor2, 1000, 2000, self.min_thrust, self.max_thrust)
        self.output.quad.motor3 = linear_scale(control_input.quad.motor3, 1000, 2000, self.min_thrust, self.max_thrust)
        self.output.quad.motor4 = linear_scale(control_input.quad.motor4, 1000, 2000, self.min_thrust, self.max_thrust)
        self.output.fw.throttle = linear_scale(control_input.fw.throttle, 1000, 2000, self.min_thrust, self.max_thrust)
        # Scale control surface deflections
        self.output.fw.aileron = linear_scale(control_input.fw.aileron, 1000, 2000, self.min_deflection, self.max_deflection)
        self.output.fw.elevator = linear_scale(control_input.fw.elevator, 1000, 2000, self.min_deflection, self.max_deflection)
        self.output.fw.rudder = linear_scale(control_input.fw.rudder, 1000, 2000, self.min_deflection, self.max_deflection)
        return self.output
