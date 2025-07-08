from Global.simdata import UAVState_class, GCSData_class, Target_data_struct, controller_flags_class, Mission_track_data
from Autonomy.Path_planning import WaypointNavigator
from Autonomy.AutoNavigation import AutoNavigation

class Flight_Mode_manager:
    def __init__(self, GCS_data: GCSData_class):
        self.navigator = WaypointNavigator()
        self.auto_nav = AutoNavigation()
        self.controller_flags = controller_flags_class()
        self.target_output = Target_data_struct()

    def run(self, GCS_data: GCSData_class, UAV_state: UAVState_class):

        match GCS_data.mode.upper():
            case "AUTO":
                self.target_output, self.controller_flags = self.auto_nav.run(GCS_data, UAV_state)
            case "QD_POSHOLD":
                mission_state = self.navigator.update(UAV_state, GCS_data.waypoint_data, GCS_data.mode)
                self.controller_flags.mode = "QD"
                wp = mission_state.curr_wp
                self.target_output.Quad.x = wp.x
                self.target_output.Quad.y = wp.y
                self.target_output.Quad.altitude = wp.z
                self.target_output.Quad.heading = wp.heading
            case "QD_ALTHOLD":
                self.controller_flags.mode = "QD"
                self.target_output.Quad.altitude = UAV_state.z + 0.1 * GCS_data.radio.channel3
            case "MANUAL":
                self.controller_flags.mode = "MANUAL"
            case _:
                self.controller_flags.mode = "SHUTDOWN"

        return self.target_output, self.controller_flags
