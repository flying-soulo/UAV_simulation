from Global.simdata import GCSData_class, UAVState_class, Target_data_struct, controller_flags_class, Mission_track_data, Waypoint_data_class
from Autonomy.guidance import FW_guidance
from Autonomy.Path_planning import WaypointNavigator

class AutoNavigation:
    def __init__(self, L1dist:float=300, takeoff_altitude:float=10):
        self.guidance = FW_guidance(min_L1_dist=L1dist)
        self.takeoff_alt = takeoff_altitude
        self.flags = controller_flags_class()
        self.navigator = WaypointNavigator()
        self._takeoff_done = False
        self._landing = False
        self._transition = False

    def run(self, GCS: GCSData_class, state: UAVState_class) -> tuple[Target_data_struct, controller_flags_class]:
        output = Target_data_struct()
        cmd = GCS.command.upper()
        armed = state.systemArmed

        mission_track = self.navigator.update(state, GCS.waypoint_data, GCS.mode)
        # if not armed:
        #     self._takeoff_done = False
        #     self._landing = False
        #     self.flags.mode = "SHUTDOWN"
        #     return output, self.flags

        match cmd.upper():
            case "LAUNCH":
                if not self._takeoff_done:
                    if state.z <= -self.takeoff_alt:
                        self._takeoff_done = True
                        self._transition = True
                        self.flags.mode = "TRANSITION"
                    else:
                        self.flags.mode = "QD"
                        output.Quad.altitude = -self.takeoff_alt
                        output.Quad.heading = state.psi
                        return output, self.flags
            case "LAND" | "ABORT":
                self._landing = True

        if self._transition:
            # TODO: Replace with proper pitch/airspeed/time condition
            self._transition = False
            self.flags.mode = "FW"

        if self._landing:
            self.flags.mode = "QD"
            output.Quad.altitude = 0
            return output, self.flags

        self.flags.mode = "FW"
        output.FW = self.guidance.run(state, mission_track)
        return output, self.flags
