from Global.simdata import UAVState, GCSData, MissionTrack, ControllerFlags, MissionPlan, TargetSetpoints
from Autonomy.AutoNavigation import AutoNavigation

class Flight_Mode_manager:
    def __init__(self, GCS_data: GCSData):
        self.auto_nav = AutoNavigation(GCS_data)
        self.controller_flags: ControllerFlags = ControllerFlags()
        self.target_output: TargetSetpoints = TargetSetpoints()

    def run(self, GCS_data: GCSData, UAV_state: UAVState):

        match GCS_data.mode.upper():
            case "AUTO":
                self.target_output, self.controller_flags = self.auto_nav.run(GCS_data, UAV_state)
            case "QD_POSHOLD":
                self.controller_flags.current_mode = "QD"
                self.target_output.quad.x = GCS_data.mission.track.target.x
                self.target_output.quad.y = GCS_data.mission.track.target.y
                self.target_output.quad.altitude = GCS_data.mission.track.target.z
                self.target_output.quad.heading = GCS_data.mission.track.target.heading
            case "QD_ALTHOLD":
                self.controller_flags.current_mode = "QD"
                self.target_output.quad.altitude = UAV_state.z + 0.1 * GCS_data.rc.throttle
            case "MANUAL":
                self.controller_flags.current_mode = "MANUAL"
            case _:
                self.controller_flags.current_mode = "SHUTDOWN"

        return self.target_output, self.controller_flags
