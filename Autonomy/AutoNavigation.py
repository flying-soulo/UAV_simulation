from Global.simdata import UAVState, GCSData, TargetSetpoints, ControllerFlags
from Autonomy.guidance import FW_guidance
from Autonomy.Path_planning import WaypointNavigator

class AutoNavigation:
    def __init__(self, gcs_data: GCSData, l1_distance: float = 300.0, takeoff_altitude: float = 10.0):
        self.gcs_data: GCSData = gcs_data
        self.guidance = FW_guidance(min_L1_dist=l1_distance)
        self.navigator = WaypointNavigator()

        self.takeoff_altitude = takeoff_altitude
        self._takeoff_done = False
        self._landing = False
        self._transition = False

        self.flags = ControllerFlags()

    def run(self, gcs_data: GCSData, state: UAVState) -> tuple[TargetSetpoints, ControllerFlags]:
        targets = TargetSetpoints()
        command = self.gcs_data.command.upper()
        armed = state.armed
        self.gcs_data = gcs_data

        # Get target waypoint from planner
        target_wp = self.navigator.update(state, self.gcs_data)

        # ----- Safety: if disarmed, hold everything -----
        # if not armed:
        #     self._takeoff_done = False
        #     self._landing = False
        #     self._transition = False
        #     self.flags.current_mode = "SHUTDOWN"
        #     return targets, self.flags

        # ----- Command Handling -----
        # match command:
        #     case "LAUNCH":
        #         if not self._takeoff_done:
        #             if state.z <= -self.takeoff_altitude:
        #                 self._takeoff_done = True
        #                 self._transition = True
        #                 self.flags.current_mode = "TRANSITION"
        #             else:
        #                 # Climb to takeoff altitude using quad
        #                 self.flags.current_mode = "QD"
        #                 targets.quad.altitude = -self.takeoff_altitude
        #                 targets.quad.heading = state.psi
        #                 return targets, self.flags

        #     case "LAND" | "ABORT":
        #         self._landing = True

        # ----- Transition to fixed wing -----
        if self._transition:
            # TODO: Replace with airspeed or pitch/altitude logic
            self._transition = False
            self.flags.current_mode = "FW"

        # ----- Landing -----
        if self._landing:
            self.flags.current_mode = "QD"
            targets.quad.altitude = 0
            return targets, self.flags

        # ----- Mission Flight (FW mode) -----
        self.flags.current_mode = "FW"
        targets.fw = self.guidance.run(state, target_wp)
        return targets, self.flags
