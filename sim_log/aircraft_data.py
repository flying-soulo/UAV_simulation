from dataclasses import dataclass, field
import pandas as pd

# ---- Central Data Frame Holder ----
@dataclass
class AircraftData:
    data_frame: pd.DataFrame = field(default_factory=lambda: pd.DataFrame(columns=[
        'time',
        # State variables (12)
        'x', 'y', 'z', 'u', 'v', 'w', 'phi', 'theta', 'psi', 'p', 'q', 'r',
        # Forces and Moments (6)
        'Fx', 'Fy', 'Fz', 'l', 'm', 'n',
        # Control Inputs (8)
        'motor1', 'motor2', 'motor3', 'motor4', 'motor5',
        'aileron', 'elevator', 'rudder',
        # Optional
        'mode'
    ]))

    def upload_data(self, data: dict):
        """Append a new row to the DataFrame."""
        self.data_frame = pd.concat([self.data_frame, pd.DataFrame([data])], ignore_index=True)

    def get_data(self):
        return self.data_frame
