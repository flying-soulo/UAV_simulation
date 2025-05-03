import pandas as pd
import numpy as np


class AircraftDataLogger:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(AircraftDataLogger, cls).__new__(cls)
            cls._instance.reset()
        return cls._instance

    def reset(self):
        """Initialize or reset the logger."""
        self.data = []
        self.time_index_map = {}  # Maps timestep to index in data list for quick updates

    def log(self, timestep, state = None, control_input=None, forces=None, moments=None, mode=None):
        """
        Log data for a single timestep.
        """
        entry = {}
        if control_input is not None:
            entry = {
                "time": timestep,
                # State
                "x": state[0], "y": state[1], "z": state[2],
                "u": state[3], "v": state[4], "w": state[5],
                "phi": state[6], "theta": state[7], "psi": state[8],
                "p": state[9], "q": state[10], "r": state[11],
            }

        if control_input is not None:
            for i, val in enumerate(control_input):
                entry[f"ctrl_{i}"] = val

        if forces is not None:
            entry.update({"Fx": forces[0], "Fy": forces[1], "Fz": forces[2]})

        if moments is not None:
            entry.update({"l": moments[0], "m": moments[1], "n": moments[2]})

        if mode is not None:
            entry["mode"] = mode

        self.time_index_map[timestep] = len(self.data)
        self.data.append(entry)

    def update(self, timestep, key, value):
        """
        Update a specific field at a specific timestep.
        """
        if timestep not in self.time_index_map:
            raise KeyError(f"Timestep {timestep} not logged yet.")
        idx = self.time_index_map[timestep]
        self.data[idx][key] = value

    def to_dataframe(self):
        """Convert logged data to a pandas DataFrame."""
        return pd.DataFrame(self.data)

    def to_numpy(self):
        """Convert logged data to a NumPy array (not recommended unless fields are uniform)."""
        df = self.to_dataframe()
        return df.to_numpy()

    def to_csv(self, filename="flight_log.csv"):
        self.to_dataframe().to_csv(filename, index=False)
