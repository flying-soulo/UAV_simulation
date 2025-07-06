import numpy as np


def wrap(value, min_val, max_val):
    """
    Wraps a given value within the specified range [min_val, max_val].

    This function ensures that the input value wraps around if it exceeds
    the maximum or falls below the minimum of the specified range.

    Args:
        value (float): The value to be wrapped.
        min_val (float): The minimum allowable value.
        max_val (float): The maximum allowable value.

    Returns:
        float: The wrapped value within the range [min_val, max_val].
    """
    range_size = max_val - min_val
    return (value - min_val) % range_size + min_val

#----------------Linear Scaling------------------
def linear_scale(input:float, in_min:float, in_max:float, out_min:float, out_max:float):
    """
    Linearly scale x from range [in_min, in_max] to [out_min, out_max].

    Parameters:
        x (float): Input value.
        in_min (float): Minimum of input range.
        in_max (float): Maximum of input range.
        out_min (float): Minimum of output range.
        out_max (float): Maximum of output range.

    Returns:
        float: Scaled output value.
    """
    if in_min == in_max:
        raise ValueError("Input range cannot be zero (in_min == in_max)")

    # Clamp input to avoid extrapolation if needed
    input = np.clip(input, in_min, in_max)

    # Linear scaling formula
    scaled = (input - in_min) / (in_max - in_min) * (out_max - out_min) + out_min
    return scaled


# ---------- Rotation Matrix ----------
def rotation_matrix(phi, theta, psi):
    cphi, sphi = np.cos(phi), np.sin(phi)
    ctheta, stheta = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)

    R = np.array(
        [
            [ctheta * cpsi, ctheta * spsi, -stheta],
            [
                sphi * stheta * cpsi - cphi * spsi,
                sphi * stheta * spsi + cphi * cpsi,
                sphi * ctheta,
            ],
            [
                cphi * stheta * cpsi + sphi * spsi,
                cphi * stheta * spsi - sphi * cpsi,
                cphi * ctheta,
            ],
        ]
    )
    return R


# LOW PASS FILTER
def lpf(new_value, prev_value, alpha):
    return [
        alpha * new + (1 - alpha) * prev for new, prev in zip(new_value, prev_value)
    ]
