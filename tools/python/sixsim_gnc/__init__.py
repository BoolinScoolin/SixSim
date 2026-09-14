"""Small GNC math utilities for SixSim tooling."""

from .attitude import q_body2ned_from_az_el_roll
from .constants import (
    SIXSIM_DEG_TO_RAD,
    SIXSIM_EPS,
    SIXSIM_EQUATORIAL_EARTH_RADIUS,
    SIXSIM_GRAVITATIONAL_PARAMETER_EARTH,
    SIXSIM_MEAN_EARTH_RADIUS,
    SIXSIM_PI,
    SIXSIM_RAD_TO_DEG,
)

__all__ = [
    "SIXSIM_DEG_TO_RAD",
    "SIXSIM_EPS",
    "SIXSIM_EQUATORIAL_EARTH_RADIUS",
    "SIXSIM_GRAVITATIONAL_PARAMETER_EARTH",
    "SIXSIM_MEAN_EARTH_RADIUS",
    "SIXSIM_PI",
    "SIXSIM_RAD_TO_DEG",
    "q_body2ned_from_az_el_roll",
]
