"""Small GNC math utilities for SixSim tooling."""

from .attitude import q_body2ned_from_az_el_roll
from .constants import (
    DEG_TO_RAD,
    EPS,
    EQUATORIAL_EARTH_RADIUS,
    GRAVITATIONAL_PARAMETER_EARTH,
    MEAN_EARTH_RADIUS,
    PI,
    RAD_TO_DEG,
)

__all__ = [
    "DEG_TO_RAD",
    "EPS",
    "EQUATORIAL_EARTH_RADIUS",
    "GRAVITATIONAL_PARAMETER_EARTH",
    "MEAN_EARTH_RADIUS",
    "PI",
    "RAD_TO_DEG",
    "q_body2ned_from_az_el_roll",
]
