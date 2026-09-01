"""Attitude conversion utilities.

Conventions used by this module:

- `q_body2ned` maps body-frame vector components into NED-frame components.
- Quaternions are returned in `(w, x, y, z)` order.
- The quaternion is passive: it represents the orientation of the body frame
  relative to the NED frame. Applying its equivalent direction cosine matrix to
  body-frame vector components gives NED-frame vector components.
- Azimuth is measured in radians clockwise from North in the NED horizontal
  plane. Zero azimuth points body +X North; +90 deg points body +X East.
- Elevation is measured in radians from the NED horizontal plane. Positive
  elevation pitches body +X upward, which is negative Down in NED.
- Roll is measured in radians about body +X using the right-hand rule.
- Rotation order is azimuth, then elevation, then roll. Equivalently, the body
  axes are formed as `Rz(azimuth) * Ry(elevation) * Rx(roll)` in NED.
"""

from math import cos, sin


def q_body2ned_from_az_el_roll(azimuth_rad, elevation_rad, roll_rad):
    """Return passive `q_body2ned` from azimuth, elevation, and roll.

    Args:
        azimuth_rad: Heading angle in radians, clockwise from North.
        elevation_rad: Positive-up elevation angle in radians.
        roll_rad: Body +X roll angle in radians, right-hand rule.

    Returns:
        Quaternion tuple `(w, x, y, z)`.
    """
    half_azimuth = 0.5 * azimuth_rad
    half_elevation = 0.5 * elevation_rad
    half_roll = 0.5 * roll_rad

    cz = cos(half_azimuth)
    sz = sin(half_azimuth)
    cy = cos(half_elevation)
    sy = sin(half_elevation)
    cx = cos(half_roll)
    sx = sin(half_roll)

    return (
        cz * cy * cx + sz * sy * sx,
        cz * cy * sx - sz * sy * cx,
        cz * sy * cx + sz * cy * sx,
        sz * cy * cx - cz * sy * sx,
    )
