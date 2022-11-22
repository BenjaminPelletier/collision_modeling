from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np
from panda3d.core import Point3


class FlightPath(object):
    """Encapsulates the 4D trajectory an aircraft will take."""
    _txyz: np.ndarray

    def __init__(self, txyz: np.ndarray):
        """Make a FlighPath instance.

        :param txyz: Nx4 float matrix with seconds since start in the first
          column, and xyz in the second-fourth columns.  Values in the time
          column must be ascending, and the first value should be 0.
        """
        self._txyz = txyz

    def offset(self, dt: float = 0, dx: float = 0, dy: float = 0, dz: float = 0) -> FlightPath:
        """Return a new FlightPath that is offset from this FlightPath.

        :param dt: Offset in time (applied to all waypoints).
        :param dx: Offset in x (applied to all waypoints).
        :param dy: Offset in y (applied to all waypoints).
        :param dz: Offset in z (applied to all waypoints).
        :return: Offset FlightPath.
        """
        m = self._txyz.copy()
        m[:, 0] += dt
        m[:, 1] += dx
        m[:, 2] += dy
        m[:, 3] += dz
        return FlightPath(m)

    def scale(self, ft: float = 1, fx: float = 1, fy: float = 1, fz: float = 1) -> FlightPath:
        """Return a new FlightPath that is scaled from this FlightPath.

        :param ft: Scale in time (applied to all waypoints).
        :param fx: Scale in x (applied to all waypoints).
        :param fy: Scale in y (applied to all waypoints).
        :param fz: Scale in z (applied to all waypoints).
        :return: Scaled FlightPath.
        """
        m = self._txyz.copy()
        m[:, 0] *= ft
        m[:, 1] *= fx
        m[:, 2] *= fy
        m[:, 3] *= fz
        return FlightPath(m)

    def location_at(self, t: float) -> Point3:
        return Point3(
            np.interp(t, self._txyz[:, 0], self._txyz[:, 1]),
            np.interp(t, self._txyz[:, 0], self._txyz[:, 2]),
            np.interp(t, self._txyz[:, 0], self._txyz[:, 3])
        )

    def t_max(self) -> float:
        return self._txyz[-1, 0]


@dataclass
class Flight(object):
    path: FlightPath
    """Time-annotated path aircraft will take."""

    op_intent: Tuple[Point3, Point3]
    """Operational intent volume.  Must be rectangular and cardinal-axes-aligned."""

    size: Point3
    """Size of the aircraft's collision bounding box in all three dimensions (this is a vector rather than a point)."""
