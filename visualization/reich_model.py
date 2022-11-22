import math
import random
from dataclasses import dataclass
from typing import List, Optional, Callable

import numpy as np
from panda3d.core import Point3

from flights import Flight, FlightPath
from vizmath import compute_sigma, M_PER_FT


@dataclass
class ParallelPathsEncounterDescriptor(object):
    S_y: float
    """Minimum planned lateral separation"""

    lambda_x: float
    """Average length"""

    lambda_y: float
    """Average wingspan"""

    lambda_z: float
    """Average height"""

    w: float
    """Operational volume dimension, half cross-section, lateral direction"""

    h: float
    """Operational volume dimension, half cross-section, vertical direction"""

    t: float
    """Length of the flight"""

    v: float
    """Nominal flight velocity of the flight"""

    delta_v: float
    """Average relative speed of aircraft flying on parallel routes"""

    YS_y: float
    """Average relative lateral speed of aircraft pair at loss of planned lateral separation"""

    delta_z: float
    """Average relative vertical speed of a co-altitude aircraft pair assigned to the same route"""

    r: Optional[random.Random] = None
    """Specific random number generator to generate paths, or None to use system default"""


def standard_parallel_paths_descriptor() -> ParallelPathsEncounterDescriptor:
    return ParallelPathsEncounterDescriptor(
        S_y=15 * M_PER_FT,
        lambda_x=2 * M_PER_FT,
        lambda_y=2 * M_PER_FT,
        lambda_z=2 * M_PER_FT,
        w=7 * M_PER_FT,
        h=7 * M_PER_FT,
        t=3600,
        v=20 * M_PER_FT,
        delta_v=5 * M_PER_FT,
        YS_y=7.75 * M_PER_FT,
        delta_z=7.75 * M_PER_FT
    )


def deviation_path(nominal_position: float, deviation_speed: float, t_overlap: float, overlap_position: float) -> Callable[[float], float]:
    """Generate a path that stays at nominal_position except for a short deviation to overlap_position and back.

    :param nominal_position: Position at which to stay for most of the time.
    :param deviation_speed: Speed at which to travel from nominal_position to overlap_position and back.
    :param t_overlap: Time at which overlap_position should be reached.
    :param overlap_position: Position at which to deviate, briefly.
    :return: Function that accepts a time and returns the position along the path at that time.
    """
    dt_transition = abs((overlap_position - nominal_position) / deviation_speed)
    m = np.array((
        (-1e9, nominal_position),
        (t_overlap - dt_transition, nominal_position),
        (t_overlap, overlap_position),
        (t_overlap + dt_transition, nominal_position),
        (1e9, nominal_position)
    ), dtype=float)
    return lambda t: np.interp(t, m[:, 0], m[:, 1])


def make_parallel_paths(encounter: Optional[ParallelPathsEncounterDescriptor] = None) -> List[Flight]:
    """Generate 2 flights on parallel paths for a longitudinal encounter."""

    # Use standard descriptor if a specific one wasn't provided
    encounter = encounter or standard_parallel_paths_descriptor()
    # Use system default random generator if a specific one wasn't provided
    r = encounter.r or random

    if encounter.delta_v == 0:
        raise NotImplementedError("Not sure how to model the movement of two aircraft flying in side-by-side formation")

    # Amount of time aircraft overlap in x
    dt_overlap_x = 2 * encounter.lambda_x / encounter.delta_v
    # Amount of time aircraft overlap in y
    dt_overlap_y = 2 * encounter.lambda_y / encounter.YS_y
    # Amount of time aircraft overlap in z
    dt_overlap_z = 2 * encounter.lambda_z / encounter.delta_z

    # Define temporal viewing period to potentially include all overlaps, plus a buffer, subject to a minimum
    dt_view = max(max(dt_overlap_x, dt_overlap_y, dt_overlap_z) * 1.2, 3)

    # === Simulate loss of longitudinal separation ===
    # Define spatial viewing window to be centered (x=0) on the "overlap in x" event
    # So, ac1_x(t = dt_view/2) = 0, ac2_x(t = dt_view/2) = 0
    # We also know ac1's x speed, so ac1_x(t) = encounter.v * t + c1
    # We also know ac2's x speed: ac2_x(t) = (encounter.v - encounter.delta_v) * t + c2
    # Therefore, ac1_x(t) = encounter.v * (t - dt_view/2)
    #            ac2_x(t) = (encounter.v - encounter.delta_v) * (t - dt_view/2)
    x1a = encounter.v * (0 - dt_view / 2)
    x1b = encounter.v * (dt_view - dt_view / 2)
    x2a = (encounter.v - encounter.delta_v) * (0 - dt_view / 2)
    x2b = (encounter.v - encounter.delta_v) * (dt_view - dt_view / 2)
    fx1 = lambda t: np.interp(t, np.array((0, dt_view), dtype=float), np.array((x1a, x1b), dtype=float), )
    fx2 = lambda t: np.interp(t, np.array((0, dt_view), dtype=float), np.array((x2a, x2b), dtype=float))
    # Keep track of all key time points within the time range of interest
    t_key1 = [0, dt_view]
    t_key2 = [0, dt_view]

    # === Simulate loss of lateral separation ===
    sigma_y = compute_sigma(encounter.w, math.pow(0.95, 1 / 2))
    # With the distributions of lateral position being Y_1 ~ N(-S_y/2, σ_y) and Y_2 ~ N(S_y/2, σ_y),
    # the distribution of position of overlap Y, given that Y_1 = Y_2, is Y ~ N(0, σ_y/sqrt(2))
    y_overlap = r.gauss(0, sigma_y / math.sqrt(2))
    # Reich assumes probability of lateral overlap is proportional to the fraction of lateral spacing
    # occupied by the aircraft, so assume the aircraft will overlap at y_overlap at some random time
    # in an interval that is larger than t_overlap_y by the same proportion as lateral spacing is
    # larger than aircraft size
    dt_lateral_overlap_interval = dt_overlap_y * encounter.S_y / encounter.lambda_y
    t_overlap_y = r.uniform(-0.5, 0.5) * dt_lateral_overlap_interval
    # We know the average relative lateral speed; generate 2 random lateral velocities that average
    # to this value (draw lateral speeds from triangular distribution peaking at 1/2 average
    # relative lateral speed and dropping to 0 at 0 speed and average lateral speed so that
    # neither speed will be negative)
    v1_y = math.sqrt((r.uniform(0, 1) % 0.5) * math.pow(encounter.YS_y, 2))
    v2_y = -math.sqrt((r.uniform(0, 1) % 0.5) * math.pow(encounter.YS_y, 2))
    # Now we can reconstruct aircrafts' lateral path
    fy1 = deviation_path(-encounter.S_y / 2, v1_y, t_overlap_y, y_overlap)
    fy2 = deviation_path(encounter.S_y / 2, v2_y, t_overlap_y, y_overlap)
    # Note the key time points associated with this deviation (when inside the time range of interest)
    dt1_y = abs((y_overlap + encounter.S_y / 2) / v1_y)
    dt2_y = abs((y_overlap - encounter.S_y / 2) / v2_y)
    t_key1.extend(t for t in (t_overlap_y, t_overlap_y - dt1_y, t_overlap_y + dt1_y) if 0 < t < dt_view)
    t_key2.extend(t for t in (t_overlap_y, t_overlap_y - dt2_y, t_overlap_y + dt2_y) if 0 < t < dt_view)

    # == Simulate potential loss of vertical separation ===
    # The "deviation" approach for lateral separation loss doesn't work for vertical separation
    # because the nominal positions of the aircraft are in overlap.  Instead, just draw a single
    # sample from vertical deviation distribution and assume the vertical deviation is roughly
    # constant for the entire duration of the encounter
    sigma_z = compute_sigma(encounter.h, math.pow(0.95, 1 / 2))
    z1 = r.gauss(0, sigma_z)
    z2 = r.gauss(0, sigma_z)

    aircraft_size = Point3(encounter.lambda_x, encounter.lambda_y, encounter.lambda_z)

    m1 = np.zeros((len(t_key1), 4), dtype=float)
    t_key1.sort()
    m1[:, 0] = np.array(t_key1, dtype=float)
    m1[:, 1] = np.array([fx1(t) for t in t_key1], dtype=float)
    m1[:, 2] = np.array([fy1(t) for t in t_key1], dtype=float)
    m1[:, 3] = z1
    path1 = FlightPath(m1)
    op_intent1_lbound = Point3(x1a - encounter.lambda_x, -encounter.S_y / 2 - encounter.w, -encounter.h)
    op_intent1_ubound = Point3(x1b + encounter.lambda_x, -encounter.S_y / 2 + encounter.w, encounter.h)
    flight1 = Flight(path=path1, op_intent=(op_intent1_lbound, op_intent1_ubound), size=aircraft_size)

    m2 = np.zeros((len(t_key2), 4), dtype=float)
    t_key2.sort()
    m2[:, 0] = np.array(t_key2, dtype=float)
    m2[:, 1] = np.array([fx2(t) for t in t_key2], dtype=float)
    m2[:, 2] = np.array([fy2(t) for t in t_key2], dtype=float)
    m2[:, 3] = z2
    path2 = FlightPath(m2)
    op_intent2_lbound = Point3(x2a - encounter.lambda_x, encounter.S_y / 2 - encounter.w, -encounter.h)
    op_intent2_ubound = Point3(x2b + encounter.lambda_x, encounter.S_y / 2 + encounter.w, encounter.h)
    flight2 = Flight(path=path2, op_intent=(op_intent2_lbound, op_intent2_ubound), size=aircraft_size)

    return [flight1, flight2]
