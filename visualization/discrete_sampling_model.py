import math
import random
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
from panda3d.core import Point3

from flights import Flight, FlightPath
import reich_model
from vizmath import compute_sigma, compute_volume_size


@dataclass
class ParallelPathsEncounterDescriptor(object):
    time_length: float
    """Duration of the encounter"""

    v1_ground: float
    """Longitudinal ground velocity (directional) of the first flight"""

    v2_ground: float
    """Longitudinal ground velocity (directional) of the second flight"""

    lateral_separation: float
    """Nominal lateral separation between the flights"""

    aircraft_size: Point3
    """Size of aircraft collision volume"""

    sampling_frequency: float
    """Frequency at which new deviation samples are drawn (defines caffeination)"""

    sigma: Point3
    """Standard deviation of deviations in each axis when a new sample is drawn"""

    r: Optional[random.Random] = None
    """Specific random number generator to generate paths, or None to use system default"""


def infer_caffeination(fraction_inside_bound: float, average_speed_at_bound_exit: float, bound_size: float) -> float:
    k_table = np.array((
        (0.8, 0.633),
        (0.9, 0.579),
        (0.95, 0.554),
        (0.99, 0.531),
        (0.999, 0.52),
    ), dtype=float)
    k = np.interp(fraction_inside_bound, k_table[:, 0], k_table[:, 1])
    dt = k * bound_size / average_speed_at_bound_exit
    return dt


def make_parallel_paths_descriptor(reich: Optional[reich_model.ParallelPathsEncounterDescriptor] = None) -> ParallelPathsEncounterDescriptor:
    """Create a discrete_sampling_model ParallelPathsEncounterDescriptor designed to match (as closely as practical) the physical setup of the provided reich_model ParallelPathsEncounterDescriptor."""

    reich = reich or reich_model.standard_parallel_paths_descriptor()
    time_length = 5  # Arbitrary time horizon as there is no obvious time-limiting built into the discrete sampling model
    op_intent_width = 2 * reich.w
    op_intent_height = 2 * reich.h
    p_one_axis = math.pow(0.95, 1 / 2)
    dt_y = infer_caffeination(p_one_axis, reich.YS_y, 2 * reich.w)
    dt_z = infer_caffeination(p_one_axis, reich.delta_z, 2 * reich.h)
    dt = min(dt_y, dt_z)  # Be conservative and pick worst-case caffeination
    sigma_y = compute_sigma(op_intent_width, p_one_axis)
    sigma_z = compute_sigma(op_intent_height, p_one_axis)
    sigma_x = 0
    return ParallelPathsEncounterDescriptor(
        time_length=time_length,
        v1_ground=reich.v,
        v2_ground=reich.v - reich.delta_v,
        lateral_separation=reich.S_y,
        aircraft_size=Point3(reich.lambda_x, reich.lambda_y, reich.lambda_z),
        sampling_frequency=1 / dt,
        sigma=Point3(sigma_x, sigma_y, sigma_z)
    )


def make_flight_path_along_x(time_length: float, dx: float, dt: float, sigma: Point3, r: Optional[random.Random] = None) -> FlightPath:
    """Make a simple canonical flight path traveling longitudinally (x axis).

    The flight will start at x=0 and t=0 and then continue nominally dx forward
    along the x axis for every discrete time step of dt.  Deviations from the
    nominal position are applied according to sigma for a specific time point,
    but do not affect the nominal position at the following time point.  This
    process is repeated until time_length is exceeded, at which point the path
    is truncated to time_length.

    :param time_length: Duration of flight path to generate.
    :param dx: Nominal longitudinal distance to travel at each discrete time step.
    :param dt: Length of each discrete time step.
    :param sigma: Scale of normal distributions for deviations from nominal position.
    :param r: Specific random number generator to use to generate deviations.
    :return: Generated flight path.
    """
    r = r or random

    # Generate key points until we pass time_length
    m = []
    t, x, y, z = -dt, -dx, 0, 0
    while t < time_length:
        t += dt
        x += dx
        y = r.gauss(0, sigma.y)
        z = r.gauss(0, sigma.z)
        x_dev = r.gauss(0, sigma.x)
        m.append((t, x + x_dev, y, z))

    # Truncate last key point to time_length
    f = (time_length - m[-2][0]) / dt
    m[-1] = tuple(f * i1 + (1 - f) * i0 for i0, i1 in zip(m[-2], m[-1]))

    return FlightPath(np.array(m, dtype=float))


def make_flight(time_length: float, ground_speed: float, sampling_frequency: float, lateral_position: float, sigma: Point3, aircraft_size: Point3, r: random.Random) -> Flight:
    path_length = time_length * ground_speed
    dt = 1 / sampling_frequency
    dx = ground_speed * dt
    path = make_flight_path_along_x(time_length, dx, dt, sigma, r).offset(dx=-path_length / 2, dy=lateral_position)
    center = Point3(0, lateral_position, 0)
    op_intent_size = Point3(
        path_length + 2 * 4 * sigma.x + aircraft_size.x,
        compute_volume_size(sigma.y, math.pow(0.95, 1 / 2)),
        compute_volume_size(sigma.z, math.pow(0.95, 1 / 2))
    )
    return Flight(path=path, op_intent=(center - op_intent_size / 2, center + op_intent_size / 2), size=aircraft_size)


def make_parallel_paths(encounter: Optional[ParallelPathsEncounterDescriptor] = None) -> List[Flight]:
    """Generate 2 flights on parallel paths for a longitudinal encounter."""

    encounter = encounter or make_parallel_paths_descriptor()
    r = encounter.r or random

    return [
        make_flight(
            time_length=encounter.time_length,
            ground_speed=encounter.v1_ground,
            sampling_frequency=encounter.sampling_frequency,
            lateral_position=-encounter.lateral_separation / 2,
            sigma=encounter.sigma,
            aircraft_size=encounter.aircraft_size,
            r=r
        ),
        make_flight(
            time_length=encounter.time_length,
            ground_speed=encounter.v2_ground,
            sampling_frequency=encounter.sampling_frequency,
            lateral_position=encounter.lateral_separation / 2,
            sigma=encounter.sigma,
            aircraft_size=encounter.aircraft_size,
            r=r
        ),
    ]
