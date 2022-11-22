import math
import random
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
from panda3d.core import Point3
from scipy.stats import norm

from flights import Flight, FlightPath


@dataclass
class ParallelPathsEncounterDescriptor(object):
    time_length: float
    ground_speed: float
    lateral_separation: float
    aircraft_size: Point3
    op_intent_size: Point3
    sampling_frequency: float
    sigma: Point3
    r: Optional[random.Random] = None


def compute_sigma(volume_size: float, p_containment: float = 0.95) -> float:
    return volume_size / 2 / norm.ppf(1 - (1 - p_containment) / 2)


def make_flight_path_along_x(path_length: float, dx: float, dt: float, sigma: Point3, r: Optional[random.Random] = None) -> FlightPath:
    if r is None:
        r = random

    m = []

    t = 0
    x = 0

    while x < path_length:
        y = r.gauss(0, sigma.y)
        z = r.gauss(0, sigma.z)
        x_dev = r.gauss(0, sigma.x)
        m.append((t, x + x_dev, y, z))
        x += dx
        t += dt
    f = (path_length - x) / dx
    t += dt * f
    x = f * (path_length + r.gauss(0, sigma.x)) + (1 - f) * x
    y = f * r.gauss(0, sigma.y) + (1 - f) * y
    z = f * r.gauss(0, sigma.z) + (1 - f) * z
    m.append((t, x, y, z))
    return FlightPath(np.array(m, dtype=float))


def make_parallel_paths_same_direction(encounter: Optional[ParallelPathsEncounterDescriptor] = None) -> List[Flight]:
    encounter = encounter or standard_discrete_sampling_model_parallel_paths_descriptor()
    path_length = encounter.time_length * encounter.ground_speed
    r = encounter.r or random
    x0 = -path_length / 2
    dt = 1 / encounter.sampling_frequency
    dx = encounter.ground_speed * dt

    center1 = Point3(0, -encounter.lateral_separation / 2, 0)
    path1 = make_flight_path_along_x(path_length, dx, dt, encounter.sigma, r).offset(dx=x0 + center1.x, dy=center1.y, dz=center1.z)
    flight1 = Flight(path=path1, op_intent=(center1 - encounter.op_intent_size / 2, center1 + encounter.op_intent_size / 2), size=encounter.aircraft_size)

    center2 = Point3(0, encounter.lateral_separation / 2, 0)
    path2 = make_flight_path_along_x(path_length, dx, dt, encounter.sigma, r).offset(dx=x0 + center2.x, dy=center2.y, dz=center2.z)
    flight2 = Flight(path=path2, op_intent=(center2 - encounter.op_intent_size / 2, center2 + encounter.op_intent_size / 2), size=encounter.aircraft_size)

    return [flight1, flight2]


def make_parallel_paths_opposite_direction(encounter: Optional[ParallelPathsEncounterDescriptor] = None) -> List[Flight]:
    encounter = encounter or standard_discrete_sampling_model_parallel_paths_descriptor()
    flight1, flight2 = make_parallel_paths_same_direction(encounter)
    flight2.path = flight2.path.scale(fx=-1)
    return [flight1, flight2]


def standard_discrete_sampling_model_parallel_paths_descriptor() -> ParallelPathsEncounterDescriptor:
    time_length = 10
    ground_speed = 4
    path_length = time_length * ground_speed
    op_intent_width = 6
    op_intent_height = 4
    sigma_y = compute_sigma(op_intent_width, math.pow(0.95, 1 / 2))
    sigma_z = compute_sigma(op_intent_height, math.pow(0.95, 1 / 2))
    sigma_x = (sigma_y + sigma_z) / 2
    return ParallelPathsEncounterDescriptor(
        time_length=time_length,
        ground_speed=ground_speed,
        lateral_separation=8,
        aircraft_size=Point3(2, 2, 2),
        op_intent_size=Point3(path_length + 4 * 2 * sigma_x, 6, 4),
        sampling_frequency=1 / 2,
        sigma=Point3(sigma_x, sigma_y, sigma_z)
    )
