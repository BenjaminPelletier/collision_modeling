from dataclasses import dataclass
from datetime import datetime, timedelta
from enum import Enum
from math import pi, sin, cos
from typing import Optional, Tuple, List, Iterable, Callable, Dict

from direct.gui.DirectGui import DirectOptionMenu
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import Point3, LineSegs, NodePath, LColor
from pandac.PandaModules import WindowProperties

from flights import Flight, FlightPath
from geometry import zoom_to_fit


def line_box(ll: Point3, ur: Point3, color: Optional[LColor], thickness: int = 4) -> LineSegs:
    lines = LineSegs('box')
    lines.set_color(color)

    # Bottom
    lines.moveTo(ll)
    lines.drawTo(ur.x, ll.y, ll.z)
    lines.drawTo(ur.x, ur.y, ll.z)
    lines.drawTo(ll.x, ur.y, ll.z)
    lines.drawTo(ll)

    # Top + ll vertical
    lines.drawTo(ll.x, ll.y, ur.z)
    lines.drawTo(ur.x, ll.y, ur.z)
    lines.drawTo(ur.x, ur.y, ur.z)
    lines.drawTo(ll.x, ur.y, ur.z)
    lines.drawTo(ll.x, ll.y, ur.z)

    # Remaining 3 verticals
    lines.moveTo(ll.x, ur.y, ll.z)
    lines.drawTo(ll.x, ur.y, ur.z)
    lines.moveTo(ur.x, ur.y, ll.z)
    lines.drawTo(ur.x, ur.y, ur.z)
    lines.moveTo(ur.x, ll.y, ll.z)
    lines.drawTo(ur.x, ll.y, ur.z)

    lines.set_thickness(thickness)
    return lines


def rotate_axes(p: Point3, d: int):
    if d % 3 == 0:
        return p
    elif d % 3 == 1:
        return Point3(p.y, p.z, p.x)
    elif d % 3 == 2:
        return Point3(p.z, p.x, p.y)
    else:
        raise RuntimeError()


def make_shadows(lower: Point3, upper: Point3, lbound: Point3, ubound: Point3) -> List[Tuple[LColor, List[Point3]]]:
    """Make shadows in the X direction.

    :param lower: Lower corner of aircraft bounding box
    :param upper: Upper corner of aircraft bounding box
    :param lbound: Lower corner of operational intent bounding box
    :param ubound: Upper corner of operational intent bounding box
    :return: List of:
        * Color that reference axis marks should be
        * Point to move to, followed by N points to draw to
    """
    bad = LColor(0.9, 0.4, 0.4, 0.6)
    shadow = LColor(0.3, 0.3, 0.3, 0.2)
    bad_shadow = LColor(0.6, 0.1, 0.1, 0.2)
    
    if lower.x >= lbound.x and upper.x <= ubound.x:
        # Aircraft fully contained in bounding box
        return [
            (shadow, [
                Point3(lbound.x, lower.y, lower.z),
                Point3(lbound.x, lower.y, upper.z),
                Point3(lbound.x, upper.y, upper.z),
                Point3(lbound.x, upper.y, lower.z),
                Point3(lbound.x, lower.y, lower.z)
            ]),
            (shadow, [
                Point3(ubound.x, lower.y, lower.z),
                Point3(ubound.x, lower.y, upper.z),
                Point3(ubound.x, upper.y, upper.z),
                Point3(ubound.x, upper.y, lower.z),
                Point3(ubound.x, lower.y, lower.z)
            ]),
        ]
    elif lower.x <= lbound.x <= upper.x:
        # Aircraft straddling lower bounding box boundary
        return [
            (bad, [
                Point3(lbound.x, lower.y, lower.z),
                Point3(lbound.x, lower.y, upper.z),
                Point3(lbound.x, upper.y, upper.z),
                Point3(lbound.x, upper.y, lower.z),
                Point3(lbound.x, lower.y, lower.z)
            ]),
            (shadow, [
                Point3(ubound.x, lower.y, lower.z),
                Point3(ubound.x, lower.y, upper.z),
                Point3(ubound.x, upper.y, upper.z),
                Point3(ubound.x, upper.y, lower.z),
                Point3(ubound.x, lower.y, lower.z)
            ]),
        ]
    elif lower.x <= ubound.x <= upper.x:
        # Aircraft straddling upper bounding box boundary
        return [
            (bad, [
                Point3(ubound.x, lower.y, lower.z),
                Point3(ubound.x, lower.y, upper.z),
                Point3(ubound.x, upper.y, upper.z),
                Point3(ubound.x, upper.y, lower.z),
                Point3(ubound.x, lower.y, lower.z)
            ]),
            (shadow, [
                Point3(lbound.x, lower.y, lower.z),
                Point3(lbound.x, lower.y, upper.z),
                Point3(lbound.x, upper.y, upper.z),
                Point3(lbound.x, upper.y, lower.z),
                Point3(lbound.x, lower.y, lower.z)
            ]),
        ]
    elif upper.x < lbound.x:
        # Aircraft under lower bounding box boundary
        return [
            (bad_shadow, [
                Point3(lbound.x, lower.y, lower.z),
                Point3(lbound.x, lower.y, upper.z),
                Point3(lbound.x, upper.y, upper.z),
                Point3(lbound.x, upper.y, lower.z),
                Point3(lbound.x, lower.y, lower.z)
            ]),
        ]
    elif lower.x > ubound.x:
        # Aircraft over upper bounding box boundary
        return [
            (bad_shadow, [
                Point3(ubound.x, lower.y, lower.z),
                Point3(ubound.x, lower.y, upper.z),
                Point3(ubound.x, upper.y, upper.z),
                Point3(ubound.x, upper.y, lower.z),
                Point3(ubound.x, lower.y, lower.z)
            ]),
        ]

    raise RuntimeError()


class Aircraft(object):
    node: NodePath
    op_intent_volume: NodePath
    bounding_volume: NodePath
    reference_axes: NodePath

    _ll: Point3
    _ur: Point3
    _location: Point3
    _size: Point3

    def __init__(self, node: NodePath, op_intent_volume: Tuple[Point3, Point3], location: Point3, size: Point3, color: LColor):
        self.node = node
        self._ll, self._ur = op_intent_volume
        self._location = location
        self._size = size

        self.op_intent_volume = NodePath(line_box(op_intent_volume[0], op_intent_volume[1], LColor(0.7, 0.7, 0.7, 0.5), thickness=1).create())

        self.bounding_volume = NodePath(line_box(-size / 2, size / 2, color).create(dynamic=True))
        self.bounding_volume.setPos(location)

        self.reference_axes = NodePath(self._make_reference_axes().create())

        self.op_intent_volume.reparentTo(self.node)
        self.bounding_volume.reparentTo(self.node)
        self.reference_axes.reparentTo(self.node)

    def _make_reference_axes(self) -> LineSegs:
        lines = LineSegs('ref_axes')

        for axis in (0, 1, 2):
            shadows = make_shadows(
                rotate_axes(self._location - self._size / 2, axis),
                rotate_axes(self._location + self._size / 2, axis),
                rotate_axes(self._ll, axis),
                rotate_axes(self._ur, axis))
            for color, pts in shadows:
                lines.set_color(color)
                lines.moveTo(rotate_axes(pts[0], -axis))
                for p in pts[1:]:
                    lines.drawTo(rotate_axes(p, -axis))

        lines.set_thickness(2)
        return lines

    def move_to(self, p: Point3):
        self.bounding_volume.setPos(p)

        self.reference_axes.removeNode()
        self.reference_axes = NodePath(self._make_reference_axes().create())
        self.reference_axes.reparentTo(self.node)

        self._location = p


@dataclass
class MovingAircraft(object):
    aircraft: Aircraft
    path: FlightPath


class EncounterVisualization(object):
    aircraft: List[MovingAircraft]
    t_start: datetime
    t_end: datetime

    def __init__(self, parent: NodePath, flights: Iterable[Flight]):
        self.aircraft = [
            MovingAircraft(
                aircraft=Aircraft(
                    parent,
                    f.op_intent, f.path.location_at(0), f.size,
                    LColor(0.9, 0.9, 0.9, 0.8)),
                path=f.path)
            for f in flights]
        self.t_start = datetime.utcnow()
        self.t_end = self.t_start + timedelta(seconds=max(ac.path.t_max() for ac in self.aircraft))

    def update(self) -> bool:
        t_now = datetime.utcnow()
        if t_now >= self.t_end:
            return True

        t = (t_now - self.t_start).total_seconds()
        for ac in self.aircraft:
            p = ac.path.location_at(t)
            ac.aircraft.move_to(p)

        return False


@dataclass
class MotionModel(object):
    name: str
    """Human-readable name of motion model"""

    make_flights: Callable[[], List[Flight]]
    """Function to generate an instance of flight motion to be visualized using the motion model"""


class EncounterVisualizer(ShowBase):
    _encounter: Optional[EncounterVisualization] = None
    motion_models: Dict[str, MotionModel]
    selected_model: str
    auto_spin: bool = False

    def __init__(self, motion_models: List[MotionModel]):
        """Make an EncounterVisualizer app instance.

        :param motion_models: List of motion models to enable.
        """
        ShowBase.__init__(self)

        props = WindowProperties()
        props.setTitle("animate_simulations")
        self.win.requestProperties(props)

        self.motion_models = {m.name: m for m in motion_models}
        self.selected_model = motion_models[0].name
        self.menu = DirectOptionMenu(
            text="Motion model", scale=0.07, command=self._on_select_model,
            items=[m.name for m in motion_models], initialitem=0,
            highlightColor=(0.65, 0.65, 0.65, 1))
        self.menu.setPos(-self.getAspectRatio(), 0, 1 - self.menu.getScale().z)

        self.disableMouse()
        self.camera.setPos(-40, -20, 20)
        self.camera.lookAt(0, 0, 0)
        #self.enableMouse()

        self.taskMgr.add(self._spin_camera_task, "SpinCameraTask")
        self.taskMgr.add(self._update_encounter_task, "UpdateEncounterTask")

    def _on_select_model(self, model_name: str):
        self.selected_model = model_name

    def _spin_camera_task(self, task):
        if not self.auto_spin:
            return Task.cont

        r = 40
        angleDegrees = task.time * 30.0
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(r * sin(angleRadians), -r * cos(angleRadians), 3)
        self.camera.setHpr(angleDegrees, 0, 0)
        return Task.cont

    def _update_encounter_task(self, task):
        if self._encounter is not None:
            complete = self._encounter.update()
            if complete:
                for ac in self._encounter.aircraft:
                    ac.aircraft.node.removeNode()
                self._encounter = None

        if self._encounter is None:
            print(f"Generating new encounter using {self.selected_model}")
            self._encounter = EncounterVisualization(
                parent=self.render.attachNewNode('encounter'),
                flights=self.motion_models[self.selected_model].make_flights())
            zoom_to_fit(self)

        return Task.cont
