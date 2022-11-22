from typing import List

from direct.showbase.ShowBase import ShowBase
from panda3d.core import Point2, Vec2, Point3, Vec3


def center_of(pts: List[Point2]) -> Point2:
    if len(pts) == 3:
        x1 = pts[0].x
        y1 = pts[0].y
        x2 = pts[1].x
        y2 = pts[1].y
        x3 = pts[2].x
        y3 = pts[2].y
        n = x3 * x3 * (y2 - y1) + x2 * x2 * (y1 - y3) + (y3 - y2) * (x1 * x1 + (y1 - y2) * (y1 - y3))
        d = 2 * (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2))
        xc = n / d
        n = (-x2 * x2 * x3 + x1 * x1 * (x3 - x2) + x3 * (y1 - y2) * (y1 + y2) + x1 * (x2 * x2 - x3 * x3 + y2 * y2 - y3 * y3) + x2 * (x3 * x3 - y1 * y1 + y3 * y3))
        d = 2 * (x3 * (y1 - y2) + x1 * (y2 - y3) + x2 * (y3 - y1))
        yc = n / d
        return Point2(xc, yc)
    else:
        indices = [0, 1, 2]
        success = False
        while not success:
            c = center_of([pts[i] for i in indices])
            r = Vec2(pts[indices[0]] - c).length()
            success = True
            for p in pts:
                rp = Vec2(p - c).length()
                if rp > r:
                    success = False
                    break
            if success:
                return c
            indices[2] += 1
            if indices[2] >= len(pts):
                indices[1] += 1
                indices[2] = indices[1] + 1
                if indices[2] >= len(pts):
                    indices[0] += 1
                    indices[1] = indices[0] + 1
                    indices[2] = indices[1] + 1


def zoom_to_fit(self: ShowBase) -> None:
    lbound = Point3()
    ubound = Point3()
    self.render.calcTightBounds(min_point=lbound, max_point=ubound)
    center = (lbound + ubound) / 2
    corners = [Point3(
        lbound.x if i % 2 == 0 else ubound.x,
        lbound.y if i % 4 < 2 else ubound.y,
        lbound.z if i < 4 else ubound.z
    ) for i in range(8)]
    def viewable():
        visible = []
        invisible = 0
        for c in corners:
            p3d = self.cam.getRelativePoint(self.render, c)
            p2d = Point2()
            if self.camLens.project(p3d, p2d):
                visible.append(p2d)
            else:
                invisible += 1
        return visible, invisible

    # Zoom out until all bounding corners are in view
    self.camera.lookAt(center)
    v_max = self.camera.getPos() - center
    visible, invisible = viewable()
    while invisible:
        v_max *= 2
        self.camera.setPos(center + v_max)
        self.camera.lookAt(center)
        visible, invisible = viewable()

    # Reorient view toward center of projected points
    # center2d = center_of(visible)
    # fov = self.camLens.getFov()
    # dh = (center2d.x / self.camLens.getAspectRatio()) * (fov.x / 2)
    # dp = -center2d.y * (fov.y / 2)
    # hpr = self.camera.getHpr()
    # hpr.x += dh
    # hpr.y += dp
    # self.camera.setHpr(hpr)
    # p2d = Point3(0.5, 0, v_max.length())
    # asdf = self.camLens.extrudeDepth(p2d, center)
    # asdf = self.cam.getRelativePoint(self.camLens, center)
    # v_max = self.camera.getPos() - center

    # Zoom in until at least one bounding corner is out of view
    v_min = Vec3(v_max)
    while not invisible:
        v_min /= 2
        self.camera.setPos(center + v_min)
        visible, invisible = viewable()

    # Perform binary search to find best-fit view
    while (v_max - v_min).length() > 0.01 * v_max.length():
        v_new = (v_min + v_max) / 2
        self.camera.setPos(center + v_new)
        visible, invisible = viewable()
        if invisible:
            v_min = v_new
        else:
            v_max = v_new
