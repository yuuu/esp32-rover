# -*- coding: utf-8 -*-
"""
Test-fit pieces (cut directly out of the frame.py shapes)

Run:     freecadcmd cad/test_fit.py
Output:  cad/stl/test/*.stl
"""
import builtins
import os
import sys

import Mesh

builtins._ROVER_IMPORT_ONLY = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import frame as rf  # noqa: E402

OUT_DIR = os.path.join(rf.OUT_DIR, "stl", "test")
BORE_TEST_CLRS = (0.35, 0.4, 0.45, 0.5)   # TestFit_Bore clearances (diameter and flat)


def crop(shape, lx, ly, cx, cy, z0, z1):
    return shape.common(rf.box(lx, ly, z1 - z0, cx, cy, z0)).removeSplitter()


def main():
    base = rf.make_base()
    top = rf.make_top()
    z_base = rf.BASE_Z

    # 1) Motor mount (left side only): pocket + saddle + base plate
    y0 = rf.GB_FRONT - rf.GB_L - rf.CAN_L - 2
    y1 = rf.GB_FRONT + 1
    motor = crop(base, 24, y1 - y0, 0, (y0 + y1) / 2, z_base, rf.AXLE_Z + 10)

    # 2) LiDAR seat: frame containing the 3 leg pads and holes (center removed to save time)
    xs = [x for x, _ in rf.LIDAR_HOLES]
    ys = [y for _, y in rf.LIDAR_HOLES]
    m = rf.LIDAR_PAD_D / 2 + 2
    x0, x1 = min(xs) - m, max(xs) + m
    y0, y1 = min(ys) - m, max(ys) + m
    lidar = crop(top, x1 - x0, y1 - y0, (x0 + x1) / 2, 0,
                 rf.TOP_Z, rf.TOP_Z + rf.PLATE_T + rf.LIDAR_PAD_H)
    lidar = lidar.cut(rf.box(2 * 29, 2 * 18, 20, 0, 0, rf.TOP_Z - 5))
    lidar = lidar.removeSplitter()

    # 3) D-cut bores: bosses with different clearances side by side. Printed like the wheel (bore vertical).
    #    The number of dots in front of a boss = its index in BORE_TEST_CLRS (1 dot = 0.35)
    pitch, boss_h, base_t = 14.0, rf.HUB_L + rf.WHEEL_W, 1.5
    n = len(BORE_TEST_CLRS)
    bore = rf.box(pitch * n, pitch, base_t, pitch * (n - 1) / 2, 0, 0)
    for i, clr in enumerate(BORE_TEST_CLRS):
        x = i * pitch
        bore = bore.fuse(rf.cyl(rf.HUB_D / 2, boss_h, x, 0, 0))
        for k in range(i + 1):
            bore = bore.fuse(rf.cyl(0.6, 0.6, x - 1.5 * i + 3 * k, -5.5, base_t))
        h = rf.d_bore(clr, boss_h + 2, -1)
        h.translate(rf.V(x, 0, 0))
        bore = bore.cut(h)
    bore = bore.removeSplitter()

    # 4) Caster: ball seat in the base + countersunk screw holes (use Caster_x2.stl as the holder)
    caster = crop(base, 2 * rf.CASTER_R + 4, 2 * rf.CASTER_R + 4, rf.CASTER_X, 0,
                  z_base, z_base + rf.PLATE_T)

    os.makedirs(OUT_DIR, exist_ok=True)
    for name, shp in (("TestFit_Motor", motor),
                      ("TestFit_LiDAR", lidar),
                      ("TestFit_Bore", bore),
                      ("TestFit_Caster", caster)):
        s = shp.copy()
        bb = s.BoundBox
        s.translate(rf.V(-(bb.XMin + bb.XMax) / 2, -(bb.YMin + bb.YMax) / 2, -bb.ZMin))
        Mesh.Mesh(s.tessellate(0.05)).write(os.path.join(OUT_DIR, name + ".stl"))
        bb = s.BoundBox
        print("%-16s valid=%s solids=%d  size %.1f x %.1f x %.1f mm  vol=%.1f cm3" % (
            name, s.isValid(), len(s.Solids), bb.XLength, bb.YLength, bb.ZLength,
            s.Volume / 1000))


main()
