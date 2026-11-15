# -*- coding: utf-8 -*-
"""
はめあい確認用テストピース (frame.py の形状をそのまま切り出す)

実行:  /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd cad/test_fit.py
出力:  cad/stl/test/*.stl
"""
import builtins
import os
import sys

import Mesh

builtins._ROVER_IMPORT_ONLY = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import frame as rf  # noqa: E402

OUT_DIR = os.path.join(rf.OUT_DIR, "stl", "test")


def crop(shape, lx, ly, cx, cy, z0, z1):
    return shape.common(rf.box(lx, ly, z1 - z0, cx, cy, z0)).removeSplitter()


def main():
    base = rf.make_base()
    top = rf.make_top()
    z_base = rf.BASE_Z

    # 1) モータ取付部 (左側1個): 取付壁 + サドル + 底板
    y0 = rf.WALL_IN - rf.GB_L - rf.CAN_L - 2
    y1 = rf.WALL_IN + rf.WALL_T + 1
    motor = crop(base, 24, y1 - y0, 0, (y0 + y1) / 2, z_base, rf.AXLE_Z + 10)

    # 2) LiDAR 座: 脚3本の座面 + 穴を含む枠 (中央を肉抜きして時間短縮)
    xs = [x for x, _ in rf.LIDAR_HOLES]
    ys = [y for _, y in rf.LIDAR_HOLES]
    m = rf.LIDAR_PAD_D / 2 + 2
    x0, x1 = min(xs) - m, max(xs) + m
    y0, y1 = min(ys) - m, max(ys) + m
    lidar = crop(top, x1 - x0, y1 - y0, (x0 + x1) / 2, 0,
                 rf.TOP_Z, rf.TOP_Z + rf.PLATE_T + rf.LIDAR_PAD_H)
    lidar = lidar.cut(rf.box(2 * 29, 2 * 18, 20, 0, 0, rf.TOP_Z - 5))
    lidar = lidar.removeSplitter()

    os.makedirs(OUT_DIR, exist_ok=True)
    for name, shp in (("TestFit_Motor", motor),
                      ("TestFit_LiDAR", lidar)):
        s = shp.copy()
        bb = s.BoundBox
        s.translate(rf.V(-(bb.XMin + bb.XMax) / 2, -(bb.YMin + bb.YMax) / 2, -bb.ZMin))
        Mesh.Mesh(s.tessellate(0.05)).write(os.path.join(OUT_DIR, name + ".stl"))
        bb = s.BoundBox
        print("%-16s valid=%s solids=%d  size %.1f x %.1f x %.1f mm  vol=%.1f cm3" % (
            name, s.isValid(), len(s.Solids), bb.XLength, bb.YLength, bb.ZLength,
            s.Volume / 1000))


main()
