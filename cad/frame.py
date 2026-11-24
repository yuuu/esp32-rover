# -*- coding: utf-8 -*-
"""
ESP32 + LiDAR SLAM rover frame prototype (FreeCAD 1.x)

Run:     freecadcmd cad/frame.py
Output:  cad/frame.FCStd, cad/stl/*.stl

Coordinates: X=forward, Y=left, Z=up, origin = on the floor directly below the midpoint of the axles
Units: mm
"""
import math
import os
import FreeCAD as App
import Part
import Mesh
from FreeCAD import Vector as V

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- Part dimensions
# Tire FS90R-W: 60 dia (wheel 56 dia), width 8
WHEEL_D, WHEEL_W = 60.0, 8.0
# Wheel (dimensioned from the shape of the distributed STL). Printed in one piece with the hub
WHEEL_RIM_D, WHEEL_RIM_IN_D = 50.0, 47.0     # rim outer dia (both edges), inner dia
WHEEL_BEAD_D, WHEEL_BEAD_W = 56.0, 4.0       # tire seat at the middle of the width
WHEEL_BOSS_D, WHEEL_BORE_D = 20.0, 3.3       # center disk, center hole
WHEEL_SPOKE_W, WHEEL_SPOKES = 6.0, 6
HUB_D, HUB_L = 9.0, 3.0                      # boss on the inner side of the wheel
BORE_CLR = 0.33         # D-cut bore clearance (diameter and flat). TestFit_Bore: 0.3 was tight, 0.35 was just right
GB_BOSS_D, GB_BOSS_L = 4.0, 1.0              # bearing boss on the gearbox face (length is estimated)
# Motor MG12A06E-060310-50 (N20 compatible)
# Gearbox width (X), height (Z), length (Y). In the orientation where the encoder board clears
# the cradle, the 12mm side is horizontal (confirmed by a test print)
GB_W, GB_H, GB_L = 12.0, 10.0, 9.0
CAN_D, CAN_L = 12.2, 22.0               # motor + encoder
CAN_H = 10.0                            # the can is flat top and bottom (Z), height 10 (same orientation as the gearbox 10mm side)
SHAFT_D, SHAFT_FLAT, SHAFT_L = 3.0, 2.5, 10.0
GB_CLR = 0.3                            # gearbox pocket clearance
GB_POCKET_WALL = 2.0                    # pocket side wall thickness
GB_POCKET_TOP = 33.0                    # top Z of the pocket side walls (gearbox top is 36)
# Positions of the cable-tie slots holding the can (distance from the gearbox rear end). The rear
# ~7mm of the can is the encoder, and a slot there is unusable (16 hit the encoder on the real part)
TIE_SLOT_OFFSETS = (6.0, 12.0)
# PCB pcb/main (main board): 60x60, M3 holes at 50mm pitch
PCB_SIZE, PCB_HOLE_PITCH, PCB_T = 60.0, 50.0, 1.6
# Battery box BH-363-1B (6x AA back to back, 9V-type snap, no lid or switch)
BAT_L, BAT_W, BAT_H = 58.0, 45.0, 29.0   # L=along the cells (Y), W=X, H=two layers (measured height; datasheet says 28)
BAT_SNAP = 2.4                           # snap protrusion (+Y end face)
# LiDAR YDLIDAR X2: 96 x 60.5 x 50.3 (datasheet FIG1/FIG2, Marutsu YDLIDAR_X2.pdf)
# The rotation center is at the origin with the motor side facing +X (forward) -> scan 0 deg is forward
LIDAR_L, LIDAR_W, LIDAR_H = 96.0, 60.5, 50.3
LIDAR_HEAD_D = 60.5
LIDAR_HOLES = [(-35.3, 0.0), (36.0, 18.0), (36.0, -18.0)]   # positions of the 3 legs (standoffs)
LIDAR_HOLE_D = 3.4      # M3 clearance hole (the datasheet gives no screw size; fastened with M3)
LIDAR_PAD_D, LIDAR_PAD_H = 8.0, 2.0   # pads under the legs (clearance for the bottom motor)
LIDAR_MOTOR_GAP = 2.5   # from the leg bottom to the LiDAR motor bottom (50.3 - 47.8)
LIDAR_LASER_Z = LIDAR_H - 7.9          # from the leg bottom to the laser center

# ---------------------------------------------------------------- Frame dimensions
BODY_D = 130.0          # round deck diameter
PLATE_T = 3.0
BASE_Z = 10.0           # ground clearance of the base bottom
AXLE_Z = WHEEL_D / 2    # axle height 30
GB_FRONT = 38.5         # Y of the gearbox face (the shaft sticks out about 9mm from here)
WHEEL_GAP = 0.5         # gap between the gearbox face and the hub
WHEEL_Y = GB_FRONT + WHEEL_GAP + HUB_L + WHEEL_W / 2   # Y of the wheel center plane (=46)
# The battery slides out rearward with the mid deck attached. To clear the rear wall, the mid
# deck bottom is at battery top + rear wall height + lift margin
BAT_REAR_WALL_H = 3.0   # rear wall height (just enough to keep the battery from sliding off)
BAT_LIFT_CLR = 1.0
MOTOR_CABLE_CLR = 5.0   # added on top of the above; keeps the motor cable connectors clear of the mid deck
MID_Z = BASE_Z + PLATE_T + BAT_H + BAT_REAR_WALL_H + BAT_LIFT_CLR + MOTOR_CABLE_CLR   # mid deck bottom (=51)
# Cable notch (rear). It is open to the outer edge, so cables go in from the side without unplugging
CABLE_NOTCH_W = 10.0
MID_NOTCH_X = -36.0     # X of the inner end of the mid deck notch
TOP_NOTCH_X = -48.0     # X of the inner end of the top deck notch
PCB_STANDOFF = 5.0
PCB_CLEAR = 25.0        # component clearance above the PCB
TOP_Z = MID_Z + PLATE_T + PCB_STANDOFF + PCB_T + PCB_CLEAR   # top deck bottom
BALL_D = 11.7           # plastic caster ball (a 1/2" steel ball was not available)
CASTER_X = 45.0
# The caster ball is held by a separate holder pressed against the base from below and fastened
# with M3 countersunk screws from above. This keeps the base bottom flat so it needs no supports
BALL_CLR = 0.4          # ball clearance in the holder (radius). 0.2 was too tight to roll
# How far the ball is raised off the floor in the model. With the ball at floor level the tires
# floated 1-2mm (on the real part). The limit is the ball top reaching the base top (Z=13),
# since the rear caster is directly under the battery
CASTER_RAISE = 1.0
BALL_Z = BALL_D / 2 + CASTER_RAISE   # ball center under load
CASTER_LIP = 0.5        # retaining lip at the holder's lower opening (per side)
# Holder bottom (ground clearance): the height where the lower opening equals ball dia - 2*CASTER_LIP (about 3.6)
CASTER_Z0 = BALL_Z - math.sqrt((BALL_D / 2 + BALL_CLR) ** 2 - (BALL_D / 2 - CASTER_LIP) ** 2)
CASTER_CHAMFER = 1.0    # chamfer on the outer edge of the holder bottom (floor side); marks which side is which
CASTER_R = 13.0         # holder radius
CASTER_SCREW_Y = 9.5    # holder screw position (+-Y)
# The ball bears on the edge of the hole in the base. The edge height is chosen first, and the
# hole diameter is the ball's cross-section diameter at that height
CASTER_SEAT_Z = BASE_Z + 0.5
CASTER_SEAT_D = 2 * math.sqrt((BALL_D / 2) ** 2 - (CASTER_SEAT_Z - BALL_Z) ** 2)
M3_CSK_D = 6.4          # countersink diameter for an M3 flat head (head 6 dia, 90 deg)
POST_D = 8.0            # mid -> top deck posts
BASE_POST_D = 8.0       # base -> mid deck posts
POST_FLARE = 3.0        # conical reinforcement at the post root (radial and height)
SELF_TAP_M3 = 2.6       # M3 self-tapping pilot hole
M3_CLR = 3.4
BASE_POSTS = [(sx * 40.0, sy * 42.0) for sx in (1, -1) for sy in (1, -1)]
MID_POSTS = [(sx * 50.0, sy * 25.0) for sx in (1, -1) for sy in (1, -1)]
# Battery: lying at the rear (cells along Y). Leaves a gap for the cable ties between its front and the motor cradles
BAT_X_FRONT = -10.5     # battery front X
BAT_Y0 = -32.2          # battery -Y end face (the snap is on the +Y side)
BAT_CLR = 0.5
TRAY_WALL = 1.5
TRAY_H = 8.0            # height of the -Y end wall
# Lightening holes in the base (shorter print time). (center X, center Y, length X, length Y, corner R),
# mirrored in Y. Keep at least 3mm from surrounding features (motor cradles, cable holes, caster
# holders, post flares, battery strap slots)
BASE_LIGHTEN = [
    (21.0, 21.0, 18.0, 22.0, 4.0),     # front (between the motor cradle and the cable hole)
    (49.0, 25.0, 12.0, 12.0, 6.0),     # front outer (between the post and the caster)
    (-21.5, 13.0, 15.0, 17.0, 4.0),    # under the battery (front)
    (-44.5, 21.0, 17.0, 8.0, 3.0),     # under the battery (rear corner)
]


def box(lx, ly, lz, cx=0, cy=0, z0=0):
    return Part.makeBox(lx, ly, lz, V(cx - lx / 2, cy - ly / 2, z0))


def cyl(r, h, x=0, y=0, z=0, d=V(0, 0, 1)):
    return Part.makeCylinder(r, h, V(x, y, z), d)


def rounded_slot(length, width, h, cx, cy, z0, along_x=True):
    r = width / 2
    s = (length - width) / 2
    if along_x:
        sh = box(length - width, width, h, cx, cy, z0)
        sh = sh.fuse([cyl(r, h, cx - s, cy, z0), cyl(r, h, cx + s, cy, z0)])
    else:
        sh = box(width, length - width, h, cx, cy, z0)
        sh = sh.fuse([cyl(r, h, cx, cy - s, z0), cyl(r, h, cx, cy + s, z0)])
    return sh.removeSplitter()


def rounded_rect(lx, ly, r, h, cx, cy, z0):
    # rectangular prism with corners rounded to radius r
    sh = cyl(r, h, cx - lx / 2 + r, cy - ly / 2 + r, z0).fuse([
        cyl(r, h, cx + lx / 2 - r, cy - ly / 2 + r, z0),
        cyl(r, h, cx - lx / 2 + r, cy + ly / 2 - r, z0),
        cyl(r, h, cx + lx / 2 - r, cy + ly / 2 - r, z0)])
    if lx > 2 * r:
        sh = sh.fuse(box(lx - 2 * r, ly, h, cx, cy, z0))
    if ly > 2 * r:
        sh = sh.fuse(box(lx, ly - 2 * r, h, cx, cy, z0))
    return sh.removeSplitter()


def cable_notch(x_in, z0):
    # notch open from the rear edge to X=x_in (inner end rounded)
    x_out = -BODY_D / 2 - 5
    return rounded_slot(x_in - x_out, CABLE_NOTCH_W, PLATE_T + 2, (x_in + x_out) / 2, 0, z0 - 1)


def disk(z0):
    return cyl(BODY_D / 2, PLATE_T, 0, 0, z0)


def flared_post(d, h, x, y, z1):
    # post with a conical flare at the root (harder to snap).
    # Where the flare reaches the outer edge (R65), it is trimmed 1mm inside the edge
    r = d / 2
    flare = Part.makeCone(r + POST_FLARE, r, POST_FLARE, V(x, y, z1 - 0.01)).common(
        cyl(BODY_D / 2 - 1, POST_FLARE + 1, 0, 0, z1 - 0.5))
    return cyl(r, h, x, y, z1).fuse(flare)


# ================================================================ Base plate
def make_base():
    z0, z1 = BASE_Z, BASE_Z + PLATE_T
    p = disk(z0)
    adds, cuts = [], []

    # --- Motor mounts (mirrored left/right)
    for s in (1, -1):
        # The motor drops in from above: gearbox into the pocket, can into the saddle.
        # There is no wall on the shaft side (so the shaft goes deeper into the wheel)
        gb_len = GB_L + GB_CLR                  # pocket length (Y)
        y_gb_rear = GB_FRONT - gb_len            # gearbox rear end = can saddle front end
        # Gearbox pocket (open top, side walls GB_POCKET_WALL)
        pw = GB_W + GB_CLR + 2 * GB_POCKET_WALL
        adds.append(box(pw, gb_len, GB_POCKET_TOP - z1, 0,
                        s * (GB_FRONT - gb_len / 2), z1))
        cuts.append(box(GB_W + GB_CLR, gb_len + 0.02, 20, 0,
                        s * (GB_FRONT - gb_len / 2), AXLE_Z - GB_H / 2))
        # Can saddle (can 12.2 dia +0.4, floor at the can's flat bottom +0.2). Its front face
        # catches the lower edge of the gearbox and stops it from moving back
        y_c = s * (y_gb_rear - CAN_L / 2)
        adds.append(box(16, CAN_L, AXLE_Z - z1, 0, y_c, z1))
        cuts.append(cyl((CAN_D + 0.4) / 2, CAN_L + 2, 0,
                        s * (GB_FRONT - GB_L), AXLE_Z, V(0, -s, 0)).common(
            box(CAN_D + 2, 2 * (CAN_L + 2), CAN_H + 0.4, 0, s * (GB_FRONT - GB_L), AXLE_Z - CAN_H / 2 - 0.2)))
        # Cable-tie slots (3 x 1.8)
        for d in TIE_SLOT_OFFSETS:
            ty = y_gb_rear - d
            cuts.append(box(16 + 2, 3, 1.8, 0, s * ty, AXLE_Z - 9))
        # Wheel clearance (through the plate)
        cuts.append(box(56, WHEEL_W + 5, PLATE_T + 2, 0, s * WHEEL_Y, z0 - 1))

    # --- Battery holder (rear)
    bx1 = BAT_X_FRONT
    bx0 = bx1 - BAT_W
    bcx = (bx0 + bx1) / 2
    by0, by1 = BAT_Y0, BAT_Y0 + BAT_L
    # Rear wall (low; the battery is lifted and slid out rearward with the mid deck attached)
    adds.append(box(TRAY_WALL, 50, BAT_REAR_WALL_H,
                    bx0 - BAT_CLR - TRAY_WALL / 2, (by0 + by1) / 2, z1))
    # -Y end wall (ends only; the strap passes through the middle)
    wy = by0 - BAT_CLR - TRAY_WALL / 2
    for (xa, xb) in ((bx1 - 5.5, bx1), (bx0 + 1.5, bx0 + 6.5)):
        adds.append(box(xb - xa, TRAY_WALL, TRAY_H, (xa + xb) / 2, wy, z1))
    # Front stopper (between the left and right motor cradles)
    adds.append(box(1.5, 8, 6, bx1 + BAT_CLR + 0.75, 0, z1))
    # Hook-and-loop strap slots (the strap wraps the battery along Y). The +Y one is past the snap;
    # the -Y one sits right under the end wall to stay clear of the post
    for sy in (by0 - BAT_CLR - TRAY_WALL - 0.5, by1 + BAT_SNAP + 1.5):
        cuts.append(rounded_slot(20, 3, PLATE_T + 2, bcx, sy, z0 - 1))

    # --- Posts (base -> mid deck)
    for (x, y) in BASE_POSTS:
        adds.append(flared_post(BASE_POST_D, MID_Z - z1, x, y, z1))
        cuts.append(cyl(SELF_TAP_M3 / 2, 12, x, y, MID_Z - 11))

    # --- Casters: ball seat + countersunk screw holes for the holder (flat bottom)
    for cx in (CASTER_X, -CASTER_X):
        cuts.append(Part.makeSphere(BALL_D / 2, V(cx, 0, BALL_Z)))
        cuts.append(cyl(CASTER_SEAT_D / 2, PLATE_T + 2, cx, 0, z0 - 1))
        for sy in (1, -1):
            cuts.append(cyl(M3_CLR / 2, PLATE_T + 2, cx, sy * CASTER_SCREW_Y, z0 - 1))
            hd = (M3_CSK_D - M3_CLR) / 2
            cuts.append(Part.makeCone(M3_CLR / 2, M3_CSK_D / 2 + 0.5, hd + 0.5,
                                      V(cx, sy * CASTER_SCREW_Y, z1 - hd)))

    # --- Cable hole / lightening
    cuts.append(rounded_slot(24, 10, PLATE_T + 2, 22, 0, z0 - 1))
    for (lcx, lcy, llx, lly, lr) in BASE_LIGHTEN:
        for sy in (1, -1):
            cuts.append(rounded_rect(llx, lly, lr, PLATE_T + 2, lcx, sy * lcy, z0 - 1))

    p = p.fuse(adds)
    p = p.cut(cuts)
    return p.removeSplitter()


# ================================================================ Caster holder (separate part)
def make_caster(cx):
    # Disk pressed against the base bottom. The ball goes in from above and is retained by the
    # lower edge (below its equator). The outer edge on the floor side is chamfered so the two
    # sides can be told apart
    z1 = BASE_Z
    c = CASTER_CHAMFER
    h = cyl(CASTER_R, z1 - CASTER_Z0 - c, cx, 0, CASTER_Z0 + c).fuse(
        Part.makeCone(CASTER_R - c, CASTER_R, c, V(cx, 0, CASTER_Z0)))
    rb = BALL_D / 2 + BALL_CLR
    cuts = [Part.makeSphere(rb, V(cx, 0, BALL_Z)),
            cyl(rb, z1 - BALL_Z + 1, cx, 0, BALL_Z)]
    for sy in (1, -1):
        cuts.append(cyl(SELF_TAP_M3 / 2, z1 - CASTER_Z0 + 2, cx, sy * CASTER_SCREW_Y, CASTER_Z0 - 1))
    return h.cut(cuts).removeSplitter()


# ================================================================ Mid deck (PCB)
def make_mid():
    z0, z1 = MID_Z, MID_Z + PLATE_T
    p = disk(z0)
    adds, cuts = [], []
    # Wheel clearance (open to the outer edge)
    for s in (1, -1):
        cuts.append(box(70, 40, PLATE_T + 2, 0, s * (WHEEL_Y + 13), z0 - 1))
    # Holes onto the base posts (M3, no counterbore)
    for (x, y) in BASE_POSTS:
        cuts.append(cyl(M3_CLR / 2, PLATE_T + 2, x, y, z0 - 1))
    # PCB standoffs
    h = PCB_HOLE_PITCH / 2
    for x in (h, -h):
        for y in (h, -h):
            adds.append(cyl(3.5, PCB_STANDOFF, x, y, z1))
            cuts.append(cyl(SELF_TAP_M3 / 2, PCB_STANDOFF + PLATE_T + 2, x, y, z0 - 1))
    # Posts to the top deck
    for (x, y) in MID_POSTS:
        adds.append(flared_post(POST_D, TOP_Z - z1, x, y, z1))
        cuts.append(cyl(SELF_TAP_M3 / 2, 12, x, y, TOP_Z - 11))
    # Cable holes (motor wires left/right, center); battery wires go through the rear notch
    for s in (1, -1):
        cuts.append(rounded_slot(20, 8, PLATE_T + 2, 0, s * 38, z0 - 1))
    cuts.append(cable_notch(MID_NOTCH_X, z0))
    cuts.append(rounded_slot(24, 8, PLATE_T + 2, 40, 0, z0 - 1, along_x=False))
    p = p.fuse(adds).cut(cuts)
    return p.removeSplitter()


# ================================================================ Top deck (LiDAR)
def make_top():
    z0, z1 = TOP_Z, TOP_Z + PLATE_T
    p = disk(z0)
    cuts = []
    for (x, y) in MID_POSTS:
        cuts.append(cyl(M3_CLR / 2, PLATE_T + 2, x, y, z0 - 1))
    # LiDAR leg pads + through holes (screwed from below)
    p = p.fuse([cyl(LIDAR_PAD_D / 2, LIDAR_PAD_H, x, y, z1) for (x, y) in LIDAR_HOLES])
    for (x, y) in LIDAR_HOLES:
        cuts.append(cyl(LIDAR_HOLE_D / 2, PLATE_T + LIDAR_PAD_H + 2, x, y, z0 - 1))
    # Cable notch (rear) / center cable and lightening hole
    cuts.append(cable_notch(TOP_NOTCH_X, z0))
    cuts.append(cyl(14, PLATE_T + 2, 0, 0, z0 - 1))
    p = p.cut(cuts)
    return p.removeSplitter()


def d_bore(clr, h, z0):
    # Bore for a D-cut shaft (3 dia, flat 2.5). clr is added to both the diameter and the flat. Flat on +Y
    br = (SHAFT_D + clr) / 2
    flat = SHAFT_FLAT + clr
    return cyl(br, h, 0, 0, z0).common(box(10, 10, h, 0, -5 + (flat - br), z0))


# ================================================================ Wheel (with integrated hub, replaces the FS90R-W wheel)
def make_wheel():
    # Built at the origin (Z+ is outward). Z=0..HUB_L is the hub, the wheel is outside it
    z0 = HUB_L
    h = cyl(HUB_D / 2, HUB_L)
    w = cyl(WHEEL_RIM_D / 2, WHEEL_W, 0, 0, z0).fuse(
        cyl(WHEEL_BEAD_D / 2, WHEEL_BEAD_W, 0, 0, z0 + (WHEEL_W - WHEEL_BEAD_W) / 2))
    win = cyl(WHEEL_RIM_IN_D / 2, WHEEL_W + 2, 0, 0, z0 - 1).cut(
        cyl(WHEEL_BOSS_D / 2, WHEEL_W + 4, 0, 0, z0 - 2))
    for k in range(WHEEL_SPOKES // 2):
        bar = box(WHEEL_RIM_D, WHEEL_SPOKE_W, WHEEL_W + 4, 0, 0, z0 - 2)
        bar.rotate(V(0, 0, 0), V(0, 0, 1), 30 + k * 360 / WHEEL_SPOKES)
        win = win.cut(bar)
    w = h.fuse(w.cut(win))
    # D-cut bore through the full length
    w = w.cut(d_bore(BORE_CLR, 20, -1))
    # Relief for the gearbox bearing boss (0.5 clearance at its tip)
    w = w.cut(cyl(GB_BOSS_D / 2 + 0.5, GB_BOSS_L - WHEEL_GAP + 0.5 + 1, 0, 0, -1))
    return w.removeSplitter()


def place_wheel(wheel, s):
    sh = wheel.copy()
    sh.rotate(V(0, 0, 0), V(1, 0, 0), -90 * s)   # Z+ -> outward (s*Y)
    sh.translate(V(0, s * (GB_FRONT + WHEEL_GAP), AXLE_Z))
    return sh


# ================================================================ Reference parts (placeholder shapes)
def ref_parts():
    refs = {}
    for s, n in ((1, "L"), (-1, "R")):
        gb = box(GB_W, GB_L, GB_H, 0, s * (GB_FRONT - GB_L / 2), AXLE_Z - GB_H / 2)
        can = cyl(CAN_D / 2, CAN_L, 0, s * (GB_FRONT - GB_L), AXLE_Z, V(0, -s, 0)).common(
            box(CAN_D, 2 * CAN_L, CAN_H, 0, s * (GB_FRONT - GB_L), AXLE_Z - CAN_H / 2))
        sha = cyl(SHAFT_D / 2, SHAFT_L + 0.7, 0, s * GB_FRONT, AXLE_Z, V(0, s, 0))
        refs["Motor_" + n] = gb.fuse([can, sha])
    pcb_z = MID_Z + PLATE_T + PCB_STANDOFF
    refs["PCB"] = box(PCB_SIZE, PCB_SIZE, PCB_T, 0, 0, pcb_z)
    bcx = BAT_X_FRONT - BAT_W / 2
    bat = box(BAT_W, BAT_L, BAT_H, bcx, BAT_Y0 + BAT_L / 2, BASE_Z + PLATE_T)
    snap = box(20, BAT_SNAP, 8, bcx, BAT_Y0 + BAT_L + BAT_SNAP / 2, BASE_Z + PLATE_T + 6)
    refs["Battery_AAx6"] = bat.fuse(snap)
    # LiDAR (rough shape from FIG1/FIG2): 3 legs / base plate / rotating head / bottom motor
    lz = TOP_Z + PLATE_T + LIDAR_PAD_H          # leg bottom
    legs = [cyl(2.5, 17, x, y, lz) for (x, y) in LIDAR_HOLES]
    plate = cyl(LIDAR_HEAD_D / 2, 2, 0, 0, lz + 17).fuse(
        [box(50, 46, 2, 25, 0, lz + 17), cyl(5, 2, -35.3, 0, lz + 17)])
    head = cyl(LIDAR_HEAD_D / 2, LIDAR_H - 19, 0, 0, lz + 19)
    lmotor = cyl(7, 17 - LIDAR_MOTOR_GAP, 44, 0, lz + LIDAR_MOTOR_GAP)
    refs["LiDAR_X2"] = head.fuse(legs + [plate, lmotor])
    for x in (CASTER_X, -CASTER_X):
        refs["Ball_%d" % x] = Part.makeSphere(BALL_D / 2, V(x, 0, BALL_Z))
    return refs


# ================================================================ Generation
def main():
    doc = App.newDocument("RoverFrame")
    printed = {
        "Base_Plate": make_base(),
        "Mid_Deck": make_mid(),
        "Top_Deck": make_top(),
    }
    wheel = make_wheel()
    printed_assy = dict(printed)
    printed_assy["Wheel_L"] = place_wheel(wheel, 1)
    printed_assy["Wheel_R"] = place_wheel(wheel, -1)
    printed_assy["Caster_F"] = make_caster(CASTER_X)
    printed_assy["Caster_B"] = make_caster(-CASTER_X)

    grp_p = doc.addObject("App::DocumentObjectGroup", "Printed")
    grp_r = doc.addObject("App::DocumentObjectGroup", "Reference")
    for name, shp in printed_assy.items():
        o = doc.addObject("Part::Feature", name)
        o.Shape = shp
        grp_p.addObject(o)
    for name, shp in ref_parts().items():
        o = doc.addObject("Part::Feature", "Ref_" + name)
        o.Shape = shp
        grp_r.addObject(o)
    doc.recompute()

    # Colors (only when the GUI is running)
    try:
        import FreeCADGui  # noqa
    except Exception:
        pass

    # It is a generated file, so delete it before overwriting to keep FreeCAD from writing a .FCBak
    fcstd = os.path.join(OUT_DIR, "frame.FCStd")
    if os.path.exists(fcstd):
        os.remove(fcstd)
    doc.saveAs(fcstd)

    # STL (print orientation: flat face down)
    stl_dir = os.path.join(OUT_DIR, "stl")
    os.makedirs(stl_dir, exist_ok=True)
    for name, shp in printed.items():
        s = shp.copy()
        s.translate(V(0, 0, -s.BoundBox.ZMin))
        Mesh.Mesh(s.tessellate(0.05)).write(os.path.join(stl_dir, name + ".stl"))
    # Wheel: outer face down
    w = wheel.copy()
    w.rotate(V(0, 0, 0), V(1, 0, 0), 180)
    w.translate(V(0, 0, -w.BoundBox.ZMin))
    Mesh.Mesh(w.tessellate(0.02)).write(os.path.join(stl_dir, "Wheel_x2.stl"))
    # Caster holder: bottom (the side the ball sticks out of) down
    c = make_caster(0)
    c.translate(V(0, 0, -c.BoundBox.ZMin))
    Mesh.Mesh(c.tessellate(0.02)).write(os.path.join(stl_dir, "Caster_x2.stl"))

    # Report
    for name, shp in list(printed_assy.items()):
        bb = shp.BoundBox
        print("%-12s valid=%s solids=%d  vol=%.0fmm3  bbox X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]" % (
            name, shp.isValid(), len(shp.Solids), shp.Volume,
            bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax))
    # Interference check (printed parts vs reference parts)
    refs = ref_parts()
    allp = dict(printed_assy)
    for pn, ps in allp.items():
        for rn, rs in refs.items():
            v = ps.common(rs).Volume
            if v > 0.5:
                print("INTERFERENCE %s x %s : %.1f mm3" % (pn, rn, v))
    leg_z = TOP_Z + PLATE_T + LIDAR_PAD_H
    print("TOP_Z=%.1f  laser_z=%.1f  total height=%.1f" % (
        TOP_Z, leg_z + LIDAR_LASER_Z, leg_z + LIDAR_H))


import builtins
if not getattr(builtins, "_ROVER_IMPORT_ONLY", False):
    main()
