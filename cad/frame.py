# -*- coding: utf-8 -*-
"""
ESP32 + LiDAR SLAM ローバー フレーム プロトタイプ (FreeCAD 1.x)

実行:  /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd cad/frame.py
出力:  cad/frame.FCStd, cad/stl/*.stl

座標系: X=前方, Y=左, Z=上, 原点=左右車輪軸の中点の真下 (床面)
単位: mm
"""
import os
import FreeCAD as App
import Part
import Mesh
from FreeCAD import Vector as V

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- 部品寸法
# タイヤ FS90R-W : Φ60 (ホイールΦ56) 幅8
WHEEL_D, WHEEL_W = 60.0, 8.0
# ホイール (配布 STL の形状を寸法化したもの)。ハブと一体で印刷する
WHEEL_RIM_D, WHEEL_RIM_IN_D = 50.0, 47.0     # リム外径 (両端), 内径
WHEEL_BEAD_D, WHEEL_BEAD_W = 56.0, 4.0       # 幅中央のタイヤ受け
WHEEL_BOSS_D, WHEEL_BORE_D = 20.0, 3.3       # 中央の円盤, 中心穴
WHEEL_SPOKE_W, WHEEL_SPOKES = 6.0, 6
HUB_D, HUB_L = 9.0, 3.0                      # ホイール内側のボス
BORE_CLR = 0.3          # Dカット穴の隙間 (径・D面とも)。0.15/0.1 では軸が入らなかった
GB_BOSS_D, GB_BOSS_L = 4.0, 1.0              # ギアボックス前面の軸受けの出っ張り (長さは推定)
# モータ MG12A06E-060310-50 (N20互換)
# ギアボックス 幅(X) 高(Z) 長(Y)。エンコーダ基板が受け台に当たらない向きでは
# 12mm 側が水平になる (テスト印刷で確認)
GB_W, GB_H, GB_L = 12.0, 10.0, 9.0
CAN_D, CAN_L = 12.2, 22.0               # モータ+エンコーダ
CAN_H = 10.0                            # 缶は上下 (Z) が平らで高さ10 (ギアボックスの 10mm 側と同じ向き)
SHAFT_D, SHAFT_FLAT, SHAFT_L = 3.0, 2.5, 10.0
GB_CLR = 0.3                            # ギアボックス ポケットの隙間
GB_POCKET_WALL = 2.0                    # ポケット側壁の厚み
GB_POCKET_TOP = 33.0                    # ポケット側壁の上端 Z (ギアボックス上面は36)
# PCB kicad/main (メイン基板) : 60x60, M3穴 50mmピッチ
PCB_SIZE, PCB_HOLE_PITCH, PCB_T = 60.0, 50.0, 1.6
# 電池ボックス BH-363-1B (単3x6 背中合わせ, 006P型スナップ, フタ・スイッチなし)
BAT_L, BAT_W, BAT_H = 58.0, 45.0, 28.0   # L=電池の軸方向 (Y), W=X, H=2段
BAT_SNAP = 2.4                           # スナップの突出 (+Y 端面)
# LiDAR YDLIDAR X2 : 96 x 60.5 x 50.3 (データシート FIG1/FIG2, Marutsu版 YDLIDAR_X2.pdf)
# 回転中心を原点に置き、モータ側を +X (前方) に向ける -> スキャン 0° が前方
LIDAR_L, LIDAR_W, LIDAR_H = 96.0, 60.5, 50.3
LIDAR_HEAD_D = 60.5
LIDAR_HOLES = [(-35.3, 0.0), (36.0, 18.0), (36.0, -18.0)]   # 脚 (スタンドオフ) 3本の位置
LIDAR_HOLE_D = 2.8      # M2.5 想定の貫通穴 (ネジ径はデータシートに記載なし)
LIDAR_PAD_D, LIDAR_PAD_H = 8.0, 2.0   # 脚を受ける座面 (底面モータとの隙間を確保)
LIDAR_MOTOR_GAP = 2.5   # 脚の下端からLiDARモータ下面まで (50.3 - 47.8)
LIDAR_LASER_Z = LIDAR_H - 7.9          # 脚の下端からレーザー中心まで

# ---------------------------------------------------------------- フレーム寸法
BODY_D = 130.0          # 円形デッキ径
PLATE_T = 3.0
BASE_Z = 10.0           # ベース下面の地上高
AXLE_Z = WHEEL_D / 2    # 車軸高さ 30
GB_FRONT = 38.5         # ギアボックス前面の Y (軸はここから約9mm 出る)
WHEEL_GAP = 0.5         # ギアボックス前面とハブの隙間
WHEEL_Y = GB_FRONT + WHEEL_GAP + HUB_L + WHEEL_W / 2   # 車輪中心面の Y (=46)
MID_Z = 43.0            # 中段デッキ下面 (電池上面 41 + 隙間2)
PCB_STANDOFF = 5.0
PCB_CLEAR = 25.0        # PCB上の部品クリアランス
TOP_Z = MID_Z + PLATE_T + PCB_STANDOFF + PCB_T + PCB_CLEAR   # 上段デッキ下面
BALL_D = 12.7           # キャスター用 1/2" 鋼球
CASTER_X = 45.0
POST_D = 8.0
SELF_TAP_M3 = 2.6       # M3 タッピング下穴
M3_CLR = 3.4
BASE_POSTS = [(sx * 40.0, sy * 42.0) for sx in (1, -1) for sy in (1, -1)]
MID_POSTS = [(sx * 50.0, sy * 25.0) for sx in (1, -1) for sy in (1, -1)]
# 電池: 後方に横置き (電池の軸が Y)。前面はモータ受けとの間に結束バンド用の隙間を残す
BAT_X_FRONT = -10.5     # 電池前面 X
BAT_Y0 = -32.2          # 電池の -Y 端面 (スナップは +Y 側)
BAT_CLR = 0.5
TRAY_WALL = 1.5
TRAY_H = 8.0


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
    return sh


def disk(z0):
    return cyl(BODY_D / 2, PLATE_T, 0, 0, z0)


# ================================================================ ベースプレート
def make_base():
    z0, z1 = BASE_Z, BASE_Z + PLATE_T
    p = disk(z0)
    adds, cuts = [], []

    # --- モータ取付 (左右対称)
    for s in (1, -1):
        # モータは上から落とし込む: ポケットにギアボックス、サドルに缶。
        # 軸側の壁は無い (軸をホイールに長く差し込むため)
        gb_len = GB_L + GB_CLR                  # ポケット長 (Y)
        y_gb_rear = GB_FRONT - gb_len            # ギアボックス後端 = 缶サドル前端
        # ギアボックス ポケット (上面開放, 側壁 GB_POCKET_WALL)
        pw = GB_W + GB_CLR + 2 * GB_POCKET_WALL
        adds.append(box(pw, gb_len, GB_POCKET_TOP - z1, 0,
                        s * (GB_FRONT - gb_len / 2), z1))
        cuts.append(box(GB_W + GB_CLR, gb_len + 0.02, 20, 0,
                        s * (GB_FRONT - gb_len / 2), AXLE_Z - GB_H / 2))
        # 缶サドル (缶Φ12.2 +0.4、底は缶の平らな下面 +0.2)。前端面がギアボックス下側の角を受けて後退を止める
        y_c = s * (y_gb_rear - CAN_L / 2)
        adds.append(box(16, CAN_L, AXLE_Z - z1, 0, y_c, z1))
        cuts.append(cyl((CAN_D + 0.4) / 2, CAN_L + 2, 0,
                        s * (GB_FRONT - GB_L), AXLE_Z, V(0, -s, 0)).common(
            box(CAN_D + 2, 2 * (CAN_L + 2), CAN_H + 0.4, 0, s * (GB_FRONT - GB_L), AXLE_Z - CAN_H / 2 - 0.2)))
        # 結束バンド用スロット (3 x 1.8)
        for ty in (y_gb_rear - 6, y_gb_rear - 16):
            cuts.append(box(16 + 2, 3, 1.8, 0, s * ty, AXLE_Z - 9))
        # 車輪逃げ (プレート貫通)
        cuts.append(box(56, WHEEL_W + 5, PLATE_T + 2, 0, s * WHEEL_Y, z0 - 1))

    # --- 電池ホルダー (後方)
    bx1 = BAT_X_FRONT
    bx0 = bx1 - BAT_W
    bcx = (bx0 + bx1) / 2
    by0, by1 = BAT_Y0, BAT_Y0 + BAT_L
    # 後ろ壁
    adds.append(box(TRAY_WALL, 50, TRAY_H,
                    bx0 - BAT_CLR - TRAY_WALL / 2, (by0 + by1) / 2, z1))
    # -Y 端の壁 (両端のみ。中央はバンドを通す)
    wy = by0 - BAT_CLR - TRAY_WALL / 2
    for (xa, xb) in ((bx1 - 5.5, bx1), (bx0 + 1.5, bx0 + 6.5)):
        adds.append(box(xb - xa, TRAY_WALL, TRAY_H, (xa + xb) / 2, wy, z1))
    # 前ストッパ (左右のモータ受けの間)
    adds.append(box(1.5, 8, 6, bx1 + BAT_CLR + 0.75, 0, z1))
    # 面ファスナーバンド用スロット (Y方向に電池を巻く)。+Y 側はスナップの先
    for sy in (by0 - BAT_CLR - TRAY_WALL - 1.5, by1 + BAT_SNAP + 1.5):
        cuts.append(rounded_slot(20, 3, PLATE_T + 2, bcx, sy, z0 - 1))

    # --- 支柱 (ベース→中段)
    for (x, y) in BASE_POSTS:
        adds.append(cyl(POST_D / 2, MID_Z - z1, x, y, z1))
        cuts.append(cyl(SELF_TAP_M3 / 2, 12, x, y, MID_Z - 11))

    # --- キャスター (前後, 1/2"鋼球スナップ式)
    for cx in (CASTER_X, -CASTER_X):
        ball_c = V(cx, 0, BALL_D / 2)
        adds.append(cyl(10, z0 - 4.0 + 0.01, cx, 0, 4.0))
        cuts.append(Part.makeSphere(BALL_D / 2 + 0.2, ball_c))
        cuts.append(cyl(BALL_D / 2 + 0.2, 6, cx, 0, BALL_D / 2))  # 上方向の抜き
        # 割り (スナップ用スリット)
        cuts.append(box(1.2, 22, 4.5, cx, 0, 3.9))

    # --- 配線穴 / 軽量化
    cuts.append(rounded_slot(24, 10, PLATE_T + 2, 22, 0, z0 - 1))

    p = p.fuse(adds)
    p = p.cut(cuts)
    return p.removeSplitter()


# ================================================================ 中段デッキ (PCB)
def make_mid():
    z0, z1 = MID_Z, MID_Z + PLATE_T
    p = disk(z0)
    adds, cuts = [], []
    # 車輪の逃げ (外周まで開放)
    for s in (1, -1):
        cuts.append(box(70, 40, PLATE_T + 2, 0, s * (WHEEL_Y + 13), z0 - 1))
    # ベース支柱への固定穴 (M3, 座ぐり無し)
    for (x, y) in BASE_POSTS:
        cuts.append(cyl(M3_CLR / 2, PLATE_T + 2, x, y, z0 - 1))
    # PCBスタンドオフ
    h = PCB_HOLE_PITCH / 2
    for x in (h, -h):
        for y in (h, -h):
            adds.append(cyl(3.5, PCB_STANDOFF, x, y, z1))
            cuts.append(cyl(SELF_TAP_M3 / 2, PCB_STANDOFF + PLATE_T + 2, x, y, z0 - 1))
    # 上段への支柱
    for (x, y) in MID_POSTS:
        adds.append(cyl(POST_D / 2, TOP_Z - z1, x, y, z1))
        cuts.append(cyl(SELF_TAP_M3 / 2, 12, x, y, TOP_Z - 11))
    # 配線穴 (モータ線 左右 / 電池線 後方 / 中央)
    for s in (1, -1):
        cuts.append(rounded_slot(20, 8, PLATE_T + 2, 0, s * 38, z0 - 1))
    cuts.append(rounded_slot(24, 8, PLATE_T + 2, -40, 0, z0 - 1, along_x=False))
    cuts.append(rounded_slot(24, 8, PLATE_T + 2, 40, 0, z0 - 1, along_x=False))
    p = p.fuse(adds).cut(cuts)
    return p.removeSplitter()


# ================================================================ 上段デッキ (LiDAR)
def make_top():
    z0, z1 = TOP_Z, TOP_Z + PLATE_T
    p = disk(z0)
    cuts = []
    for (x, y) in MID_POSTS:
        cuts.append(cyl(M3_CLR / 2, PLATE_T + 2, x, y, z0 - 1))
    # LiDAR 脚の座面 + 貫通穴 (下からネジ止め)
    p = p.fuse([cyl(LIDAR_PAD_D / 2, LIDAR_PAD_H, x, y, z1) for (x, y) in LIDAR_HOLES])
    for (x, y) in LIDAR_HOLES:
        cuts.append(cyl(LIDAR_HOLE_D / 2, PLATE_T + LIDAR_PAD_H + 2, x, y, z0 - 1))
    # ケーブル穴 (後方) / 中央の配線・軽量化穴
    cuts.append(rounded_slot(24, 8, PLATE_T + 2, -52, 0, z0 - 1, along_x=False))
    cuts.append(cyl(14, PLATE_T + 2, 0, 0, z0 - 1))
    p = p.cut(cuts)
    return p.removeSplitter()


# ================================================================ ホイール (ハブ一体, FS90R-W のホイール部)
def make_wheel():
    # 原点基準で作成 (Z+ が外側)。Z=0〜HUB_L がハブ、その外側がホイール
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
    # Dカット穴 (軸Φ3・D面2.5 に BORE_CLR を足す) を全長に通す
    br = (SHAFT_D + BORE_CLR) / 2
    flat = SHAFT_FLAT + BORE_CLR
    bore = cyl(br, 20, 0, 0, -1).common(box(10, 10, 20, 0, -5 + (flat - br), -1))
    w = w.cut(bore)
    # ギアボックスの軸受けの出っ張りの逃げ (先端との隙間 0.5)
    w = w.cut(cyl(GB_BOSS_D / 2 + 0.5, GB_BOSS_L - WHEEL_GAP + 0.5 + 1, 0, 0, -1))
    return w.removeSplitter()


def place_wheel(wheel, s):
    sh = wheel.copy()
    sh.rotate(V(0, 0, 0), V(1, 0, 0), -90 * s)   # Z+ → 外側 (s*Y)
    sh.translate(V(0, s * (GB_FRONT + WHEEL_GAP), AXLE_Z))
    return sh


# ================================================================ 参照部品 (ダミー形状)
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
    # LiDAR (FIG1/FIG2 からの概形): 脚3本 / ベース板 / 回転ヘッド / 底面モータ
    lz = TOP_Z + PLATE_T + LIDAR_PAD_H          # 脚の下端
    legs = [cyl(2.5, 17, x, y, lz) for (x, y) in LIDAR_HOLES]
    plate = cyl(LIDAR_HEAD_D / 2, 2, 0, 0, lz + 17).fuse(
        [box(50, 46, 2, 25, 0, lz + 17), cyl(5, 2, -35.3, 0, lz + 17)])
    head = cyl(LIDAR_HEAD_D / 2, LIDAR_H - 19, 0, 0, lz + 19)
    lmotor = cyl(7, 17 - LIDAR_MOTOR_GAP, 44, 0, lz + LIDAR_MOTOR_GAP)
    refs["LiDAR_X2"] = head.fuse(legs + [plate, lmotor])
    for x in (CASTER_X, -CASTER_X):
        refs["Ball_%d" % x] = Part.makeSphere(BALL_D / 2, V(x, 0, BALL_D / 2))
    return refs


# ================================================================ 生成
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

    # 色 (GUI 起動時のみ有効)
    try:
        import FreeCADGui  # noqa
    except Exception:
        pass

    # 生成物なので上書き前に消し、FreeCAD に .FCBak を作らせない
    fcstd = os.path.join(OUT_DIR, "frame.FCStd")
    if os.path.exists(fcstd):
        os.remove(fcstd)
    doc.saveAs(fcstd)

    # STL (印刷向き: 平らな面を下)
    stl_dir = os.path.join(OUT_DIR, "stl")
    os.makedirs(stl_dir, exist_ok=True)
    for name, shp in printed.items():
        s = shp.copy()
        s.translate(V(0, 0, -s.BoundBox.ZMin))
        Mesh.Mesh(s.tessellate(0.05)).write(os.path.join(stl_dir, name + ".stl"))
    # ホイール: 外側の面を下にする
    w = wheel.copy()
    w.rotate(V(0, 0, 0), V(1, 0, 0), 180)
    w.translate(V(0, 0, -w.BoundBox.ZMin))
    Mesh.Mesh(w.tessellate(0.02)).write(os.path.join(stl_dir, "Wheel_x2.stl"))

    # レポート
    for name, shp in list(printed_assy.items()):
        bb = shp.BoundBox
        print("%-12s valid=%s solids=%d  vol=%.0fmm3  bbox X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]" % (
            name, shp.isValid(), len(shp.Solids), shp.Volume,
            bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax))
    # 干渉チェック (印刷部品 vs 参照部品)
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
