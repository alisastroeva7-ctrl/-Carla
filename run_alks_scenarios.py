# run_alks_scenarios.py  —  ALKS §4.1–§4.6  (без ScenarioRunner, без встроенных карт)
# Запуск: C:\Users\popes\AppData\Local\Programs\Python\Python37\python.exe D:\files\run_alks_scenarios.py

import sys, os, subprocess, time, queue, math, threading, socket, random
import numpy as np

EGG       = r"C:\Carla simulator 0.14.0\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.14-py3.7-win-amd64.egg"
CARLA_API = r"C:\Carla simulator 0.14.0\WindowsNoEditor\PythonAPI"
CARLA_EXE = r"C:\Carla simulator 0.14.0\WindowsNoEditor\CarlaUE4.exe"
FILES_DIR = r"D:\files"
OUT_DIR   = r"D:\files\output"
XODR_FILE = os.path.join(FILES_DIR, "alks_highway.xodr")

for p in [EGG, os.path.join(CARLA_API, "carla"), CARLA_API]:
    if p not in sys.path:
        sys.path.insert(0, p)

import carla
import cv2
import pygame

# ══ ПАРАМЕТРЫ ════════════════════════════════════════════════════
VIDEO_W, VIDEO_H = 1280, 720
VIDEO_FPS        = 20
LIDAR_W, LIDAR_H = 800, 800
LIDAR_SCALE      = 3.5
EGO_SPEED        = 27.78   # 100 км/ч
BRAKE_DIST       = 60.0
STOP_DIST        = 6.0

# ══ СЦЕНАРИИ ═════════════════════════════════════════════════════
SCENARIOS = [
    ("4_1",    "ALKS §4.1  Удержание полосы",                 90),
    ("4_2a",   "ALKS §4.2a Неподвижный авто",                 30),
    ("4_2b",   "ALKS §4.2b Неподвижный мотоцикл PTW",         30),
    ("4_2c",   "ALKS §4.2c Неподвижный пешеход",              30),
    ("4_2d",   "ALKS §4.2d Пешеход пересекает полосу",        35),
    ("4_2e",   "ALKS §4.2e Заблокированная полоса",           30),
    ("4_2f",   "ALKS §4.2f Препятствие частично в полосе",    30),
    ("4_2g",   "ALKS §4.2g Последовательные препятствия",     35),
    ("4_2h",   "ALKS §4.2h Препятствие на дуге",              45),
    ("4_3a",   "ALKS §4.3a Следование — переменная скорость", 60),
    ("4_3b",   "ALKS §4.3b Следование — лидер PTW",           50),
    ("4_3f",   "ALKS §4.3f Экстренное торможение 6 м/с²",     35),
    ("4_4a",   "ALKS §4.4a Cut-in — авто",                    30),
    ("4_4d",   "ALKS §4.4d Cut-in — PTW",                     30),
    ("4_5a",   "ALKS §4.5a Cutout — лидер уходит",            40),
]

# ══ МЕНЮ ═════════════════════════════════════════════════════════
def show_menu():
    print("\n" + "="*64)
    print("  ALKS — ВЫБЕРИТЕ СЦЕНАРИЙ")
    print("="*64)
    print("  {:2d}  — Запустить ВСЕ сценарии".format(0))
    for i, (key, name, dur) in enumerate(SCENARIOS, 1):
        print("  {:2d}  [{}] {}  ({} s)".format(i, key, name, dur))
    print("="*64)
    while True:
        try:
            n = int(input("Номер (0–{}): ".format(len(SCENARIOS))).strip())
            if 0 <= n <= len(SCENARIOS):
                return n
        except (ValueError, EOFError):
            pass

# ══ CARLA ════════════════════════════════════════════════════════
def port_open():
    try:
        s = socket.create_connection(("127.0.0.1", 2000), timeout=1)
        s.close(); return True
    except: return False

def ensure_carla():
    if port_open():
        print("CARLA: port OK"); return
    print("Zapuskayu CARLA...", end="", flush=True)
    subprocess.Popen(
        [CARLA_EXE, "-carla-rpc-port=2000",
         "-quality-level=Epic",
         "-windowed", "-ResX=1280", "-ResY=720"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    while not port_open():
        print(".", end="", flush=True); time.sleep(1)
    print(" OK")
    time.sleep(12)

def connect():
    c = carla.Client("127.0.0.1", 2000)
    c.set_timeout(60.0)
    w = c.get_world()
    print("Podklyucheno: " + w.get_map().name)
    return c, w

def load_map(client):
    print("Gruzhu kartu...", flush=True)
    with open(XODR_FILE, "r", encoding="utf-8") as f:
        xodr = f.read()
    client.generate_opendrive_world(xodr,
        carla.OpendriveGenerationParameters(
            vertex_distance=0.3,
            max_road_length=500.0,
            wall_height=0.0,
            additional_width=0.5,
            smooth_junctions=True,
            enable_mesh_visibility=True))
    time.sleep(12)
    c2 = carla.Client("127.0.0.1", 2000)
    c2.set_timeout(60.0)
    w2 = c2.get_world()
    print("Karta: " + w2.get_map().name)
    w2.set_weather(carla.WeatherParameters(
        cloudiness=5.0, precipitation=0.0, precipitation_deposits=0.0,
        wind_intensity=0.0, sun_azimuth_angle=230.0, sun_altitude_angle=75.0,
        fog_density=0.0, wetness=0.0))
    return c2, w2

# ══ SPAWN ════════════════════════════════════════════════════════
def spawn_at(world, bp_lib, bp_name, road_id, lane_id, s, offset=0.0,
             color=None, role="simulation"):
    wp = world.get_map().get_waypoint_xodr(road_id, lane_id, float(s))
    if wp is None:
        print("[WARN] wp not found road={} lane={} s={}".format(road_id, lane_id, s))
        return None
    tf = wp.transform
    tf.location.z += 0.3
    if abs(offset) > 0.01:
        r = tf.get_right_vector()
        tf.location.x -= r.x * offset
        tf.location.y -= r.y * offset
    bp = bp_lib.find(bp_name)
    if bp is None: return None
    bp.set_attribute("role_name", role)
    if color and bp.has_attribute("color"):
        try: bp.set_attribute("color", color)
        except: pass
    a = world.try_spawn_actor(bp, tf)
    if a is None:
        tf.location.z += 0.5
        a = world.try_spawn_actor(bp, tf)
    return a

def spawn_ped(world, bp_lib, road_id, lane_id, s, offset=0.0):
    bps = list(bp_lib.filter("walker.pedestrian.*"))
    if not bps: return None
    wp = world.get_map().get_waypoint_xodr(road_id, lane_id, float(s))
    if wp is None: return None
    tf = wp.transform
    tf.location.z += 1.0
    if abs(offset) > 0.01:
        r = tf.get_right_vector()
        tf.location.x -= r.x * offset
        tf.location.y -= r.y * offset
    return world.try_spawn_actor(bps[0], tf)

def full_stop(actor):
    if actor is None: return
    if "walker" in actor.type_id:
        wc = carla.WalkerControl(); wc.speed = 0.0
        try: actor.apply_control(wc)
        except: pass
    else:
        ctrl = carla.VehicleControl()
        ctrl.brake=1.0; ctrl.throttle=0.0
        ctrl.hand_brake=True; ctrl.reverse=False
        try: actor.apply_control(ctrl)
        except: pass

# ══ ВИДЕО — правильный кодек без артефактов ══════════════════════
def make_writer(path, w, h):
    """
    Используем mp4v в .mp4 — работает во всех Windows плеерах
    без чересстрочных артефактов MJPG.
    """
    # Попытка 1: mp4v (встроен в OpenCV, без артефактов)
    vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), VIDEO_FPS, (w, h))
    if vw.isOpened(): return vw, path
    # Попытка 2: XVID .avi
    p2 = path.replace(".mp4", ".avi")
    vw = cv2.VideoWriter(p2, cv2.VideoWriter_fourcc(*"XVID"), VIDEO_FPS, (w, h))
    return vw, p2

# ══ РАЗМЕТКА ПОЛОС НА КАДР ═══════════════════════════════════════
def draw_lanes_on_frame(frame):
    """
    Рисует разметку поверх кадра фронтальной камеры.
    Камера: x=0.5, z=1.5, pitch=-5, fov=90
    Горизонт ≈ 43% высоты кадра
    """
    H, W = frame.shape[:2]
    hy  = int(H * 0.43)   # горизонт
    by  = H                # низ
    cx  = W // 2           # центр по X

    # Ширина полосы в пикселях (полуширина)
    hw_b = int(W * 0.155)  # внизу
    hw_h = int(W * 0.038)  # на горизонте

    # Границы текущей полосы ego
    ll_b, lr_b = cx - hw_b, cx + hw_b   # low left/right
    ll_h, lr_h = cx - hw_h, cx + hw_h   # horizon left/right

    # Полупрозрачная зелёная заливка текущей полосы
    overlay = frame.copy()
    pts = np.array([[ll_b, by], [lr_b, by], [lr_h, hy], [ll_h, hy]], np.int32)
    cv2.fillPoly(overlay, [pts], (0, 100, 0))
    cv2.addWeighted(overlay, 0.20, frame, 0.80, 0, frame)

    # Жёлтые сплошные линии — границы текущей полосы
    cv2.line(frame, (ll_b, by), (ll_h, hy), (0, 230, 230), 4)
    cv2.line(frame, (lr_b, by), (lr_h, hy), (0, 230, 230), 4)

    # Пунктирные белые линии — соседние полосы (слева и справа)
    ll2_b, ll2_h = ll_b - 2*hw_b, ll_h - 2*hw_h
    lr2_b, lr2_h = lr_b + 2*hw_b, lr_h + 2*hw_h
    SEGS = 10
    for i in range(SEGS):
        if i % 2 == 0:
            t0 = i / SEGS
            t1 = min(1.0, (i + 0.5) / SEGS)
            # Левая соседняя
            x0 = int(ll2_b + (ll2_h - ll2_b) * t0)
            y0 = int(by    + (hy    - by)    * t0)
            x1 = int(ll2_b + (ll2_h - ll2_b) * t1)
            y1 = int(by    + (hy    - by)    * t1)
            cv2.line(frame, (x0, y0), (x1, y1), (200, 200, 200), 2)
            # Правая соседняя
            x0r = int(lr2_b + (lr2_h - lr2_b) * t0)
            x1r = int(lr2_b + (lr2_h - lr2_b) * t1)
            cv2.line(frame, (x0r, y0), (x1r, y1), (200, 200, 200), 2)

    # Белая пунктирная центральная линия
    for i in range(SEGS):
        if i % 2 == 0:
            t0 = i / SEGS
            t1 = min(1.0, (i + 0.5) / SEGS)
            y0 = int(by + (hy - by) * t0)
            y1 = int(by + (hy - by) * t1)
            cv2.line(frame, (cx, y0), (cx, y1), (255, 255, 255), 2)
    return frame

# ══ LIDAR ════════════════════════════════════════════════════════
def draw_lidar(screen, pts, cx, cy, font, name, elapsed, dist_m, braking, spd_kmh):
    screen.fill((8, 12, 24))
    for r_m in [20, 40, 60, 80, 100]:
        r_px = int(r_m * LIDAR_SCALE)
        pygame.draw.circle(screen, (20, 26, 45), (cx, cy), r_px, 1)
        screen.blit(font.render(str(r_m)+"m", True, (40, 45, 70)), (cx+r_px+2, cy-7))
    for p in pts:
        sx = int(cx - float(p[1]) * LIDAR_SCALE)
        sy = int(cy - float(p[0]) * LIDAR_SCALE)
        if 0 <= sx < LIDAR_W and 0 <= sy < LIDAR_H:
            nz = min(1.0, max(0.0, (float(p[2])+3.0)/6.0))
            screen.set_at((sx, sy), (255, int(180+75*nz), int(160*nz)))
    col = (255, 60, 60) if braking else (200, 120, 120)
    pygame.draw.line(screen, col, (cx-14, cy), (cx+14, cy), 2)
    pygame.draw.line(screen, col, (cx, cy-14), (cx, cy+14), 2)
    safe_px = int(BRAKE_DIST * LIDAR_SCALE)
    pygame.draw.line(screen, (200, 180, 0), (cx-40, cy-safe_px), (cx+40, cy-safe_px), 1)
    lines = [
        name[:55],
        "t={}s  v={}km/h".format(elapsed, spd_kmh),
        ("BRAKE  " if braking else "") + ("dist={}m".format(round(dist_m,1)) if dist_m<9999 else ""),
    ]
    for i, line in enumerate(lines):
        col2 = (255, 70, 70) if "BRAKE" in line else (100, 220, 130)
        screen.blit(font.render(line, True, col2), (10, 10 + i*17))

# ══ ЗАПУСК СЦЕНАРИЯ ══════════════════════════════════════════════
def run_scenario(idx, client, world):
    key, name, duration = SCENARIOS[idx]
    print("\n" + "="*64)
    print("  [{}] {}".format(idx+1, name))
    print("  Длительность: {} сек".format(duration))
    print("="*64)

    out_dir   = os.path.join(OUT_DIR, key)
    lidar_dir = os.path.join(out_dir, "lidar")
    os.makedirs(lidar_dir, exist_ok=True)

    bp         = world.get_blueprint_library()
    stop_flag  = [False]
    all_actors = []
    sensors    = []
    vw_f = vw_b = vw_l = None
    pf = pb = pl = ""

    try:
        # ── СПАВН ────────────────────────────────────────────
        ego = None
        if key == "4_1":
            ego  = spawn_at(world,bp,"vehicle.tesla.model3",  1,-2, 50,  role="hero",color="255,0,0")
            car  = spawn_at(world,bp,"vehicle.audi.a2",       1,-2,200,  color="0,100,255")
            ptw  = spawn_at(world,bp,"vehicle.kawasaki.ninja",1,-2,550,  color="255,140,0")
            side = spawn_at(world,bp,"vehicle.audi.a2",       1,-1, 20,  color="180,180,180")
            all_actors = [ego,car,ptw,side]
        elif key == "4_2a":
            ego = spawn_at(world,bp,"vehicle.tesla.model3",1,-2, 50, role="hero",color="255,0,0")
            obs = spawn_at(world,bp,"vehicle.audi.a2",     1,-2,280, color="0,0,200")
            all_actors = [ego,obs]
        elif key == "4_2b":
            ego = spawn_at(world,bp,"vehicle.tesla.model3",   1,-2, 50, role="hero",color="255,0,0")
            obs = spawn_at(world,bp,"vehicle.kawasaki.ninja", 1,-2,280, color="255,140,0")
            all_actors = [ego,obs]
        elif key == "4_2c":
            ego = spawn_at(world,bp,"vehicle.tesla.model3",1,-2, 50, role="hero",color="255,0,0")
            ped = spawn_ped(world,bp,1,-2,280)
            all_actors = [ego,ped]
        elif key == "4_2d":
            ego = spawn_at(world,bp,"vehicle.tesla.model3",1,-2, 50, role="hero",color="255,0,0")
            ped = spawn_ped(world,bp,1,-3,250)
            all_actors = [ego,ped]
        elif key == "4_2e":
            ego = spawn_at(world,bp,"vehicle.tesla.model3",1,-2, 50, role="hero",color="255,0,0")
            obs = spawn_at(world,bp,"vehicle.audi.a2",     1,-2,280, color="255,80,0")
            all_actors = [ego,obs]
        elif key == "4_2f":
            ego = spawn_at(world,bp,"vehicle.tesla.model3",1,-2, 50,     role="hero",color="255,0,0")
            obs = spawn_at(world,bp,"vehicle.audi.a2",     1,-2,280, 1.6, color="200,80,0")
            all_actors = [ego,obs]
        elif key == "4_2g":
            ego = spawn_at(world,bp,"vehicle.tesla.model3",   1,-2, 50, role="hero",color="255,0,0")
            m1  = spawn_at(world,bp,"vehicle.kawasaki.ninja", 1,-2,250, color="255,0,100")
            m2  = spawn_at(world,bp,"vehicle.audi.a2",        1,-2,295, color="0,0,180")
            all_actors = [ego,m1,m2]
        elif key == "4_2h":
            # Ego стартует прямо на дуге (road 2)
            ego = spawn_at(world,bp,"vehicle.tesla.model3",2,-2, 50, role="hero",color="255,0,0")
            obs = spawn_at(world,bp,"vehicle.audi.a2",     2,-2,310, color="0,180,80")
            all_actors = [ego,obs]
        elif key == "4_3a":
            ego  = spawn_at(world,bp,"vehicle.tesla.model3",    1,-2, 50, role="hero",color="255,0,0")
            lead = spawn_at(world,bp,"vehicle.bmw.grandtourer", 1,-2,130, color="0,120,255")
            all_actors = [ego,lead]
        elif key == "4_3b":
            ego  = spawn_at(world,bp,"vehicle.tesla.model3",   1,-2, 50, role="hero",color="255,0,0")
            lead = spawn_at(world,bp,"vehicle.kawasaki.ninja", 1,-2,130, color="255,140,0")
            all_actors = [ego,lead]
        elif key == "4_3f":
            ego  = spawn_at(world,bp,"vehicle.tesla.model3",    1,-2, 50, role="hero",color="255,0,0")
            lead = spawn_at(world,bp,"vehicle.bmw.grandtourer", 1,-2,130, color="0,120,255")
            all_actors = [ego,lead]
        elif key == "4_4a":
            ego = spawn_at(world,bp,"vehicle.tesla.model3",1,-2, 50,  role="hero",color="255,0,0")
            ci  = spawn_at(world,bp,"vehicle.audi.a2",     1,-1,160,  color="0,0,200")
            all_actors = [ego,ci]
        elif key == "4_4d":
            ego = spawn_at(world,bp,"vehicle.tesla.model3",   1,-2, 50,  role="hero",color="255,0,0")
            ci  = spawn_at(world,bp,"vehicle.kawasaki.ninja", 1,-1,160,  color="255,50,150")
            all_actors = [ego,ci]
        elif key == "4_5a":
            ego  = spawn_at(world,bp,"vehicle.tesla.model3",    1,-2, 50,  role="hero",color="255,0,0")
            lead = spawn_at(world,bp,"vehicle.bmw.grandtourer", 1,-2,130,  color="0,120,255")
            obs  = spawn_at(world,bp,"vehicle.audi.a2",         1,-2,210,  color="50,50,50")
            all_actors = [ego,lead,obs]

        all_actors = [a for a in all_actors if a is not None]
        if ego is None:
            print("OSHIBKA: ego ne spawnilas!"); return

        print("Ego id={} type={}".format(ego.id, ego.type_id))

        # ── НАЧАЛЬНЫЕ СКОРОСТИ ────────────────────────────────
        for a in all_actors: full_stop(a)
        time.sleep(0.5)

        # ── ЦЕЛЕВЫЕ СКОРОСТИ ──────────────────────────────────
        target_spd = {ego.id: EGO_SPEED}
        for a in all_actors:
            if a and a.id != ego.id:
                target_spd[a.id] = 0.0

        # Кто едет с начала
        if key == "4_1":
            for a in all_actors[1:]:
                if a: target_spd[a.id] = EGO_SPEED
        elif key in ("4_3a","4_3b","4_3f"):
            if len(all_actors)>1 and all_actors[1]:
                target_spd[all_actors[1].id] = EGO_SPEED
        elif key in ("4_4a","4_4d"):
            if len(all_actors)>1 and all_actors[1]:
                target_spd[all_actors[1].id] = EGO_SPEED
        elif key == "4_5a":
            if len(all_actors)>1 and all_actors[1]:
                target_spd[all_actors[1].id] = EGO_SPEED

        # ── WAYPOINT STEERING ─────────────────────────────────
        _map = world.get_map()

        def get_steer(actor):
            """Удерживает полосу через ближайший waypoint — работает на дуге."""
            try:
                tf  = actor.get_transform()
                wp  = _map.get_waypoint(tf.location,
                          project_to_road=True,
                          lane_type=carla.LaneType.Driving)
                if wp is None: return 0.0
                nwps = wp.next(5.0)
                if not nwps: return 0.0
                nwp = nwps[0]
                dx  = nwp.transform.location.x - tf.location.x
                dy  = nwp.transform.location.y - tf.location.y
                right = tf.get_right_vector()
                cross = dx*right.x + dy*right.y
                return max(-0.8, min(0.8, cross * 0.45))
            except: return 0.0

        # ── CRUISE CONTROL ────────────────────────────────────
        dist_state  = [9999.0]
        brake_state = [False]

        def cruise_thread():
            while not stop_flag[0]:
                try:
                    for a in all_actors:
                        if a is None: continue
                        if "walker" in a.type_id: continue
                        tgt = target_spd.get(a.id, 0.0)
                        v   = a.get_velocity()
                        cur = math.sqrt(v.x**2+v.y**2+v.z**2)
                        st  = get_steer(a)
                        ctrl = carla.VehicleControl()
                        ctrl.hand_brake = False
                        ctrl.reverse    = False
                        ctrl.steer      = st
                        if tgt < 0.2:
                            ctrl.brake=1.0; ctrl.throttle=0.0
                            ctrl.hand_brake=True; ctrl.steer=0.0
                        elif cur < tgt - 0.5:
                            ctrl.throttle = min(0.9,(tgt-cur)/tgt*1.5)
                            ctrl.brake    = 0.0
                        elif cur > tgt + 0.5:
                            ctrl.throttle = 0.0
                            ctrl.brake    = min(0.8,(cur-tgt)/tgt*2)
                        else:
                            ctrl.throttle = 0.15
                            ctrl.brake    = 0.0
                        a.apply_control(ctrl)
                except: pass
                time.sleep(0.04)

        # ── SAFETY CONTROLLER (EGO) ───────────────────────────
        def safety_thread():
            while not stop_flag[0]:
                try:
                    ego_tf  = ego.get_transform()
                    ego_loc = ego_tf.location
                    ego_fwd = ego_tf.get_forward_vector()
                    min_d   = 9999.0
                    for a in (list(world.get_actors().filter("vehicle.*")) +
                              list(world.get_actors().filter("walker.*"))):
                        if getattr(a,"id",None) == ego.id: continue
                        o = a.get_transform().location
                        dx = o.x-ego_loc.x; dy = o.y-ego_loc.y
                        if dx*ego_fwd.x + dy*ego_fwd.y < 0.5: continue
                        d = math.sqrt(dx*dx+dy*dy)
                        if d < min_d: min_d = d
                    dist_state[0] = min_d
                    v   = ego.get_velocity()
                    spd = math.sqrt(v.x**2+v.y**2+v.z**2)
                    st  = get_steer(ego)
                    if min_d < STOP_DIST:
                        target_spd[ego.id] = 0.0
                        ctrl = carla.VehicleControl()
                        ctrl.throttle=0.0; ctrl.brake=1.0
                        ctrl.hand_brake=True; ctrl.reverse=False; ctrl.steer=0.0
                        ego.apply_control(ctrl)
                        brake_state[0] = True
                    elif min_d < BRAKE_DIST:
                        factor = (min_d-STOP_DIST)/(BRAKE_DIST-STOP_DIST)
                        target = EGO_SPEED*factor
                        target_spd[ego.id] = target
                        if spd > target+0.5:
                            bv = min(0.85,(spd-target)/EGO_SPEED*1.5)
                            ctrl = carla.VehicleControl()
                            ctrl.throttle=0.0; ctrl.brake=bv
                            ctrl.hand_brake=False; ctrl.reverse=False
                            ctrl.steer=st
                            ego.apply_control(ctrl)
                        brake_state[0] = True
                    else:
                        if not brake_state[0]:
                            target_spd[ego.id] = EGO_SPEED
                        brake_state[0] = False
                except: pass
                time.sleep(0.04)

        threading.Thread(target=cruise_thread, daemon=True).start()
        threading.Thread(target=safety_thread, daemon=True).start()

        # ── СОБЫТИЯ ──────────────────────────────────────────
        def schedule(t_sec, fn):
            def _r():
                time.sleep(t_sec)
                if not stop_flag[0]: fn()
            threading.Thread(target=_r, daemon=True).start()

        if key == "4_1":
            car_actor = all_actors[1] if len(all_actors)>1 else None
            def car_slow(): 
                if car_actor: target_spd[car_actor.id] = 18.0
            def car_evasion():
                if car_actor:
                    tf = car_actor.get_transform()
                    r  = tf.get_right_vector()
                    tf.location.x += r.x*1.3; tf.location.y += r.y*1.3
                    car_actor.set_transform(tf)
            def car_back():
                if car_actor:
                    tf = car_actor.get_transform()
                    r  = tf.get_right_vector()
                    tf.location.x -= r.x*1.3; tf.location.y -= r.y*1.3
                    car_actor.set_transform(tf)
                    target_spd[car_actor.id] = EGO_SPEED
            schedule(30, car_slow); schedule(55, car_evasion); schedule(65, car_back)

        elif key == "4_2d":
            ped = all_actors[1] if len(all_actors)>1 else None
            def ped_cross():
                if ped:
                    wc = carla.WalkerControl()
                    wc.speed = 1.4
                    wc.direction = carla.Vector3D(0.0, -1.0, 0.0)
                    try: ped.apply_control(wc)
                    except: pass
            schedule(15, ped_cross)

        elif key == "4_3a":
            lead = all_actors[1] if len(all_actors)>1 else None
            def lead_slow():
                if lead: target_spd[lead.id] = 12.0
            def lead_accel():
                if lead: target_spd[lead.id] = EGO_SPEED
            def lead_stop():
                if lead: target_spd[lead.id] = 0.0
            schedule(20,lead_slow); schedule(38,lead_accel); schedule(52,lead_stop)

        elif key in ("4_3b","4_3f"):
            lead = all_actors[1] if len(all_actors)>1 else None
            def lead_brake():
                if lead: target_spd[lead.id] = 0.0
            schedule(20 if key=="4_3f" else 30, lead_brake)

        elif key in ("4_4a","4_4d"):
            ci = all_actors[1] if len(all_actors)>1 else None
            def do_cutin():
                if ci:
                    tf = ci.get_transform()
                    wp_new = _map.get_waypoint_xodr(1, -2, float(ci.get_transform().location.x))
                    if wp_new:
                        new_tf = wp_new.transform
                        new_tf.location.z = tf.location.z + 0.3
                        ci.set_transform(new_tf)
            schedule(10, do_cutin)

        elif key == "4_5a":
            lead = all_actors[1] if len(all_actors)>1 else None
            def lead_cutout():
                if lead:
                    tf = lead.get_transform()
                    r  = tf.get_right_vector()
                    tf.location.x -= r.x*4.5; tf.location.y -= r.y*4.5
                    lead.set_transform(tf)
                    target_spd[lead.id] = EGO_SPEED
            schedule(12, lead_cutout)

        # ── СЕНСОРЫ ──────────────────────────────────────────
        cam_f = bp.find("sensor.camera.rgb")
        cam_f.set_attribute("image_size_x", str(VIDEO_W))
        cam_f.set_attribute("image_size_y", str(VIDEO_H))
        cam_f.set_attribute("fov", "90")

        cam_b = bp.find("sensor.camera.rgb")
        cam_b.set_attribute("image_size_x", str(VIDEO_W))
        cam_b.set_attribute("image_size_y", str(VIDEO_H))
        cam_b.set_attribute("fov", "110")

        lid_bp = bp.find("sensor.lidar.ray_cast")
        for k,v in [("channels","64"),("range","120.0"),
                    ("points_per_second","250000"),("rotation_frequency","20.0"),
                    ("upper_fov","2.0"),("lower_fov","-24.9")]:
            lid_bp.set_attribute(k,v)

        s_f = world.spawn_actor(cam_f,
            carla.Transform(carla.Location(x=0.5,z=1.5),carla.Rotation(pitch=-5)),
            attach_to=ego)
        s_b = world.spawn_actor(cam_b,
            carla.Transform(carla.Location(x=-10,z=5),carla.Rotation(pitch=-20)),
            attach_to=ego)
        s_l = world.spawn_actor(lid_bp,
            carla.Transform(carla.Location(z=2.2)), attach_to=ego)

        sensors = [s_f, s_b, s_l]
        q_f = queue.Queue(maxsize=60)
        q_b = queue.Queue(maxsize=60)
        q_l = queue.Queue(maxsize=60)
        s_f.listen(lambda img: q_f.put(img) if not q_f.full() else None)
        s_b.listen(lambda img: q_b.put(img) if not q_b.full() else None)
        s_l.listen(lambda img: q_l.put(img) if not q_l.full() else None)
        print("Sensory OK")

        # ── SPECTATOR ─────────────────────────────────────────
        spectator = world.get_spectator()
        def follow():
            while not stop_flag[0]:
                try:
                    t = ego.get_transform()
                    r = math.radians(t.rotation.yaw)
                    spectator.set_transform(carla.Transform(
                        carla.Location(x=t.location.x-10*math.cos(r),
                                       y=t.location.y-10*math.sin(r),
                                       z=t.location.z+5),
                        carla.Rotation(pitch=-20,yaw=t.rotation.yaw)))
                except: pass
                time.sleep(0.05)
        threading.Thread(target=follow, daemon=True).start()

        # ── PYGAME ────────────────────────────────────────────
        if not pygame.get_init(): pygame.init()
        screen = pygame.display.set_mode((LIDAR_W, LIDAR_H))
        pygame.display.set_caption(name)
        font = pygame.font.SysFont("monospace", 13)
        cx, cy = LIDAR_W//2, LIDAR_H//2

        # ── ВИДЕОЗАПИСЫВАЮЩИЕ ПОТОКИ ─────────────────────────
        vw_f, pf = make_writer(os.path.join(out_dir,key+"_front.mp4"),VIDEO_W,VIDEO_H)
        vw_b, pb = make_writer(os.path.join(out_dir,key+"_bird.mp4"), VIDEO_W,VIDEO_H)
        vw_l, pl = make_writer(os.path.join(out_dir,key+"_lidar.mp4"),LIDAR_W,LIDAR_H)

        def write_front():
            while not stop_flag[0]:
                try:
                    img = q_f.get(timeout=0.1)
                    arr = np.frombuffer(img.raw_data, dtype=np.uint8)
                    # BGRA → BGR (без конвертации цветовых каналов)
                    frame = arr.reshape((VIDEO_H, VIDEO_W, 4))[:, :, :3].copy()
                    # Рисуем разметку полос
                   # frame = draw_lanes_on_frame(frame)
                    vw_f.write(frame)
                except queue.Empty: pass
                except: pass

        def write_bird():
            while not stop_flag[0]:
                try:
                    img = q_b.get(timeout=0.1)
                    arr = np.frombuffer(img.raw_data, dtype=np.uint8)
                    frame = arr.reshape((VIDEO_H, VIDEO_W, 4))[:, :, :3].copy()
                    vw_b.write(frame)
                except queue.Empty: pass
                except: pass

        threading.Thread(target=write_front, daemon=True).start()
        threading.Thread(target=write_bird,  daemon=True).start()

        # ── ГЛАВНЫЙ ЦИКЛ ──────────────────────────────────────
        frame  = 0
        t0     = time.time()
        print("Zapis {} sek...\n".format(duration))

        while time.time()-t0 < duration:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: raise KeyboardInterrupt

            if not q_l.empty():
                data    = q_l.get_nowait()
                pts     = np.frombuffer(data.raw_data,dtype=np.float32).reshape(-1,4)
                pos     = ego.get_transform().location
                elapsed = round(time.time()-t0, 1)
                dist_m  = dist_state[0]
                braking = brake_state[0]
                v       = ego.get_velocity()
                spd_kmh = round(math.sqrt(v.x**2+v.y**2+v.z**2)*3.6, 1)

                print("  [{}s] kadr {}  v={}km/h  dist={}  {}".format(
                    elapsed, str(frame).zfill(4), spd_kmh,
                    str(round(dist_m,1))+"m" if dist_m<9999 else "---",
                    "BRAKE!" if braking else ""))

                draw_lidar(screen,pts,cx,cy,font,name,elapsed,dist_m,braking,spd_kmh)
                pygame.display.flip()

                # LiDAR видео
                arr3d = pygame.surfarray.array3d(screen)
                vw_l.write(arr3d.transpose((1,0,2))[:,:,::-1])

                # PLY каждые 5 кадров
                if frame % 5 == 0:
                    ply = os.path.join(lidar_dir,"f"+str(frame).zfill(5)+".ply")
                    with open(ply,"w") as f:
                        f.write("ply\nformat ascii 1.0\n")
                        f.write("element vertex {}\n".format(len(pts)))
                        f.write("property float x\nproperty float y\n"
                                "property float z\nproperty float intensity\nend_header\n")
                        for p in pts:
                            f.write("{} {} {} {}\n".format(
                                round(float(p[0]),3),round(float(p[1]),3),
                                round(float(p[2]),3),round(float(p[3]),3)))
                frame += 1

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nOstanovleno.")

    finally:
        # ── ЧИСТОЕ ЗАВЕРШЕНИЕ — нет краша CARLA ──────────────
        stop_flag[0] = True
        time.sleep(0.5)   # даём потокам завершиться

        # Останавливаем и сохраняем видео ДО уничтожения сенсоров
        try:
            if vw_f: vw_f.release()
        except: pass
        try:
            if vw_b: vw_b.release()
        except: pass
        try:
            if vw_l: vw_l.release()
        except: pass

        # Сначала останавливаем listen(), потом destroy()
        for s in sensors:
            try: s.stop()
            except: pass
        time.sleep(0.3)
        for s in sensors:
            try: s.destroy()
            except: pass

        # Уничтожаем акторы по одному с задержкой
        for a in all_actors:
            try:
                if a: a.destroy()
            except: pass
            time.sleep(0.05)

        try: pygame.display.quit()
        except: pass

        print("\n" + "="*64)
        print("ZAVERSHEN: " + name)
        if pf: print("  Front -> " + pf)
        if pb: print("  Bird  -> " + pb)
        if pl: print("  LiDAR -> " + pl)
        print("="*64)

# ══ ТОЧКА ВХОДА ══════════════════════════════════════════════════
if __name__ == "__main__":
    ensure_carla()
    client, world = connect()
    client, world = load_map(client)

    choice = show_menu()

    if choice == 0:
        print("\nZapusk vsekh...\n")
        for i in range(len(SCENARIOS)):
            try:
                run_scenario(i, client, world)
            except Exception as e:
                print("OSHIBKA {}: {}".format(SCENARIOS[i][0], e))
            if i < len(SCENARIOS)-1:
                print("Pauza 5 sek...")
                time.sleep(5)
                # Переподключение после каждого сценария
                try: world = client.get_world()
                except:
                    try: client, world = connect()
                    except: pass
        print("\nVse zaversheny!")
    else:
        run_scenario(choice-1, client, world)
