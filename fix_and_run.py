# fix_and_run.py — 4.2(a): Tesla (red) edet po pryamoy k stoящему Audi (blue)
# C:\Users\popes\AppData\Local\Programs\Python\Python37\python.exe D:\dthcbz\project\scenarios\python\fix_and_run.py

import sys, os, time, queue, math, threading
import numpy as np

EGG       = r"C:\Carla simulator 0.14.0\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.14-py3.7-win-amd64.egg"
CARLA_API = r"C:\Carla simulator 0.14.0\WindowsNoEditor\PythonAPI"
for p in [EGG, os.path.join(CARLA_API, "carla"), CARLA_API]:
    if p not in sys.path:
        sys.path.insert(0, p)

import carla
import cv2

OUTPUT_DIR   = r"D:\dthcbz\project\scenarios\output\4_2a"
DURATION_SEC = 60
VIDEO_W      = 1280
VIDEO_H      = 720
VIDEO_FPS    = 20

os.makedirs(OUTPUT_DIR, exist_ok=True)
os.makedirs(os.path.join(OUTPUT_DIR, "lidar"), exist_ok=True)

# ── Реальные координаты из find_road.py ───────────────────────────
# Прямой участок 300м, road_id=6 lane_id=-2, yaw=90.4
EGO_X   = -514.23
EGO_Y   =  177.52
EGO_Z   =    0.30
EGO_YAW =   90.36   # едет вверх по оси Y

OBS_X   = -514.37   # 80м вперёд по той же полосе
OBS_Y   =  259.08
OBS_Z   =    0.30
OBS_YAW =   90.36

# ── 1. Подключение ────────────────────────────────────────────────
print("Podklyuchenie...")
client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world  = client.get_world()

if "Town04" not in world.get_map().name:
    print("Gruzhu Town04...")
    world = client.load_world("Town04")
    time.sleep(10)
print("OK: " + world.get_map().name)

bp_lib    = world.get_blueprint_library()
carla_map = world.get_map()

# ── 2. Очищаем сцену ──────────────────────────────────────────────
print("Ochishchayu scenu...")
for actor in world.get_actors().filter("vehicle.*"):
    actor.destroy()
for actor in world.get_actors().filter("walker.*"):
    actor.destroy()
time.sleep(1)
world = client.get_world()

# ── 3. Спавним красную Tesla ──────────────────────────────────────
ego_transform = carla.Transform(
    carla.Location(x=EGO_X, y=EGO_Y, z=EGO_Z),
    carla.Rotation(yaw=EGO_YAW)
)

ego_bp = bp_lib.find("vehicle.tesla.model3")
ego_bp.set_attribute("role_name", "hero")
ego_bp.set_attribute("color", "255,0,0")

ego = world.try_spawn_actor(ego_bp, ego_transform)
if ego is None:
    # Пробуем чуть сместить
    for dz in [0.5, 1.0, 1.5]:
        t = carla.Transform(
            carla.Location(x=EGO_X, y=EGO_Y, z=EGO_Z + dz),
            carla.Rotation(yaw=EGO_YAW))
        ego = world.try_spawn_actor(ego_bp, t)
        if ego:
            break

if ego is None:
    print("OSHIBKA: Tesla ne spawnilas!")
    sys.exit(1)
print("Tesla (red)  id=" + str(ego.id) +
      "  pos: x=" + str(round(ego.get_transform().location.x,1)) +
      " y=" + str(round(ego.get_transform().location.y,1)))

# ── 4. Спавним синий Audi (стоит в полосе, 80м вперёд) ───────────
obs_transform = carla.Transform(
    carla.Location(x=OBS_X, y=OBS_Y, z=OBS_Z),
    carla.Rotation(yaw=OBS_YAW)
)

car_bp = bp_lib.find("vehicle.audi.tt")
car_bp.set_attribute("color", "0,0,255")

car = world.try_spawn_actor(car_bp, obs_transform)
if car is None:
    for dz in [0.5, 1.0]:
        t = carla.Transform(
            carla.Location(x=OBS_X, y=OBS_Y, z=OBS_Z + dz),
            carla.Rotation(yaw=OBS_YAW))
        car = world.try_spawn_actor(car_bp, t)
        if car:
            break

if car is None:
    print("OSHIBKA: Audi ne spawnilas!")
    ego.destroy()
    sys.exit(1)

car.set_simulate_physics(False)
dist = ego.get_transform().location.distance(car.get_transform().location)
print("Audi TT (blue) id=" + str(car.id) +
      "  dist=" + str(round(dist,1)) + "m ot Tesla")

# ── 5. Записываем XOSC ───────────────────────────────────────────
xosc = (
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<OpenSCENARIO>\n'
    '  <FileHeader description="4.2a Stationary Car" author="ALKS"'
    ' revMajor="1" revMinor="0" date="2024-01-01T00:00:00"/>\n'
    '  <ParameterDeclarations/>\n'
    '  <CatalogLocations/>\n'
    '  <RoadNetwork><LogicFile filepath="Town04"/></RoadNetwork>\n'
    '  <Entities>\n'
    '    <ScenarioObject name="hero">\n'
    '      <Vehicle name="vehicle.tesla.model3" vehicleCategory="car">\n'
    '        <BoundingBox><Center x="1.5" y="0.0" z="0.9"/>'
    '<Dimensions width="2.1" length="4.5" height="1.8"/></BoundingBox>\n'
    '        <Performance maxSpeed="69.0" maxAcceleration="10.0" maxDeceleration="10.0"/>\n'
    '        <Axles>\n'
    '          <FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="1.8" positionX="3.1" positionZ="0.3"/>\n'
    '          <RearAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.8" positionX="0.0" positionZ="0.3"/>\n'
    '        </Axles>\n'
    '        <Properties>\n'
    '          <Property name="type" value="ego_vehicle"/>\n'
    '          <Property name="color" value="255,0,0"/>\n'
    '        </Properties>\n'
    '      </Vehicle>\n'
    '    </ScenarioObject>\n'
    '    <ScenarioObject name="StationaryCar">\n'
    '      <Vehicle name="vehicle.audi.tt" vehicleCategory="car">\n'
    '        <BoundingBox><Center x="1.5" y="0.0" z="0.75"/>'
    '<Dimensions width="2.0" length="4.0" height="1.5"/></BoundingBox>\n'
    '        <Performance maxSpeed="0.0" maxAcceleration="0.0" maxDeceleration="0.0"/>\n'
    '        <Axles>\n'
    '          <FrontAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.8" positionX="2.7" positionZ="0.3"/>\n'
    '          <RearAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.8" positionX="0.0" positionZ="0.3"/>\n'
    '        </Axles>\n'
    '        <Properties><Property name="type" value="simulation"/></Properties>\n'
    '      </Vehicle>\n'
    '    </ScenarioObject>\n'
    '  </Entities>\n'
    '  <Storyboard>\n'
    '    <Init><Actions>\n'
    '      <Private entityRef="hero">\n'
    '        <PrivateAction><TeleportAction><Position>\n'
    '          <WorldPosition x="EGO_X" y="EGO_Y" z="EGO_Z" h="EGO_YAW"/>\n'
    '        </Position></TeleportAction></PrivateAction>\n'
    '        <PrivateAction><LongitudinalAction><SpeedAction>\n'
    '          <SpeedActionDynamics dynamicsShape="step" value="0.0" dynamicsDimension="time"/>\n'
    '          <SpeedActionTarget><AbsoluteTargetSpeed value="13.89"/></SpeedActionTarget>\n'
    '        </SpeedAction></LongitudinalAction></PrivateAction>\n'
    '      </Private>\n'
    '      <Private entityRef="StationaryCar">\n'
    '        <PrivateAction><TeleportAction><Position>\n'
    '          <WorldPosition x="OBS_X" y="OBS_Y" z="OBS_Z" h="OBS_YAW"/>\n'
    '        </Position></TeleportAction></PrivateAction>\n'
    '      </Private>\n'
    '    </Actions></Init>\n'
    '    <Story name="Story"><Act name="Act">\n'
    '      <ManeuverGroup name="MG" maximumExecutionCount="1">\n'
    '        <Actors selectTriggeringEntities="false"><EntityRef entityRef="hero"/></Actors>\n'
    '        <Maneuver name="M"><Event name="Drive" priority="overwrite">\n'
    '          <Action name="Speed"><PrivateAction><LongitudinalAction><SpeedAction>\n'
    '            <SpeedActionDynamics dynamicsShape="step" value="0.0" dynamicsDimension="time"/>\n'
    '            <SpeedActionTarget><AbsoluteTargetSpeed value="13.89"/></SpeedActionTarget>\n'
    '          </SpeedAction></LongitudinalAction></PrivateAction></Action>\n'
    '          <StartTrigger><ConditionGroup><Condition name="T1" delay="0" conditionEdge="none">\n'
    '            <ByValueCondition><SimulationTimeCondition value="1.0" rule="greaterThan"/></ByValueCondition>\n'
    '          </Condition></ConditionGroup></StartTrigger>\n'
    '        </Event></Maneuver>\n'
    '      </ManeuverGroup>\n'
    '      <StartTrigger><ConditionGroup><Condition name="Go" delay="0" conditionEdge="none">\n'
    '        <ByValueCondition><SimulationTimeCondition value="0.0" rule="greaterThan"/></ByValueCondition>\n'
    '      </Condition></ConditionGroup></StartTrigger>\n'
    '      <StopTrigger><ConditionGroup><Condition name="End" delay="0" conditionEdge="none">\n'
    '        <ByValueCondition><SimulationTimeCondition value="60.0" rule="greaterThan"/></ByValueCondition>\n'
    '      </Condition></ConditionGroup></StopTrigger>\n'
    '    </Act></Story>\n'
    '    <StopTrigger><ConditionGroup><Condition name="End2" delay="0" conditionEdge="none">\n'
    '      <ByValueCondition><SimulationTimeCondition value="60.0" rule="greaterThan"/></ByValueCondition>\n'
    '    </Condition></ConditionGroup></StopTrigger>\n'
    '  </Storyboard>\n'
    '</OpenSCENARIO>'
)
xosc = xosc.replace("EGO_X", str(EGO_X)).replace("EGO_Y", str(EGO_Y))
xosc = xosc.replace("EGO_Z", str(EGO_Z)).replace("EGO_YAW", str(EGO_YAW))
xosc = xosc.replace("OBS_X", str(OBS_X)).replace("OBS_Y", str(OBS_Y))
xosc = xosc.replace("OBS_Z", str(OBS_Z)).replace("OBS_YAW", str(OBS_YAW))

xosc_path = os.path.join(OUTPUT_DIR, "4_2a_stationary_car.xosc")
with open(xosc_path, "w") as f:
    f.write(xosc)
print("XOSC: " + xosc_path)

# ── 6. Движение Tesla по полосе через waypoints ───────────────────
# Получаем цепочку waypoints от ego до obstacle по полосе
ego_wp = carla_map.get_waypoint(
    ego.get_transform().location,
    project_to_road=True,
    lane_type=carla.LaneType.Driving)

# Собираем waypoints до препятствия
route = []
cur = ego_wp
while True:
    route.append(cur.transform.location)
    nexts = cur.next(2.0)
    if not nexts:
        break
    cur = nexts[0]
    # Останавливаемся немного не доезжая до Audi
    d = cur.transform.location.distance(car.get_transform().location)
    if d < 20.0:
        break

print("Marshrut: " + str(len(route)) + " tochek, dist=" + str(round(dist,1)) + "m")

# Движение по waypoints — вызываем set_target_velocity по направлению к следующей точке
route_idx = [0]
SPEED     = 13.89  # 50 km/h

def drive_loop():
    while route_idx[0] < len(route) - 1:
        try:
            cur_loc  = ego.get_transform().location
            tgt      = route[route_idx[0]]
            dx       = tgt.x - cur_loc.x
            dy       = tgt.y - cur_loc.y
            dist_tgt = math.sqrt(dx*dx + dy*dy)
            if dist_tgt < 3.0:
                route_idx[0] += 1
                continue
            norm = dist_tgt if dist_tgt > 0 else 1.0
            ego.set_target_velocity(carla.Vector3D(
                x=SPEED * dx / norm,
                y=SPEED * dy / norm,
                z=0.0))
        except Exception:
            break
        time.sleep(0.05)
    # Остановка перед Audi
    try:
        ego.set_target_velocity(carla.Vector3D(x=0, y=0, z=0))
    except Exception:
        pass

drive_thread = threading.Thread(target=drive_loop, daemon=True)
drive_thread.start()

# Аварийная остановка — если Tesla слишком близко к Audi
def safety_brake():
    while not stop_flag[0]:
        try:
            d = ego.get_transform().location.distance(car.get_transform().location)
            if d < 15.0:
                ego.set_target_velocity(carla.Vector3D(x=0,y=0,z=0))
                # Применяем тормоз
                ctrl = carla.VehicleControl()
                ctrl.brake = 1.0
                ctrl.throttle = 0.0
                ego.apply_control(ctrl)
        except Exception:
            pass
        time.sleep(0.1)

threading.Thread(target=safety_brake, daemon=True).start()
print("Drive loop zapushchen + safety brake OK")

# ── 7. Spectator следит за Tesla ─────────────────────────────────
spectator = world.get_spectator()
stop_flag = [False]

def follow():
    while not stop_flag[0]:
        try:
            t = ego.get_transform()
            r = math.radians(t.rotation.yaw)
            spectator.set_transform(carla.Transform(
                carla.Location(
                    x=t.location.x - 12*math.cos(r),
                    y=t.location.y - 12*math.sin(r),
                    z=t.location.z + 6),
                carla.Rotation(pitch=-20, yaw=t.rotation.yaw)))
        except Exception:
            pass
        time.sleep(0.05)

threading.Thread(target=follow, daemon=True).start()
print("Spectator OK")

# ── 8. Сенсоры ───────────────────────────────────────────────────
sensors = []
q_ego   = queue.Queue()
q_bird  = queue.Queue()
q_lidar = queue.Queue()

cam_bp = bp_lib.find("sensor.camera.rgb")
cam_bp.set_attribute("image_size_x", str(VIDEO_W))
cam_bp.set_attribute("image_size_y", str(VIDEO_H))
cam_bp.set_attribute("fov", "90")

c1 = world.spawn_actor(cam_bp,
     carla.Transform(carla.Location(x=0.5,z=1.4),carla.Rotation(pitch=-5)),
     attach_to=ego)
c1.listen(q_ego.put); sensors.append(c1)

c2 = world.spawn_actor(cam_bp,
     carla.Transform(carla.Location(x=-12,z=6),carla.Rotation(pitch=-20)),
     attach_to=ego)
c2.listen(q_bird.put); sensors.append(c2)

lid_bp = bp_lib.find("sensor.lidar.ray_cast")
lid_bp.set_attribute("channels",           "64")
lid_bp.set_attribute("range",              "100.0")
lid_bp.set_attribute("points_per_second",  "500000")
lid_bp.set_attribute("rotation_frequency", "20.0")
lid_bp.set_attribute("upper_fov",          "2.0")
lid_bp.set_attribute("lower_fov",          "-24.9")
lid = world.spawn_actor(lid_bp,
      carla.Transform(carla.Location(z=2.2)), attach_to=ego)
lid.listen(q_lidar.put); sensors.append(lid)
print("Kamery + LiDAR OK")

# ── LiDAR окно визуализации (pygame) ─────────────────────────────
import pygame
pygame.init()
LIDAR_W, LIDAR_H = 800, 800
lidar_screen = pygame.display.set_mode((LIDAR_W, LIDAR_H))
pygame.display.set_caption("LiDAR — 4.2a Stationary Car")
lidar_font = pygame.font.SysFont("monospace", 16)
LIDAR_SCALE = 3.5   # пикселей на метр
print("LiDAR window OK\n")

# ── 9. Запись ─────────────────────────────────────────────────────
lidar_dir = os.path.join(OUTPUT_DIR, "lidar")
fourcc    = cv2.VideoWriter_fourcc(*"mp4v")
vw_ego    = cv2.VideoWriter(os.path.join(OUTPUT_DIR,"4_2a_ego.mp4"),
                             fourcc,VIDEO_FPS,(VIDEO_W,VIDEO_H))
vw_bird   = cv2.VideoWriter(os.path.join(OUTPUT_DIR,"4_2a_birdseye.mp4"),
                             fourcc,VIDEO_FPS,(VIDEO_W,VIDEO_H))
vw_lidar  = cv2.VideoWriter(os.path.join(OUTPUT_DIR,"4_2a_lidar.mp4"),
                             fourcc,VIDEO_FPS,(800,800))
frame = 0
t0    = time.time()
print("Zapis " + str(DURATION_SEC) + " sek...\n")

# Видео пишем в отдельном потоке — не зависит от LiDAR
def write_video():
    while not stop_flag[0]:
        while not q_ego.empty():
            try:
                arr = np.frombuffer(q_ego.get_nowait().raw_data,dtype=np.uint8)
                vw_ego.write(arr.reshape((VIDEO_H,VIDEO_W,4))[:,:,:3])
            except Exception:
                pass
        while not q_bird.empty():
            try:
                arr = np.frombuffer(q_bird.get_nowait().raw_data,dtype=np.uint8)
                vw_bird.write(arr.reshape((VIDEO_H,VIDEO_W,4))[:,:,:3])
            except Exception:
                pass
        time.sleep(0.01)

video_thread = threading.Thread(target=write_video, daemon=True)
video_thread.start()
print("Video thread zapushchen")

try:
    while time.time()-t0 < DURATION_SEC:

        # Обработка закрытия pygame окна
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        # LiDAR — визуализация + запись PLY
        if not q_lidar.empty():
            data = q_lidar.get_nowait()
            pts  = np.frombuffer(data.raw_data,dtype=np.float32).reshape(-1,4)
            pos  = ego.get_transform().location
            elapsed = round(time.time()-t0,1)
            print("  ["+str(elapsed)+"s] kadr "+str(frame).zfill(4)+
                  " | Tesla x="+str(round(pos.x,1))+" y="+str(round(pos.y,1))+
                  " | "+str(len(pts))+" pts")

            # Рисуем LiDAR точки
            lidar_screen.fill((0, 0, 0))
            cx_px = LIDAR_W // 2
            cy_px = LIDAR_H // 2

            for p in pts:
                lx = float(p[0])   # вперёд
                ly = float(p[1])   # влево
                lz = float(p[2])   # вверх

                # Проецируем на 2D вид сверху
                sx = int(cx_px - ly * LIDAR_SCALE)
                sy = int(cy_px - lx * LIDAR_SCALE)

                if 0 <= sx < LIDAR_W and 0 <= sy < LIDAR_H:
                    # Цвет: жёлтый (низко) -> белый (высоко) как на скриншоте
                    norm_z = min(1.0, max(0.0, (lz + 3.0) / 6.0))
                    r = 255
                    g = int(200 + 55 * norm_z)
                    b = int(0   + 200 * norm_z)
                    lidar_screen.set_at((sx, sy), (r, g, b))

            # Красный крест — позиция Tesla
            pygame.draw.line(lidar_screen,(255,0,0),(cx_px-12,cy_px),(cx_px+12,cy_px),2)
            pygame.draw.line(lidar_screen,(255,0,0),(cx_px,cy_px-12),(cx_px,cy_px+12),2)

            # Подпись
            txt = lidar_font.render(
                "t="+str(elapsed)+"s  pts="+str(len(pts))+"  kadr="+str(frame),
                True, (180,180,180))
            lidar_screen.blit(txt, (10,10))
            txt2 = lidar_font.render(
                "Tesla x="+str(round(pos.x,1))+" y="+str(round(pos.y,1)),
                True, (180,180,180))
            lidar_screen.blit(txt2, (10,30))

            pygame.display.flip()

            # Сохраняем кадр LiDAR визуализации в видео
            lidar_arr = pygame.surfarray.array3d(lidar_screen)
            lidar_arr = lidar_arr.transpose((1,0,2))  # pygame -> numpy HWC
            lidar_bgr = lidar_arr[:,:,::-1]           # RGB -> BGR для OpenCV
            vw_lidar.write(lidar_bgr)

            # Сохраняем PLY
            ply = os.path.join(lidar_dir,"frame_"+str(frame).zfill(5)+".ply")
            with open(ply,"w") as f:
                f.write("ply\nformat ascii 1.0\n")
                f.write("element vertex "+str(len(pts))+"\n")
                f.write("property float x\nproperty float y\n"
                        "property float z\nproperty float intensity\nend_header\n")
                for p in pts:
                    f.write(str(round(float(p[0]),4))+" "+
                            str(round(float(p[1]),4))+" "+
                            str(round(float(p[2]),4))+" "+
                            str(round(float(p[3]),4))+"\n")
            frame += 1

        time.sleep(0.005)
except KeyboardInterrupt:
    print("\nOstanovleno.")

pygame.quit()

# ── 10. Завершение ────────────────────────────────────────────────
stop_flag[0] = True
vw_ego.release(); vw_bird.release(); vw_lidar.release()
for s in sensors:
    try: s.stop(); s.destroy()
    except Exception: pass
try: ego.destroy()
except Exception: pass
try: car.destroy()
except Exception: pass

print("\n==================================================")
print("XOSC              -> " + xosc_path)
print("Video iz mashiny  -> " + OUTPUT_DIR + "\\4_2a_ego.mp4")
print("Video so storony  -> " + OUTPUT_DIR + "\\4_2a_birdseye.mp4")
print("Video LiDAR       -> " + OUTPUT_DIR + "\\4_2a_lidar.mp4")
print("Video so storony  -> " + OUTPUT_DIR + "\\4_2a_birdseye.mp4")
print("LiDAR             -> " + lidar_dir + "  (" + str(frame) + " faylov .ply)")
print("==================================================")
