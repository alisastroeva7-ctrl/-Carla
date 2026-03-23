# run_xosc.py
# Zapusk:
# C:\Users\popes\AppData\Local\Programs\Python\Python37\python.exe D:\dthcbz\project\scenarios\python\run_xosc.py

import sys
import os
import subprocess
import time
import queue
import math
import threading
import numpy as np

EGG = r"C:\Carla simulator 0.14.0\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.14-py3.7-win-amd64.egg"
CARLA_API = r"C:\Carla simulator 0.14.0\WindowsNoEditor\PythonAPI"
CARLA_EXE = r"C:\Carla simulator 0.14.0\WindowsNoEditor\CarlaUE4.exe"
CARLA_DIR = r"C:\Carla simulator 0.14.0\WindowsNoEditor"

for p in [EGG, os.path.join(CARLA_API, "carla"), CARLA_API]:
    if p not in sys.path:
        sys.path.insert(0, p)

import carla
import cv2
import pygame

# ══ НАСТРОЙКИ ════════════════════════════════════════════════════
XOSC_PATH       = r"D:\dthcbz\project\scenarios\output\4_2a\4_2a_stationary_car_full.xosc"
SCENARIO_RUNNER = r"D:\scenario_runner\scenario_runner.py"
OUTPUT_DIR      = r"D:\dthcbz\project\scenarios\output\4_2a"
DURATION_SEC    = 60
VIDEO_W         = 1280
VIDEO_H         = 720
VIDEO_FPS       = 20
LIDAR_W         = 800
LIDAR_H         = 800
LIDAR_SCALE     = 3.5
# ═════════════════════════════════════════════════════════════════

os.makedirs(OUTPUT_DIR, exist_ok=True)
os.makedirs(os.path.join(OUTPUT_DIR, "lidar"), exist_ok=True)

# ── 0. Патч OSC2 ─────────────────────────────────────────────────
stub = "# disabled\nclass OSC2Scenario(object): pass\nclass OSC2Helper(object): pass\n"
for f in [r"D:\scenario_runner\srunner\tools\osc2_helper.py",
          r"D:\scenario_runner\srunner\scenarios\osc2_scenario.py"]:
    if os.path.exists(f):
        with open(f, "r") as fh:
            c = fh.read()
        if "antlr4" in c or "list[" in c or len(c) < 10:
            with open(f, "w") as fh:
                fh.write(stub)
            print("patched: " + f)


print("Podklyuchenie k CARLA...")
client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world  = client.get_world()
print("OK: " + world.get_map().name)

# ── 3. Патч OSC2 и запуск ScenarioRunner ─────────────────────────
sr_env = os.environ.copy()
sr_env["PYTHONPATH"] = EGG + ";" + os.path.join(CARLA_API, "carla") + ";" + CARLA_API

sr_dir = os.path.dirname(os.path.abspath(SCENARIO_RUNNER))
cmd = [sys.executable, SCENARIO_RUNNER,
       "--openscenario", XOSC_PATH,
       "--timeout", "120",
       "--waitForEgo"]

print("ScenarioRunner: " + XOSC_PATH)
sr_proc = subprocess.Popen(cmd, cwd=sr_dir, env=sr_env)

# ── 4. Ищем ego ──────────────────────────────────────────────────
print("Ishchu ego", end="", flush=True)
ego = None
for _ in range(60):
    world = client.get_world()
    for a in world.get_actors().filter("vehicle.*"):
        if a.attributes.get("role_name") in ("hero", "ego_vehicle", "ego"):
            ego = a
            break
    if ego:
        break
    print(".", end="", flush=True)
    time.sleep(0.5)

if ego is None:
    print("\nEgo ne nayden!")
    sr_proc.terminate()
    sys.exit(1)
print(" OK: " + ego.type_id + " id=" + str(ego.id))

# ── 5. Spectator ─────────────────────────────────────────────────
spectator = world.get_spectator()
stop_flag = [False]

def follow():
    while not stop_flag[0]:
        try:
            t = ego.get_transform()
            r = math.radians(t.rotation.yaw)
            spectator.set_transform(carla.Transform(
                carla.Location(
                    x=t.location.x - 12 * math.cos(r),
                    y=t.location.y - 12 * math.sin(r),
                    z=t.location.z + 6),
                carla.Rotation(pitch=-20, yaw=t.rotation.yaw)))
        except Exception:
            pass
        time.sleep(0.05)

threading.Thread(target=follow, daemon=True).start()
print("Spectator OK")

# ── 6. Сенсоры ───────────────────────────────────────────────────
sensors = []
q_ego   = queue.Queue()
q_bird  = queue.Queue()
q_lidar = queue.Queue()
bp_lib  = world.get_blueprint_library()

cam_bp = bp_lib.find("sensor.camera.rgb")
cam_bp.set_attribute("image_size_x", str(VIDEO_W))
cam_bp.set_attribute("image_size_y", str(VIDEO_H))
cam_bp.set_attribute("fov", "90")

c1 = world.spawn_actor(cam_bp,
     carla.Transform(carla.Location(x=0.5, z=1.4), carla.Rotation(pitch=-5)),
     attach_to=ego)
c1.listen(q_ego.put)
sensors.append(c1)

c2 = world.spawn_actor(cam_bp,
     carla.Transform(carla.Location(x=-12, z=6), carla.Rotation(pitch=-20)),
     attach_to=ego)
c2.listen(q_bird.put)
sensors.append(c2)

lid_bp = bp_lib.find("sensor.lidar.ray_cast")
lid_bp.set_attribute("channels",           "64")
lid_bp.set_attribute("range",              "100.0")
lid_bp.set_attribute("points_per_second",  "500000")
lid_bp.set_attribute("rotation_frequency", "20.0")
lid_bp.set_attribute("upper_fov",          "2.0")
lid_bp.set_attribute("lower_fov",          "-24.9")
lid = world.spawn_actor(lid_bp,
      carla.Transform(carla.Location(z=2.2)), attach_to=ego)
lid.listen(q_lidar.put)
sensors.append(lid)
print("Kamery + LiDAR OK")

# ── 7. pygame LiDAR окно ─────────────────────────────────────────
pygame.init()
lidar_screen = pygame.display.set_mode((LIDAR_W, LIDAR_H))
pygame.display.set_caption("LiDAR — 4.2a")
lidar_font = pygame.font.SysFont("monospace", 16)
print("LiDAR window OK\n")

# ── 8. Видео writers ─────────────────────────────────────────────
fourcc   = cv2.VideoWriter_fourcc(*"mp4v")
vw_ego   = cv2.VideoWriter(os.path.join(OUTPUT_DIR, "4_2a_ego.mp4"),
                            fourcc, VIDEO_FPS, (VIDEO_W, VIDEO_H))
vw_bird  = cv2.VideoWriter(os.path.join(OUTPUT_DIR, "4_2a_birdseye.mp4"),
                            fourcc, VIDEO_FPS, (VIDEO_W, VIDEO_H))
vw_lidar = cv2.VideoWriter(os.path.join(OUTPUT_DIR, "4_2a_lidar.mp4"),
                            fourcc, VIDEO_FPS, (LIDAR_W, LIDAR_H))

# Видео в отдельном потоке
def write_video():
    while not stop_flag[0]:
        while not q_ego.empty():
            try:
                arr = np.frombuffer(q_ego.get_nowait().raw_data, dtype=np.uint8)
                vw_ego.write(arr.reshape((VIDEO_H, VIDEO_W, 4))[:, :, :3])
            except Exception:
                pass
        while not q_bird.empty():
            try:
                arr = np.frombuffer(q_bird.get_nowait().raw_data, dtype=np.uint8)
                vw_bird.write(arr.reshape((VIDEO_H, VIDEO_W, 4))[:, :, :3])
            except Exception:
                pass
        time.sleep(0.01)

threading.Thread(target=write_video, daemon=True).start()
print("Video thread OK")

# ── 9. Главный цикл ──────────────────────────────────────────────
lidar_dir = os.path.join(OUTPUT_DIR, "lidar")
frame = 0
t0    = time.time()
print("Zapis " + str(DURATION_SEC) + " sek...\n")

try:
    while time.time() - t0 < DURATION_SEC:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        if not q_lidar.empty():
            data    = q_lidar.get_nowait()
            pts     = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
            pos     = ego.get_transform().location
            elapsed = round(time.time() - t0, 1)

            print("  [" + str(elapsed) + "s] kadr " + str(frame).zfill(4) +
                  " | x=" + str(round(pos.x, 1)) +
                  " y=" + str(round(pos.y, 1)) +
                  " | " + str(len(pts)) + " pts")

            # Рисуем LiDAR
            lidar_screen.fill((0, 0, 0))
            cx = LIDAR_W // 2
            cy = LIDAR_H // 2

            for p in pts:
                sx = int(cx - float(p[1]) * LIDAR_SCALE)
                sy = int(cy - float(p[0]) * LIDAR_SCALE)
                if 0 <= sx < LIDAR_W and 0 <= sy < LIDAR_H:
                    norm_z = min(1.0, max(0.0, (float(p[2]) + 3.0) / 6.0))
                    lidar_screen.set_at((sx, sy), (255, int(200 + 55*norm_z), int(200*norm_z)))

            pygame.draw.line(lidar_screen, (255,0,0), (cx-12,cy), (cx+12,cy), 2)
            pygame.draw.line(lidar_screen, (255,0,0), (cx,cy-12), (cx,cy+12), 2)

            t1 = lidar_font.render("t=" + str(elapsed) + "s  pts=" + str(len(pts)), True, (200,200,200))
            t2 = lidar_font.render("x=" + str(round(pos.x,1)) + " y=" + str(round(pos.y,1)), True, (200,200,200))
            t3 = lidar_font.render("4.2a Stationary Car — ALKS Test", True, (100,200,100))
            lidar_screen.blit(t1, (10, 10))
            lidar_screen.blit(t2, (10, 30))
            lidar_screen.blit(t3, (10, LIDAR_H - 25))

            pygame.display.flip()

            # LiDAR видео
            lidar_arr = pygame.surfarray.array3d(lidar_screen)
            vw_lidar.write(lidar_arr.transpose((1,0,2))[:,:,::-1])

            # PLY файл
            ply = os.path.join(lidar_dir, "frame_" + str(frame).zfill(5) + ".ply")
            with open(ply, "w") as f:
                f.write("ply\nformat ascii 1.0\n")
                f.write("element vertex " + str(len(pts)) + "\n")
                f.write("property float x\nproperty float y\nproperty float z\nproperty float intensity\nend_header\n")
                for p in pts:
                    f.write(str(round(float(p[0]),4)) + " " +
                            str(round(float(p[1]),4)) + " " +
                            str(round(float(p[2]),4)) + " " +
                            str(round(float(p[3]),4)) + "\n")
            frame += 1

        time.sleep(0.005)

except KeyboardInterrupt:
    print("\nOstanovleno.")

# ── 10. Завершение ────────────────────────────────────────────────
stop_flag[0] = True
pygame.quit()
time.sleep(0.5)
vw_ego.release()
vw_bird.release()
vw_lidar.release()

for s in sensors:
    try:
        s.stop()
        s.destroy()
    except Exception:
        pass

sr_proc.terminate()
sr_proc.wait()

print("\n==================================================")
print("XOSC        -> " + XOSC_PATH)
print("Ego video   -> " + OUTPUT_DIR + "\\4_2a_ego.mp4")
print("Bird video  -> " + OUTPUT_DIR + "\\4_2a_birdseye.mp4")
print("LiDAR video -> " + OUTPUT_DIR + "\\4_2a_lidar.mp4")
print("LiDAR PLY   -> " + lidar_dir + "  (" + str(frame) + " faylov)")
print("==================================================")
