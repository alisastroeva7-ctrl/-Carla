# find_road.py — найти длинный прямой участок в Town04
# C:\Users\popes\AppData\Local\Programs\Python\Python37\python.exe D:\dthcbz\project\scenarios\python\find_road.py

import sys, time

EGG       = r"C:\Carla simulator 0.14.0\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.14-py3.7-win-amd64.egg"
CARLA_API = r"C:\Carla simulator 0.14.0\WindowsNoEditor\PythonAPI"
for p in [EGG, CARLA_API]:
    if p not in sys.path:
        sys.path.insert(0, p)

import carla

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world  = client.get_world()

if "Town04" not in world.get_map().name:
    world = client.load_world("Town04")
    time.sleep(10)

carla_map = world.get_map()

# Генерируем waypoints по всей карте с шагом 2м
all_wp = carla_map.generate_waypoints(2.0)
print("Vsego waypoints: " + str(len(all_wp)))

# Для каждого waypoint считаем длину прямого участка вперёд
def straight_ahead(start_wp, max_dist=300.0, step=5.0):
    wps   = [start_wp]
    dist  = 0.0
    cur   = start_wp
    while dist < max_dist:
        nexts = cur.next(step)
        if not nexts:
            break
        nxt      = nexts[0]
        dy       = abs(nxt.transform.rotation.yaw - cur.transform.rotation.yaw)
        if dy > 180: dy = 360 - dy
        if dy > 4:
            break
        cur  = nxt
        dist += step
        wps.append(cur)
    return dist, wps

print("\nIshchu dlinnye pryamye uchastki...")
results = []
checked = set()

for wp in all_wp:
    if wp.lane_type != carla.LaneType.Driving:
        continue
    key = (round(wp.transform.location.x, 10), round(wp.transform.location.y, 10))
    if key in checked:
        continue
    checked.add(key)
    length, wps = straight_ahead(wp)
    if length >= 150.0:
        results.append((length, wp, wps))

results.sort(key=lambda r: -r[0])
print("Nayden " + str(len(results)) + " uchastkov >= 150m\n")

print("TOP 5 pryamykh uchastkov:")
for i, (length, wp, wps) in enumerate(results[:5]):
    start = wp.transform
    end   = wps[-1].transform
    print("  [" + str(i) + "] " + str(round(length)) + "m" +
          "  start: x=" + str(round(start.location.x,1)) +
          " y=" + str(round(start.location.y,1)) +
          " yaw=" + str(round(start.rotation.yaw,1)) +
          "  end: x=" + str(round(end.location.x,1)) +
          " y=" + str(round(end.location.y,1)) +
          "  road_id=" + str(wp.road_id) +
          " lane_id=" + str(wp.lane_id))

# Берём лучший участок и выводим точки для ego и obstacle
if results:
    best_len, best_wp, best_wps = results[0]
    print("\nLUCHSHIY UCHASTOK: " + str(round(best_len)) + "m")
    print("EGO start:")
    t = best_wps[0].transform
    print("  x=" + str(round(t.location.x,3)) +
          " y=" + str(round(t.location.y,3)) +
          " z=" + str(round(t.location.z+0.3,3)) +
          " yaw=" + str(round(t.rotation.yaw,3)))

    # Obstacle — 80м вперёд (16 шагов по 5м)
    obs_idx = min(16, len(best_wps)-1)
    t2 = best_wps[obs_idx].transform
    print("OBSTACLE (80m vperyod):")
    print("  x=" + str(round(t2.location.x,3)) +
          " y=" + str(round(t2.location.y,3)) +
          " z=" + str(round(t2.location.z+0.3,3)) +
          " yaw=" + str(round(t2.rotation.yaw,3)))

    print("\nSKOPIRUYTE eti koordinaty v fix_and_run.py")
