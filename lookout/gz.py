"""Small helpers shared by the lookout scripts (run inside the container).

Gazebo (Ignition Fortress) is driven through the `ign service` CLI: set_pose,
world control (pause / multi-step), entity removal. Lidar clouds arrive over
ros_gz_bridge as PointCloud2 and are decoded to per-beam arrays here.
"""
import math
import os
import subprocess
import time

import numpy as np


SVC_RETRIES = []        # (service, request, error) of every call that had to be repeated


def _svc(world, name, reqtype, req, timeout_ms=10000, reptype="ignition.msgs.Boolean", tries=4):
    """One `ign service` call, repeated on failure. A reply can be lost in
    transport ("Host unreachable" in the server log, once in ~400 calls in
    the walk-driver test); every call used here is safe to repeat (set_pose
    is idempotent; an extra step of an unchanged scene only renders it again,
    which scans.step_fresh absorbs)."""
    cmd = ["ign", "service", "-s", f"/world/{world}/{name}", "--reqtype", reqtype,
           "--reptype", reptype, "--timeout", str(timeout_ms), "--req", req]
    err = None
    for i in range(tries):
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_ms / 1000 + 10)
            if "data: true" in out.stdout:
                return
            err = f"{out.stdout.strip()} {out.stderr.strip()}"
        except subprocess.TimeoutExpired as e:
            err = f"subprocess timeout {e}"
        SVC_RETRIES.append((name, req, err))
        print(f"[gz] ign service {name} failed (try {i + 1}/{tries}): {req!r} -> {err}", flush=True)
        time.sleep(1.0)
    raise RuntimeError(f"ign service {name} failed {tries} times: {req!r} -> {err}")


def quat_z(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


GZSVC = "/runs/lookout/bin/gzsvc"     # persistent client (lookout/gzsvc.cc); CLI fallback
_client = None


def _fast(line):
    """Send one request to the persistent client; False if it is not built."""
    global _client
    if _client is None:
        if not os.path.exists(GZSVC):
            return False
        _client = subprocess.Popen([GZSVC], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
    _client.stdin.write(line + "\n")
    _client.stdin.flush()
    rep = _client.stdout.readline().split()
    if not rep:
        raise RuntimeError(f"gzsvc died on {line!r}")
    tries = int(rep[1])
    if tries > 1 or rep[0] != "ok":
        SVC_RETRIES.append((line.split()[0], line, " ".join(rep)))
        print(f"[gz] gzsvc {line!r} -> {' '.join(rep)}", flush=True)
    if rep[0] != "ok":
        raise RuntimeError(f"gzsvc failed: {line!r} -> {' '.join(rep)}")
    return True


def sim_time(world, timeout=30):
    """(sim time in s, paused) from one /world/<world>/stats message. The
    text format leaves out zero fields, so sec or nsec may be missing."""
    import re
    out = subprocess.run(["ign", "topic", "-e", "-t", f"/world/{world}/stats", "-n", "1"],
                         capture_output=True, text=True, timeout=timeout).stdout
    m = re.search(r"sim_time\s*\{([^}]*)\}", out)
    if not m:
        raise RuntimeError(f"no sim_time in /world/{world}/stats: {out[:300]!r}")
    sec = re.search(r"\bsec:\s*(\d+)", m.group(1))
    nsec = re.search(r"\bnsec:\s*(\d+)", m.group(1))
    t = (int(sec.group(1)) if sec else 0) + (int(nsec.group(1)) if nsec else 0) * 1e-9
    return t, bool(re.search(r"paused:\s*true", out))


def set_pose(world, model, x, y, z, yaw):
    qx, qy, qz, qw = quat_z(yaw)
    if _fast(f"pose {world} {model} {x:.6f} {y:.6f} {z:.6f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}"):
        return
    _svc(world, "set_pose", "ignition.msgs.Pose",
         f'name: "{model}", position: {{x: {x:.6f}, y: {y:.6f}, z: {z:.6f}}}, '
         f"orientation: {{x: {qx:.9f}, y: {qy:.9f}, z: {qz:.9f}, w: {qw:.9f}}}")


def pause(world, on=True):
    if _fast(f"pause {world} {1 if on else 0}"):
        return
    _svc(world, "control", "ignition.msgs.WorldControl", f"pause: {'true' if on else 'false'}")


def step(world, n):
    if _fast(f"step {world} {int(n)}"):
        return
    _svc(world, "control", "ignition.msgs.WorldControl", f"pause: true, multi_step: {int(n)}")


def remove_model(world, model):
    if _fast(f"remove {world} {model}"):
        return
    # ignition.msgs.Entity type 2 = MODEL
    _svc(world, "remove", "ignition.msgs.Entity", f'name: "{model}", type: 2')


# ------------------------------------------------------------------ clouds --
_DT = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16, 5: np.int32, 6: np.uint32,
       7: np.float32, 8: np.float64}


def decode(msg):
    """PointCloud2 -> dict of (H, W) arrays: x, y, z (sensor frame), ring (if
    present), range. The gpu_lidar cloud is organised: row = channel (ring),
    column = azimuth sample; a no-return beam is +/-inf."""
    n = msg.width * msg.height
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
    out = {}
    for f in msg.fields:
        dt = np.dtype(_DT[f.datatype]).newbyteorder(">" if msg.is_bigendian else "<")
        col = buf[:, f.offset:f.offset + dt.itemsize].copy().view(dt)[:, 0]
        out[f.name] = col.reshape(msg.height, msg.width)
    x, y, z = (out[k].astype(np.float64) for k in "xyz")
    with np.errstate(invalid="ignore"):
        out["range"] = np.sqrt(x * x + y * y + z * z)
    return out


def stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_to_R(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


# ------------------------------------------------------------- model info --
def _pose_after(lines, i):
    """Parse the two lines after a 'Pose [ XYZ (m) ] [ RPY (rad) ]:' header."""
    xyz = [float(v) for v in lines[i + 1].strip().strip("[]").split()]
    rpy = [float(v) for v in lines[i + 2].strip().strip("[]").split()]
    return xyz, rpy


def model_info(model, sensor=None, timeout=600, tries=4):
    """World pose of `model` and (optionally) the pose of its sensor
    `sensor` relative to the model, as reported by Gazebo (`ign model -m`).
    Returns ((xyz, rpy), (xyz, rpy) or None). The sensor pose `ign model`
    prints is relative to its link; every lookout/mulcher link sits at the
    model origin (checked here: the link pose must be identity). The CLI
    rebuilds the whole world state (~680 oak meshes): 44-72 s with two sims
    on the host (2026-09-24 pilots), so the old 60 s timeout was too tight.
    Repeated on a timeout or a reply with no pose, as _svc is (a read)."""
    for k in range(tries):
        try:
            out = subprocess.run(["ign", "model", "-m", model], capture_output=True, text=True,
                                 timeout=timeout).stdout
            err = f"no pose in output: {out[:300]!r}"
        except subprocess.TimeoutExpired as e:
            out, err = "", f"subprocess timeout {e}"
        lines = out.splitlines()
        hdr = [i for i, l in enumerate(lines) if "Pose [ XYZ (m) ] [ RPY (rad) ]" in l]
        if hdr:
            break
        SVC_RETRIES.append(("model_info", model, err))
        print(f"[gz] ign model -m {model} failed (try {k + 1}/{tries}): {err}", flush=True)
        time.sleep(1.0)
    else:
        raise RuntimeError(f"ign model -m {model} failed {tries} times: {err}")
    model_pose = _pose_after(lines, hdr[0])
    if sensor is None:
        return model_pose, None
    names = [i for i, l in enumerate(lines) if l.strip() == f"- Name: {sensor}"]
    if not names:
        raise RuntimeError(f"ign model -m {model}: no sensor {sensor}")
    # the sensor's parent link: the last '- Link' block before the sensor
    link_hdr = [i for i in hdr if i < names[0]]
    link_pose = _pose_after(lines, link_hdr[-1]) if len(link_hdr) >= 2 else ([0, 0, 0], [0, 0, 0])
    if max(abs(v) for v in link_pose[0] + link_pose[1]) > 1e-6:
        raise RuntimeError(f"{model}: sensor link is not at the model origin: {link_pose}")
    s_hdr = [i for i in hdr if i > names[0]][0]
    return model_pose, _pose_after(lines, s_hdr)


def compose(model_pose, sensor_pose):
    """World (R, t) of a sensor from the model's world pose and the sensor's
    pose in the model, each ((x, y, z), (roll, pitch, yaw))."""
    Rm = rpy_to_R(*model_pose[1]); tm = np.array(model_pose[0], dtype=float)
    Rs = rpy_to_R(*sensor_pose[1]); ts = np.array(sensor_pose[0], dtype=float)
    return Rm @ Rs, tm + Rm @ ts


def odom_sensor(odom, sensor_pose):
    """World (R, t) of a sensor from a ground-truth Odometry of its model."""
    p, q = odom.pose.pose.position, odom.pose.pose.orientation
    Rm = quat_to_R(q.x, q.y, q.z, q.w); tm = np.array([p.x, p.y, p.z])
    Rs = rpy_to_R(*sensor_pose[1]); ts = np.array(sensor_pose[0], dtype=float)
    return Rm @ Rs, tm + Rm @ ts


def to_world(d, R, t):
    """(H, W, 3) world points from a decoded cloud (sensor frame)."""
    P = np.stack([d["x"], d["y"], d["z"]], -1).astype(np.float64)
    with np.errstate(invalid="ignore"):
        return P @ R.T + t
