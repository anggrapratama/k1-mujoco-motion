"""
my_motion.py - Final Challenge (KPP)
====================================
Judul gerakan: "Salam Sapa Sambil Mengangguk"

Dasar: solutions/04_humanoid_wave.py (rumus PD tidak diubah).
    Run:  uv run python my_motion.py
"""

import time

import mujoco
import mujoco.viewer
import numpy as np


# ======================================================================
# 1. Load robot K1 (sama seperti 04, file model TIDAK diubah)
# ======================================================================
spec = mujoco.MjSpec.from_file("models/k1/K1_22dof.xml")
for j in [j for j in spec.joints if j.type == mujoco.mjtJoint.mjJNT_FREE]:
    spec.delete(j)
model = spec.compile()
data = mujoco.MjData(model)

# [TAMBAHAN] Cetak daftar 22 nama actuator untuk dicek (beri # jika tidak perlu)
for i in range(model.nu):
    print(i, model.actuator(i).name)


# ======================================================================
# 2. Lookup joint berdasarkan nama (sama seperti 04)
# ======================================================================
names = [model.actuator(i).name for i in range(model.nu)]
idx = {n: i for i, n in enumerate(names)}
joint = [model.actuator_trnid[i, 0] for i in range(model.nu)]
qadr = [model.jnt_qposadr[j] for j in joint]
vadr = [model.jnt_dofadr[j] for j in joint]

KP = np.full(model.nu, 30.0)   # gain sama dengan 04
KD = np.full(model.nu, 1.5)

# [TAMBAHAN] Batas sudut tiap joint, dipakai agar target selalu aman
lo = np.array([model.jnt_range[j, 0] for j in joint])
hi = np.array([model.jnt_range[j, 1] for j in joint])
limited = np.array([model.jnt_limited[j] for j in joint], dtype=bool)


def find(*keywords):
    """[TAMBAHAN] Cari nama actuator yang memuat semua kata kunci (huruf kecil).
    Berguna untuk joint kepala, karena penulisan namanya bisa berbeda."""
    for n in names:
        if all(k in n.lower() for k in keywords):
            return n
    return None


HEAD_PITCH = find("head", "pitch")   # anggukan
HEAD_YAW = find("head", "yaw")       # menoleh
print("Joint kepala terdeteksi:", HEAD_PITCH, "|", HEAD_YAW)


# ======================================================================
# 3. Gerakan: target sudut semua joint pada waktu t
# ======================================================================
# Jadwal (total ~6 detik):
#   0.0 - 1.2 s : lengan kanan terangkat halus (ramp)
#   1.2 - 4.2 s : lambaian sinus + kepala mengangguk + menoleh pelan
#   4.2 - 5.5 s : lengan kanan turun perlahan, gelombang mengecil
#   > 5.5 s     : robot diam di pose akhir

T_UP = 1.2      # detik selesai mengangkat
T_DOWN = 4.2    # detik mulai menurunkan
T_END = 5.5     # detik selesai menurunkan


def smooth(x):
    """[TAMBAHAN] Smoothstep 0->1, mulus di awal dan akhir (tanpa hentakan)."""
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def target_angles(t):
    up = smooth(t / T_UP)                       # 0 -> 1 saat lengan naik
    down = smooth((t - T_DOWN) / (T_END - T_DOWN))   # 0 -> 1 saat lengan turun
    env = smooth((t - T_UP) / 0.5) * (1 - down)      # amplitudo gelombang (fade in/out)
    tw = t - T_UP                                    # waktu sejak mulai melambai

    target = np.zeros(model.nu)

    # Lengan kiri: tetap turun (sama seperti 04)
    target[idx["left_shoulder_roll_joint"]] = -1.3 * up

    # [DIUBAH] Lengan kanan: naik dengan smoothstep, lalu turun di akhir.
    # Saat turun, roll menuju +1.3 (kebalikan dari -1.3 di sisi kiri).
    target[idx["right_shoulder_roll_joint"]] = -1.4 * up * (1 - down) + 1.3 * down
    target[idx["right_elbow_pitch_joint"]] = -0.9 * up * (1 - down)

    # [DIUBAH] Lambaian: sinus yang amplitudonya dikalikan env (fade in/out)
    target[idx["right_elbow_yaw_joint"]] = env * (0.3 + 0.6 * np.sin(5 * tw))

    # [BARU] Kepala mengangguk (sinus pitch) mengikuti irama lambaian
    if HEAD_PITCH:
        target[idx[HEAD_PITCH]] = env * 0.25 * np.sin(3 * tw)

    # [BARU] Kepala menoleh pelan ke arah tangan yang melambai
    if HEAD_YAW:
        target[idx[HEAD_YAW]] = env * 0.3 * np.sin(1.5 * tw)

    # [BARU] Jaga semua target tetap di dalam batas joint
    return np.where(limited, np.clip(target, lo, hi), target)


# ======================================================================
# 4. Control loop (PD dan viewer sama seperti 04)
# ======================================================================
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        t0 = time.time()

        q = data.qpos[qadr]
        qvel = data.qvel[vadr]

        # Rumus PD tidak diubah
        data.ctrl[:] = KP * (target_angles(data.time) - q) - KD * qvel

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(max(0, model.opt.timestep - (time.time() - t0)))