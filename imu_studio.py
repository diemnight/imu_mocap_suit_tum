import serial
import serial.tools.list_ports
import struct
import math
import threading
import os
import csv
from datetime import datetime
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np


PACKET_SIZE = 35
NUM_IMUS    = 6
IMU_IDS     = list(range(1, NUM_IMUS + 1))  # 1..6

BAUD_RATE   = 921600

# ── Per-IMU shared state ───────────────────────────────────────────────────────
latest_q = [[1.0, 0.0, 0.0, 0.0] for _ in range(NUM_IMUS)]
latest_a = [[0.0, 0.0, 0.0]      for _ in range(NUM_IMUS)]
q_lock   = threading.Lock()

q_ref      = [[1.0, 0.0, 0.0, 0.0] for _ in range(NUM_IMUS)]
calibrated = [False] * NUM_IMUS
active_imu = 0
recording  = False

BUFFER_SIZE = 1000
t_buf  = [deque(maxlen=BUFFER_SIZE) for _ in range(NUM_IMUS)]
qw_buf = [deque(maxlen=BUFFER_SIZE) for _ in range(NUM_IMUS)]
qi_buf = [deque(maxlen=BUFFER_SIZE) for _ in range(NUM_IMUS)]
qj_buf = [deque(maxlen=BUFFER_SIZE) for _ in range(NUM_IMUS)]
qk_buf = [deque(maxlen=BUFFER_SIZE) for _ in range(NUM_IMUS)]
ax_buf = [deque(maxlen=BUFFER_SIZE) for _ in range(NUM_IMUS)]
ay_buf = [deque(maxlen=BUFFER_SIZE) for _ in range(NUM_IMUS)]
az_buf = [deque(maxlen=BUFFER_SIZE) for _ in range(NUM_IMUS)]
buf_lock = threading.Lock()

session_t  = [[] for _ in range(NUM_IMUS)]
session_qw = [[] for _ in range(NUM_IMUS)]
session_qi = [[] for _ in range(NUM_IMUS)]
session_qj = [[] for _ in range(NUM_IMUS)]
session_qk = [[] for _ in range(NUM_IMUS)]
session_ax = [[] for _ in range(NUM_IMUS)]
session_ay = [[] for _ in range(NUM_IMUS)]
session_az = [[] for _ in range(NUM_IMUS)]
session_lock = threading.Lock()

script_dir = os.path.dirname(os.path.abspath(__file__))
save_dir   = os.path.join(script_dir, 'imu_recordings')
os.makedirs(save_dir, exist_ok=True)
csv_file   = None
csv_writer = None

# ── Auto-detect ESP32 serial port ─────────────────────────────────────────────
def find_serial_port():
    # ports = serial.tools.list_ports.comports()
    # for p in ports:
    #     desc = (p.description or '').lower()
    #     if any(x in desc for x in ['cp210', 'ch340', 'usb serial', 'uart', 'esp']):
    #         return p.device
    # # fallback — just return first available port
    # if ports:
    #     return ports[0].device
    # return None
    return '/dev/ttyACM0'

# ── Quaternion math ────────────────────────────────────────────────────────────
def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ]

def quat_inverse(q):
    return [q[0], -q[1], -q[2], -q[3]]

def apply_calibration(q, ref):
    return quat_multiply(quat_inverse(ref), q)

# ── Serial receiver (background thread) ───────────────────────────────────────
def serial_receiver(port):
    print(f"Opening serial port {port} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1.0)
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return

    print("Serial connected — waiting for packets...")
    buf = bytearray()

    while True:
        try:
            chunk = ser.read(256)
            if not chunk:
                continue
            buf.extend(chunk)

            # Scan for valid packet headers
            while len(buf) >= PACKET_SIZE:
                # Find 0xAA 0xFF marker
                idx = -1
                for i in range(len(buf) - 1):
                    if buf[i] == 0xAA and buf[i+1] == 0xFF:
                        idx = i
                        break

                if idx == -1:
                    # No header found — discard all but last byte
                    buf = buf[-1:]
                    break

                if idx > 0:
                    # Discard bytes before header
                    buf = buf[idx:]

                if len(buf) < PACKET_SIZE:
                    break

                # Extract packet
                packet = buf[:PACKET_SIZE]
                buf = buf[PACKET_SIZE:]

                board_id = packet[2]
                if board_id not in IMU_IDS:
                    continue

                idx2 = board_id - 1

                ts, qw, qi, qj, qk, ax, ay, az = struct.unpack_from('<Ifffffff', packet, 3)

                mag = math.sqrt(qw**2 + qi**2 + qj**2 + qk**2)
                if not (0.9 < mag < 1.1):
                    continue
                print(f"IMU {board_id} qw={qw:.3f} qi={qi:.3f} qj={qj:.3f} qk={qk:.3f}")

                with q_lock:
                    latest_q[idx2][:] = [qw, qi, qj, qk]
                    latest_a[idx2][:] = [ax, ay, az]

                t_sec = ts / 1_000_000

                with buf_lock:
                    t_buf[idx2].append(t_sec)
                    qw_buf[idx2].append(qw); qi_buf[idx2].append(qi)
                    qj_buf[idx2].append(qj); qk_buf[idx2].append(qk)
                    ax_buf[idx2].append(ax); ay_buf[idx2].append(ay)
                    az_buf[idx2].append(az)

                if recording:
                    with session_lock:
                        session_t[idx2].append(t_sec)
                        session_qw[idx2].append(qw); session_qi[idx2].append(qi)
                        session_qj[idx2].append(qj); session_qk[idx2].append(qk)
                        session_ax[idx2].append(ax); session_ay[idx2].append(ay)
                        session_az[idx2].append(az)
                    if csv_writer:
                        csv_writer.writerow([board_id, ts, qw, qi, qj, qk, ax, ay, az])
                        csv_file.flush()

        except Exception as e:
            print(f"Serial error: {e}")
            break

# ── Session graph ──────────────────────────────────────────────────────────────
def show_session_graph():
    with session_lock:
        any_data = any(len(session_t[i]) >= 2 for i in range(NUM_IMUS))
        if not any_data:
            print("Not enough data to plot")
            return
        snaps = [(
            np.array(session_t[i]),
            np.array(session_qw[i]), np.array(session_qi[i]),
            np.array(session_qj[i]), np.array(session_qk[i]),
            np.array(session_ax[i]), np.array(session_ay[i]),
            np.array(session_az[i]),
        ) for i in range(NUM_IMUS)]

    plt.style.use('dark_background')
    fig, axes = plt.subplots(NUM_IMUS, 2, figsize=(14, 3 * NUM_IMUS))
    fig.suptitle('IMU Session — all sensors', fontsize=14, color='white', y=1.01)

    for i, (t, qw, qi, qj, qk, axs, ays, azs) in enumerate(snaps):
        if len(t) < 2:
            continue
        t = t - t[0]
        roll  = np.degrees(np.arctan2(2*(qw*qi + qj*qk), 1 - 2*(qi**2 + qj**2)))
        pitch = np.degrees(np.arcsin(np.clip(2*(qw*qj - qk*qi), -1, 1)))
        yaw   = np.degrees(np.arctan2(2*(qw*qk + qi*qj), 1 - 2*(qj**2 + qk**2)))

        ax_euler = axes[i][0]
        ax_euler.plot(t, roll,  color='#FF6B6B', linewidth=0.8, label='Roll')
        ax_euler.plot(t, pitch, color='#4ECDC4', linewidth=0.8, label='Pitch')
        ax_euler.plot(t, yaw,   color='#FFE66D', linewidth=0.8, label='Yaw')
        ax_euler.set_title(f'IMU {i+1} — Euler', color='white')
        ax_euler.set_ylabel('deg'); ax_euler.legend(fontsize=7)
        ax_euler.set_xlim(0, t[-1]); ax_euler.grid(alpha=0.15)

        ax_acc = axes[i][1]
        ax_acc.plot(t, axs, color='#FF6B6B', linewidth=0.8, label='x')
        ax_acc.plot(t, ays, color='#4ECDC4', linewidth=0.8, label='y')
        ax_acc.plot(t, azs, color='#FFE66D', linewidth=0.8, label='z')
        ax_acc.set_title(f'IMU {i+1} — Accel', color='white')
        ax_acc.set_ylabel('m/s²'); ax_acc.legend(fontsize=7)
        ax_acc.set_xlim(0, t[-1]); ax_acc.grid(alpha=0.15)

    plt.tight_layout()
    plt.savefig(
        os.path.join(save_dir, f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"),
        dpi=150, bbox_inches='tight', facecolor='#1a1a1a'
    )
    plt.show()

# ── Live graph ─────────────────────────────────────────────────────────────────
def run_live_graph():
    plt.style.use('dark_background')
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
    ax1.set_ylim(-1.1, 1.1); ax1.set_ylabel('value')
    ax2.set_ylabel('m/s²')

    lw, = ax1.plot([], [], color='#FF6B6B', label='w', linewidth=0.9)
    li, = ax1.plot([], [], color='#4ECDC4', label='i', linewidth=0.9)
    lj, = ax1.plot([], [], color='#FFE66D', label='j', linewidth=0.9)
    lk, = ax1.plot([], [], color='#A8E6CF', label='k', linewidth=0.9)
    lx, = ax2.plot([], [], color='#FF6B6B', label='x', linewidth=0.9)
    ly, = ax2.plot([], [], color='#4ECDC4', label='y', linewidth=0.9)
    lz, = ax2.plot([], [], color='#FFE66D', label='z', linewidth=0.9)

    ax1.legend(loc='upper right', fontsize=8)
    ax2.legend(loc='upper right', fontsize=8)
    ax1.grid(alpha=0.15); ax2.grid(alpha=0.15)

    def update(frame):
        idx = active_imu
        fig.suptitle(f'IMU Live — IMU {idx+1} (Tab to switch)', color='white')
        ax1.set_title('Quaternion', color='white')
        ax2.set_title('Linear Acceleration (m/s²)', color='white')

        with buf_lock:
            if len(t_buf[idx]) < 2:
                return
            t  = np.array(t_buf[idx])
            w  = np.array(qw_buf[idx]); ii = np.array(qi_buf[idx])
            j  = np.array(qj_buf[idx]); k  = np.array(qk_buf[idx])
            xa = np.array(ax_buf[idx]); ya = np.array(ay_buf[idx])
            za = np.array(az_buf[idx])

        t = t - t[-1]
        lw.set_data(t, w);  li.set_data(t, ii)
        lj.set_data(t, j);  lk.set_data(t, k)
        lx.set_data(t, xa); ly.set_data(t, ya); lz.set_data(t, za)
        ax1.set_xlim(t[0], 0); ax2.set_xlim(t[0], 0)
        if len(xa):
            margin = 0.5
            mn = min(xa.min(), ya.min(), za.min()) - margin
            mx = max(xa.max(), ya.max(), za.max()) + margin
            ax2.set_ylim(mn, mx)

    ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

# ── OpenGL ─────────────────────────────────────────────────────────────────────
vertices = [
    [ 1,  0.1,  0.5], [-1,  0.1,  0.5], [-1, -0.1,  0.5], [ 1, -0.1,  0.5],
    [ 1,  0.1, -0.5], [-1,  0.1, -0.5], [-1, -0.1, -0.5], [ 1, -0.1, -0.5],
]
faces = [
    ([0,1,2,3], (0.8,0.2,0.2)), ([4,5,6,7], (0.2,0.8,0.2)),
    ([0,3,7,4], (0.2,0.2,0.8)), ([1,2,6,5], (0.8,0.8,0.2)),
    ([0,1,5,4], (0.8,0.5,0.2)), ([3,2,6,7], (0.5,0.2,0.8)),
]
box_edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]

def draw_box(qw_, qi_, qj_, qk_):
    angle = 2 * math.acos(max(-1.0, min(1.0, qw_)))
    mag   = math.sqrt(qi_**2 + qj_**2 + qk_**2)
    bx, by, bz = (qi_/mag, qj_/mag, qk_/mag) if mag > 0.0001 else (0, 1, 0)
    if mag <= 0.0001: angle = 0
    glPushMatrix()
    glRotatef(math.degrees(angle), bx, by, bz)
    glBegin(GL_QUADS)
    for fi, color in faces:
        glColor3f(*color)
        for vi in fi: glVertex3f(*vertices[vi])
    glEnd()
    glColor3f(0, 0, 0)
    glBegin(GL_LINES)
    for a, b in box_edges:
        glVertex3f(*vertices[a]); glVertex3f(*vertices[b])
    glEnd()
    glPopMatrix()

def draw_axes():
    glBegin(GL_LINES)
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(2,0,0)
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,2,0)
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,2)
    glEnd()

# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    global q_ref, calibrated, recording, csv_file, csv_writer, active_imu

    # Find and open serial port
    port = find_serial_port()
    if port is None:
        print("ERROR: No serial port found. Plug in the ESP32 and try again.")
        return
    print(f"Found serial port: {port}")

    threading.Thread(target=serial_receiver, args=(port,), daemon=True).start()
    #threading.Thread(target=run_live_graph, daemon=True).start()

    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

    glEnable(GL_DEPTH_TEST)
    glClearColor(0.1, 0.1, 0.1, 1)
    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, display[0]/display[1], 0.1, 50.0)
    glMatrixMode(GL_MODELVIEW)

    clock = pygame.time.Clock()

    print("Controls:")
    print("  Tab    — cycle active IMU (viewport + live graph)")
    print("  C      — calibrate active IMU (hold still first)")
    print("  R      — reset calibration for active IMU")
    print("  A      — calibrate ALL IMUs at once")
    print("  SPACE  — start / stop recording")
    print("  G      — show session graph (after recording)")
    print("  Escape — quit")

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit(); return
            if event.type == KEYDOWN:

                if event.key == K_ESCAPE:
                    pygame.quit(); return

                if event.key == K_TAB:
                    active_imu = (active_imu + 1) % NUM_IMUS
                    print(f"Active IMU: {active_imu + 1}")

                if event.key == K_c:
                    with q_lock:
                        q_ref[active_imu] = list(latest_q[active_imu])
                    calibrated[active_imu] = True
                    print(f"IMU {active_imu+1} calibrated")

                if event.key == K_r:
                    q_ref[active_imu] = [1.0, 0.0, 0.0, 0.0]
                    calibrated[active_imu] = False
                    print(f"IMU {active_imu+1} calibration reset")

                if event.key == K_a:
                    with q_lock:
                        for i in range(NUM_IMUS):
                            q_ref[i] = list(latest_q[i])
                            calibrated[i] = True
                    print("All IMUs calibrated")

                if event.key == K_SPACE:
                    if not recording:
                        with session_lock:
                            for i in range(NUM_IMUS):
                                session_t[i].clear();  session_qw[i].clear()
                                session_qi[i].clear(); session_qj[i].clear()
                                session_qk[i].clear(); session_ax[i].clear()
                                session_ay[i].clear(); session_az[i].clear()
                        fname = os.path.join(save_dir, f"imu_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
                        csv_file   = open(fname, 'w', newline='')
                        csv_writer = csv.writer(csv_file)
                        csv_writer.writerow(['board_id','timestamp_us','qw','qi','qj','qk','ax','ay','az'])
                        recording = True
                        print(f"● Recording started → {fname}")
                    else:
                        recording = False
                        if csv_file:
                            csv_file.close()
                            csv_file = None; csv_writer = None
                        print("○ Recording stopped — press G to view session graph")

                if event.key == K_g:
                    if not recording:
                        threading.Thread(target=show_session_graph, daemon=True).start()
                    else:
                        print("Stop recording first (Space), then press G")

        idx = active_imu
        with q_lock:
            qw_, qi_, qj_, qk_ = latest_q[idx]
            ax_, ay_, az_       = latest_a[idx]

        if calibrated[idx]:
            qw_, qi_, qj_, qk_ = apply_calibration([qw_, qi_, qj_, qk_], q_ref[idx])

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0, -4)
        draw_axes()
        draw_box(qw_, qi_, qj_, qk_)

        rec = "● REC" if recording else "○"
        cal = "CAL" if calibrated[idx] else "RAW"
        pygame.display.set_caption(
            f"IMU {idx+1}/{NUM_IMUS}  {rec}  {cal}  "
            f"w={qw_:+.3f} i={qi_:+.3f} j={qj_:+.3f} k={qk_:+.3f}  "
            f"a=({ax_:+.2f},{ay_:+.2f},{az_:+.2f})"
        )

        pygame.display.flip()
        clock.tick(60)

if __name__ == '__main__':
    main()