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
import matplotlib.gridspec as gridspec
from collections import deque
import numpy as np
import serial


PACKET_SIZE = 35
PORT = '/dev/ttyACM0'
BAUD = 921600

# ── Shared state ──────────────────────────────────────────────────────────────
latest_q = [1.0, 0.0, 0.0, 0.0]
latest_a = [0.0, 0.0, 0.0]
q_lock = threading.Lock()

q_ref = [1.0, 0.0, 0.0, 0.0]
calibrated = False
recording = False

BUFFER_SIZE = 1000
t_buf  = deque(maxlen=BUFFER_SIZE)
qw_buf = deque(maxlen=BUFFER_SIZE)
qi_buf = deque(maxlen=BUFFER_SIZE)
qj_buf = deque(maxlen=BUFFER_SIZE)
qk_buf = deque(maxlen=BUFFER_SIZE)
ax_buf = deque(maxlen=BUFFER_SIZE)
ay_buf = deque(maxlen=BUFFER_SIZE)
az_buf = deque(maxlen=BUFFER_SIZE)
buf_lock = threading.Lock()

session_t  = []
session_qw = []
session_qi = []
session_qj = []
session_qk = []
session_ax = []
session_ay = []
session_az = []
session_lock = threading.Lock()

script_dir = os.path.dirname(os.path.abspath(__file__))
save_dir = os.path.join(script_dir, 'imu_recordings')
os.makedirs(save_dir, exist_ok=True)
csv_file   = None
csv_writer = None

# ── Quaternion math ───────────────────────────────────────────────────────────
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

def serial_receiver():
    ser = serial.Serial(PORT, BAUD, timeout=1)

    def find_header():
        while True:
            try:
                b1 = ser.read(1)
                if b1 and b1[0] == 0xAA:
                    b2 = ser.read(1)
                    if b2 and b2[0] == 0xFF:
                        return True
            except Exception:
                pass

    find_header()
    while True:
        try:
            raw = ser.read(33)  # 35 - 2 header bytes
            if len(raw) < 33:
                find_header()
                continue

            board_id = raw[0]
            ts, qw, qi, qj, qk, ax, ay, az = struct.unpack_from('<Ifffffff', raw, 1)

            mag = math.sqrt(qw**2 + qi**2 + qj**2 + qk**2)
            if not (0.9 < mag < 1.1):
                find_header()
                continue

            with q_lock:
                latest_q[:] = [qw, qi, qj, qk]
                latest_a[:] = [ax, ay, az]

            t_sec = ts / 1_000_000

            with buf_lock:
                t_buf.append(t_sec)
                qw_buf.append(qw); qi_buf.append(qi)
                qj_buf.append(qj); qk_buf.append(qk)
                ax_buf.append(ax); ay_buf.append(ay)
                az_buf.append(az)

            if recording:
                with session_lock:
                    session_t.append(t_sec)
                    session_qw.append(qw); session_qi.append(qi)
                    session_qj.append(qj); session_qk.append(qk)
                    session_ax.append(ax); session_ay.append(ay)
                    session_az.append(az)
                if csv_writer:
                    csv_writer.writerow([board_id, ts, qw, qi, qj, qk, ax, ay, az])
                    csv_file.flush()

            find_header()

        except Exception as e:
            print(f"Serial error: {e}")
            find_header()
            
                        
# ── Session graph ─────────────────────────────────────────────────────────────
def show_session_graph():
    with session_lock:
        if len(session_t) < 2:
            print("Not enough data to plot")
            return
        t  = np.array(session_t)
        qw = np.array(session_qw)
        qi = np.array(session_qi)
        qj = np.array(session_qj)
        qk = np.array(session_qk)
        ax = np.array(session_ax)
        ay = np.array(session_ay)
        az = np.array(session_az)

    t = t - t[0]
    roll  = np.degrees(np.arctan2(2*(qw*qi + qj*qk), 1 - 2*(qi**2 + qj**2)))
    pitch = np.degrees(np.arcsin(np.clip(2*(qw*qj - qk*qi), -1, 1)))
    yaw   = np.degrees(np.arctan2(2*(qw*qk + qi*qj), 1 - 2*(qj**2 + qk**2)))
    a_mag = np.sqrt(ax**2 + ay**2 + az**2)

    plt.style.use('dark_background')
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(f'IMU Session — {t[-1]:.1f}s  ({len(t)} samples)',
                 fontsize=14, color='white', y=0.98)
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(t, roll,  color='#FF6B6B', linewidth=0.8, label='Roll')
    ax1.plot(t, pitch, color='#4ECDC4', linewidth=0.8, label='Pitch')
    ax1.plot(t, yaw,   color='#FFE66D', linewidth=0.8, label='Yaw')
    ax1.set_title('Orientation — Euler Angles', color='white')
    ax1.set_ylabel('degrees'); ax1.set_xlabel('time (s)')
    ax1.legend(loc='upper right'); ax1.set_xlim(0, t[-1])
    ax1.grid(alpha=0.15); ax1.axhline(0, color='white', linewidth=0.3, alpha=0.3)

    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(t, qw, color='#FF6B6B', linewidth=0.8, label='w')
    ax2.plot(t, qi, color='#4ECDC4', linewidth=0.8, label='i')
    ax2.plot(t, qj, color='#FFE66D', linewidth=0.8, label='j')
    ax2.plot(t, qk, color='#A8E6CF', linewidth=0.8, label='k')
    ax2.set_title('Raw Quaternion', color='white')
    ax2.set_ylabel('value'); ax2.set_xlabel('time (s)')
    ax2.set_ylim(-1.1, 1.1); ax2.legend(loc='upper right', fontsize=8)
    ax2.set_xlim(0, t[-1]); ax2.grid(alpha=0.15)

    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(t, ax, color='#FF6B6B', linewidth=0.8, label='x')
    ax3.plot(t, ay, color='#4ECDC4', linewidth=0.8, label='y')
    ax3.plot(t, az, color='#FFE66D', linewidth=0.8, label='z')
    ax3.set_title('Linear Acceleration', color='white')
    ax3.set_ylabel('m/s²'); ax3.set_xlabel('time (s)')
    ax3.legend(loc='upper right', fontsize=8)
    ax3.set_xlim(0, t[-1]); ax3.grid(alpha=0.15)
    ax3.axhline(0, color='white', linewidth=0.3, alpha=0.3)

    ax4 = fig.add_subplot(gs[2, 0])
    ax4.plot(t, a_mag, color='#FF9F43', linewidth=0.8)
    ax4.fill_between(t, 0, a_mag, alpha=0.2, color='#FF9F43')
    ax4.set_title('Acceleration Magnitude', color='white')
    ax4.set_ylabel('m/s²'); ax4.set_xlabel('time (s)')
    ax4.set_xlim(0, t[-1]); ax4.set_ylim(0); ax4.grid(alpha=0.15)

    ax5 = fig.add_subplot(gs[2, 1])
    q_mag = np.sqrt(qw**2 + qi**2 + qj**2 + qk**2)
    ax5.plot(t, q_mag, color='#A29BFE', linewidth=0.8)
    ax5.axhline(1.0, color='white', linewidth=0.5, linestyle='--', alpha=0.5, label='ideal = 1.0')
    ax5.set_title('Quaternion Magnitude (health check)', color='white')
    ax5.set_ylabel('magnitude'); ax5.set_xlabel('time (s)')
    ax5.set_xlim(0, t[-1]); ax5.set_ylim(0.95, 1.05)
    ax5.legend(fontsize=8); ax5.grid(alpha=0.15)

    plt.savefig(
        os.path.join(save_dir, f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"),
        dpi=150, bbox_inches='tight', facecolor='#1a1a1a'
    )
    plt.show()

# ── Live graph ────────────────────────────────────────────────────────────────
def run_live_graph():
    plt.style.use('dark_background')
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
    fig.suptitle('IMU Live — last 5 seconds', color='white')

    ax1.set_title('Quaternion', color='white')
    ax1.set_ylim(-1.1, 1.1); ax1.set_ylabel('value')
    ax2.set_title('Linear Acceleration (m/s²)', color='white')
    ax2.set_ylabel('m/s²')

    lw, = ax1.plot([], [], color='#FF6B6B', label='w', linewidth=0.9)
    li, = ax1.plot([], [], color='#4ECDC4', label='i', linewidth=0.9)
    lj, = ax1.plot([], [], color='#FFE66D', label='j', linewidth=0.9)
    lk, = ax1.plot([], [], color='#A8E6CF', label='k', linewidth=0.9)
    lx, = ax2.plot([], [], color='#FF6B6B', label='x', linewidth=0.9)
    ly, = ax2.plot([], [], color='#4ECDC4', label='y', linewidth=0.9)
    lz, = ax2.plot([], [], color='#FFE66D', label='z', linewidth=0.9)

    ax1.legend(loc='upper right', fontsize=8); ax2.legend(loc='upper right', fontsize=8)
    ax1.grid(alpha=0.15); ax2.grid(alpha=0.15)

    def update(frame):
        with buf_lock:
            if len(t_buf) < 2:
                return
            t  = np.array(t_buf)
            w  = np.array(qw_buf); i = np.array(qi_buf)
            j  = np.array(qj_buf); k = np.array(qk_buf)
            xa = np.array(ax_buf); ya = np.array(ay_buf); za = np.array(az_buf)

        t = t - t[-1]
        lw.set_data(t, w); li.set_data(t, i)
        lj.set_data(t, j); lk.set_data(t, k)
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

# ── OpenGL ────────────────────────────────────────────────────────────────────
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

def draw_box(qw, qi, qj, qk):
    angle = 2 * math.acos(max(-1.0, min(1.0, qw)))
    mag = math.sqrt(qi**2 + qj**2 + qk**2)
    bx, by, bz = (qi/mag, qj/mag, qk/mag) if mag > 0.0001 else (0, 1, 0)
    if mag <= 0.0001: angle = 0
    glPushMatrix()
    glRotatef(math.degrees(angle), bx, by, bz)
    glBegin(GL_QUADS)
    for fi, color in faces:
        glColor3f(*color)
        for i in fi: glVertex3f(*vertices[i])
    glEnd()
    glColor3f(0,0,0)
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

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    global q_ref, calibrated, recording, csv_file, csv_writer

    threading.Thread(target=serial_receiver, daemon=True).start()
    threading.Thread(target=run_live_graph, daemon=True).start()

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
    print("  C     — calibrate (hold T-pose first)")
    print("  R     — reset calibration")
    print("  SPACE — start / stop recording")
    print("  G     — show full session graph (after recording)")
    print("  Escape — quit")

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                return
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.quit()
                    return

                if event.key == K_c:
                    with q_lock:
                        q_ref = list(latest_q)
                    calibrated = True
                    print(f"Calibrated — w={q_ref[0]:+.4f} i={q_ref[1]:+.4f} j={q_ref[2]:+.4f} k={q_ref[3]:+.4f}")

                if event.key == K_r:
                    q_ref = [1.0, 0.0, 0.0, 0.0]
                    calibrated = False
                    print("Calibration reset")

                if event.key == K_SPACE:
                    if not recording:
                        with session_lock:
                            session_t.clear(); session_qw.clear()
                            session_qi.clear(); session_qj.clear()
                            session_qk.clear(); session_ax.clear()
                            session_ay.clear(); session_az.clear()
                        fname = os.path.join(save_dir, f"imu_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
                        csv_file = open(fname, 'w', newline='')
                        csv_writer = csv.writer(csv_file)
                        csv_writer.writerow(['board_id','timestamp_us','qw','qi','qj','qk','ax','ay','az'])
                        recording = True
                        print(f"● Recording started → {fname}")
                    else:
                        recording = False
                        if csv_file:
                            csv_file.close()
                            csv_file = None
                            csv_writer = None
                        print("○ Recording stopped — press G to view session graph")

                if event.key == K_g:
                    if not recording:
                        threading.Thread(target=show_session_graph, daemon=True).start()
                    else:
                        print("Stop recording first (Space), then press G")

        with q_lock:
            qw, qi, qj, qk = latest_q
            ax, ay, az = latest_a

        if calibrated:
            qw, qi, qj, qk = apply_calibration([qw, qi, qj, qk], q_ref)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0, -4)
        draw_axes()
        draw_box(qw, qi, qj, qk)

        rec = "● REC" if recording else "○"
        cal = "CAL" if calibrated else "RAW"
        pygame.display.set_caption(
            f"{rec}  {cal}  "
            f"w={qw:+.3f} i={qi:+.3f} j={qj:+.3f} k={qk:+.3f}  "
            f"a=({ax:+.2f},{ay:+.2f},{az:+.2f})"
        )

        pygame.display.flip()
        clock.tick(60)

if __name__ == '__main__':
    main()