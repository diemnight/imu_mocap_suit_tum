import serial
import struct
import math
import threading
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

PORT = '/dev/ttyACM0'
BAUD = 921600
HEADER_1 = 0xAA
HEADER_2 = 0xFF

PACKET_SIZE = 34

latest_q = [1.0, 0.0, 0.0, 0.0]
latest_a = [0.0, 0.0, 0.0]
q_lock = threading.Lock()

# Reference quaternion for calibration (starts as identity = no rotation)
q_ref = [1.0, 0.0, 0.0, 0.0]  # qw, qi, qj, qk
calibrated = False

vertices = [
    [ 1,  0.1,  0.5],
    [-1,  0.1,  0.5],
    [-1, -0.1,  0.5],
    [ 1, -0.1,  0.5],
    [ 1,  0.1, -0.5],
    [-1,  0.1, -0.5],
    [-1, -0.1, -0.5],
    [ 1, -0.1, -0.5],
]

faces = [
    ([0,1,2,3], (0.8, 0.2, 0.2)),
    ([4,5,6,7], (0.2, 0.8, 0.2)),
    ([0,3,7,4], (0.2, 0.2, 0.8)),
    ([1,2,6,5], (0.8, 0.8, 0.2)),
    ([0,1,5,4], (0.8, 0.5, 0.2)),
    ([3,2,6,7], (0.5, 0.2, 0.8)),
]

def quat_multiply(a, b):
    """Multiply two quaternions [w, x, y, z]"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ]

def quat_inverse(q):
    """Inverse of a unit quaternion — just flip x, y, z"""
    return [q[0], -q[1], -q[2], -q[3]]

def apply_calibration(q_current, q_ref):
    """Returns rotation relative to the calibration pose"""
    return quat_multiply(quat_inverse(q_ref), q_current)

def serial_reader():
    ser = serial.Serial(PORT, BAUD, timeout=1)

    def find_header():
        while True:
            b1 = ser.read(1)
            if b1 and b1[0] == HEADER_1:
                b2 = ser.read(1)
                if b2 and b2[0] == HEADER_2:
                    return True

    find_header()
    while True:
        try:
            raw = ser.read(32)
            if len(raw) < 32:
                find_header()
                continue
            ts, qw, qi, qj, qk, ax, ay, az = struct.unpack('<Ifffffff', raw)
            mag = math.sqrt(qw**2 + qi**2 + qj**2 + qk**2)
            if 0.9 < mag < 1.1:
                with q_lock:
                    latest_q[0] = qw
                    latest_q[1] = qi
                    latest_q[2] = qj
                    latest_q[3] = qk
                    latest_a[0] = ax
                    latest_a[1] = ay
                    latest_a[2] = az
            find_header()
        except Exception as e:
            print(f"Serial error: {e}")
            find_header()

def draw_box(qw, qi, qj, qk):
    angle = 2 * math.acos(max(-1.0, min(1.0, qw)))
    mag = math.sqrt(qi**2 + qj**2 + qk**2)
    if mag > 0.0001:
        ax, ay, az = qi/mag, qj/mag, qk/mag
    else:
        ax, ay, az = 0, 1, 0
        angle = 0

    glPushMatrix()
    glRotatef(math.degrees(angle), ax, ay, az)

    glBegin(GL_QUADS)
    for face_indices, color in faces:
        glColor3f(*color)
        for i in face_indices:
            glVertex3f(*vertices[i])
    glEnd()

    glColor3f(0, 0, 0)
    glBegin(GL_LINES)
    edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
    for a, b in edges:
        glVertex3f(*vertices[a])
        glVertex3f(*vertices[b])
    glEnd()

    glPopMatrix()

def draw_axes():
    glBegin(GL_LINES)
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(2,0,0)
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,2,0)
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,2)
    glEnd()

def main():
    global q_ref, calibrated

    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

    glEnable(GL_DEPTH_TEST)
    glClearColor(0.1, 0.1, 0.1, 1)

    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, display[0]/display[1], 0.1, 50.0)
    glMatrixMode(GL_MODELVIEW)

    clock = pygame.time.Clock()
    print("Press C to calibrate (hold T-pose first)")
    print("Press R to reset calibration")
    print("Press Escape to quit")

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
                    # Save current orientation as reference
                    with q_lock:
                        q_ref = list(latest_q)
                    calibrated = True
                    print(f"Calibrated! Reference: w={q_ref[0]:+.4f} i={q_ref[1]:+.4f} j={q_ref[2]:+.4f} k={q_ref[3]:+.4f}")

                if event.key == K_r:
                    # Reset to identity — back to raw sensor output
                    q_ref = [1.0, 0.0, 0.0, 0.0]
                    calibrated = False
                    print("Calibration reset")

        with q_lock:
            qw, qi, qj, qk = latest_q
            ax, ay, az = latest_a

        # Apply calibration if set
        if calibrated:
            qw, qi, qj, qk = apply_calibration(
                [qw, qi, qj, qk], q_ref
            )

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0, -4)

        draw_axes()
        draw_box(qw, qi, qj, qk)

        status = "CALIBRATED" if calibrated else "RAW  (press C to calibrate)"
        pygame.display.set_caption(
            f"IMU  {status}  w={qw:+.3f}  i={qi:+.3f}  j={qj:+.3f}  k={qk:+.3f}"
            f"  a=({ax:+.2f},{ay:+.2f},{az:+.2f})m/s²"
        )

        pygame.display.flip()
        clock.tick(60)

if __name__ == '__main__':
    main()