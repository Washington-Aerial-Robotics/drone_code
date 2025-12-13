import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---------------- Serial ----------------
PORT = "/dev/cu.usbserial-02C554D8"        # change if needed
BAUD = 115200
ser = serial.Serial(PORT, BAUD, timeout=1)

# ---------------- Rotation math ----------------
def rotation_matrix(roll, pitch, yaw):
    """ roll, pitch, yaw in radians """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr, cr]])

    Ry = np.array([[cp, 0, sp],
                   [0, 1, 0],
                   [-sp, 0, cp]])

    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [0,   0,  1]])

    return Rz @ Ry @ Rx   # ZYX (yaw-pitch-roll)

# ---------------- Cube definition ----------------
cube_vertices = np.array([
    [-0.5, -0.5, -0.5],
    [ 0.5, -0.5, -0.5],
    [ 0.5,  0.5, -0.5],
    [-0.5,  0.5, -0.5],
    [-0.5, -0.5,  0.5],
    [ 0.5, -0.5,  0.5],
    [ 0.5,  0.5,  0.5],
    [-0.5,  0.5,  0.5]
])

faces = [
    [0,1,2,3],
    [4,5,6,7],
    [0,1,5,4],
    [2,3,7,6],
    [1,2,6,5],
    [4,7,3,0]
]

# ---------------- Plot ----------------
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

cube_poly = Poly3DCollection([], alpha=0.6, facecolor="cyan")
ax.add_collection3d(cube_poly)

print("Listening for roll,pitch,yaw ... Ctrl+C to stop")

# ---------------- Main loop ----------------
while True:
    line = ser.readline().decode("utf-8", errors="ignore").strip()
    if not line:
        continue

    try:
        roll, pitch, yaw = map(float, line.split(","))
    except ValueError:
        continue

    # Convert degrees → radians
    roll  = np.deg2rad(roll)
    pitch = np.deg2rad(pitch)
    yaw   = np.deg2rad(yaw)

    R = rotation_matrix(roll, pitch, yaw)
    rotated = cube_vertices @ R.T

    face_vertices = [[rotated[i] for i in face] for face in faces]
    cube_poly.set_verts(face_vertices)

    plt.pause(0.01)

