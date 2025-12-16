import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R

# input quaternion repeatedly
# output real time view of orientaiton
class visualizeAttitude:
    def __init__(self, MaxHz):
        self.MaxHz = MaxHz

        w, d, h = 1.0, 0.6, 0.3
        self.vertices = np.array([
            [-w/2, -d/2, -h/2],
            [ w/2, -d/2, -h/2],
            [ w/2,  d/2, -h/2],
            [-w/2,  d/2, -h/2],
            [-w/2, -d/2,  h/2],
            [ w/2, -d/2,  h/2],
            [ w/2,  d/2,  h/2],
            [-w/2,  d/2,  h/2],
        ])

        self.faces = [
            [0, 1, 2, 3],
            [4, 5, 6, 7],
            [0, 1, 5, 4],
            [2, 3, 7, 6],
            [1, 2, 6, 5],
            [0, 3, 7, 4],
        ]

        self.plt = plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.poly = Poly3DCollection(
            [[self.vertices[i] for i in face] for face in self.faces],
            alpha=0.6,
            facecolor="cyan",
            edgecolor="black"
        )
        self.ax.add_collection3d(self.poly)
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(-1, 1)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        # Acceleration vector (red)
        self.acc_quiver = self.ax.quiver(
            0, 0, 0,
            0, 0, 0,
            color="red",
            linewidth=2,
            length=1.0,
            normalize=True
        )

        # Magnetic field vector (blue)
        self.mag_quiver = self.ax.quiver(
            0, 0, 0,
            0, 0, 0,
            color="blue",
            linewidth=2,
            length=1.0,
            normalize=True
        )

        self.ax.set_box_aspect((1, 1, 1))
        self.ax.view_init(elev=20, azim=45)
        
    
    # input quaternion
    def update_attitude(
        self, qw=None, qx=None, qy=None, qz=None,
        *,
        acc=None,
        mag=None
    ):
        # --- Rotation ---
        rot = R.from_quat([qx, qy, qz, qw])
        Rbw = rot.as_matrix()

        # cube (body → world)
        rotated_vertices = (Rbw @ self.vertices.T).T
        self.poly.set_verts([[rotated_vertices[i] for i in face] for face in self.faces])

        if acc is not None:
            acc_world = rot.apply(np.asarray(acc))
            self.acc_quiver.set_segments([[(0,0,0), acc_world]])

        if mag is not None:
            mag_world = rot.apply(np.asarray(mag))
            self.acc_quiver.set_segments([[(0,0,0), mag_world]])

        plt.pause(1.0 / self.MaxHz)


        
        


if __name__ == "__main__":
    import time
    import numpy as np

    viz = visualizeAttitude(MaxHz=60)

    print("Testing quaternion visualization...")
    print("Ctrl+C to exit")

    t0 = time.time()
    while True:
        t = time.time() - t0

        # --- Smooth rotation about Z axis ---
        angle = t
        qw = np.cos(angle / 2)
        qx = 0.0
        qy = 0.0
        qz = np.sin(angle / 2)

        # --- Test vectors (BODY frame) ---
        acc = np.array([0.0, 0.0, -1.0])   # gravity straight "down"
        mag = np.array([1.0, 0.0, 0.0])    # magnetic north along X

        viz.update_attitude(
            qw, qx, qy, qz,
            acc=acc,
            mag=mag
        )






