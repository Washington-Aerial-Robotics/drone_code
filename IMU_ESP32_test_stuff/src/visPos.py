import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R

# input quaternion repeatedly
# output real time view of orientaiton
class visPos:
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
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_zlim(-10, 10)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        # Acceleration vector (red)
        self.acc2_quiver = self.ax.quiver(
            0, 0, 0,
            0, 0, 0,
            color="red",
            linewidth=2,
            length=1.0,
            normalize=True
        )

        self.velo = [0, 0, 0]
        self.pos = [0, 0, 0]
        self.curr_acc = [0, 0, 0]

        self.ax.set_box_aspect((1, 1, 1))
        self.ax.view_init(elev=20, azim=45)
        
    
    # input quaternion
    def update(
        self, qw=None, qx=None, qy=None, qz=None,
        *,
        acc=None,
        dt=None
    ):
        
    

        

        # --- Rotation ---
        rot = R.from_quat([qx, qy, qz, qw])
        #rot = R.from_quat([qw, qx, qy, qz])
        Rbw = rot.as_matrix()

        # cube (body → world)
        rotated_vertices = (Rbw @ self.vertices.T).T
        

        if acc is not None:
            #acc_world = rot.inv().apply(np.asarray(acc))
            #self.acc_quiver.set_segments([[(0,0,0), acc_world]])

            acc2_world = rot.apply(np.asarray(acc))
            self.acc2_quiver.set_segments([[(0,0,0), acc2_world]])
            
        # calculate pos and velo
        avg_acc = (self.curr_acc + acc2_world)/2
        final_v = avg_acc*dt + self.velo
        avg_v = (final_v+self.velo)/2
        
        self.velo = final_v
        self.curr_acc = acc2_world
        self.pos = 1/2*dt*dt*avg_acc + avg_v*dt + self.pos

        translated_vert = rotated_vertices + self.pos
        self.poly.set_verts([[translated_vert[i] for i in face] for face in self.faces])



        plt.pause(1.0 / self.MaxHz)

if __name__ == "__main__":
    import time
    import numpy as np

    viz = visPos(MaxHz=100)

    print("Testing position + attitude integration")
    print("Ctrl+C to exit")

    t_prev = time.time()
    t0 = t_prev

    while True:
        t_now = time.time()
        dt = t_now - t_prev
        t_prev = t_now

        t = t_now - t0

        # --- Smooth rotation about Z axis ---
        angle = 0.5 * t  # rad/s
        qw = np.cos(angle / 2)
        qx = 0.0
        qy = 0.0
        qz = np.sin(angle / 2)

        # --- Constant acceleration in BODY frame ---
        # Forward acceleration along body X
        acc_body = np.array([0.5, 0.0, 0.0])

        viz.update(
            qw, qx, qy, qz,
            acc=acc_body,
            dt=dt
        )

        
        

