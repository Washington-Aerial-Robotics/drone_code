# drone_code
WAAR Drone Code Base

Under IMU_ESP32_test_stuff, visual_task.py allows for attutide visualization.
To run visual_task.py, first connect to the esp32 and update PORT under visual_task.py and platformio.ini. Ensure all packages under requirements.txt are installed, then simply run visual_task.py


TODO:

In general, start making objects instead of making big scripts.

Ensure proper madgewick filtering. Change visualization to use quaternions from madgewick filter. Optimize refresh rates. Add option to print out script suitable for the magneto calibration tool.

Design simple kalman filter for position. Create alternate visualizer which tracks position as well as attiude. Consider adding offset parameters for how far the imu is away from the cg of the drone.


Object for visualization, simple rotations, and for general positioning, both using quaternions.

