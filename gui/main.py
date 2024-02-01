import sys

import ahrs
import numpy as np
import serial
from vispy import app, scene
from vispy.util.quaternion import Quaternion
from vispy.visuals import transforms

from gui.kalman import KalmanFilter

N = 1000
ACC_CAMERA_VIEW = (-0.1, -10.0, 1.2, 50.0)
GYRO_CAMERA_VIEW = (-0.1, -1000.0, 1.2, 4000.0)
MAG_CAMERA_VIEW = (-0.1, -100.0, 1.2, 300.0)

sum_gyro = np.zeros(3)
sum_acc = np.zeros(3)

# madwick = ahrs.filters.Madgwick()
# mahony = ahrs.filters.Mahony(frequency=10, k_P=0.8)

quat = np.asarray([1.0, 0.0, 0.0, 0.0])
cube = None


def update(event):
    global sum_gyro
    global sum_acc
    global quat

    serial_port.write(b"Give")
    m = np.frombuffer(serial_port.read(52), dtype=np.float32)

    sum_acc += m[0:3]
    sum_gyro += np.rad2deg(m[3:6])

    # print("acc", m[0:3])
    # print("gyro", m[3:6])
    # print("mag", m[6:9])
    # print("angle", m[9:])

    acc.roll_data(sum_acc)
    gyro.roll_data(sum_gyro)
    mag.roll_data(m[6:9])

    # if not m[6:9].any():
    #     quat = mahony.updateMARG(quat, np.deg2rad(m[3:6]), m[0:3], m[6:9])
    # else:
    #     quat = mahony.updateIMU(
    #         quat,
    #         np.deg2rad(m[3:6]),
    #         m[0:3],
    #     )

    quat = Quaternion(*m[9:])
    # quat = Quaternion.create_from_euler_angles(*m[9:], degrees=False)
    # quat = Quaternion(*quat)

    rotate_cube(cube, quat)

    # norm = m[6:9] / np.linalg.norm(m[6:9])
    # azim = np.arctan2(norm[1], norm[0])
    # print(np.rad2deg(azim))

    # print("acc", m[0:3].min(), m[0:3].max())
    # print("gyro", m[3:6].min(), m[3:6].max())
    # print("mag", m[6:9])

    # kf.predict()
    # kf.update(m)
    # print("+", kf.x)
    # acc.roll_data(kf.x[0:3])
    # gyro.roll_data(kf.x[3:6])
    # mag.roll_data(kf.x[6:9])


def get_widget(grid, row, col, rect, offset):
    view = grid.add_view(row=row, col=col, camera="panzoom", border_color="grey")
    view.camera.rect = rect
    _gridlines = scene.GridLines(color=(1, 1, 1, 0.5), parent=view.scene)

    pos_offset = np.zeros((3, 3), dtype=np.float32)
    pos_offset[:, 1] = offset

    widget = scene.ScrollingLines(
        n_lines=3,
        line_size=N,
        dx=1 / N,
        pos_offset=pos_offset,
        color=np.asarray(
            [[1.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 1.0], [0.0, 0.0, 1.0, 1.0]]
        ),
        parent=view.scene,
    )
    widget.set_data(0, np.zeros(N))
    widget.set_data(1, np.zeros(N))
    widget.set_data(2, np.zeros(N))
    return widget


def get_3d(grid, row, col):
    view = grid.add_view(row=row, col=col, bgcolor="w")
    view.camera = "turntable"
    view.camera.scale_factor = 1
    # view.camera = scene.cameras.ArcballCamera(parent=view.scene)
    cube = scene.Box(
        0.25, 0.25, 0.5, color="red", edge_color="black", parent=view.scene
    )
    cube.transform = transforms.MatrixTransform()
    cube.transform.translate((0, 0))
    return cube


def rotate_cube(cube, quat):
    cube.transform.matrix = quat.get_matrix()


if __name__ == "__main__":
    canvas = scene.SceneCanvas(keys="interactive", show=True, fullscreen=False)
    grid = canvas.central_widget.add_grid()

    acc = get_widget(grid, 0, 0, ACC_CAMERA_VIEW, [0, 0, 0])
    gyro = get_widget(grid, 0, 1, GYRO_CAMERA_VIEW, [0, 1000, 2000])
    mag = get_widget(grid, 1, 0, MAG_CAMERA_VIEW, [0, 0, 0])
    cube = get_3d(grid, 1, 1)

    timer = app.Timer("auto", connect=update, start=True)
    timer.start()

    serial_port = serial.Serial(
        "COM6", baudrate=115200, timeout=1, inter_byte_timeout=1
    )
    serial_port.write(b"Start")

    app.run()

    # print("close")
    # serial_port.cancel_read()
    # serial_port.close()
    # print(f"CLosed: {serial_port.closed}")
    # del serial_port

    # while True:
    #     m = np.frombuffer(serial_port.read(36), dtype=np.float32)
    #     print(m)
