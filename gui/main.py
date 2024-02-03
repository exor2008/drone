import sys

import ahrs
import numpy as np
import serial
from vispy import app, scene
from vispy.util.quaternion import Quaternion
from vispy.visuals import transforms

N = 1000
ACC_CAMERA_VIEW = (-0.1, -10.0, 1.2, 50.0)
GYRO_CAMERA_VIEW = (-0.1, -1000.0, 1.2, 4000.0)
MAG_CAMERA_VIEW = (-0.1, -100.0, 1.2, 300.0)
BGCOLOR = "#828893"

sum_gyro = np.zeros(3)
sum_acc = np.zeros(3)

# mahony = ahrs.filters.Mahony(frequency=10.0, k_P=0.5, k_I=0.01)

quat = np.asarray([1.0, 0.0, 0.0, 0.0])
cube = None
mag_markers = None
mag_raw = []


def update(event):
    global sum_gyro
    global sum_acc
    global quat
    global mag_raw

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

    # if m[6:9].any():
    #     quat = mahony.updateMARG(quat, np.deg2rad(m[3:6]), m[0:3], m[6:9])
    # else:
    #     quat = mahony.updateIMU(
    #         quat,
    #         np.deg2rad(m[3:6]),
    #         m[0:3],
    #     )
    # print(quat)
    # quat = Quaternion(*quat)

    quat = Quaternion(*m[9:])

    rotate_cube(cube, quat)

    if m[6:9].any():
        mag_raw.append(m[6:9])
        mag_markers.set_data(np.asarray(mag_raw))


def get_plot_view(grid, row, col, rect, offset):
    view = grid.add_view(
        row=row,
        col=col,
        col_span=2,
        camera="panzoom",
        border_color="black",
        bgcolor=BGCOLOR,
    )
    view.camera.rect = rect
    _gridlines = scene.GridLines(color=(0.1, 0.1, 0.1, 1.0), parent=view.scene)

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


def get_position_view(grid, row, col):
    view = grid.add_view(row=row, col=col, row_span=4, bgcolor=BGCOLOR)
    view.camera = "turntable"
    view.camera.scale_factor = 1
    cube = scene.Box(
        0.25, 0.25, 0.5, color="red", edge_color="black", parent=view.scene
    )
    cube.transform = transforms.MatrixTransform()
    cube.transform.translate((0, 0))
    return cube


def get_magnitometer_view(grid, row, col):
    view = grid.add_view(row=row, col=col, row_span=4, bgcolor=BGCOLOR)
    view.camera = "turntable"
    view.camera.scale_factor = 5
    view.camera.zoom_factor = 5

    markers = scene.Markers()
    markers.transform = transforms.MatrixTransform()
    markers.transform.translate((0, 0))

    view.add(markers)
    return markers


def rotate_cube(cube, quat):
    cube.transform.matrix = quat.get_matrix()


if __name__ == "__main__":
    canvas = scene.SceneCanvas(keys="interactive", show=True, fullscreen=False)
    grid = canvas.central_widget.add_grid()

    acc = get_plot_view(grid, 0, 0, ACC_CAMERA_VIEW, [0, 0, 0])
    gyro = get_plot_view(grid, 1, 0, GYRO_CAMERA_VIEW, [0, 1000, 2000])
    mag = get_plot_view(grid, 2, 0, MAG_CAMERA_VIEW, [0, 0, 0])
    cube = get_position_view(grid, 3, 0)
    mag_markers = get_magnitometer_view(grid, 3, 1)

    timer = app.Timer("auto", connect=update, start=True)
    timer.start()

    serial_port = serial.Serial(
        "COM6", baudrate=115200, timeout=1, inter_byte_timeout=1
    )
    serial_port.write(b"Start")

    app.run()

    # while True:
    #     m = np.frombuffer(serial_port.read(36), dtype=np.float32)
    #     print(m)
