import sys

import ahrs
import numpy as np
import serial
from vispy import app, scene
from vispy.io.stl import load_stl_binary
from vispy.util.quaternion import Quaternion
from vispy.visuals import transforms

N = 1000
ACC_CAMERA_VIEW = (-0.1, -10.0, 1.2, 50.0)
GYRO_CAMERA_VIEW = (-0.1, -100.0, 1.2, 400.0)
MAG_CAMERA_VIEW = (-0.1, -100.0, 1.2, 300.0)
BGCOLOR = "#828893"

sum_gyro = np.zeros(3)
sum_acc = np.zeros(3)

# mahony = ahrs.filters.Mahony(frequency=20.0, k_P=0.4, k_I=0.01)

quat = np.asarray([1.0, 0.0, 0.0, 0.0])
quat_ar = np.asarray([1.0, 0.0, 0.0, 0.0])
cube = None
mag_markers = None
mag_raw = []


def update(event):
    global sum_gyro
    global sum_acc
    global quat
    global mag_raw
    global quat_ar
    global rotation

    serial_port.write(b"Give")
    m = np.frombuffer(serial_port.read(52), dtype=np.float32)

    sum_acc += m[0:3]
    sum_gyro += np.rad2deg(m[3:6])

    # print("acc", m[0:3])
    # print("gyro", m[3:6])
    # print("mag", m[6:9])
    # print("angle", m[9:])

    acc.roll_data(m[0:3])
    gyro.roll_data(np.rad2deg(m[3:6]))
    mag.roll_data(m[6:9])

    # if m[6:9].any():
    #     quat_ar = mahony.updateMARG(quat_ar, m[3:6], m[0:3], m[6:9])
    # else:
    #     quat_ar = mahony.updateIMU(
    #         quat_ar,
    #         m[3:6],
    #         m[0:3],
    #     )
    # quat = Quaternion(
    #     quat_ar[1],
    #     quat_ar[2],
    #     quat_ar[3],
    #     quat_ar[0],
    # )
    # quat = Quaternion(*quat_ar)

    # quat = Quaternion(m[9], m[10], m[11], m[12], normalize=False)
    # quat = Quaternion.create_from_euler_angles(m[9], m[10], m[11])

    # quat = Quaternion(m[10], m[11], m[12], m[9], False)
    quat = Quaternion(*m[9:], False)

    # print(quat)

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

    _gridlines = scene.GridLines(color=(0.5, 0.5, 0.5, 1.0), parent=view.scene)

    mesh_data = load_stl_binary(open(r".\resources\rocket.stl", mode="rb"))
    cube = scene.visuals.Mesh(
        mesh_data["vertices"],
        mesh_data["faces"],
        color=(0.2, 0.3, 0.2, 1),
        shading="smooth",
        parent=view.scene,
    )
    cube.transform = transforms.MatrixTransform()
    cube.transform.translate((0, 0))

    return cube


def titles():
    scene.visuals.Text(
        "Accelerometer",
        pos=(10, 10),
        color="white",
        font_size=12,
        bold=False,
        anchor_x="left",
        anchor_y="bottom",
        parent=canvas.scene,
    )

    scene.visuals.Text(
        "Gyro",
        pos=(10, 125),
        color="white",
        font_size=12,
        bold=False,
        anchor_x="left",
        anchor_y="bottom",
        parent=canvas.scene,
    )

    scene.visuals.Text(
        "Magnetometer",
        pos=(10, 245),
        color="white",
        font_size=12,
        bold=False,
        anchor_x="left",
        anchor_y="bottom",
        parent=canvas.scene,
    )

    scene.visuals.Text(
        "3D",
        pos=(10, 360),
        color="white",
        font_size=12,
        bold=False,
        anchor_x="left",
        anchor_y="bottom",
        parent=canvas.scene,
    )

    scene.visuals.Text(
        "3D Magnitometer",
        pos=(800, 360),
        color="white",
        font_size=12,
        bold=False,
        anchor_x="left",
        anchor_y="bottom",
        parent=canvas.scene,
    )


def get_magnitometer_view(grid, row, col):
    view = grid.add_view(row=row, col=col, row_span=4, bgcolor=BGCOLOR)
    view.camera = "turntable"
    view.camera.scale_factor = 5
    view.camera.zoom_factor = 5

    _gridlines = scene.GridLines(color=(0.5, 0.5, 0.5, 1.0), parent=view.scene)

    markers = scene.Markers()
    markers.transform = transforms.MatrixTransform()
    markers.transform.translate((0, 0))

    view.add(markers)
    return markers


def rotate_cube(cube, quat):
    cube.transform.matrix = quat.get_matrix().T


if __name__ == "__main__":
    canvas = scene.SceneCanvas(keys="interactive", show=True, fullscreen=False)
    grid = canvas.central_widget.add_grid()

    acc = get_plot_view(grid, 0, 0, ACC_CAMERA_VIEW, [0, 0, 0])
    gyro = get_plot_view(grid, 1, 0, GYRO_CAMERA_VIEW, [0, 100, 200])
    mag = get_plot_view(grid, 2, 0, MAG_CAMERA_VIEW, [0, 0, 0])
    cube = get_position_view(grid, 3, 0)
    mag_markers = get_magnitometer_view(grid, 3, 1)
    titles()

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
