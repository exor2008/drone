import sys

import numpy as np
import serial
from vispy import app, scene

N = 1000
ACC_CAMERA_VIEW = (-0.1, -10.0, 1.2, 50.0)
GYRO_CAMERA_VIEW = (-0.1, -1000.0, 1.2, 4000.0)
MAG_CAMERA_VIEW = (-0.1, -100.0, 1.2, 300.0)


def update(event):
    serial_port.write(b"Give")
    m = np.frombuffer(serial_port.read(36), dtype=np.float32)
    acc.roll_data(m[0:3])
    gyro.roll_data(m[3:6])
    mag.roll_data(m[6:9])
    # print("acc", m[0:3].min(), m[0:3].max())
    # print("gyro", m[3:6].min(), m[3:6].max())
    # print("mag", m[6:9].min(), m[6:9].max())


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


if __name__ == "__main__":
    win = scene.SceneCanvas(keys="interactive", show=True, fullscreen=False)
    grid = win.central_widget.add_grid()

    acc = get_widget(grid, 0, 0, ACC_CAMERA_VIEW, [0, 0, 0])
    gyro = get_widget(grid, 1, 0, GYRO_CAMERA_VIEW, [0, 1000, 2000])
    mag = get_widget(grid, 2, 0, MAG_CAMERA_VIEW, [0, 0, 0])

    timer = app.Timer("auto", connect=update, start=True)
    timer.start()

    serial_port = serial.Serial(
        "COM6", baudrate=115200, timeout=1, inter_byte_timeout=1
    )
    serial_port.write(b"Start")

    app.run()

    print("close")
    serial_port.cancel_read()
    serial_port.close()
    print(f"CLosed: {serial_port.closed}")
    del serial_port

    # while True:
    #     m = np.frombuffer(serial_port.read(36), dtype=np.float32)
    #     print(m)
