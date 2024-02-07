import json

import numpy as np
import serial
from fastapi import FastAPI, Response
from fastapi.responses import FileResponse, JSONResponse

app = FastAPI()

serial_port = serial.Serial("COM6", baudrate=115200, timeout=1, inter_byte_timeout=1)
serial_port.write(b"Start")


@app.get("/assets/{file_path:path}")
async def get_assets_file(file_path: str):
    return FileResponse(
        f"static/assets/{file_path}", media_type="application/javascript"
    )


@app.get("/gltf/{file_path:path}")
async def get_gltf_file(file_path: str):
    return FileResponse(
        f"static/gltf/{file_path}", media_type="application/octet-stream"
    )


@app.get("/")
async def get_assets_file():
    return FileResponse(f"static/index.html", media_type="text/html")


@app.get("/angle")
async def get_angle():
    serial_port.write(b"Give")
    m = np.frombuffer(serial_port.read(52), dtype=np.float32)

    json_str = json.dumps({"d": m[9:].tolist()})
    return Response(content=json_str, media_type="application/json")
