# FastAPI
import asyncio
import telnetlib
from fastapi import FastAPI, BackgroundTasks, Path, Query, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.openapi.models import Server

import uuid

# Socket IO
import socketio

# Models
from pydantic import BaseModel, Field

# json helpers
import json

# Types
from datetime import datetime
from enum import Enum
from typing import List, Optional

# Models
from models import CANInterfaceModel, NomadBLDCModel, AddNomadBLDCModel, UpdateNomadBLDCModel, TelemetryData

# DB
from tinydb import TinyDB, Query, where

# TODO: Wrap DB Code below in a Class
db = TinyDB("db.json")
db.drop_tables()

can_interface_table = db.table('CAN_INTERFACES')
nomad_bldc_device_table = db.table('NOMAD_BLDC_DEVICES')

Interfaces = Query()
NomadDevices = Query()

globals()['BLAH'] = 2
# test = CANInterfaceModel(id=1, bitrate=1000, d_bitrate=3000,
#                          sample_point=87.5, d_sample_point=63.0, clock_freq=80, mode_fd=1)


# can_interface_table.insert(test.dict())

# results = can_interface_table.search(Interfaces.id == 1)
# print(results)
# print(can_interface_table.all())


# test.bitrate = 1234
# can_interface_table.update(test.dict(), Interfaces.id == 1)
# print(can_interface_table.all())


# Create Socket IO Server
sio = socketio.AsyncServer(cors_allowed_origins='*', async_mode='asgi')


# Setup our App and Mount SIO sub app
app = FastAPI(title="NomadBLDC API",
              description="API for Nomad BLDC motor controllers",
              version="0.1",
              contact={
                  "name": "Quincy Jones",
                  "url": "http://www.implementedrobotics.com",
                  "email": "quincy@implementedrobotics.com",
              },
              license_info={
                  "name": "MIT",
                  "url": "https://opensource.org/licenses/MIT",
              },
              servers=[{"url": "http://localhost:8000", "description": "test"}])
app.mount('/ws', socketio.ASGIApp(socketio_server=sio, socketio_path="socket.io"))

origins = ["*"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class NomadBLDC():
    def __init__(self):

        # TODO: Really only need a wrapper and store PyNomad object
        self.can_id = 0
        self.connected = False
        self.in_use = False


class NomadBLDCManager():
    def __init__(self):
        self.devices = {}

    # run on parameter update in DB
    # make easy and just disconnect, idle and reload
    # Also reset on CAN Interface parameter changes?
    # Error if try to change settings while actively connected(FOR CAN INTERFACE CHANGES)
    # Actually make no changes while actively connected....

    def reload(self):
        print("Changes Detected Reload")

    def load(self):
        print("LOADING FROM DB")

    def scan(self):
        print("Scanning")

    def connect(self):
        print("CONNECTING")

    def get_device(self):
        print("RETRIEVING DEVICE")


class NomadState(int, Enum):
    IDLE = 1
    ERROR = 2
    FOC = 3
    CALIBRATION = 4


# TODO: Move to models.py


# TODO: Change to "Session Data"
mutex = {"stop": False}


class ResponseMessage(BaseModel):
    message: str

# TODO: Update Rate as Parameter
# TODO: Also return if no user connected


async def stream_data(update_rate: float):

    test = [{"Name": "X", "type": 'float'},
            {"Name": "Y", "type": 'float'}]
    i = 0
    while(mutex["stop"] != True):
        i = i + 1
        stop = mutex["stop"]
        print(f"READ DATA: {stop} : {i}")

        await sio.emit('hey', json.dumps(test))
        await asyncio.sleep(update_rate)

    print("WE OUT!")

    # if(mutex["stop"] == True):
    #     print("BAILED")
    #     return

    #     # while(mutex["stop"] != True):
    #     # while(i < 30):
    # i = i + 1
    # stop = mutex["stop"]
    # print(f"READ DATA: {stop} : {i}")

    # await sio.emit('hey', 'joe')
    # sio.start_background_task(target=read_data)
    # print("WE OUT!")

# Socket IO


@sio.event
async def connect(sid, environ, auth):
    print('connect ', sid)
    await sio.emit('hey', 'joe')


@sio.event
def disconnect(sid):
    mutex["stop"] = True
    print('disconnect ', sid)


@app.get("/")
def root():
    return "NOMAD BLDC API V0.1"


@app.get("/devices", response_model=List[NomadBLDCModel])
def get_devices():
    # return {"message": "Item received"}
    return nomad_bldc_device_table.all()


@app.get("/devices/{device_id}")
def get_device(device_id: str):
    return nomad_bldc_device_table.search(where('id') == device_id)


@app.post("/devices")
def create_device(device: AddNomadBLDCModel) -> NomadBLDCModel:
    add_device = NomadBLDCModel(
        id=uuid.uuid4().hex, name=device.name, can_bus_id=device.can_bus_id)

    nomad_bldc_device_table.insert(add_device.dict())
    return add_device


# TODO: Pass CAN INTERFACE DEVICE HERE as query param
# TODO: Enable Streaming/SocketIO with query param for register

@app.get("/devices/connect/{device_id}")
async def connect_device(device_id: str, background_tasks: BackgroundTasks):

    # TODO: Check Task Started
    mutex["stop"] = False
    background_tasks.add_task(stream_data, update_rate=0.2)
    # sio.start_background_task(target=stream_data)
    return "Connected"


@app.get("/devices/disconnect/{device_id}")
def disconnect_device(device_id: str):
    mutex["stop"] = True
    return "Disconnected"


@app.get("/devices/status/{device_id}")
def connect_device(device_id: str):
    return "STATUS"


@app.post("/stream")
def stream_telemetry(telemetry: TelemetryData):
    print(telemetry)
    return "Success"
