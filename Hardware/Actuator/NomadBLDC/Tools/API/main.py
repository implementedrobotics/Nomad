# FastAPI
import asyncio
from fastapi import FastAPI, BackgroundTasks, Path, Query, HTTPException, status
import uuid

# Socket IO
import socketio

# Models
from pydantic import BaseModel, Field

# Types
from datetime import datetime
from enum import Enum
from typing import Optional

# DB
from tinydb import TinyDB, Query, where

# TODO: Wrap DB Code below in a Class
db = TinyDB("db.json")
db.drop_tables()

can_interface_table = db.table('CAN_INTERFACES')
nomad_bldc_device_table = db.table('NOMAD_BLDC_DEVICES')

Interfaces = Query()
NomadDevices = Query()

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
app = FastAPI()
app.mount('/ws', socketio.ASGIApp(socketio_server=sio, socketio_path="socket.io"))


class NomadBLDC():
    def __init__(self):

        # TODO: Really on need a wrapper and store PyNomad object
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

class CANInterfaceModel(BaseModel):
    id: int = Field(description="The UID of the PEAK CAN Interface/Adapter")
    bitrate: int = Field(gt=0, le=1000000, default=1000000,
                         description="CAN BUS desired nominal bitrate (1mbps max)")
    d_bitrate: int = Field(gt=0, le=5000000, default=5000000,
                           description="CAN BUS desired data bitrate (5mbps max)")
    sample_point: float = Field(gt=0, le=100, default=80,
                                description="CAN BUS desired nominal sample point")
    d_sample_point: float = Field(gt=0, le=100, default=62.5,
                                  description="CAN BUS desired data sample point")
    clock_freq: int = Field(gt=0, le=8000000, default=8000000,
                            description="PEAK CAN Device Driver Clock Frequency (0-80mhz)")
    mode_fd: int = Field(ge=0, le=1, default=1,
                         description="CAN BUS Flexible Data Rate Enable")


class NomadBLDCModel(BaseModel):
    id: str = Field(description="The UID of the Nomad BLDC Device")
    name: str = Field(description="Friendly name of the Nomad BLDC Device")
    # can_interface_id: int = Field(default=None,
    #                               description="The UID of the CAN Transport Device")
    can_bus_id: int = Field(
        ge=0, le=31, description="The CAN BUS network identifier")
    connected: bool = Field(default=False,
                            description="Connection Status of Nomad BLDC Device")
    last_connected: datetime = Field(default=None,
                                     description="The last time Nomad BLDC Device was connected")


class UpdateNomadBLDCModel(BaseModel):
    id: str = Field(description="The UID of the Nomad BLDC Device")
    name: Optional[str] = Field(
        description="Friendly name of the Nomad BLDC Device")
    can_bus_id: Optional[int] = Field(
        description="The CAN BUS Identifier for Nomad BLDC Device")


class AddNomadBLDCModel(BaseModel):
    name: str = Field(description="Friendly name of the Nomad BLDC Device")
    can_bus_id: int = Field(
        description="The CAN BUS Identifier for Nomad BLDC Device")


# TODO: Change to "Session Data"
mutex = {"stop": False}


# TODO: Update Rate as Parameter
async def stream_data(update_rate: float):

    i = 0
    while(mutex["stop"] != True):
        i = i + 1
        stop = mutex["stop"]
        print(f"READ DATA: {stop} : {i}")

        await sio.emit('hey', 'joe')
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
    print('disconnect ', sid)


@app.get("/")
def root():
    return "NOMAD BLDC API V0.1"


@app.get("/devices")
def get_devices():
    return nomad_bldc_device_table.all()


@app.get("/devices/{device_id}")
def get_device(device_id: str):
    return nomad_bldc_device_table.search(where('id') == device_id)

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


@app.post("/devices")
def create_device(device: AddNomadBLDCModel) -> NomadBLDCModel:
    add_device = NomadBLDCModel(
        id=uuid.uuid4().hex, name=device.name, can_bus_id=device.can_bus_id)

    nomad_bldc_device_table.insert(add_device.dict())
    return add_device
