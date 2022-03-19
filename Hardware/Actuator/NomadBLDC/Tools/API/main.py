from fastapi import FastAPI, Path, Query, HTTPException, status
from typing import Optional
from pydantic import BaseModel
from tinydb import TinyDB, Query


db = TinyDB("db.json")
app = FastAPI()


class NomadBLDC():
    def __init__(self):

        # TODO: Really on need a wrapper and store PyNomad object
        self.can_id = 0
        self.connected = False
        self.in_use = False


class NomadBLDCManager():
    def __init__(self):
        self.devices = []

    def scan(self):
        print("Scanning")

    def connect(self):
        print("CONNECTING")

    def get_device(self):
        print("RETRIEVING DEVICE")


class NomadBLDCModel(BaseModel):
    id: str
    name: str
    can_id: int
    connected: bool
    state: int
    last_connected: int


class UpdateNomadBLDCModel(BaseModel):
    name: Optional[str] = None
    can_id: Optional[int] = None


@app.get("/devices")
def get_devices():
    return {"Devices": "1,2,3"}


@app.get("/devices/{device_id}")
def get_device(device_id: int):
    return {"Device": device_id}


@app.get("/devices/connect/{device_id}")
def connect_device(device_id: int):
    return "Connected"


@app.get("/devices/disconnect/{device_id}")
def disconnect_device(device_id: int):
    return "Disconnected"


@app.post("/devices")
def create_device(device: NomadBLDCModel):
    return "Created Device"
