
# Models
from pydantic import BaseModel, Field

# Types
from datetime import datetime
from enum import Enum
from typing import List, Optional


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


class TelemetryData(BaseModel):
    nomad_bldc_id: str = Field(description="The UID of the Nomad BLDC Device")
    registers: List[int] = Field(
        description="List of registers to stream telemetry data")
