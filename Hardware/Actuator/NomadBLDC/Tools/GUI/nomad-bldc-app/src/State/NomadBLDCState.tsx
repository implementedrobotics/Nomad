import { atomFamily, atom, selectorFamily, selector } from 'recoil';

type DeviceStatusRegister1State = {
    faultMode: number | undefined;
    controlMode: number | undefined;
    Vbus: number | undefined;
    Ibus: number | undefined;
    fetTemp: number | undefined;
};

type DeviceStatusRegister2State = {
    fwMajor: number | undefined;
    fwMinor: number | undefined;
    uid1: number | undefined;
    uid2: number | undefined;
    uid3: number | undefined;
    upTime: number | undefined;
};

export const deviceStatusRegister1Default = {
    faultMode: undefined,
    controlMode: undefined,
    Vbus: undefined,
    Ibus: undefined,
    fetTemp: undefined,
};

export const deviceStatusRegister2Default = {
    fwMajor: undefined,
    fwMinor: undefined,
    uid1: undefined,
    uid2: undefined,
    uid3: undefined,
    upTime: undefined,
};

type Registers = {
    deviceStatusRegister1: DeviceStatusRegister1State;
    deviceStatusRegister2: DeviceStatusRegister2State;
};

type NomadBLDCDeviceState = {
    id: string;
    connected: boolean;
    calibrated: boolean;
    enabled: boolean;
    registers: Registers;
};

export const nomadBLDCDeviceState = atomFamily<NomadBLDCDeviceState, number>({
    key: 'NomadBLDCDevice',
    default: () => ({
        id: '123', // TODO: UUIDV4()?
        connected: false,
        calibrated: false,
        enabled: false,
        registers: {
            deviceStatusRegister1: deviceStatusRegister1Default,
            deviceStatusRegister2: deviceStatusRegister2Default,
        },
    }),
});
