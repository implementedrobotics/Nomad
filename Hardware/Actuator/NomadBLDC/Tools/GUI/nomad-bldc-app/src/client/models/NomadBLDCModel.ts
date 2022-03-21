/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */

export type NomadBLDCModel = {
    /**
     * The UID of the Nomad BLDC Device
     */
    id: string;
    /**
     * Friendly name of the Nomad BLDC Device
     */
    name: string;
    /**
     * The CAN BUS network identifier
     */
    can_bus_id: number;
    /**
     * Connection Status of Nomad BLDC Device
     */
    connected?: boolean;
    /**
     * The last time Nomad BLDC Device was connected
     */
    last_connected?: string;
};
