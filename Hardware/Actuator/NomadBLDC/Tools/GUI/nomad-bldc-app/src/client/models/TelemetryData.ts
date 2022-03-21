/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */

export type TelemetryData = {
    /**
     * The UID of the Nomad BLDC Device
     */
    nomad_bldc_id: string;
    /**
     * List of registers to stream telemetry data
     */
    registers: Array<number>;
};
