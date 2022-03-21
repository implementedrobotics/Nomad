/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { AddNomadBLDCModel } from '../models/AddNomadBLDCModel';
import type { NomadBLDCModel } from '../models/NomadBLDCModel';
import type { TelemetryData } from '../models/TelemetryData';

import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';

export class DefaultService {

    /**
     * Root
     * @returns any Successful Response
     * @throws ApiError
     */
    public static rootGet(): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/',
        });
    }

    /**
     * Get Devices
     * @returns NomadBLDCModel Successful Response
     * @throws ApiError
     */
    public static getDevicesDevicesGet(): CancelablePromise<Array<NomadBLDCModel>> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/devices',
        });
    }

    /**
     * Create Device
     * @param requestBody
     * @returns any Successful Response
     * @throws ApiError
     */
    public static createDeviceDevicesPost(
        requestBody: AddNomadBLDCModel,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/devices',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }

    /**
     * Get Device
     * @param deviceId
     * @returns any Successful Response
     * @throws ApiError
     */
    public static getDeviceDevicesDeviceIdGet(
        deviceId: string,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/devices/{device_id}',
            path: {
                'device_id': deviceId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }

    /**
     * Connect Device
     * @param deviceId
     * @returns any Successful Response
     * @throws ApiError
     */
    public static connectDeviceDevicesConnectDeviceIdGet(
        deviceId: string,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/devices/connect/{device_id}',
            path: {
                'device_id': deviceId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }

    /**
     * Disconnect Device
     * @param deviceId
     * @returns any Successful Response
     * @throws ApiError
     */
    public static disconnectDeviceDevicesDisconnectDeviceIdGet(
        deviceId: string,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/devices/disconnect/{device_id}',
            path: {
                'device_id': deviceId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }

    /**
     * Connect Device
     * @param deviceId
     * @returns any Successful Response
     * @throws ApiError
     */
    public static connectDeviceDevicesStatusDeviceIdGet(
        deviceId: string,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/devices/status/{device_id}',
            path: {
                'device_id': deviceId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }

    /**
     * Stream Telemetry
     * @param requestBody
     * @returns any Successful Response
     * @throws ApiError
     */
    public static streamTelemetryStreamPost(
        requestBody: TelemetryData,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/stream',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }

}