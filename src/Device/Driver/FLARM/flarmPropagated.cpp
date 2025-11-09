/*
 * Copyright (C) 2010-2013 triadis engineering GmbH all rights reserved.
 *
 * flarmPropagated.cpp
 *
 * 
 *
 * Created on: 14.03.2013
 *     Author: sam
 */

#include <math.h>

#include <Device/Driver/FLARM/flarmPropagated.hpp>


//static int16_t calcBearing(int RelEast, int RelNorth);

uint32_t getFlarmId(const CanasMessage *canasMessage);

/*
 * restauriert flarm FLAU daten auf einer sequenz von CANas messages
 * msg: die CAN message mit id FLARM_STATE_ID
 * alt: altitude
 * objectData:   pointer auf FlarmMostImportantObjectData_t
 * flarmState:   pointer auf FlarmDeviceState_t
 * return: true wenn daten komplet
 */
bool canasFlarmStatePropagated(const CanasMessage *canasMessage, int altitude, FlarmMostImportantObjectData *objectData,
                               FlarmState *flarmState) {

    switch (canasMessage->service_code) {
        case 0: {
            objectData->TargetVectorValid = canasMessage->data.container.CHAR4[0] >> 4 & 0x01;
            flarmState->RxDevicesCount = canasMessage->data.container.UCHAR4[1];
            flarmState->TxState = canasMessage->data.container.UCHAR4[0] & 0x01;
            flarmState->GpsState = canasMessage->data.container.UCHAR4[0] >> 1 & 0x03;
            flarmState->PowerState = canasMessage->data.container.UCHAR4[0] >> 3 & 0x01;
            flarmState->State = canasMessage->data.container.UCHAR4[2];
            flarmState->ErrorCode = canasMessage->data.container.UCHAR4[3];
            flarmState->GpsAltitude = altitude;  // GPS altitude from flarm. TODO isu hat genauere GPS alt
            if (!(canasMessage->data.container.UCHAR4[0] >> 4 & 0x01)) {
                objectData->AlarmLevel = 3;
                return true;
            }
            break;
        }

        case 1: {
            objectData->AlarmLevel = canasMessage->data.container.UCHAR4[0] & 0x7;
            objectData->AlarmType = canasMessage->data.container.UCHAR4[0] >> 3 & 0x07;
            objectData->ID = getFlarmId(canasMessage);
            break;
        }

        case 2: {
            objectData->RelVertical = canasMessage->data.container.USHORT2[0];
            objectData->RelHorizontal = canasMessage->data.container.USHORT2[1];
            break;
        }

        case 3: {
            objectData->RelBearing = canasMessage->data.container.SHORT;
            if (objectData->AlarmType == FLATOBSTACLEALARM) {
                objectData->RelBearing = 0;
                objectData->RelVertical = 0;
                objectData->ID = 0;
            }
            return true;
        }
    }
    return false;
}

/*
 * restauriert flarm FLAU daten auf einer sequenz von CANas messages
 * msg: die CAN message mit id FLARM_STATE_ID
 * flarmObjectData:   pointer auf FlarmMostImportantObjectData_t
 * S:   pointer auf FlarmDeviceState_t
 * return: true wenn daten komplet
 */
bool canasFlarmObjectPropagated(const CanasMessage *canasMessage, int canid, FlarmObjectData *flarmObjectData) {

    auto messageindex = canasMessage->service_code & 0x0f;
    auto validFlags = canasMessage->service_code >> 4 & 0x0f;

    switch (messageindex) {
        case 0: {
            flarmObjectData->AlarmLevel = FLARM_OBJECT_AL0_ID - canid;
            flarmObjectData->RelNorth = canasMessage->data.container.USHORT2[0];
            flarmObjectData->RelEast = canasMessage->data.container.USHORT2[1];
            flarmObjectData->RelHorizontal = sqrtl(flarmObjectData->RelNorth * flarmObjectData->RelNorth +
                                                   flarmObjectData->RelEast * flarmObjectData->RelEast);
            break;
        }

        case 1: {
            flarmObjectData->RelVertical = canasMessage->data.container.USHORT2[0];
            flarmObjectData->Track = canasMessage->data.container.USHORT2[1];
            flarmObjectData->valid.track = (validFlags & 0x2f) != 0;
            break;
        }

        case 2: {
            flarmObjectData->GroundSpeed = canasMessage->data.container.UCHAR4[0];
            flarmObjectData->Type = canasMessage->data.container.UCHAR4[1];
            flarmObjectData->ClimbRate = canasMessage->data.container.CHAR4[2];
            flarmObjectData->TurnRate = canasMessage->data.container.UCHAR4[3];
            flarmObjectData->valid.groundSpeed = (validFlags & 0x1f) != 0;
            flarmObjectData->valid.climbRate = (validFlags & 0x4f) != 0;
            flarmObjectData->valid.turnRate = (validFlags & 0x8f) != 0;
            break;
        }

        case 3: {
            flarmObjectData->IdType = canasMessage->data.container.UCHAR4[0];
            flarmObjectData->ID = getFlarmId(canasMessage);
            return true;
        }
    }
    return false;
}

uint32_t getFlarmId(const CanasMessage *canasMessage) {
    return (canasMessage->data.container.UCHAR4[3] & 0xFF) |
           (canasMessage->data.container.UCHAR4[2] & 0xFF) << 8 |
           (canasMessage->data.container.UCHAR4[1] & 0xFF) << 16;
}
