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


static int16_t calcBearing(int RelEast, int RelNorth);

/*
 * restauriert flarm FLAU daten auf einer sequenz von CANas messages
 * msg: die CAN message mit id FLARM_STATE_ID
 * alt: altitude
 * O:   pointer auf FlarmMostImportantObjectData_t
 * S:   pointer auf FlarmDeviceState_t
 * return: true wenn daten komplet
 */
bool canasFlarmStatePropagated(const CanasMessageData *phost, int altitude, FlarmMostImportantObjectData *O,
                               FlarmState *S) {

    const CanasMessage *canasMessage = (const CanasMessage *) phost;
    switch (canasMessage->service_code) {

        case 0: {
            O->TargetVectorValid = phost->container.CHAR4[0] >> 4 & 0x01;

            //S->TmLastReceive = getTicker();
            S->RxDevicesCount = phost->container.UCHAR4[1];
            S->TxState = phost->container.UCHAR4[0] & 0x01;
            S->GpsState = phost->container.UCHAR4[0] >> 1 & 0x03;
            S->PowerState = phost->container.UCHAR4[0] >> 3 & 0x01;
            S->State = phost->container.UCHAR4[2];
            S->ErrorCode = phost->container.UCHAR4[3];
            S->GpsAltitude = altitude;  // GPS altitude from flarm. TODO isu hat genauere GPS alt
            if (!(phost->container.UCHAR4[0] >> 4 & 0x01)) {
                O->AlarmLevel = 3;
                return true;
            }
            break;
        }

        case 1: {
            O->AlarmLevel = phost->container.UCHAR4[0] & 0x7;
            O->AlarmType = phost->container.CHAR4[0] >> 3 & 0x07;
            O->ID = phost->container.UCHAR3[0]; // TODO -- correct ???
            break;
        }

        case 2: {
            O->RelVertical = phost->container.SHORT2[0];
            O->RelHorizontal = phost->container.SHORT2[1];
            break;
        }

        case 3: {
            O->RelBearing = phost->container.SHORT;
            if (O->AlarmType == FLATOBSTACLEALARM) {
                O->RelBearing = 0;
                O->RelVertical = 0;
                O->ID = 0;
            }
            return true;
        }
    }
    return false;
}

/*
 * restauriert flarm FLAU daten auf einer sequenz von CANas messages
 * msg: die CAN message mit id FLARM_STATE_ID
 * E:   pointer auf FlarmMostImportantObjectData_t
 * S:   pointer auf FlarmDeviceState_t
 * return: true wenn daten komplet
 */
bool canasFlarmObjectPropagated(const CanasMessageData *phost, int heading, int canid, FlarmObjectData *E) {

    const CanasMessage *canasMessage = (const CanasMessage *) phost;
    int messageindex = canasMessage->service_code & 0x0f;

    unsigned validFlags = (canasMessage->service_code >> 4) & 0x0f;

    switch (messageindex) {
        case 0: {
            E->AlarmLevel = FLARM_OBJECT_AL0_ID - canid;
            E->RelNorth = phost->container.SHORT2[0];
            E->RelEast = phost->container.SHORT2[1];

/*
            E->RelHorizontal = (int16_t) isqrt3((uint32_t) ((int32_t) E->RelNorth * (int32_t) E->RelNorth +
                                                            (int32_t) E->RelEast * (int32_t) E->RelEast));
*/
            if (abs(heading) <= 360) {  // is not int_max
                heading = heading - calcBearing(E->RelEast, E->RelNorth);
                //heading = iroundf(angleWrap360((float) heading));
                E->RelBearing = heading;
            } else {
                E->RelBearing = 0;
            }
            break;
        }

        case 1: {
            E->RelVertical = phost->container.SHORT2[0];
            E->Track = phost->container.SHORT2[1];

            E->valid.track = (validFlags & 2) != 0;
            break;
        }
        case 2: {
            E->GroundSpeed = phost->container.UCHAR4[0];
            E->Type = phost->container.UCHAR4[1];
            E->ClimbRate = phost->container.UCHAR4[2];
            E->TurnRate = phost->container.UCHAR4[3];

            E->valid.groundSpeed = (validFlags & 1) != 0;
            E->valid.climbRate = (validFlags & 4) != 0;
            E->valid.turnRate = (validFlags & 8) != 0;
            break;

        }
        case 3: {
            E->IdType = phost->container.UCHAR;
            E->ID = phost->container.USHORT; // todo -- ??
            return true;
        }
    }
    return false;
}

static int16_t calcBearing(int RelEast, int RelNorth) {
    int16_t a = 0;

    if (RelNorth == 0 && RelEast == 0) {
        return (0);
    } else if (RelNorth == 0) {
        if (RelEast > 0)
            return (90);
        else
            return (-90);
    } else if (RelEast == 0) {
        return (0);
    } else {
        float x;
        x = (float) RelEast / (float) RelNorth;
        a = (int16_t) (atan(x) * (float) (360.0 / (2 * 3.14159)));
    }

    if (RelNorth < 0)
        a = (a + (int16_t) 180);
    else if (RelEast < 0)
        a = (a + (int16_t) 360);

    return (a);

}
