/*
 * Copyright (C) 2010-2012 triadis engineering GmbH all rights reserved.
 *
 * flarm_datatypes.h
 *
 * 
 *
 * Created on: 2012-10-1
 *     Author: sam
 */

#ifndef FLARM_DATATYPES_H
#define FLARM_DATATYPES_H

#ifndef FLARM_USEBITFIELDS
#define FLARM_USEBITFIELDS   0
#endif

#ifdef __cplusplus
extern "C"
{
#endif


typedef enum {
    flTxNo,
    flTxOK
} FlarmTransmitionState_t;

typedef enum {
    flCmText,
    flCmBinary
} FlarmComMode_t;

typedef enum {
    flGpsNo,
    flGpsOK
} FlarmGpsState_t;

typedef enum {
    flPwUnderOverVoltage,
    flPwOK
} FlarmPowerState_t;

typedef enum {
    flAtAircraftTraffic,
    flAtSilentAircraftTraffic,
    flAtAircraftAlarm,
    flAtObstacleAlarm
} FlarmAlarmType_t;

typedef enum {
    flIdStateless,
    flIdICAO,
    flIdPseudo,
    flIdStatic
} FlarmIdType_t;

typedef enum {
    flOtUnknown,
    flOtGlider,
    flOtTwoPlane,
    flOtHelicopter,
    flOtParachute,
    flOtDropPlane,
    flOtFixedHangGlider,
    flOtSoftParaGlider,
    flOtEngine,
    flOtJet,
    flOtUFO,
    flOtBaloon,
    flOtZeppelin,
    flOtUAV,
    flOtStatic = 0x0f
} FlarmObjectType_t;

typedef enum {
    flStNormalOperation,
    flStInfoNormalOperation,
    flStReducedOper,
    flStFatalProblem
} FlarmState_t;

typedef enum {
    flAlNo,
    flAlLow,
    flAlImportant,
    flAlUrgent
} FlarmAlarmLevel_t;

typedef enum {
    flColdReset = 0,
    flSilentColdReset = 1,
    flFactoryDefaultReset = 99
} FlarmReset_t;

// foreward declaration of message object pointer
struct FlarmObjectMessage_t;
typedef struct FlarmObjectMessage_t FlarmObjectMessage_t;

struct FlarmObjectData {
    ListItem_t ListPtr;                                // must be the first element in struct!!!
#if (FLARM_USEBITFIELDS)
    signed            AlarmLevel:4;
    signed            newAlarmLevel:4;
    signed            IdType:4;
    signed            MessageAckLevel:4;
    unsigned          Type:4;
    unsigned          Fill:4;
#else
    int8_t AlarmLevel;
    int8_t newAlarmLevel;
    int8_t IdType;
    int8_t MessageAckLevel;
    uint8_t Type;
#endif
    int16_t RelNorth;
    int16_t RelEast;
    int16_t RelVertical;
    int16_t RelHorizontal;
    int16_t RelBearing;
    uint32_t ID;
    int16_t Track;
    int8_t TurnRate;
    int8_t GroundSpeed;
    int8_t ClimbRate;      // in cm/s!!!!!!!!!
    int8_t TimeOutCnt;
    struct {
        unsigned track: 1;
        unsigned turnRate: 1;
        unsigned groundSpeed: 1;
        unsigned climbRate: 1;
    } valid;
    FlarmObjectMessage_t *MessagePtr;
    /*  ungeschützte private daten
     *  E->newAlarmLevel = 0;
     *  E->MessageAckLevel = 0;
     *  E->TimeOutCnt = 0;
     *  E->ListPtr = 0;
     *  E->MessagePtr = 0;
     */

};

typedef struct FlarmObjectData FlarmObjectData_t;

struct FlarmState {
    uint32_t TmLastReceive;
    unsigned flarmId;
    int16_t GpsAltitude;
    uint8_t RxDevicesCount;
    uint8_t TxState;
    uint8_t GpsState;
    uint8_t PowerState;
    uint8_t State;
    uint8_t ErrorCode;
};

typedef struct FlarmState FlarmDeviceState_t;

typedef struct {
#if (FLARM_USEBITFIELDS)
    signed            AlarmLevel:4;
    signed            AlarmType:4;
    unsigned          TargetVectorValid:1;
#else
    int8_t AlarmLevel;
    int8_t AlarmType;
    int8_t TargetVectorValid: 1;
#endif
    int16_t RelVertical;
    int16_t RelHorizontal;
    int16_t RelBearing;
    uint32_t ID;
} FlarmMostImportantObjectData_t;


#ifdef __cplusplus
}
#endif


#endif // FLARM_DATATYPES_H
