//
// Created by andreas on 01.05.20.
//

#pragma once

typedef struct {
    unsigned flarmId;
    int16_t GpsAltitude;
    uint8_t RxDevicesCount;
    uint8_t TxState;
    uint8_t GpsState;
    uint8_t PowerState;
    uint8_t State;
    uint8_t ErrorCode;
} FlarmState;

typedef struct {
    int8_t AlarmLevel;
    int8_t AlarmType;
    int8_t TargetVectorValid: 1;
    int16_t RelVertical;
    int16_t RelHorizontal;
    int16_t RelBearing;
    uint32_t ID;
} FlarmMostImportantObjectData;

struct FlarmObjectData {
    int8_t AlarmLevel;
    int8_t newAlarmLevel;
    int8_t IdType;
    int8_t MessageAckLevel;
    uint8_t Type;
    int16_t RelNorth;
    int16_t RelEast;
    int16_t RelVertical;
    int16_t RelHorizontal;
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
};