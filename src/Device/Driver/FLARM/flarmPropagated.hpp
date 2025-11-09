/*
 * Copyright (C) 2010-2013 triadis engineering GmbH all rights reserved.
 *
 * flarmPropagated.hpp
 *
 * 
 *
 * Created on: 14.03.2013
 *     Author: sam
 */

#ifndef FLARMPROPAGATED_H
#define FLARMPROPAGATED_H

#include <Device/Driver/CANaerospace/marshal.h>
#include <Device/Driver/CANaerospace/FlarmMessage.h>


#ifdef __cplusplus
extern "C"
{
#endif

#define FLATOBSTACLEALARM 3

bool
canasFlarmStatePropagated(const CanasMessage *phost, int altitude, FlarmMostImportantObjectData *objectData, FlarmState *flarmState);
bool canasFlarmObjectPropagated(const CanasMessage *phost, int canid, FlarmObjectData *flarmObjectData);


#ifdef __cplusplus
}
#endif

#endif // FLARMPROPAGATED_H
