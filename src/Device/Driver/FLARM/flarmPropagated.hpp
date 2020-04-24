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


#ifdef __cplusplus
extern "C"
{
#endif

#define FLATOBSTACLEALARM 3

bool
canasFlarmStatePropagated(const CanasMessageData *phost, int altitude, FlarmMostImportantObjectData *O, FlarmState *S);
bool canasFlarmObjectPropagated(const CanasMessageData *phost, int heading, int canid, FlarmObjectData *E);


#ifdef __cplusplus
}
#endif

#endif // FLARMPROPAGATED_H
