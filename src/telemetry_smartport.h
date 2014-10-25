/*
 * telemetry_smartport.h
 *
 *  Created on: 24 October 2014
 *      Author: Frank26080115
 */

#ifndef TELEMETRY_SMARTPORT_H_
#define TELEMETRY_SMARTPORT_H_

extern char smartPortHasTimedOut;

void handleSmartPortTelemetry(void);
void checkSmartPortTelemetryState(void);

void configureSmartPortTelemetryPort(void);
void freeSmartPortTelemetryPort(void);

#endif /* TELEMETRY_SMARTPORT_H_ */
