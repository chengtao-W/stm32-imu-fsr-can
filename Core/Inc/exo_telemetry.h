#ifndef __EXO_TELEMETRY_H
#define __EXO_TELEMETRY_H

/** Initialize the sensor-only telemetry module. */
void ExoTelemetry_Init(void);

/** Assemble and send IMU and foot-pressure telemetry. */
void ExoTelemetry_Push(void);

#endif
