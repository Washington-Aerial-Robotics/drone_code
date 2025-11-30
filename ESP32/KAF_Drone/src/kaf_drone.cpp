#include "kaf_drone.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <math.h>
#include <stdio.h>
#endif

struct global_variables kafenv;

void kaf_dumpmemory() {
  //KF_REGVAR( 'c', "kafenv.u.deviceID", kafenv.u.deviceID );
}

bool kaf_triggercom() {
  kafenv.n.replySize = sizeof( kafenv.c.tx.data.bytes ) + 1;
  communicationStep();
  return kafenv.n.replySize <= sizeof( kafenv.c.tx.data.bytes );
}

void kaf_pidreset( pid* pid ) {
  pid->integ = 0;
  pid->deriv = 0; 
  pid->error = 0; 
  pid->desired = 0; 
  pid->prevmeas = NAN;
  float r = tan( ( (float)M_PI ) / pid->derlimit );
  pid->lowpass[0] = r * r / ( 1 + sqrt( 2 ) * r + r * r );
  pid->lowpass[1] = pid->lowpass[0] * ( 2 - 2 / ( r * r ) );
  pid->lowpass[2] = pid->lowpass[0] * ( 1 / ( r * r ) - sqrt( 2 ) / r + 1  );
  pid->lowpass[3] = 0;
  pid->lowpass[4] = 0;
}

float kaf_pidstep( pid* pid, float desired, float measured, float dt ) {
  pid->desired = desired;
  pid->error = pid->desired - measured;
  if( pid->error > pid->modula / 2 ) {
    pid->error = pid->error - pid->modula;
  } else if( pid->error < -pid->modula / 2 ) {
    pid->error = pid->error + pid->modula;
  }
  pid->deriv = ( pid->prevmeas - measured ) / dt;
  pid->prevmeas = measured;
  if( isfinite( pid->deriv ) ) {
    pid->deriv = 0;
  } else {
    float datalp0 = pid->deriv - pid->lowpass[1] * pid->lowpass[3] - pid->lowpass[2] * pid->lowpass[4];
    pid->deriv = pid->lowpass[0] * ( datalp0 + 2 * pid->lowpass[3] + pid->lowpass[4] );
    pid->lowpass[4] = pid->lowpass[3];
    pid->lowpass[3] = datalp0;
  }
  pid->integ = pid->integ + pid->error * dt;
  KF_BOUND( pid->integ, -pid->intlimit, pid->intlimit );
  float output = pid->Kp * pid->error + pid->Ki * pid->integ + pid->Kd * pid->deriv + pid->Kf * pid->desired;
  KF_BOUND( output, -pid->outlimit, pid->outlimit );
  return output;
}

unsigned char kaf_getdevice( unsigned char deviceID ) {
  unsigned char index = kafenv.n.deviceLookup[deviceID] - 1;
  if( kafenv.n.deviceLookup[deviceID] == 0 ) {
    if( kafenv.n.deviceCount < DEVICE_COUNT ) {
      index = kafenv.n.deviceCount;
      kafenv.n.deviceCount++;
    } else {
      unsigned long disconnectTime = kafenv.n.currentTime - 8000;
      for( index = 0; index < DEVICE_COUNT - 1; index++ ) {
        if( kafenv.n.devices[index].lastSeen < disconnectTime ) {
          break;
        }
      }
    }
    kafenv.n.deviceLookup[ kafenv.n.devices[index].deviceID ] = 0;
    kafenv.n.deviceLookup[deviceID] = index + 1;
    kafenv.n.devices[index].deviceID = deviceID;
  }
  return index;
}