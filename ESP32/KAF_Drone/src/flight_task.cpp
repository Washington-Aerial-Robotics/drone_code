#include "kaf_drone.h"
#include "communication.h"

void controlsInit() {
  kafenv.s.flightMode = NULL_MODE;
  KF_VEC3( i, kaf_pidreset( &( kafenv.s.positionPID[i] ) ) );
  KF_VEC3( i, kaf_pidreset( &( kafenv.s.velocityPID[i] ) ) );
  KF_VEC3( i, kaf_pidreset( &( kafenv.s.attitudePID[i] ) ) );
  KF_VEC3( i, kaf_pidreset( &( kafenv.s.attiratePID[i] ) ) );
}

void controlsStep() {
  switch( kafenv.s.flightMode ) {
    case NULL_MODE : {
      KF_VECN( i, MOTOR_COUNT, kafenv.f.motorOutput[i] = 0 );
      break;
    }
    case TRAJECTORY_MODE : {
    }
    case POS_SETPOINT_MODE : {
      float timeStep = kafenv.f.timeStep * 1e-6F;
      KF_VEC3( i, kafenv.s.cascadeSetpoint[i] = kaf_pidstep( &( kafenv.s.positionPID[i] ), kafenv.s.posSetpoint[i],     kafenv.u.stateEstimate.x.f[i], timeStep ) );
      KF_VEC3( i, kafenv.s.cascadeSetpoint[i] = kaf_pidstep( &( kafenv.s.velocityPID[i] ), kafenv.s.cascadeSetpoint[i], kafenv.u.stateEstimate.v.f[i], timeStep ) );
      //wip
      KF_VEC3( i, kafenv.s.cascadeSetpoint[i] = kaf_pidstep( &( kafenv.s.attitudePID[i] ), kafenv.s.cascadeSetpoint[i], kafenv.u.stateEstimate.t.f[i], timeStep ) );
      KF_VEC3( i, kafenv.s.cascadeSetpoint[i] = kaf_pidstep( &( kafenv.s.attiratePID[i] ), kafenv.s.cascadeSetpoint[i], kafenv.u.stateEstimate.w.f[i], timeStep ) );
      //wip
    }
    case MOTOR_SETPOINT_MODE : {
      KF_VECN( i, MOTOR_COUNT, kafenv.f.motorOutput[i] = kafenv.s.motorSetpoint[i] );
    }
    default : {}
  }
}