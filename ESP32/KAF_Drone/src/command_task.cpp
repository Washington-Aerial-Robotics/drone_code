#include "kaf_drone.h"
#include "communication.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <string.h>
#endif

void communicationStep() {
  if( kafenv.c.rx.data.toID == kafenv.u.deviceID ) { // respond to requests and commands
    kafenv.c.tx.data.fromID = kafenv.u.deviceID;
    kafenv.c.tx.data.toID = kafenv.c.rx.data.fromID;
    switch( kafenv.c.rx.data.messageType ) {
      case COM_ACK : case COM_NACK : case COM_PONG: break;
      case COM_PING : {
        kafenv.c.tx.data.messageType = COM_PONG;
        kafenv.n.replySize = 0;
        break;
      }
      case COM_TRIGGER : {
        kafenv.p.trigger = kafenv.c.rx.data.byte;
        kafenv.c.tx.data.messageType = COM_ACK;
        kafenv.n.replySize = 0;
        break;
      }
      case COM_REQUEST_DEVICES : {
        int offset = kafenv.c.rx.data.byte;
        kafenv.c.tx.data.messageType = COM_REPLY_DEVICES;
        for( int i = 0; i < sizeof( kafenv.c.tx.data.deviceList.deviceID ) && i + offset < DEVICE_COUNT; i++ ) {
          kafenv.c.tx.data.deviceList.deviceID[i] = kafenv.n.devices[ i + offset ].deviceID;
          kafenv.c.tx.data.deviceList.timeLastSeen[i] = ( unsigned int )( kafenv.n.currentTime - kafenv.n.devices[ i + offset ].lastSeen );
        }
        kafenv.n.replySize = sizeof( kafenv.c.tx.data.deviceList );
        break;
      }
      case COM_REQUEST_STATE : {
        kafenv.c.tx.data.messageType = COM_REPLY_STATE;
        kafenv.c.tx.data.coord = kafenv.u.stateEstimate.x;
        kafenv.n.replySize = sizeof( kafenv.c.tx.data.coord );
        break;
      }
      case COM_REQUEST_WIFI_IP : {
        kafenv.c.tx.data.messageType = COM_REPLY_WIFI_IP;
        memcpy( kafenv.c.tx.data.bytes, kafenv.n.networkAddress, strlen( kafenv.n.networkAddress ) );
        kafenv.n.replySize = strlen( kafenv.n.networkAddress );
        break;
      }
      case COM_REQUEST_POS : {
        kafenv.c.tx.data.messageType = COM_REPLY_POS;
        kafenv.c.tx.data.coord = kafenv.u.stateEstimate.x;
        kafenv.n.replySize = sizeof( kafenv.c.tx.data.coord );
        break;
      }
      case COM_REQUEST_GLOBAL : {
        unsigned int startPos = kafenv.c.rx.data.globalVarTransfer.startPos;
        int length = kafenv.c.rx.data.globalVarTransfer.length;
        if( startPos >= 0 && startPos + length < sizeof( kafenv ) && length > 0 && length <= sizeof( kafenv.c.tx.data.bytes ) ) {
          kafenv.c.tx.data.messageType = COM_REQUEST_GLOBAL;
          memcpy( kafenv.c.tx.data.bytes, ( (char*)(void*)( &kafenv ) ) + startPos, length );
          kafenv.n.replySize = length;
        } else {
          kafenv.c.tx.data.messageType = COM_NACK;
          kafenv.n.replySize = 0;
        }
        break;
      }
      case COM_REQUEST_ST_EST : {
        kafenv.c.tx.data.messageType = COM_REPLY_ST_EST;
        kafenv.c.tx.data.stateEst = kafenv.u.stateEstimate;
        kafenv.n.replySize = sizeof( kafenv.c.tx.data.stateEst );
        break;
      }
      case COM_SET_GLOBAL : {
        unsigned int startPos = kafenv.c.rx.data.globalVarTransfer.startPos;
        int length = kafenv.c.rx.data.globalVarTransfer.length;
        if( startPos >= 0 && startPos + length < sizeof( kafenv ) && length > 0 && length <= sizeof( kafenv.c.tx.data.globalVarTransfer.bytes ) ) {
          kafenv.c.tx.data.messageType = COM_ACK;
          memcpy( ( (char*)(void*)( &kafenv ) ) + startPos, kafenv.c.rx.data.globalVarTransfer.bytes, length );
        } else {
          kafenv.c.tx.data.messageType = COM_NACK;
        }
        kafenv.n.replySize = 0;
        break;
      }
      case COM_SET_POS : {
        kafenv.u.stateEstimate.x = kafenv.c.rx.data.coord;
        kafenv.c.tx.data.messageType = COM_ACK;
        kafenv.n.replySize = 0;
        break;
      }
      case COM_SET_ST_EST : {
        kafenv.u.stateEstimate = kafenv.c.rx.data.stateEst;
        kafenv.c.tx.data.messageType = COM_ACK;
        kafenv.n.replySize = 0;
        break;
      }
      case COM_SET_CTRL_MODE : {
        unsigned char flightMode = kafenv.c.rx.data.byte;
        if( flightMode <= FLIGHT_MODE_BOUNDS ) {
          kafenv.s.flightMode = flightMode;
          kafenv.c.tx.data.messageType = COM_ACK;
        } else{
          kafenv.c.tx.data.messageType = COM_NACK;
        }
        kafenv.n.replySize = 0;
        break;
      }
      case COM_SET_MOTOR_CMD : {
        kafenv.s.flightMode = MOTOR_SETPOINT_MODE;
        KF_VECN( i, 4, kafenv.s.motorSetpoint[i] = kafenv.c.rx.data.coord.f[i] );
        kafenv.c.tx.data.messageType = COM_ACK;
        kafenv.n.replySize = 0;
        break;
      }
      case COM_SET_POS_CMD : {
        kafenv.s.flightMode = POS_SETPOINT_MODE;
        KF_VEC3( i, kafenv.s.posSetpoint[i] = kafenv.c.rx.data.coord.f[i] );
        kafenv.c.tx.data.messageType = COM_ACK;
        kafenv.n.replySize = 0;
        break;
      }
      default : {
        if( kafenv.c.rx.data.messageType < 0x10 || kafenv.c.rx.data.messageType > 0x1F ) {
          kafenv.c.tx.data.messageType = COM_NACK;
          kafenv.n.replySize = 0;
        }
      }
    }
  } else { // eavesdrop
  }
}