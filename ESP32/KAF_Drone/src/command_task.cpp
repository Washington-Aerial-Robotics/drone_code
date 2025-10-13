#include "kaf_drone.h"
#include <stdio.h>
#include <string.h>

#define D2D_TIMEOUT         8000//ms device invalidation timeout period
//communication ids
#define COM_ACK             0x01//
#define COM_NACK            0x02//
#define COM_PING            0x03//
#define COM_PONG            0x04//
#define COM_REQUEST_DEVICES 0x20//
#define COM_REPLY_DEVICES   0x21//
#define COM_REQUEST_STATE   0x22//
#define COM_REPLY_STATE     0x23//
#define COM_REQUEST_GLOBAL  0x24//
#define COM_REPLY_GLOBAL    0x25//
#define COM_SET_GLOBAL      0x26//
#define COM_REQUEST_ST_EST  0x30
#define COM_REPLY_ST_EST    0x31
#define COM_SET_ST_EST      0x32
#define COM_REQUEST_POS     0x34//
#define COM_REPLY_POS       0x35//
#define COM_SET_POS         0x36//
#define COM_REQUEST_POSANG  0x38
#define COM_REPLY_POSANG    0x39
#define COM_SET_POSANG      0x3A
#define COM_SET_NULL_MODE   0x40
#define COM_SET_MOTOR_MODE  0x42
#define COM_SET_MOTOR_CMD   0x43
#define COM_SET_ANGLE_MODE  0x44
#define COM_SET_ANGLE_CMD   0x45
#define COM_SET_VELOC_MODE  0x46
#define COM_SET_VELOC_CMD   0x47
#define COM_SET_POS_MODE    0x48
#define COM_SET_POS_CMD     0x49
#define COM_SET_TRAJ_MODE   0x4A
#define COM_SET_TRAJ_CMD    0x4B
#define COM_REQUEST_WIFI_IP 0x50//
#define COM_REPLY_WIFI_IP   0x51//
#define COM_REQUEST_FWDMSG  0x52
#define COM_REPLY_FWDMSG    0x53

unsigned char getDeviceIndex( unsigned char deviceID ) {
  unsigned char index = kafenv.c.deviceLookup[deviceID] - 1;
  if( kafenv.c.deviceLookup[deviceID] == 0 ) {
    if( kafenv.c.deviceCount < DEVICE_COUNT ) {
      index = kafenv.c.deviceCount;
      kafenv.c.deviceCount++;
    } else {
      unsigned long disconnectTime = kafenv.s.rtos.task[CMD_TASK].currentTime - D2D_TIMEOUT;
      for( index = 0; index < DEVICE_COUNT - 1; index++ ) {
        if( kafenv.c.devices[index].lastSeen < disconnectTime ) {
          break;
        }
      }
    }
    kafenv.c.deviceLookup[ kafenv.c.devices[index].deviceID ] = 0;
    kafenv.c.deviceLookup[deviceID] = index + 1;
    kafenv.c.devices[index].deviceID = deviceID;
  }
  return index;
}

void handleTransmission( void(*sendFunc)( int ) ) {
  if( kafenv.c.rx.data.toID == kafenv.u.deviceID ) { // respond to requests and commands
    kafenv.c.tx.data.toID = kafenv.c.rx.data.fromID;
    switch( kafenv.c.rx.data.messageType ) {
      case COM_ACK : case COM_NACK : case COM_PONG: break;
      case COM_PING : {
        kafenv.c.tx.data.messageType = COM_PONG;
        sendFunc( 0 );
        break;
      }
      case COM_REQUEST_DEVICES : {
        int offset = kafenv.c.rx.data.byte;
        kafenv.c.tx.data.messageType = COM_REPLY_DEVICES;
        for( int i = 0; i < sizeof( kafenv.c.tx.data.deviceList.deviceID ) && i + offset < DEVICE_COUNT; i++ ) {
          kafenv.c.tx.data.deviceList.deviceID[i] = kafenv.c.devices[ i + offset ].deviceID;
          kafenv.c.tx.data.deviceList.timeLastSeen[i] = ( unsigned int )( kafenv.s.rtos.task[CMD_TASK].currentTime - kafenv.c.devices[ i + offset ].lastSeen );
        }
        sendFunc( sizeof( kafenv.c.tx.data.deviceList ) );
        break;
      }
      case COM_REQUEST_STATE : {
        kafenv.c.tx.data.messageType = COM_REPLY_STATE;
        kafenv.c.tx.data.coord = kafenv.u.stateEstimate.x;
        sendFunc( sizeof( kafenv.c.tx.data.coord ) );
        break;
      }
      case COM_REQUEST_WIFI_IP : {
        if( kafenv.s.wifi.working ) {
          kafenv.c.tx.data.messageType = COM_REPLY_WIFI_IP;
          memcpy( kafenv.c.tx.data.bytes, kafenv.s.wifi.localIP, strlen( kafenv.s.wifi.localIP ) );
          sendFunc( strlen( kafenv.s.wifi.localIP ) );
        } else {
          kafenv.c.tx.data.messageType = COM_NACK;
          sendFunc( 0 );
        }
        break;
      }
      case COM_REQUEST_POS : {
        kafenv.c.tx.data.messageType = COM_REPLY_POS;
        kafenv.c.tx.data.coord = kafenv.u.stateEstimate.x;
        sendFunc( sizeof( kafenv.c.tx.data.coord ) );
        break;
      }
      case COM_REQUEST_GLOBAL : {
        unsigned int startPos = kafenv.c.rx.data.globalVarTransfer.startPos;
        int length = kafenv.c.rx.data.globalVarTransfer.length;
        if( startPos >= 0 && startPos + length < sizeof( kafenv ) && length > 0 && length <= sizeof( kafenv.c.tx.data.bytes ) ) {
          kafenv.c.tx.data.messageType = COM_REQUEST_GLOBAL;
          memcpy( kafenv.c.tx.data.bytes, ( (char*)(void*)( &kafenv ) ) + startPos, length );
          sendFunc( length );
        } else {
          kafenv.c.tx.data.messageType = COM_NACK;
          sendFunc( 0 );
        }
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
        sendFunc( 0 );
        break;
      }
      case COM_SET_POS : {
        kafenv.u.stateEstimate.x = kafenv.c.rx.data.coord;
        kafenv.c.tx.data.messageType = COM_ACK;
        sendFunc( 0 );
        break;
      }
      default : {
        kafenv.c.tx.data.messageType = COM_NACK;
        sendFunc( 0 );
      }
    }
  } else { // eavesdrop
  }
}

void debugUpdate() {
  printf( "TIME: cmd=%d, flt=%d\n", kafenv.s.rtos.task[0].currentCycle, kafenv.s.rtos.task[1].currentCycle );
  for( int i = 0; i < kafenv.c.deviceCount; i++ ) {
    if( kafenv.s.rtos.task[CMD_TASK].currentTime - kafenv.c.devices[i].lastSeen < D2D_TIMEOUT ) {
      printf( "DEVICE %d: { ID: %c, lu=%lu, x=<%.2f,%.2f,%.2f>, d=%f }\n", 
        i,
        kafenv.c.devices[i].deviceID,
        kafenv.c.devices[i].lastSeen,
        kafenv.c.devices[i].position.x,
        kafenv.c.devices[i].position.y,
        kafenv.c.devices[i].position.z,
        kafenv.c.devices[i].distance );
    }
  }
}