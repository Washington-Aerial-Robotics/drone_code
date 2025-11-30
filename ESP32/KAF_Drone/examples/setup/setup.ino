#include "kaf_drone.h"
#include "string.h"

//DEBUG PRINT_________________________________________________________________________________________________
#define RECORD_COUNT 1000
static struct {
  int burstID = RECORD_COUNT + 1;
  int readerID = RECORD_COUNT + 1;
  int burstTime[RECORD_COUNT];
  float burstData[6 * RECORD_COUNT];
} debugger;

static void loopDebugFlight() {
  if( debugger.burstID < RECORD_COUNT ) {
    debugger.burstTime[debugger.burstID] = kafenv.f.currentTime;
    KF_VEC3( q, debugger.burstData[ debugger.burstID * 6 + q ] = kafenv.f.accelInput.f[q] );
    KF_VEC3( q, debugger.burstData[ debugger.burstID * 6 + 3 + q ] = kafenv.f.gyroInput.f[q] );
    debugger.burstID++;
  }
}

static void loopDebugCommand() {
  if( debugger.readerID < RECORD_COUNT ) {
    Serial.printf( "%u", debugger.burstTime[debugger.readerID] );
    KF_VECN( q, 6, Serial.printf( ",%f", debugger.burstData[ debugger.readerID * 6 + q ] ) );
    Serial.printf( "\n" );
    debugger.readerID++;
  }
  if( kafenv.p.trigger == 0x82 ) {
    debugger.readerID = 0;
    debugger.burstID = 0;
    kafenv.p.trigger = 0;
  }
}

static void initDebug( int id ) {
  strcpy( kafenv.p.peripheral[id].name, "DW3000" );
  kafenv.p.peripheral[id].enabled = true;
  kafenv.p.peripheral[id].loopType = 3;
  kafenv.p.peripheral[id].dataSize = sizeof( debugger );
  kafenv.p.peripheral[id].initFunction = &initDebug;
  kafenv.p.peripheral[id].loopFunction = &loopDebugFlight;
  kafenv.p.peripheral[id].auxLoopFunction = &loopDebugCommand;
  kafenv.p.peripheral[id].data = &debugger;
}

void setup() {
  kafenv.u.deviceID                 = 'A';
  kafenv.n.comTaskDelay             = 0;
  strcpy( kafenv.n.networkAddress,    "iPhone" );
  strcpy( kafenv.n.networkPassword,   "password" );
  initDebug( 6 );
  firmwareSetup();
}
void loop() {}