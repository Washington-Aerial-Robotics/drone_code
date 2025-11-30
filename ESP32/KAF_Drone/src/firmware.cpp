#include "kaf_drone.h"
#include "communication.h"
#if FIRMWARE_ENABLE
#if ALT_DEFINE
#include "altdef.h"
#else
#include "lib/ESP32Servo/ESP32Servo.h"
#include "lib/Dw3000/dw3000.h"
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <stdio.h>
#endif

//PIN DEFINITIONS_____________________________________________________________________________________________
#define DW_IRQ   34
#define DW_RST   27
#define DW_SS     4
#define MPU_SDA  21
#define MPU_SCL  22
#define ESC_PINS 33, 35, 25, 26

//FIRMWARE____________________________________________________________________________________________________
#define RTOS_FLIGHT_PERIOD       4 //ms
#define NULL_LOOP                0
#define FLIGHT_LOOP              1
#define COMMAND_LOOP             2
#define BOTH_LOOP                3

static struct {
  unsigned char numInits = 0;
  void( *initFuncs[PERIPHERAL_COUNT] )( int );
  unsigned char numComs = 0;
  void( *comFuncs[PERIPHERAL_COUNT] )();
  unsigned char numFlights = 0;
  void( *flightFuncs[PERIPHERAL_COUNT] )();
} firmware;

static void loopFirmwareCommand() {
  vTaskDelay( 1 );
  kafenv.n.currentTime = millis();
  delay( kafenv.n.comTaskDelay );
#if ALLOW_CUSTOM_FIRMWARE
  for( int i = 0; i < firmware.numComs; i++ ) {
    firmware.comFuncs[i]();
  }
#endif
}

static void loopFirmwareFlight() {
  vTaskDelay( 0 );
  unsigned long oldTime = kafenv.f.currentTime;
  int delayTime = RTOS_FLIGHT_PERIOD - (int)( millis() - oldTime );
  if( delayTime > 0 ) {
    delay( delayTime );
  }
  unsigned long newTime = millis();
  kafenv.f.currentTime = newTime;
  kafenv.f.timeStep = newTime - oldTime;
#if ALLOW_CUSTOM_FIRMWARE
  for( int i = 0; i < firmware.numFlights; i++ ) {
    firmware.flightFuncs[i]();
  }
#endif
}

static void initFirmware( int id ) {
  strcpy( kafenv.p.peripheral[id].name, "firmware" );
  kafenv.p.peripheral[id].enabled = true;
  kafenv.p.peripheral[id].loopType = NULL_LOOP;
  kafenv.p.peripheral[id].dataSize = sizeof( firmware );
  kafenv.p.peripheral[id].initFunction = &initFirmware;
  kafenv.p.peripheral[id].loopFunction = &loopFirmwareFlight;
  kafenv.p.peripheral[id].auxLoopFunction = &loopFirmwareCommand;
  kafenv.p.peripheral[id].data = &firmware;
  kafenv.p.version = 1;
  kafenv.p.trigger = 0;
  for( int i = 0; i < PERIPHERAL_COUNT; i++ ) {
    if( kafenv.p.peripheral[i].enabled ) {
      if( kafenv.p.peripheral[i].initFunction != NULL ) {
        firmware.initFuncs[ firmware.numInits++ ] = kafenv.p.peripheral[i].initFunction;
      }
      if( kafenv.p.peripheral[i].loopFunction != NULL ) {
        if( kafenv.p.peripheral[i].loopType == FLIGHT_LOOP ) {
          firmware.flightFuncs[ firmware.numFlights++ ] = kafenv.p.peripheral[i].loopFunction;
        } else if( kafenv.p.peripheral[i].loopType == COMMAND_LOOP ) {
          firmware.comFuncs[ firmware.numComs++ ] = kafenv.p.peripheral[i].loopFunction;
        }
      }
      if( kafenv.p.peripheral[i].auxLoopFunction != NULL && kafenv.p.peripheral[i].loopType == BOTH_LOOP ) {
        firmware.flightFuncs[ firmware.numFlights++ ] = kafenv.p.peripheral[i].loopFunction;
        firmware.comFuncs[ firmware.numComs++ ] = kafenv.p.peripheral[i].auxLoopFunction;
      }
    }
  }
  Serial.printf( "ESP32 Drone Entering Nominal Operations\n" );
  delay( 1000 );
}

//ESCS________________________________________________________________________________________________________
#define ESC_MAX               2000
#define ESC_MIN               1000
#define ESC_ARM                500
#define ESC_RAMP                10

static struct {
  unsigned int pins[MOTOR_COUNT] = { ESC_PINS };
  unsigned short setpoints[MOTOR_COUNT];
  Servo servos[MOTOR_COUNT];
} escs;

static void loopESCs() {
  for( int i = 0; i < MOTOR_COUNT; i++ ) {
    unsigned short value = (unsigned short)( ( ESC_MAX - ESC_MIN ) * kafenv.f.motorOutput[i] + ESC_MIN ) - escs.setpoints[i];
    KF_BOUND( value, -ESC_RAMP, ESC_RAMP );
    value = escs.setpoints[i] + value;
    KF_BOUND( value, ESC_MIN, ESC_MAX );
    if( value != escs.setpoints[i] ) {
      escs.setpoints[i] = value;
      escs.servos[i].writeMicroseconds( value );
    }
  }
}

static void initESCs( int id ) {
  strcpy( kafenv.p.peripheral[id].name, "ESCs" );
  kafenv.p.peripheral[id].enabled = true;
  kafenv.p.peripheral[id].loopType = NULL_LOOP;
  kafenv.p.peripheral[id].dataSize = sizeof( escs );
  kafenv.p.peripheral[id].initFunction = &initESCs;
  kafenv.p.peripheral[id].loopFunction = &loopESCs;
  kafenv.p.peripheral[id].auxLoopFunction = NULL;
  kafenv.p.peripheral[id].data = &escs;
  //arm all the pins and set default values
  for( int i = 0; i < MOTOR_COUNT; i++ ) {
    pinMode( escs.pins[i], OUTPUT );
    escs.servos[i].attach( escs.pins[i] );
    escs.servos[i].writeMicroseconds( ESC_ARM );
    escs.setpoints[i] = ESC_ARM;
  }
}

//MPU6050_____________________________________________________________________________________________________
#define MPU_I2C               0x68
#define MPU_READ_SIZE           14
#define MPU_ACCEL_RANGE       0x10 //0x00              0x08             0x10              0x18
#define MPU_GYRO_RANGE        0x10 //0x00              0x08             0x10              0x18
#define MPU_ACCEL_SCALE        1.0 //2g       16384.0, 4g       8192.0, 8g        4096.0, 16g,      2048.0
#define MPU_GYRO_SCALE         1.0 //250deg/s 131.072, 500deg/s 65.536, 1000deg/s 32.768, 2000deg/s 16.384

static struct {
  bool working = false;
  unsigned int missCount = 0;
  float accelCalib[3] = { 0, 0, 0 };
  float angvelCalib[3] = { 0, 0, 0 };
  union {
    short imuData[ MPU_READ_SIZE / 2 ];
    unsigned char imuBytes[MPU_READ_SIZE];
  };
} mpu;

static void loopMPU6050() {
  if( mpu.working ) {
    Wire.beginTransmission( MPU_I2C );
    Wire.write( 0x3B );
    Wire.endTransmission( false );
    if( Wire.requestFrom( MPU_I2C, MPU_READ_SIZE, true ) == MPU_READ_SIZE ) {
      Wire.readBytes( mpu.imuBytes, MPU_READ_SIZE );
      for( int i = 0; i < MPU_READ_SIZE / 2; i++ ) {
        unsigned char temp = mpu.imuBytes[i];
        mpu.imuBytes[i] = mpu.imuBytes[ i + 1 ];
        mpu.imuBytes[ i + 1 ] = temp;
      }
      KF_VEC3( q, kafenv.f.accelInput.f[q] = mpu.imuData[q] / MPU_ACCEL_SCALE - mpu.accelCalib[q] );
      KF_VEC3( q, kafenv.f.gyroInput.f[q] = mpu.imuData[ 4 + q ] / MPU_GYRO_SCALE - mpu.angvelCalib[q] );
      mpu.missCount = 0;
    } else if( ++mpu.missCount > 50 ) {
      mpu.working = false;
    }
  } else {
    KF_VEC3( q, kafenv.f.accelInput.f[q] = 0 );
    KF_VEC3( q, kafenv.f.gyroInput.f[q] = 0 );
  }
}

static void initMPU6050( int id ) {
  strcpy( kafenv.p.peripheral[id].name, "MPU6050" );
  kafenv.p.peripheral[id].enabled = true;
  kafenv.p.peripheral[id].loopType = NULL_LOOP;
  kafenv.p.peripheral[id].dataSize = sizeof( mpu );
  kafenv.p.peripheral[id].initFunction = &initMPU6050;
  kafenv.p.peripheral[id].loopFunction = &loopMPU6050;
  kafenv.p.peripheral[id].auxLoopFunction = NULL;
  kafenv.p.peripheral[id].data = &mpu;
  //init wire
  Wire.begin( MPU_SDA, MPU_SCL );
  Wire.setTimeOut( 50 );
  //write a reset command
  Wire.beginTransmission( MPU_I2C );
  Wire.write( 0x6B );
  Wire.write( 0x00 );
  int success = Wire.endTransmission( true );
  delay( 20 );
  //configure acceleration
  Wire.beginTransmission( MPU_I2C );
  Wire.write( 0x1C );
  Wire.write( MPU_ACCEL_RANGE );
  Wire.endTransmission( true );
  //configure acceleration
  Wire.beginTransmission( MPU_I2C );
  Wire.write( 0x1B );
  Wire.write( MPU_GYRO_RANGE );
  Wire.endTransmission( true );
  mpu.working = success == 0;
}

//WIFI________________________________________________________________________________________________________
static struct {
  bool wifiConnected = false;
  bool clientConnected = false;
  WiFiServer server;
  WiFiClient client;
} wifi;

static void loopWiFi() {
  bool lastWorking = wifi.wifiConnected;
  wifi.wifiConnected = WiFi.status() == WL_CONNECTED;
  if( wifi.wifiConnected ) {
    if( !lastWorking ) {
      strcpy( kafenv.n.networkAddress, WiFi.localIP().toString().c_str() );
      wifi.server.begin();
#if DEBUGPRINT
      Serial.printf( "WiFi started successfully at IP: %s\n", kafenv.n.networkAddress );
#endif
    }
    wifi.clientConnected = wifi.client || ( wifi.client = wifi.server.accept() );
    if( wifi.clientConnected && wifi.client.available() > 0 ) {
      wifi.client.setTimeout( 100 );
      int length = wifi.client.available();
      length = length < sizeof( kafenv.c.rx.raw ) ? length : sizeof( kafenv.c.rx.raw );
      wifi.client.readBytes( kafenv.c.rx.raw, length );
      if( kaf_triggercom() ) {
        wifi.client.write( kafenv.c.tx.raw, COM_HEADER_LEN + kafenv.n.replySize );//2ms
      }
    }
  } else if( kafenv.n.attemptReconnect ) {
    WiFi.begin( kafenv.n.networkName, kafenv.n.networkPassword );
    kafenv.n.attemptReconnect = false;
  }
}

static void initWiFi( int id ) {
  strcpy( kafenv.p.peripheral[id].name, "WiFi" );
  kafenv.p.peripheral[id].enabled = true;
  kafenv.p.peripheral[id].loopType = NULL_LOOP;
  kafenv.p.peripheral[id].dataSize = sizeof( wifi );
  kafenv.p.peripheral[id].initFunction = &initWiFi;
  kafenv.p.peripheral[id].loopFunction = &loopWiFi;
  kafenv.p.peripheral[id].auxLoopFunction = NULL;
  kafenv.p.peripheral[id].data = &wifi;
  WiFi.begin( kafenv.n.networkAddress, kafenv.n.networkPassword );
  WiFiServer server( 23 );
  wifi.server = server;
}

//SERIAL______________________________________________________________________________________________________
static void loopSerial() {
  int serialLength;
  if( ( serialLength = Serial.available() ) > 0 ) {
#if DEBUGPRINT
    int activeCount = 0;
    unsigned char current = 0;
    int index = 0;
    for( int i = 0; i < serialLength; i++ ) {
      short b = Serial.read();
      short bn;
      unsigned char lower;
      if( ( bn = b - 'A' ) <= ( 'F' - 'A' ) && bn >= 0 ) {
        lower = (unsigned char)( bn + 0xA );
      } else if( ( bn = b - 'a' ) <= ( 'f' - 'a' ) && bn >= 0  ) {
        lower = (unsigned char)( bn + 0xA );
      } else if( ( bn = b - '0' ) <= ( '9' - '0' ) && bn >= 0  ) {
        lower = (unsigned char)bn;
      } else if( b == ' ' || b == '\n' ) {
        if( activeCount > 0 ) {
          kafenv.c.rx.raw[index++] = current;
          current = 0;
          activeCount = 0;
          if( b == '\n' ) {
            break;
          }
        }
        continue;
      } else {
        break;
      }
      if( activeCount++ > 1 ) {
        break;
      }
      current = ( current << 4 ) + lower;
    }
#else
    serialLength = serialLength < sizeof( kafenv.c.rx.raw ) ? serialLength : sizeof( kafenv.c.rx.raw );
    Serial.readBytes( kafenv.c.rx.raw, serialLength );
#endif
    if( kafenv.c.rx.data.messageType == COM_REQUEST_MEMORY && kafenv.c.rx.data.toID == kafenv.u.deviceID ) {
      Serial.write( (char*)(void*)( &kafenv ), sizeof( kafenv ) );
    } else if( kafenv.c.rx.data.messageType == COM_SET_MEMORY && kafenv.c.rx.data.toID == kafenv.u.deviceID ) {
      delay( 200 );
      if( Serial.available() >= sizeof( kafenv ) ) {
        Serial.readBytes( (char*)(void*)( &kafenv ), sizeof( kafenv ) );
      }
    } else if( kaf_triggercom() ) {
#if DEBUGPRINT
      Serial.printf( "msg: { " );
      for( int i = 0; i < COM_HEADER_LEN + kafenv.n.replySize; i++ ) {
        Serial.printf( "%2x ", kafenv.c.tx.raw[i] );
      }
      Serial.printf( "}\n" );
#else
      Serial.write( kafenv.c.tx.raw, COM_HEADER_LEN + kafenv.n.replySize );//2ms
#endif
    }
  }
}

static void initSerial( int id ) {
  strcpy( kafenv.p.peripheral[id].name, "Serial" );
  kafenv.p.peripheral[id].enabled = true;
  kafenv.p.peripheral[id].loopType = NULL_LOOP;
  kafenv.p.peripheral[id].dataSize = 0;
  kafenv.p.peripheral[id].initFunction = &initSerial;
  kafenv.p.peripheral[id].loopFunction = &loopSerial;
  kafenv.p.peripheral[id].auxLoopFunction = NULL;
  kafenv.p.peripheral[id].data = NULL;
  Serial.begin( 38400 );
  Serial.setTimeout( 250 );
  delay( 1000 );
  Serial.printf( "ESP32 Drone Code Started\n" );
  kaf_dumpmemory();
}

//DW3000______________________________________________________________________________________________________
#define DW_TX_ANT_DLY        16385 //TX antenna delay
#define DW_RX_ANT_DLY        16385 //RX antenna delay
#define DW_PRE_TIMEOUT           5
#define DW_R1_SCAN_PERIOD    30000
#define DW_PERIOD_R12          900
#define DW_PERIOD_R23          700
#define DW_RANGE_TOLERANCE     200
#define DW_SPIN_TIMEOUT        300

static struct {
  bool working = false;
  dwt_config_t config = { 
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_STD, 
    DWT_PHRRATE_STD, (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
  };
} dw;

static bool sendMessageDW( unsigned char msgLen, unsigned char sendMask ) {
  dwt_write32bitreg( SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK );
  dwt_writetxdata( COM_HEADER_LEN + msgLen, kafenv.c.tx.raw, 0 );
  dwt_writetxfctrl( COM_HEADER_LEN + msgLen + FCS_LEN, 0, 1 );
  if( dwt_starttx( sendMask ) == DWT_SUCCESS ) {
    if( !( sendMask & DWT_RESPONSE_EXPECTED ) ) {
      unsigned long endTime = micros() + 200;
      while( !( dwt_read32bitreg( SYS_STATUS_ID ) & SYS_STATUS_TXFRS_BIT_MASK ) && micros() < endTime );
      dwt_write32bitreg( SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK );
    }
    return true;
  }
  return false;
}

static void sendMessageDW() {
  dwt_setdelayedtrxtime( ( get_rx_timestamp_u64() + ( 2000 * UUS_TO_DWT_TIME ) ) >> 8 );
  sendMessageDW( kafenv.n.replySize, DWT_START_TX_DELAYED );
}

static bool receiveMessageDW( unsigned int timeout ) {
  unsigned int status = 0;
  unsigned long endTime = micros() + timeout + DW_SPIN_TIMEOUT;
  while( !( ( status = dwt_read32bitreg( SYS_STATUS_ID ) ) & ( SYS_STATUS_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR ) ) && micros() < endTime );
  if ( status & SYS_STATUS_RXFCG_BIT_MASK ) {
    dwt_write32bitreg( SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK|SYS_STATUS_TXFRS_BIT_MASK );
    unsigned int frameLength = dwt_read32bitreg( RX_FINFO_ID ) & RXFLEN_MASK;
    frameLength = frameLength < sizeof( kafenv.c.rx.raw ) ? frameLength : sizeof( kafenv.c.rx.raw );
    dwt_readrxdata( kafenv.c.rx.raw, frameLength, 0 );
    return true;
  } else {
    dwt_write32bitreg( SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR );
    return false;
  }
}

static void processRangingData( unsigned char devID, unsigned int tof1, unsigned int tof2, coordinate* position ) {
  unsigned long currentTime = millis();
  unsigned char index = kaf_getdevice( devID );
  kafenv.n.devices[index].lastSeen = currentTime;
  kafenv.n.devices[index].lastRanging = currentTime;
  kafenv.n.devices[index].position = *position;
  kafenv.n.devices[index].distance = 0.5F * ( tof1 + tof2 ) * DWT_TIME_UNITS * SPEED_OF_LIGHT;
}

static void loopDW3000() {
  if( dw.working ) {
    //Initiate DW 5ms
    kafenv.c.tx.data.fromID = kafenv.u.deviceID;
    kafenv.c.tx.data.toID = 0;
    kafenv.c.tx.data.messageType = COM_RANGING_1;
    kafenv.c.tx.data.ranging1.position = kafenv.u.stateEstimate.x;
    dwt_setrxaftertxdelay( DW_PERIOD_R12 - DW_RANGE_TOLERANCE );
    dwt_setrxtimeout( DW_RANGE_TOLERANCE * 2 );
    dwt_setpreambledetecttimeout( DW_PRE_TIMEOUT );
    if ( sendMessageDW( sizeof( kafenv.c.tx.data.ranging1 ), DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED ) 
        && receiveMessageDW( DW_PERIOD_R12 + DW_RANGE_TOLERANCE ) ) {
          kafenv.n.currentTime = millis();
      if( kafenv.c.rx.data.messageType == COM_RANGING_2 && kafenv.c.rx.data.toID == kafenv.u.deviceID ) {
        unsigned char fromID = kafenv.c.rx.data.fromID;
        uint64_t range1TxTime = get_tx_timestamp_u64();
        uint64_t range2RxTime = get_rx_timestamp_u64();
        uint32_t final_tx_time = ( range2RxTime + ( DW_PERIOD_R23 * UUS_TO_DWT_TIME ) ) >> 8;
        uint64_t range3TxTime = ( ( (uint64_t)( final_tx_time & 0xFFFFFFFEUL ) ) << 8 ) + DW_TX_ANT_DLY;
        dwt_setdelayedtrxtime( final_tx_time );
        unsigned int timeOfFlight = ( (unsigned int)( range2RxTime - range1TxTime ) ) - kafenv.c.rx.data.ranging2.processingTime;
        kafenv.c.tx.data.toID = fromID;
        kafenv.c.tx.data.messageType = COM_RANGING_3;
        kafenv.c.tx.data.ranging3.processingTime = (unsigned int)( range3TxTime - range2RxTime );
        kafenv.c.tx.data.ranging3.timeOfFlight = timeOfFlight;
        if( sendMessageDW( sizeof( kafenv.c.tx.data.ranging3 ), DWT_START_TX_DELAYED ) ) {
          processRangingData( fromID, timeOfFlight, timeOfFlight, &kafenv.c.rx.data.ranging2.position );
        }
      } else if( kaf_triggercom() ) {
        sendMessageDW();
      }
    }
    //Respond DW 35ms
    dwt_setpreambledetecttimeout( 0 );
    dwt_setrxtimeout( DW_R1_SCAN_PERIOD );
    dwt_rxenable( DWT_START_RX_IMMEDIATE );
    kafenv.c.tx.data.ranging2.position = kafenv.u.stateEstimate.x;
    if ( receiveMessageDW( DW_R1_SCAN_PERIOD ) ) {//30ms
      if( kafenv.c.rx.data.messageType == COM_RANGING_1 ) {
        uint64_t range1RxTime = get_rx_timestamp_u64();
        uint32_t resp_tx_time = ( range1RxTime + ( DW_PERIOD_R12 * UUS_TO_DWT_TIME ) ) >> 8;
        uint64_t range2TxTime = ( ( (uint64_t)( resp_tx_time & 0xFFFFFFFEUL ) ) << 8 ) + DW_TX_ANT_DLY;
        dwt_setdelayedtrxtime( resp_tx_time );
        dwt_setrxaftertxdelay( DW_PERIOD_R23 - DW_RANGE_TOLERANCE );
        dwt_setrxtimeout( DW_RANGE_TOLERANCE * 2 );
        dwt_setpreambledetecttimeout( DW_PRE_TIMEOUT );
        unsigned char fromID = kafenv.c.rx.data.fromID;
        kafenv.c.tx.data.toID = fromID;
        kafenv.c.tx.data.messageType = COM_RANGING_2;
        kafenv.c.tx.data.ranging2.processingTime = ( unsigned int )( range2TxTime - range1RxTime );
        if( sendMessageDW( sizeof( kafenv.c.tx.data.ranging2 ), DWT_START_TX_DELAYED|DWT_RESPONSE_EXPECTED ) ) {
          coordinate pos = kafenv.c.rx.data.ranging1.position;
          if( receiveMessageDW( DW_PERIOD_R23 + DW_RANGE_TOLERANCE ) && kafenv.c.rx.data.messageType == COM_RANGING_3 && 
              kafenv.c.rx.data.fromID == fromID && kafenv.c.rx.data.toID == kafenv.u.deviceID ) {
            range2TxTime = get_tx_timestamp_u64();
            uint64_t range3RxTime = get_rx_timestamp_u64();
            processRangingData( fromID, ( (unsigned int)( range3RxTime - range2TxTime ) ) - 
                kafenv.c.rx.data.ranging3.processingTime, kafenv.c.rx.data.ranging3.timeOfFlight, &pos );
          }
        }
      } else if( kaf_triggercom() ) {
        sendMessageDW();
      }
    }
  }
}

static void initDW3000( int id ) {
  strcpy( kafenv.p.peripheral[id].name, "DW3000" );
  kafenv.p.peripheral[id].enabled = true;
  kafenv.p.peripheral[id].loopType = NULL_LOOP;
  kafenv.p.peripheral[id].dataSize = sizeof( dw );
  kafenv.p.peripheral[id].initFunction = &initDW3000;
  kafenv.p.peripheral[id].loopFunction = &loopDW3000;
  kafenv.p.peripheral[id].auxLoopFunction = NULL;
  kafenv.p.peripheral[id].data = &dw;
  test_run_info( (unsigned char*)"DS TWR RESP" );
  extern SPISettings _fastSPI;
  _fastSPI = SPISettings( 16000000L, MSBFIRST, SPI_MODE0 );
  spiBegin( DW_IRQ, DW_RST );
  spiSelect( DW_SS );
  delay( 20 );
  if( !dwt_checkidlerc() ) {// Need to make sure DW IC is in IDLE_RC before proceeding
    dw.working = false;
  } else if( dwt_initialise( DWT_DW_INIT ) == DWT_ERROR ) {
    dw.working = false;
  } else if( dwt_configure( &dw.config ) ) {
    dw.working = false;
  } else {
    dw.working = true;
    extern dwt_txconfig_t txconfig_options;
    dwt_configuretxrf( &txconfig_options );// Configure the TX spectrum parameters (power, PG delay and PG count)
    dwt_setlnapamode( DWT_LNA_ENABLE|DWT_PA_ENABLE );
    dwt_setleds( DWT_LEDS_ENABLE|DWT_LEDS_INIT_BLINK );
    dwt_setrxantennadelay( DW_RX_ANT_DLY );// Apply default antenna delay value. See NOTE 2 below.
    dwt_settxantennadelay( DW_TX_ANT_DLY );
  }
}

//ENTRY POINT_________________________________________________________________________________________________
static void commandTask( void* pvParameters ) {
  for(;;) {
    loopFirmwareCommand();
    loopDW3000();
    loopSerial();
    loopWiFi();
  }
}

static void flightTask( void* pvParameters ) {
  for(;;) {
    loopESCs();
    loopFirmwareFlight();
    loopMPU6050();
    controlsStep();
  }
}

void firmwareSetup() {
  initSerial( 0 );
  initMPU6050( 1 );
  initESCs( 2 );
  initWiFi( 3 );
  initDW3000( 4 );
  initFirmware( 5 );
  controlsInit();
  xTaskCreatePinnedToCore( &flightTask,     "flight_task",     5000, NULL, 1, NULL, 0 );
  xTaskCreatePinnedToCore( &commandTask,    "command_task",    5000, NULL, 1, NULL, 1 );
}
#endif