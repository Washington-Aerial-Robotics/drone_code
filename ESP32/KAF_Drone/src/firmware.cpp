#include "kaf_drone.h"
#include "dw3000.h"
#include "lib/RC_ESC/ESC.h"
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <stdio.h>

//DEFINITIONS_________________________________________________________________________________________________
//pin definitions
#define DW_IRQ   34
#define DW_RST   27
#define DW_SS     4
#define MPU_SDA  21
#define MPU_SCL  22
#define ESC_PINS 33, 35, 25, 26
//transmission options
#define ESC_MAX  2000
#define ESC_MIN  1000
#define ESC_ARM   500
#define ESC_RAMP   10
#define MPU_I2C         0x68
#define MPU_ACCEL_RANGE 0x02
#define MPU_GYRO_RANGE  0x02
#define MPU_ACCEL_SCALE ( 2048.0 / 9.81 ) //2g 16384.0, 4g 8192.0, 8g 4096.0, 16g, 2048.0
#define MPU_GYRO_SCALE  16.384            //250deg/s 131.072, 500deg/s 65.536, 1000deg/s 32.768, 2000deg/s 16.384
#define MPU_CALIB_LEN   256
#define RTOS_CMD_CORE              0
#define RTOS_FLIGHT_CORE           1
#define RTOS_CMD_PERIOD           40
#define RTOS_FLIGHT_PERIOD        10
#define DW_TX_ANT_DLY      16385//TX antenna delay
#define DW_RX_ANT_DLY      16385//RX antenna delay
#define DW_PRE_TIMEOUT         5
#define DW_R1_SCAN_PERIOD  30000
#define DW_PERIOD_R12        900
#define DW_PERIOD_R23        700
#define DW_RANGE_TOLERANCE   200
#define DW_SPIN_TIMEOUT      300
#define COM_HEADER_LEN         4//length of header
#define COM_KILL            0x10// WIFI only
#define COM_RANGING_1       0x11//UWB only
#define COM_RANGING_2       0x12//UWB only
#define COM_RANGING_3       0x13//UWB only
#define COM_REQUEST_MEMORY  0x16//SERIAL only
#define COM_REPLY_MEMORY    0x17//SERIAL only
#define COM_SET_MEMORY      0x18//SERIAL only
//utility commands
#define VEC3( CODE ) for( int q = 0; q < 3; q++ ) CODE

//PERIPHERAL OBJECTS__________________________________________________________________________________________
struct global_variables kafenv;
static unsigned int esc_pins[MOTOR_COUNT] = { ESC_PINS };
static ESC* esc_array[MOTOR_COUNT];
static WiFiServer server( 23 );
static WiFiClient client;
extern SPISettings _fastSPI;
extern dwt_txconfig_t txconfig_options;
//configs for dw3000
static dwt_config_t config = {
  5,                /* Channel number. */
  DWT_PLEN_128,     /* Preamble length. Used in TX only. */
  DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
  9,                /* TX preamble code. Used in TX only. */
  9,                /* RX preamble code. Used in RX only. */
  1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
  DWT_BR_6M8,       /* Data rate. */
  DWT_PHRMODE_STD,  /* PHY header mode. */
  DWT_PHRRATE_STD,  /* PHY header rate. */
  (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
  DWT_STS_MODE_OFF, /* STS disabled */
  DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
  DWT_PDOA_M0       /* PDOA mode off */
};

//UTILITY FUNCTIONS___________________________________________________________________________________________
static void rtosStep( unsigned char index, unsigned short minPeriod ) {
  vTaskDelay( index );
  unsigned long oldTime = kafenv.s.rtos.task[ index ].currentTime;
  unsigned long taskPeriod = millis() - oldTime;
  kafenv.s.rtos.task[ index ].taskPeriod = taskPeriod;
  int delayTime = (int)( minPeriod ) - (int)( taskPeriod );
  if( delayTime > 0 ) {
    delay( delayTime );
  }
  unsigned long newTime = millis();
  kafenv.s.rtos.task[ index ].totalPeriod = newTime - oldTime;
  kafenv.s.rtos.task[ index ].currentTime = newTime;
  kafenv.s.rtos.task[ index ].currentCycle++;
}

static void readIMU() {
  //read accelerometer data
  unsigned long currentTime;
  if( kafenv.s.mpu.working ) {
    Wire.beginTransmission( MPU_I2C );
    Wire.write( 0x3B );
    Wire.endTransmission( false );
    int reqRes = Wire.requestFrom( MPU_I2C, 14, true );
    currentTime = millis();
    if( reqRes == 14 ) {
      VEC3( kafenv.s.mpu.accel[q] = ( Wire.read() << 8 | Wire.read() ) / MPU_ACCEL_SCALE - kafenv.s.mpu.accelCalib[q] );
      Wire.read();Wire.read();//skip temperature readout
      VEC3( kafenv.s.mpu.angvel[q] = ( Wire.read() << 8 | Wire.read() ) / MPU_GYRO_SCALE - kafenv.s.mpu.angvelCalib[q] );
      kafenv.s.mpu.missCount = 0;
    } else if( ++kafenv.s.mpu.missCount > 50 ) {
      kafenv.s.mpu.working = false;
    }
    kafenv.s.mpu.lastRead = currentTime;
  } else {
    VEC3( kafenv.s.mpu.accel[q] = 0 );
    VEC3( kafenv.s.mpu.angvel[q] = 0 );
    currentTime = millis();
  }
  kafenv.s.mpu.timeStep = currentTime - kafenv.s.mpu.currentStep;
  kafenv.s.mpu.currentStep = currentTime;
}

static void controlMotors() {
  int numActive = 0;
  for( int i = 0; i < kafenv.s.escs.motorCount; i++ ) {
    if( kafenv.s.escs.esc[i].working ) {
      float targetParam = kafenv.s.escs.esc[i].targetSpeed;
      kafenv.s.escs.esc[i].targetRaw = targetParam < 0.000001 ? ESC_ARM : (int)( ( ESC_MAX - ESC_MIN ) * targetParam + ESC_MIN );
      int diff = kafenv.s.escs.esc[i].targetRaw - kafenv.s.escs.esc[i].currentRaw;
      if( diff != 0 ) {
        kafenv.s.escs.esc[i].currentRaw += diff / abs( diff ) * min( abs( diff ), 10 );
        esc_array[i]->speed( kafenv.s.escs.esc[i].currentRaw );
        numActive++;
      }
    }
  }
  kafenv.s.escs.atTarget = numActive == 0;
}

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

static void sendMessageDW( int msgLen ) {
  dwt_setdelayedtrxtime( ( get_rx_timestamp_u64() + ( 2000 * UUS_TO_DWT_TIME ) ) >> 8 );
  sendMessageDW( (unsigned char)msgLen, DWT_START_TX_DELAYED );
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
  unsigned char index = getDeviceIndex( devID );
  kafenv.c.devices[index].lastSeen = currentTime;
  kafenv.c.devices[index].lastRanging = currentTime;
  kafenv.c.devices[index].position = *position;
  kafenv.c.devices[index].distance = 0.5F * ( tof1 + tof2 ) * DWT_TIME_UNITS * SPEED_OF_LIGHT;
}

static void commandTask( void* pvParameters ) {
  for(;;) {
    rtosStep( CMD_TASK, RTOS_CMD_PERIOD + kafenv.s.rtos.comTaskPeriodOffset );
#if DEBUGPRINT
    debugUpdate();
#endif
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
      } else {
        handleTransmission( &sendMessageDW );
      }
    }
    //Respond DW 35ms
    dwt_setpreambledetecttimeout( 0 );
    dwt_setrxtimeout( DW_R1_SCAN_PERIOD );
    dwt_rxenable( DWT_START_RX_IMMEDIATE );
    kafenv.c.tx.data.ranging2.position = kafenv.u.stateEstimate.x;
    if ( receiveMessageDW( DW_R1_SCAN_PERIOD ) ) {//30ms
      if( kafenv.s.dw.doRanging && kafenv.c.rx.data.messageType == COM_RANGING_1 ) {
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
      } else {
        handleTransmission( &sendMessageDW );
      }
    }
    //Serial 7ms
    if( Serial.available() > 0 ) {
      delay( 2 );//2ms
      int length = Serial.available();
      length = length < sizeof( kafenv.c.rx.raw ) ? length : sizeof( kafenv.c.rx.raw );
      Serial.readBytes( kafenv.c.rx.raw, length );
      if( kafenv.c.rx.data.messageType == COM_REQUEST_MEMORY && kafenv.c.rx.data.toID == kafenv.u.deviceID ) {
        Serial.write( (char*)(void*)( &kafenv ), sizeof( kafenv ) );
      } else if( kafenv.c.rx.data.messageType == COM_SET_MEMORY && kafenv.c.rx.data.toID == kafenv.u.deviceID ) {
        delay( 200 );
        if( Serial.available() >= sizeof( kafenv ) ) {
          Serial.readBytes( (char*)(void*)( &kafenv ), sizeof( kafenv ) );
        }
      } else {
        handleTransmission( []( int msgLen ) {//3ms
#if DEBUGPRINT
          Serial.printf( "msg: { " );
          for( int i = 0; i < COM_HEADER_LEN + msgLen; i++ ) {
            Serial.printf( "%2x ", kafenv.c.tx.raw[i] );
          }
          Serial.printf( "}\n" );
#else
          Serial.write( kafenv.c.tx.raw, COM_HEADER_LEN + msgLen );//2ms
#endif
        } );
      }
    }
    //WiFi 7ms
    bool lastWorking = kafenv.s.wifi.working;
    kafenv.s.wifi.working = WiFi.status() == WL_CONNECTED;
    if( kafenv.s.wifi.working ) {
      if( !lastWorking ) {
        strcpy( kafenv.s.wifi.localIP, WiFi.localIP().toString().c_str() );
        server.begin();
#ifdef DEBUGPRINT
        Serial.printf( "WiFi started successfully at IP: %s\n", kafenv.s.wifi.localIP );
#endif
      }
      kafenv.s.wifi.clientConnected = client || ( WiFi.isConnected() && ( client = server.accept() ) );
      if( kafenv.s.wifi.clientConnected && client.available() > 0 ) {
        client.setTimeout( 100 );
        delay( 2 );//2ms
        int length = client.available();
        length = length < sizeof( kafenv.c.rx.raw ) ? length : sizeof( kafenv.c.rx.raw );
        client.readBytes( kafenv.c.rx.raw, length );
        handleTransmission( []( int msgLen ) {//3ms
          if( kafenv.s.wifi.clientConnected ) {
            client.write( kafenv.c.tx.raw, COM_HEADER_LEN + msgLen );//2ms
          }
        } );
      }
    } else if( kafenv.s.wifi.resetConnection ) {
      WiFi.begin( kafenv.s.wifi.wifiName, kafenv.s.wifi.wifiPassword );
    }
  }
}

static void flightTask( void* pvParameters ) {
  for(;;) {
    rtosStep( FLIGHT_TASK, RTOS_FLIGHT_PERIOD );
    readIMU();
    doStateEstimate();
    commandStep();
    controlMotors();
  }
}

//ENTRY POINT_________________________________________________________________________________________________
void drone_setup() {
  setupDroneConfiguration();
  // >>> Serial <<<
  Serial.begin( 38400 );
  Serial.setTimeout( 250 );
  delay( 1000 );
  Serial.printf( "ESP32 Drone Code Started\n" );
  // >>> MPU6050 <<<
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
  kafenv.s.mpu.working = success == 0;
  //calibrate mpu
  if( kafenv.s.mpu.working && kafenv.s.mpu.calibrateMPU ) {
    for( int i = 0; i < MPU_CALIB_LEN; i++ ) {
      readIMU();
      VEC3( kafenv.s.mpu.accelCalib[q] += kafenv.s.mpu.accel[q] );
      VEC3( kafenv.s.mpu.angvelCalib[q] += kafenv.s.mpu.angvel[q] );
    }
    VEC3( kafenv.s.mpu.accelCalib[q] /= MPU_CALIB_LEN );
    VEC3( kafenv.s.mpu.angvelCalib[q] /= MPU_CALIB_LEN );
  }
  // >>> ESC <<<
  //arm all the pins and set default values
  for( int i = 0; i < kafenv.s.escs.motorCount; i++ ) {
    pinMode( esc_pins[i], OUTPUT );
    esc_array[i] = new ESC( esc_pins[i], ESC_MIN, ESC_MAX, ESC_ARM );
    esc_array[i]->arm();
    kafenv.s.escs.esc[i].working = true;
    kafenv.s.escs.esc[i].currentRaw = ESC_ARM;
    kafenv.s.escs.esc[i].targetRaw = 0;
    kafenv.s.escs.esc[i].targetSpeed = 0;
  }
  // >>> WIFI <<<
  WiFi.begin( kafenv.s.wifi.wifiName, kafenv.s.wifi.wifiPassword );
  kafenv.s.wifi.working = false;
  // >>> DW3000 <<<
  test_run_info( (unsigned char*)"DS TWR RESP" );
  _fastSPI = SPISettings( 16000000L, MSBFIRST, SPI_MODE0 );
  spiBegin( DW_IRQ, DW_RST );
  spiSelect( DW_SS );
  delay( 20 );
  if( !dwt_checkidlerc() ) {// Need to make sure DW IC is in IDLE_RC before proceeding
    kafenv.s.dw.working = false;
  } else if( dwt_initialise( DWT_DW_INIT ) == DWT_ERROR ) {
    kafenv.s.dw.working = false;
  } else if( dwt_configure( &config ) ) {
    kafenv.s.dw.working = false;
  } else {
    kafenv.s.dw.working = true;
    dwt_configuretxrf( &txconfig_options );// Configure the TX spectrum parameters (power, PG delay and PG count)
    dwt_setlnapamode( DWT_LNA_ENABLE|DWT_PA_ENABLE );
    dwt_setleds( DWT_LEDS_ENABLE|DWT_LEDS_INIT_BLINK );
    dwt_setrxantennadelay( DW_RX_ANT_DLY );// Apply default antenna delay value. See NOTE 2 below.
    dwt_settxantennadelay( DW_TX_ANT_DLY );
  }
  // >>> TASKS <<<
  Serial.printf( "ESP32 Drone Entering Nominal Operations\n" );
  delay( 1000 );
  xTaskCreatePinnedToCore( commandTask,    "command_task",    5000, NULL, 1, NULL, RTOS_CMD_CORE );
  xTaskCreatePinnedToCore( flightTask,     "flight_task",     5000, NULL, 1, NULL, RTOS_FLIGHT_CORE );
}