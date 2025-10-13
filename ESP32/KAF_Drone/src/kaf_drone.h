//PREAMBLE____________________________________________________________________________________________________
#ifdef _MSC_VER
#define PACKED 
#else
#define PACKED __attribute__((__packed__))
#endif
#ifndef NULL
#define NULL 0
#endif

//CONFIGURATIONS______________________________________________________________________________________________
//define once
#ifndef ESP32_PNT_DRONE
#define ESP32_PNT_DRONE
//ids for each task
#define CMD_TASK 0
#define FLIGHT_TASK 1
//maximum number of motors/devices
#define MOTOR_COUNT 4
#define DEVICE_COUNT 16
//debug control
#define DEBUGPRINT 1

//STRUCTURES__________________________________________________________________________________________________
//coordinate definition
typedef PACKED struct coordinate {
 float x;
 float y;
 float z;
 float stdev;
};

//full state struct definition
typedef PACKED struct state {
  coordinate x;//position
  coordinate v;//velocity
  coordinate a;//acceleration
  coordinate t;//orientation
  coordinate w;//angular velocity
  coordinate e;//angular acceleration
};

//GLOBAL VARIABLES____________________________________________________________________________________________
struct global_variables {
  struct {//DRONE STATE DATA
    unsigned char deviceID = 'U';//WRITABLE INIT, LOOP
    state stateEstimate;
  } u;
  struct {//PERIPHERAL DATA
    struct {
      bool working = false;
      bool calibrateMPU = true;//WRITABLE INIT
      unsigned long currentStep = 0;
      unsigned long timeStep = 0;
      unsigned long lastRead = 0;
      unsigned int missCount = 0;
      float accelCalib[3] = { 0, 0, 0 };//WRITABLE INIT, LOOP
      float angvelCalib[3] = { 0, 0, 0 };//WRITABLE INIT, LOOP
      float accel[3] = { 0, 0, 0 };//raw data, not in universal coords
      float angvel[3] = { 0, 0, 0 };//raw data
    } mpu;
    struct {
      bool working = true;
      unsigned short motorCount = 4;//WRITABLE INIT
      bool atTarget = false;
      struct {
        bool working = false;
        int currentRaw = 0;
        int targetRaw = 0;
        float targetSpeed = 0;//WRITABLE LOOP 0<=1
      } esc[MOTOR_COUNT];
    } escs;
    struct {
      bool working = false;
      bool doRanging = true;
      bool usedByStateEstimate = false;
      coordinate position;
    } dw;
    struct {
      bool working = false;
      char wifiName[32] = "default";//WRITABLE INIT
      char wifiPassword[25] = "default";//WRITABLE INIT
      char localIP[127] = "default";
      bool resetConnection = false;
      bool clientConnected = false;
    } wifi;
    struct {
      bool working = false;
      unsigned short comTaskPeriodOffset = 0;
      struct {
        unsigned int currentCycle = 0;
        unsigned long taskPeriod = 0;
        unsigned long totalPeriod = 0;
        unsigned long currentTime = 0;
      } task[2];
    } rtos;
  } s;
  struct {//COMMUNICATION DATA
    union {
      PACKED struct {
        unsigned char fromID;
        unsigned char toID;
        unsigned char messageType;
        union {
          unsigned char byte;
          unsigned char bytes[248];
          coordinate coord;
          PACKED struct {
            coordinate position;
          } ranging1;
          PACKED struct {
            unsigned int processingTime;
            coordinate position;
          } ranging2;
          PACKED struct {
            unsigned int processingTime;
            unsigned int timeOfFlight;
          } ranging3;
          PACKED struct {
            unsigned char deviceID[4];
            unsigned int timeLastSeen[4];
          } deviceList;
          PACKED struct {
            unsigned int startPos;
            unsigned char length;
            unsigned char bytes[200];
          } globalVarTransfer;
        };
      } data;
      unsigned char raw[ sizeof( data ) ];
    } rx, tx;
    unsigned int deviceCount = 0;
    unsigned char deviceLookup[0xFF];
    struct {
      unsigned char deviceID;
      unsigned long lastSeen;
      unsigned long lastRanging;
      coordinate position;
      float distance;
    } devices[DEVICE_COUNT];
  } c;
};
extern struct global_variables kafenv;

//SOFTWARE FUNCTIONS__________________________________________________________________________________________
void setupDroneConfiguration();
void debugUpdate();
unsigned char getDeviceIndex( unsigned char deviceID );
void handleTransmission( void(*sendFunc)( int ) );
void doStateEstimate();
void commandStep();

//FIRMWARE____________________________________________________________________________________________________
void drone_setup();
#endif