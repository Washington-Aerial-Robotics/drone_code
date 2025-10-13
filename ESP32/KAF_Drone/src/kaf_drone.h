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
#define DEVICE_COUNT 4
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

//quaternion definition
typedef PACKED struct quaternion {
  float e0;
  float e1;
  float e2;
  float e3;
}

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
    unsigned char deviceID = 'U';                    //Character designating a unique ID of a drone
    state stateEstimate;                             //State estimate of the drone in the global reference
                                                     //frame. Position is cartesian, oreintation is Euler.
  } u;                                               //Drone data
  struct {//PERIPHERAL DATA
    struct {
      bool working = false;                          //True if the ESP32 is connected to the MPU 
      bool calibrateMPU = true;                      //If true, calibrates MPU upon drone code setup
      unsigned long currentStep = 0;                 //Timestamp in ms of call of readIMU
      unsigned long timeStep = 0;                    //Time intervalin ms between the last two readIMU calls
      unsigned long lastRead = 0;                    //Time in ms since the last update from the IMU
      unsigned int missCount = 0;                    //Number of consecutive unsuccessful attemps of readIMU
      float accelCalib[3] = { 0, 0, 0 };             //Calibration offsets for acceleration m/s2
      float angvelCalib[3] = { 0, 0, 0 };            //Calibration offset for angular acceleration in rad/s2 
      float accel[3] = { 0, 0, 0 };                  //Acceleration in IMU reference frame m/s2
      float angvel[3] = { 0, 0, 0 };                 //Angular acceleration in Euler, IMU reference in rad/s2
    } mpu;                                           //Structure containing IMU data
    struct {
      bool working = true;                           //True if the PWM library is working
      unsigned short motorCount = MOTOR_COUNT;       //Number of motors controlled by the drone
      bool atTarget = false;                         //True if current motor voltages match the target volt
      struct {
        bool working = false;                        //True if the PWM at the selected pin is working
        int currentRaw = 0;                          //Current voltage input of the PWM
        int targetRaw = 0;                           //Target voltage of the PWM
        float targetSpeed = 0;                       //Variable to write to in order to set the target PWM
                                                     //voltage, specified in an unitless scalar 0<=1
      } esc[MOTOR_COUNT];                            //Structure containing data for each PWM
    } escs;                                          //Structure for PWM information
    struct {
      bool working = false;                          //True if the ESP32 is connected to the DW3000
      bool doRanging = true;                         //If true, drone will attempt ranging 2 & 3
    } dw;                                            //Structure containing DW3000 data
    struct {
      bool working = false;                          //True if the ESP32 is connected to a WiFi network
      char wifiName[32] = "default";                 //Specifies the name of the WiFi network to connect to
                                                     //upon initial setup or when resetConnection is raised
      char wifiPassword[25] = "default";             //Specifies the password of the WiFi network
      char localIP[127] = "default";                 //If connected, this records the IP address of the drone
      bool resetConnection = false;                  //If true, the drone will reattempt to connect to a WiFi
                                                     //network with the name wifiPassword
      bool clientConnected = false;                  //True if a downlink device is connected to the ESP32
    } wifi;                                          //Structure containing information about ESP#2 WiFi
    struct {
      bool working = ture;                           //True if FreeRTOS is successfully initialized
      unsigned short comTaskPeriodOffset = 0;        //Time in ms to deviate from the command task period
      struct {
        unsigned int currentCycle = 0;               //Number of cycles the task successfully executed
        unsigned long taskPeriod = 0;                //Time in ms it took to execute the task computations
        unsigned long totalPeriod = 0;               //Total time it took to run a single cycle of the task
        unsigned long currentTime = 0;               //Current time step in ms
      } task[2];                                     //Structure for each task. 0 is command, 1 is flight
    } rtos;                                          //Information about FreeRTOS management
  } s;                                               //Sensor data
  struct {//COMMUNICATION DATA
    union {
      PACKED struct {
        unsigned char fromID;                        //ID of the sender of the communication packet
        unsigned char toID;                          //ID of the intended receiver of the packet
        unsigned char messageType;                   //Message type of the communication packet
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
      } data;                                       //Data representation of transmission data
      unsigned char raw[ sizeof( data ) ];          //Buffer representation of transmission data
    } rx, tx;                                       //Structs for loading receive/send coms data
    unsigned int deviceCount = 0;                   //Number of peer drones observed by this drone
    unsigned char deviceLookup[0xFF];               //Drone ID to devices struct array index lookup
    struct {
      unsigned char deviceID;                       //Unique id of peer drone
      unsigned long lastSeen;                       //Last observation of peer drone in ms
      unsigned long lastRanging;                    //Last successful ranging with peer drone in ms
      coordinate position;                          //Position estimate in m of peer drone
      float distance;                               //Distance in m between peer drone and this drone
    } devices[DEVICE_COUNT];                        //Structure for each peer that the drone can observe
  } c;                                              //Communication data
};
extern struct global_variables kafenv;

//SOFTWARE FUNCTIONS__________________________________________________________________________________________
void setupDroneConfiguration();                         //Called at the start to set initial drone config
void debugUpdate();                                     //Called in command task every cycle if DEBUGPRINT = 1
unsigned char getDeviceIndex( unsigned char deviceID ); //Method that gets the index of peer with id deviceID
void handleTransmission( void(*sendFunc)( int ) );      //Called in command task process communication data
void doStateEstimate();                                 //Called in flight task every cycle to calculate state
void commandStep();                                     //Feedback controller, called every flight task cycle

//FIRMWARE____________________________________________________________________________________________________
void drone_setup();                                     //Function call to start the drone firmware, call ONCE
#endif