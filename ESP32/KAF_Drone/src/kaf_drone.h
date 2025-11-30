//PREAMBLE____________________________________________________________________________________________________
#ifdef _MSC_VER
#define ALT_DEFINE 1
#define PACKED 
#else
#define ALT_DEFINE 0
#define PACKED __attribute__((__packed__))
#endif
#ifndef NULL
#define NULL 0
#endif

//CONFIGURATIONS______________________________________________________________________________________________
//define once
#ifndef ESP32_PNT_DRONE
#define ESP32_PNT_DRONE
//maximum number of motors/devices
#define MOTOR_COUNT              4
#define DEVICE_COUNT             6
#define PERIPHERAL_COUNT        10
//debug control
#define DEBUGPRINT               1
#define FIRMWARE_ENABLE          1
#define ALLOW_CUSTOM_FIRMWARE    1

//UTILITY MACROS______________________________________________________________________________________________
#define KF_REGVAR( P, T, N ) printf( "%c,%s,%04x,%04x\n", T, N, sizeof( P )                        \
                 (unsigned short)( ( (char*)( (void*)&( P ) ) ) - ( (char*)( (void*)&kafenv ) ) ) )
#define KF_VEC3( N, C ) for( int N = 0; N < 3; N++ ) C
#define KF_VECN( N, A, C ) for( int N = 0; N < A; N++ ) C
#define KF_BOUND( V, LB, UB ) V = V > UB ? UB : ( V < LB ? LB : V )

//STRUCTURES__________________________________________________________________________________________________
//coordinate definition
typedef union coordinate {
  struct {
    float x;
    float y;
    float z;
    float stdev;
  };
  float f[4];
};

//pid definition
typedef PACKED struct pid {
  float Kf;
  float Kp;
  float Ki;
  float Kd;
  float intlimit;
  float outlimit;
  float derlimit;
  float modula;
  float integ;
  float deriv;
  float error;
  float desired;
  float prevmeas;
  float lowpass[5];
};

//full state struct definition
typedef PACKED struct state {
  coordinate x;//position
  coordinate v;//velocity
  coordinate t;//orientation
  coordinate w;//angular velocity
};

//GLOBAL VARIABLES____________________________________________________________________________________________
struct global_variables {
  struct {//DRONE STATE DATA
    unsigned char deviceID = 'U';                    //Character designating a unique ID of a drone
    unsigned int version = 0x00901900;               //
    state stateEstimate;                             //State estimate of the drone in the global reference
                                                     //frame. Position is cartesian, orientation is axial.
  } u;                                               //Drone data
  struct {//DRONE CONTROL DATA
    unsigned char flightMode = 0;                    //
    float motorSetpoint[MOTOR_COUNT];                //
    float posSetpoint[3] = { 0, 0, 0 };              //
    float cascadeSetpoint[3];                        //
    pid positionPID[3];                              //
    pid velocityPID[3];                              //
    pid attitudePID[3];                              //
    pid attiratePID[3];                              //
  } s;                                               //
  struct {//FLIGHT DATA
    unsigned long currentTime = 0;                  //Timestamp of flight sensor data acquisition time in ms
    unsigned int timeStep = 0;                      //Time interval in us between the last two flight ticks
    float motorOutput[MOTOR_COUNT];                 //Variable to write to in order to set the target PWM
                                                    //voltage, specified in an unitless scalar 0<=1
    coordinate accelInput = { 0, 0, 0, 1e10 };      //
    coordinate gyroInput = { 0, 0, 0, 1e10 };       //
    coordinate posInput = { 0, 0, 0, 1e10 };        //
    coordinate baroInput = { 0, 0, 0, 1e10 };       //
  } f;                                              //Flight peripheral data
  struct {//NETWORK DATA
    unsigned long currentTime = 0;                  //Timestamp of communication received
    char networkName[32] = "default";               //Specifies the name of the WiFi network to connect to
                                                    //upon initial setup or when resetConnection is raised
    char networkPassword[25] = "default";           //Specifies the password of the WiFi network
    char networkAddress[127] = "disconnected";      //If connected, this records the IP address of the drone
    bool attemptReconnect = true;                   //
    unsigned short comTaskDelay = 0;                //
    unsigned char replySize = 0;                    //
    unsigned int deviceCount = 0;                   //Number of peer drones observed by this drone
    unsigned char deviceLookup[0xFF];               //Drone ID to devices struct array index lookup
    struct {
      unsigned char deviceID;                       //Unique id of peer drone
      unsigned long lastSeen;                       //Last observation of peer drone in ms
      unsigned long lastRanging;                    //Last successful ranging with peer drone in ms
      coordinate position;                          //Position estimate in m of peer drone
      float distance;                               //Distance in m between peer drone and this drone
    } devices[DEVICE_COUNT];                        //Structure for each peer that the drone can observe
  } n;
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
          state stateEst;
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
  } c;                                              //Communication data
  struct {
    unsigned char version = 99;                     //
    unsigned char trigger = 0;                      //
    struct  {
      char name[15] = "default";                    //
      bool enabled = false;                         //
      unsigned char loopType;                       //
      unsigned int dataSize = 0;                    //
      void( *initFunction )( int ) = NULL;          //
      void( *loopFunction )() = NULL;               //
      void( *auxLoopFunction )() = NULL;            //
      void* data = NULL;                            //
    } peripheral[PERIPHERAL_COUNT];                 //
  } p;                                              //Peripheral data
};
extern struct global_variables kafenv;

//FUNCTIONS DEFINITONS________________________________________________________________________________________
//kaf drone utility functions
void kaf_dumpmemory();                                           //
bool kaf_triggercom();                                           //
void kaf_pidreset( pid* pid );                                   //
float kaf_pidstep( pid* pid, float set, float meas, float dt  ); //
unsigned char kaf_getdevice( unsigned char deviceID );           //Method that gets the index of peer with id
//software functions
void communicationStep();                                        //Called to communication data
void controlsStep();                                             //Called every cycle to calculate states
void controlsInit();                                             //
//firmware functions
#if FIRMWARE_ENABLE
void firmwareSetup();                                            //Function call to start the drone firmware
#endif

#endif