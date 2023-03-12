/* add  machine  raise and lower management  by  A1 A2  or D2 D5
 improvement  12/03/2023  PW2 noise/watchdog    if  timing  set 255  it is continous  activation  and can be  reactivate on request  if not  only one time   
 
  
 UDP Autosteer code for ENC28J60 module
  For AgOpenGPS
  4 Feb 2021, Brian Tischler
  Like all Arduino code - copied from somewhere else :)
  So don't claim it as your own
*/


#define  Hyd_up A1  ///   define   the  pin  to up and down       can be   D2 D5 A1 A2 ...  like A1 in this line
#define  Hyd_down A2




#include <Wire.h>
#include <EEPROM.h>
#include "zADS1115.h"
#include "EtherCard_AOG.h"
#include <IPAddress.h>
#include "BNO08x_AOG.h"

////////////////// User Settings /////////////////////////

//How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 3.0

/*  PWM Frequency ->
    490hz (default) = 0
    122hz = 1
    3921hz = 2
*/
#define PWM_Frequency 0

// Change this number to reset and reload default parameters To EEPROM
#define EEP_Ident 0x3378

struct ConfigIP {
  uint8_t ipOne = 192;
  uint8_t ipTwo = 168;
  uint8_t ipThree = 5;
};  ConfigIP networkAddress;   //3 bytes

/////////////////////////////////////////////////////////////////////////////////////////////////

// ethernet interface ip address and host address 126
static uint8_t myip[] = { 0, 0, 0, 126 };

// gateway ip address
static uint8_t gwip[] = { 0, 0, 0, 1 };

//DNS- you just need one anyway
static uint8_t myDNS[] = { 8, 8, 8, 8 };

//mask
static uint8_t mask[] = { 255, 255, 255, 0 };

//this is port of this autosteer module
uint16_t portMy = 5126;

//sending back to where and which port
static uint8_t ipDestination[] = {0, 0, 0, 255};
uint16_t portDestination = 9999; //AOG port that listens

// ethernet mac address - must be unique on your network - 126 = 7E
static uint8_t mymac[] = { 0x00, 0x00, 0x56, 0x00, 0x00, 0x7E };

// Address of CMPS14 shifted right one bit for arduino wire library
#define CMPS14_ADDRESS 0x60

// BNO08x definitions
#define REPORT_INTERVAL 90 //Report interval in ms (same as the delay at the bottom)

//   ***********  Motor drive connections  **************888
//Connect ground only for cytron, Connect Ground and +5v for IBT2

//Dir1 for Cytron Dir, Both L and R enable for IBT2
#define DIR1_RL_ENABLE  4  //PD4

//PWM1 for Cytron PWM, Left PWM for IBT2
#define PWM1_LPWM  3  //PD3

//Not Connected for Cytron, Right PWM for IBT2
#define PWM2_RPWM  9 //D9

//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN 6 //PD6
#define WORKSW_PIN 7  //PD7
#define REMOTE_PIN 8  //PB0

//ethercard 10,11,12,13
// Arduino Nano = 10 depending how CS of Ethernet Controller ENC28J60 is Connected
#define CS_Pin 10

//Define sensor pin for current or pressure sensor
#define ANALOG_SENSOR_PIN A0

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115

uint8_t Ethernet::buffer[200]; // udp send and receive buffer

//loop time variables in microseconds
const uint16_t LOOP_TIME = 25;  //40Hz
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;

//Heart beat hello AgIO
uint8_t helloFromAutoSteer[] = { 128, 129, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
int16_t helloSteerPosition = 0;

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t PGN_253[] = {128, 129, 126, 253, 8, 0, 0, 0, 0, 0, 0, 0, 0, 12 };
int8_t PGN_253_Size = sizeof(PGN_253) - 1;

//fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = { 128, 129, 126, 250, 8, 0, 0, 0, 0, 0, 0, 0, 0, 12 };
int8_t PGN_250_Size = sizeof(PGN_250) - 1;

uint8_t aog2Count = 0;
float sensorReading, sensorSample;

// booleans to see if we are using CMPS or BNO08x
bool useCMPS = false;
bool useBNO08x = false;

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = {0x4A, 0x4B};
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;

float bno08xHeading = 0;
double bno08xRoll = 0;
double bno08xPitch = 0;

int16_t bno08xHeading10x = 0;
int16_t bno08xRoll10x = 0;

//EEPROM
int16_t EEread = 0;

//Relays
bool isRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t xte = 0;

//Switches
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

//On Off
uint8_t guidanceStatus = 0;
uint8_t prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;

//speed sent as *10
float gpsSpeed = 0;

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0; //the desired angle from AgOpen
int16_t steeringPosition = 0; //from steering sensor
float steerAngleError = 0; //setpoint - actual

//pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float pValue = 0;
float errorAbs = 0;
float highLowPerDeg = 0;

//Steer switch button  ***********************************************************************************************************
uint8_t currentState = 1, reading, previous = 0;
uint8_t pulseCount = 0; // Steering Wheel Encoder
bool encEnable = false; //debounce flag
uint8_t thisEnc = 0, lastEnc = 0;

//Variables for settings
struct Storage {
  uint8_t Kp = 120;  //proportional gain
  uint8_t lowPWM = 30;  //band of no action
  int16_t wasOffset = 0;
  uint8_t minPWM = 25;
  uint8_t highPWM = 160;//max PWM value
  float steerSensorCounts = 30;
  float AckermanFix = 1;     //sent as percent
};  Storage steerSettings;  //11 bytes

//Variables for settings - 0 is false
struct Setup {
  uint8_t InvertWAS = 0;
  uint8_t IsRelayActiveHigh = 0; //if zero, active low (default)
  uint8_t MotorDriveDirection = 0;
  uint8_t SingleInputWAS = 1;
  uint8_t CytronDriver = 1;
  uint8_t SteerSwitch = 0;  //1 if switch selected
  uint8_t SteerButton = 0;  //1 if button selected
  uint8_t ShaftEncoder = 0;
  uint8_t PressureSensor = 0;
  uint8_t CurrentSensor = 0;
  uint8_t PulseCountMax = 5;
  uint8_t IsDanfoss = 0;
  uint8_t IsUseX_Axis = 0;
};  Setup steerConfig;          //9 bytes



//////////////////////////////////////////////

//Machine Control - Brian Tee - Cut and paste from everywhere

/*
    //-----------------------------------------------------------------------------------------------
    // Change this number to reset and reload default parameters To EEPROM
    #define EEP_Ident 0x5425

    //the default network address
    struct ConfigIP {
        uint8_t ipOne = 192;
        uint8_t ipTwo = 168;
        uint8_t ipThree = 5;
    };  ConfigIP networkAddress;   //3 bytes
    //-----------------------------------------------------------------------------------------------

    #include <EEPROM.h>
    #include <Wire.h>
    #include "EtherCard_AOG.h"
    #include <IPAddress.h>

    // ethernet interface ip address
    static uint8_t myip[] = { 0,0,0,123 };

    // gateway ip address
    static uint8_t gwip[] = { 0,0,0,1 };

    //DNS- you just need one anyway
    static uint8_t myDNS[] = { 8,8,8,8 };

    //mask
    static uint8_t mask[] = { 255,255,255,0 };

    //this is port of this autosteer module
    uint16_t portMy = 5123;

    //sending back to where and which port
    static uint8_t ipDestination[] = { 0,0,0,255 };
    uint16_t portDestination = 9999; //AOG port that listens

    // ethernet mac address - must be unique on your network
    static uint8_t mymac[] = { 0x00,0x00,0x56,0x00,0x00,0x7B };

    uint8_t Ethernet::buffer[200]; // udp send and receive buffer
*/
//Variables for config - 0 is false
struct Config {
  uint8_t raiseTime = 2;
  uint8_t lowerTime = 4;
  uint8_t enableToolLift = 0;
  uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

  uint8_t user1 = 0; //user defined values set in machine tab
  uint8_t user2 = 0;
  uint8_t user3 = 0;
  uint8_t user4 = 0;

};  Config aogConfig;   //4 bytes

/*    //Program counter reset
    void(*resetFunc) (void) = 0;

    //ethercard 10,11,12,13 Nano = 10 depending how CS of ENC28J60 is Connected
    #define CS_Pin 10
*/
/*
  Functions as below assigned to pins
  0: -
  1 thru 16: Section 1,Section 2,Section 3,Section 4,Section 5,Section 6,Section 7,Section 8,
            Section 9, Section 10, Section 11, Section 12, Section 13, Section 14, Section 15, Section 16,
  17,18    Hyd Up, Hyd Down,
  19 Tramline,
  20: Geo Stop
  21,22,23 - unused so far*/
uint8_t pin[] = { 1, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

//read value from Machine data and set 1 or zero according to list
uint8_t relayState[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

//hello from AgIO
uint8_t helloFromMachine[] = { 128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71 };

//    const uint8_t LOOP_TIME = 200; //5hz
//    uint32_t lastTime = LOOP_TIME;
//    uint32_t currentTime = LOOP_TIME;
uint32_t fifthTime = 0;
uint16_t count = 0;

//Comm checks
//    uint8_t watchdogTimer = 20; //make sure we are talking to AOG
uint8_t serialResetTimer = 0; //if serial buffer is getting full, empty it

bool isRaise = false, isLower = false;



//Communication with AgOpenGPS
//    int16_t temp, EEread = 0;

//Parsing PGN
//   bool isPGNFound = false, isHeaderFound = false;
//    uint8_t pgn = 0, dataLength = 0, idx = 0;
//    int16_t tempHeader = 0;

//settings pgn
uint8_t PGN_237[] = { 0x80, 0x81, 0x7f, 237, 8, 1, 2, 3, 4, 0, 0, 0, 0, 0xCC };
int8_t PGN_237_Size = sizeof(PGN_237) - 1;

//The variables used for storage
//   uint8_t relayHi = 0, relayLo = 0, tramline = 0, uTurn = 0, hydLift = 0, geoStop = 0;
uint8_t  relayLo = 0, tramline = 0,  hydLift = 0, geoStop = 0;
// float gpsSpeed;
uint8_t  lastTrigger = 0;

uint16_t raiseTimer = 0;
uint16_t lowerTimer = 0;
uint32_t clockTime = 0;


//////////////////////////////////////////////


//reset function
void(* resetFunc) (void) = 0;

void setup()
{
  //PWM rate settings
  if (PWM_Frequency == 1)
  {
    TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 to 256 for PWM frequency of   122.55 Hz
    TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 to 256 for PWM frequency of   122.55 Hz
  }

  else if (PWM_Frequency == 2)
  {
    TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 to 8 for PWM frequency of  3921.16 Hz
    TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 to 8 for PWM frequency of  3921.16 Hx
  }

  //keep pulled high and drag low to activate, noise free safe
  pinMode(WORKSW_PIN, INPUT_PULLUP);
  pinMode(STEERSW_PIN, INPUT_PULLUP);
  pinMode(REMOTE_PIN, INPUT_PULLUP);
  pinMode(DIR1_RL_ENABLE, OUTPUT);

  if (steerConfig.CytronDriver) pinMode(PWM2_RPWM, OUTPUT);

  //set up communication
  Wire.begin();
  Serial.begin(38400);

  //test if CMPS working
  uint8_t error;
  Wire.beginTransmission(CMPS14_ADDRESS);
  error = Wire.endTransmission();

  if (error == 0)
  {
    Serial.println("Error = 0");
    Serial.print("CMPS14 ADDRESs: 0x");
    Serial.println(CMPS14_ADDRESS, HEX);
    Serial.println("CMPS14 Ok.\r\n");
    useCMPS = true;
  }
  else
  {
    Serial.println("Error = 4");
    Serial.println("CMPS not Connected or Found\r\n");
    useCMPS = false;
  }

  // Check for BNO08x
  if (!useCMPS)
  {
    for (int16_t i = 0; i < nrBNO08xAdresses; i++)
    {
      bno08xAddress = bno08xAddresses[i];

      Serial.print("\r\nChecking for BNO08X on ");
      Serial.println(bno08xAddress, HEX);
      Wire.beginTransmission(bno08xAddress);
      error = Wire.endTransmission();

      if (error == 0)
      {
        Serial.println("Error = 0");
        Serial.print("BNO08X ADDRESs: 0x");
        Serial.println(bno08xAddress, HEX);
        Serial.println("BNO08X Ok.\r\n");

        // Initialize BNO080 lib
        if (bno08x.begin(bno08xAddress))
        {
          Wire.setClock(400000); //Increase I2C data rate to 400kHz

          // Use gameRotationVector
          bno08x.enableGameRotationVector(REPORT_INTERVAL); //Send data update every REPORT_INTERVAL in ms for BNO085

          // Retrieve the getFeatureResponse report to check if Rotation vector report is corectly enable
          if (bno08x.getFeatureResponseAvailable() == true)
          {
            if (bno08x.checkReportEnable(SENSOR_REPORTID_GAME_ROTATION_VECTOR, REPORT_INTERVAL) == false) bno08x.printGetFeatureResponse();

            // Break out of loop
            useBNO08x = true;
            break;
          }
          else
          {
            Serial.println("BNO08x init fails!!\r\n");
          }
        }
        else
        {
          Serial.println("BNO080 not detected at given I2C address.");
        }
      }
      else
      {
        Serial.println("Error = 4");
        Serial.println("BNO08X not Connected or Found");
      }
    }
  }

  EEPROM.get(0, EEread);              // read identifier

  if (EEread != EEP_Ident)   // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(10, steerSettings);
    EEPROM.put(40, steerConfig);
    EEPROM.put(60, networkAddress);
    EEPROM.put(100, aogConfig);
    EEPROM.put(120, pin);
  }
  else
  {
    EEPROM.get(10, steerSettings);     // read the Settings
    EEPROM.get(40, steerConfig);
    EEPROM.get(60, networkAddress);
    EEPROM.get(100, aogConfig);
    EEPROM.get(120, pin);
  }

  pinMode(Hyd_up, OUTPUT);
  pinMode(Hyd_down, OUTPUT);
  digitalWrite(Hyd_up, isLower);
  digitalWrite(Hyd_down, isRaise);
  lowerTimer = 0 ;
  raiseTimer = 0 ;

  // for PWM High to Low interpolator
  highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;

  if (ether.begin(sizeof Ethernet::buffer, mymac, CS_Pin) == 0)
    Serial.println(F("Failed to access Ethernet controller"));

  //grab the ip from EEPROM
  myip[0] = networkAddress.ipOne;
  myip[1] = networkAddress.ipTwo;
  myip[2] = networkAddress.ipThree;

  gwip[0] = networkAddress.ipOne;
  gwip[1] = networkAddress.ipTwo;
  gwip[2] = networkAddress.ipThree;

  ipDestination[0] = networkAddress.ipOne;
  ipDestination[1] = networkAddress.ipTwo;
  ipDestination[2] = networkAddress.ipThree;

  Serial.println();
  //set up connection
  ether.staticSetup(myip, gwip, myDNS, mask);
  ether.printIp("_IP_: ", ether.myip);
  ether.printIp("GWay: ", ether.gwip);
  ether.printIp("AgIO: ", ipDestination);

  //register to port 8888
  ether.udpServerListenOnPort(&udpSteerRecv, 8888);

  Serial.println("\r\nSetup complete, waiting for AgOpenGPS");

  adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); //128 samples per second
  adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);

}// End of Setup

void loop()
{
  // Loop triggers every 100 msec and sends back gyro heading, and roll, steer angle etc
  currentTime = millis();

  if (currentTime - lastTime >= LOOP_TIME)
  {
    lastTime = currentTime;

    //reset debounce
    encEnable = true;

    //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250) watchdogTimer = WATCHDOG_FORCE_VALUE;

    //read all the switches
    workSwitch = digitalRead(WORKSW_PIN);  // read work switch

    if (steerConfig.SteerSwitch == 1)         //steer switch on - off
    {
      steerSwitch = digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off
      
    }
    else if (steerConfig.SteerButton == 1)     //steer Button momentary
    {
      reading = digitalRead(STEERSW_PIN);
      if (reading == LOW && previous == HIGH)
      {
        if (currentState == 1)
        {
          currentState = 0;
          steerSwitch = 0;
        }
        else
        {
          currentState = 1;
          steerSwitch = 1;
        }
      }
      previous = reading;
      
    }
    else                                      // No steer switch and no steer button
    {
      // So set the correct value. When guidanceStatus = 1,
      // it should be on because the button is pressed in the GUI
      // But the guidancestatus should have set it off first
      if (guidanceStatusChanged && guidanceStatus == 1 && steerSwitch == 1 && previous == 0)
      {
        steerSwitch = 0;
        previous = 1;
        
      }

      // This will set steerswitch off and make the above check wait until the guidanceStatus has gone to 0
      if (guidanceStatusChanged && guidanceStatus == 0 && steerSwitch == 0 && previous == 1)
      {
        steerSwitch = 1;
        previous = 0;
        
       
      }
      
        
    }

    if (steerConfig.ShaftEncoder && pulseCount >= steerConfig.PulseCountMax)
    {
      steerSwitch = 1; // reset values like it turned off
      currentState = 1;
      previous = 0;
    }

    // Pressure sensor?
    if (steerConfig.PressureSensor)
    {
      sensorSample = (float)analogRead(ANALOG_SENSOR_PIN);
      sensorSample *= 0.25;
      sensorReading = sensorReading * 0.6 + sensorSample * 0.4;
      if (sensorReading >= steerConfig.PulseCountMax)
      {
        steerSwitch = 1; // reset values like it turned off
        currentState = 1;
        previous = 0;
      }
    }

    //Current sensor?
    if (steerConfig.CurrentSensor)
    {
      sensorSample = (float)analogRead(ANALOG_SENSOR_PIN);
      sensorSample = (abs(512 - sensorSample)) * 0.5;
      sensorReading = sensorReading * 0.7 + sensorSample * 0.3;
      if (sensorReading >= steerConfig.PulseCountMax)
      {
        steerSwitch = 1; // reset values like it turned off
        currentState = 1;
        previous = 0;
      }
    }


    remoteSwitch = digitalRead(REMOTE_PIN); //read auto steer enable switch open = 0n closed = Off
    switchByte = 0;
    switchByte |= (remoteSwitch << 2); //put remote in bit 2
    switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
    switchByte |= workSwitch;

    /*
      #if Relay_Type == 1
        SetRelays();       //turn on off section relays
      #elif Relay_Type == 2
        SetuTurnRelays();  //turn on off uTurn relays
      #endif
    */

    //get steering position
    if (steerConfig.SingleInputWAS)   //Single Input ADS
    {
      adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
      steeringPosition = adc.getConversion();
      adc.triggerConversion();//ADS1115 Single Mode

      steeringPosition = (steeringPosition >> 1); //bit shift by 2  0 to 13610 is 0 to 5v
      helloSteerPosition = steeringPosition - 6800;
    }
    else    //ADS1115 Differential Mode
    {
      adc.setMux(ADS1115_REG_CONFIG_MUX_DIFF_0_1);
      steeringPosition = adc.getConversion();
      adc.triggerConversion();


      steeringPosition = (steeringPosition >> 1); //bit shift by 2  0 to 13610 is 0 to 5v
      helloSteerPosition = steeringPosition - 6800;
    }

    //DETERMINE ACTUAL STEERING POSITION

    //convert position to steer angle. 32 counts per degree of steer pot position in my case
    //  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
    if (steerConfig.InvertWAS)
    {
      steeringPosition = (steeringPosition - 6805 - steerSettings.wasOffset);   // 1/2 of full scale
      steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
    }
    else
    {
      steeringPosition = (steeringPosition - 6805 + steerSettings.wasOffset);   // 1/2 of full scale
      steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
    }

    //Ackerman fix
    if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);

    if (watchdogTimer < WATCHDOG_THRESHOLD)
    {
       
      //Enable H Bridge for IBT2, hyd aux, etc for cytron
      if (steerConfig.CytronDriver)
      {
        if (steerConfig.IsRelayActiveHigh)
        {
          digitalWrite(PWM2_RPWM, 0);
        }
        else
        {
          digitalWrite(PWM2_RPWM, 1);
        }
      }
      else digitalWrite(DIR1_RL_ENABLE, 1);

      steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error
      //if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;

      calcSteeringPID();  //do the pid
      motorDrive();       //out to motors the pwm value
    }
    else
    {
       
      //we've lost the comm to AgOpenGPS, or just stop request
      //Disable H Bridge for IBT2, hyd aux, etc for cytron
      if (steerConfig.CytronDriver)
      {
        if (steerConfig.IsRelayActiveHigh)
        {
          digitalWrite(PWM2_RPWM, 1);
        }
        else
        {
          digitalWrite(PWM2_RPWM, 0);
        }
      }
      else digitalWrite(DIR1_RL_ENABLE, 0); //IBT2

      pwmDrive = 0; //turn off steering motor
      motorDrive(); //out to motors the pwm value
      pulseCount = 0;

      
    }
    ////////////////////////

    if (watchdogTimer  > WATCHDOG_THRESHOLD ) // > 20)
    {
      if (aogConfig.isRelayActiveHigh) {
        relayLo = 255;
        relayHi = 255;
      }
      else {
        relayLo = 0;
        relayHi = 0;
      }
    }

    //hydraulic lift



    if (aogConfig.enableToolLift = 1)
    {



      ///////////////////
      clockTime = millis() / 1000 ;


      if (( lowerTimer > 0) & (aogConfig.lowerTime >= 255 ))  lowerTimer =  aogConfig.lowerTime  +  clockTime;
      if (( raiseTimer > 0) & (aogConfig.raiseTime >= 255 ))  raiseTimer =  aogConfig.raiseTime  + clockTime ;


      if (hydLift != lastTrigger && (hydLift == 1 || hydLift == 2))
      {
        lastTrigger = hydLift;

        //20 msec per frame so 50 per second
        switch (hydLift)
        {
          //lower
          case 1:
            lowerTimer =  aogConfig.lowerTime + clockTime ;
            raiseTimer = 0;
            break;

          //raise
          case 2:
            lowerTimer = 0;
            raiseTimer =  aogConfig.raiseTime + clockTime  ;

            break;
        }
      }

      //if anything wrong, shut off hydraulics, reset last
      if ((hydLift != 1 && hydLift != 2) || watchdogTimer >=  WATCHDOG_THRESHOLD ) //|| gpsSpeed < 2)
      {
        lowerTimer = 0 ;
        raiseTimer = 0 ;
        
       if ((aogConfig.lowerTime >= 255 ) || (aogConfig.raiseTime >= 255 )) lastTrigger = 0;

      }

      if (aogConfig.isRelayActiveHigh)
      {
        isLower = isRaise = false;
        if (lowerTimer > clockTime ) isLower = true;
        if (raiseTimer > clockTime ) isRaise = true;
      }
      else
      {
        isLower = isRaise = true;
        if (lowerTimer > clockTime) isLower = false;
        if (raiseTimer > clockTime) isRaise = false;
      }
    }



    //section relays
    SetRelays();

    //checksum
    int16_t CK_A = 0;
    for (uint8_t i = 2; i < PGN_237_Size; i++)
    {
      CK_A = (CK_A + PGN_237[i]);
    }
    PGN_237[PGN_237_Size] = CK_A;

    //off to AOG
    ether.sendUdp(PGN_237, sizeof(PGN_237), portMy, ipDestination, portDestination);





    ///////





  } //end of timed loop

  //This runs continuously, outside of the timed loop, keeps checking for new udpData, turn sense
  delay(1);

  //this must be called for ethercard functions to work. Calls udpSteerRecv() defined way below.
  ether.packetLoop(ether.packetReceive());

  if (encEnable)
  {
    thisEnc = digitalRead(REMOTE_PIN);
    if (thisEnc != lastEnc)
    {
      lastEnc = thisEnc;
      if (lastEnc) EncoderFunc();
    }
  }

} // end of main loop

//callback when received packets
void udpSteerRecv(uint16_t dest_port, uint8_t src_ip[IP_LEN], uint16_t src_port, uint8_t* udpData, uint16_t len)
{
  /* IPAddress src(src_ip[0],src_ip[1],src_ip[2],src_ip[3]);
    Serial.print("dPort:");  Serial.print(dest_port);
    Serial.print("  sPort: ");  Serial.print(src_port);
    Serial.print("  sIP: ");  ether.printIp(src_ip);  Serial.println("  end");

    //for (int16_t i = 0; i < len; i++) {
    //Serial.print(udpData[i],HEX); Serial.print("\t"); } Serial.println(len);
  */
  //if (sizeof(udpData) < 5) return;

  if (udpData[0] == 128 && udpData[1] == 129 && udpData[2] == 127) //Data
  {
    if (udpData[3] == 254)
    {
      gpsSpeed = ((float)(udpData[5] | udpData[6] << 8)) * 0.1;

      prevGuidanceStatus = guidanceStatus;

      guidanceStatus = udpData[7];
      guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

      //Bit 8,9    set point steer angle * 100 is sent
      steerAngleSetPoint = ((float)(udpData[8] | udpData[9] << 8)) * 0.01; //high low bytes

      //Serial.println(gpsSpeed);
      


    
      if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1))
      {
        watchdogTimer = WATCHDOG_FORCE_VALUE; //turn off steering motor
      }
      else          //valid conditions to turn on autosteer
      {
        watchdogTimer = 0;  //reset watchdog
      }

      //Bit 10 Tram
      xte = udpData[10];

      //Bit 11
      relay = udpData[11];

      //Bit 12
      relayHi = udpData[12];

      //----------------------------------------------------------------------------
      //Serial Send to agopenGPS

      int16_t sa = (int16_t)(steerAngleActual * 100);

      PGN_253[5] = (uint8_t)sa;
      PGN_253[6] = sa >> 8;

      if (useCMPS)
      {
        Wire.beginTransmission(CMPS14_ADDRESS);
        Wire.write(0x02);
        Wire.endTransmission();

        Wire.requestFrom(CMPS14_ADDRESS, 2);
        while (Wire.available() < 2);

        //the heading x10
        PGN_253[8] = Wire.read();
        PGN_253[7] = Wire.read();

        Wire.beginTransmission(CMPS14_ADDRESS);
        Wire.write(0x1C);
        Wire.endTransmission();

        Wire.requestFrom(CMPS14_ADDRESS, 2);
        while (Wire.available() < 2);

        //the roll x10
        PGN_253[10] = Wire.read();
        PGN_253[9] = Wire.read();
      }
      else if (useBNO08x)
      {
        if (bno08x.dataAvailable() == true)
        {
          bno08xHeading = (bno08x.getYaw()) * CONST_180_DIVIDED_BY_PI; // Convert yaw / heading to degrees
          bno08xHeading = -bno08xHeading; //BNO085 counter clockwise data to clockwise data

          if (bno08xHeading < 0 && bno08xHeading >= -180) //Scale BNO085 yaw from [-180°;180°] to [0;360°]
          {
            bno08xHeading = bno08xHeading + 360;
          }

          if (steerConfig.IsUseX_Axis)
            bno08xRoll = (bno08x.getPitch()) * CONST_180_DIVIDED_BY_PI; // Convert pitch to degrees
          else
            bno08xRoll = (bno08x.getRoll()) * CONST_180_DIVIDED_BY_PI; //Convert roll to degrees

          bno08xHeading10x = (int16_t)(bno08xHeading * 10);
          bno08xRoll10x = (int16_t)(bno08xRoll * 10);

          //Serial.print(bno08xHeading10x);
          //Serial.print(",");
          //Serial.println(bno08xRoll10x);

          //the heading x10
          PGN_253[7] = (uint8_t)bno08xHeading10x;
          PGN_253[8] = bno08xHeading10x >> 8;


          //the roll x10
          PGN_253[9] = (uint8_t)bno08xRoll10x;
          PGN_253[10] = bno08xRoll10x >> 8;
        }
      }
      else
      {
        //heading
        PGN_253[7] = (uint8_t)9999;
        PGN_253[8] = 9999 >> 8;

        //roll
        PGN_253[9] = (uint8_t)8888;
        PGN_253[10] = 8888 >> 8;
      }

      PGN_253[11] = switchByte;
      PGN_253[12] = (uint8_t)pwmDisplay;

      //checksum
      int16_t CK_A = 0;
      for (uint8_t i = 2; i < PGN_253_Size; i++)
        CK_A = (CK_A + PGN_253[i]);

      PGN_253[PGN_253_Size] = CK_A;

      //off to AOG
      ether.sendUdp(PGN_253, sizeof(PGN_253), portMy, ipDestination, portDestination);

      //Steer Data 2 -------------------------------------------------
      if (steerConfig.PressureSensor || steerConfig.CurrentSensor)
      {
        if (aog2Count++ > 2)
        {
          //Send fromAutosteer2
          PGN_250[5] = (byte)sensorReading;

          //add the checksum for AOG2
          CK_A = 0;
          for (uint8_t i = 2; i < PGN_250_Size; i++)
          {
            CK_A = (CK_A + PGN_250[i]);
          }
          PGN_250[PGN_250_Size] = CK_A;

          //off to AOG
          ether.sendUdp(PGN_250, sizeof(PGN_250), portMy, ipDestination, portDestination);
          aog2Count = 0;
        }
      }

      //Serial.println(steerAngleActual);
      //--------------------------------------------------------------------------
    }

    else if (udpData[3] == 200) // Hello from AgIO
    {
      int16_t sa = (int16_t)(steerAngleActual * 100);

      helloFromAutoSteer[5] = (uint8_t)sa;
      helloFromAutoSteer[6] = sa >> 8;

      helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
      helloFromAutoSteer[8] = helloSteerPosition >> 8;
      helloFromAutoSteer[9] = switchByte;

      ether.sendUdp(helloFromAutoSteer, sizeof(helloFromAutoSteer), portMy, ipDestination, portDestination);
    }

    //steer settings
    else if (udpData[3] == 252)
    {
      //PID values
      steerSettings.Kp = udpData[5];   // read Kp from AgOpenGPS

      steerSettings.highPWM = udpData[6]; // read high pwm

      steerSettings.lowPWM = udpData[7];   // read lowPWM from AgOpenGPS

      steerSettings.minPWM = udpData[8]; //read the minimum amount of PWM for instant on

      float temp = steerSettings.minPWM;
      temp *= 1.2;
      steerSettings.lowPWM = (uint8_t)temp;

      steerSettings.steerSensorCounts = udpData[9]; //sent as setting displayed in AOG

      steerSettings.wasOffset = (udpData[10]);  //read was zero offset Lo

      steerSettings.wasOffset |= (udpData[11] << 8);  //read was zero offset Hi

      steerSettings.AckermanFix = (float)udpData[12] * 0.01;

      //crc
      //udpData[13];

      //store in EEPROM
      EEPROM.put(10, steerSettings);

      Serial.println("mise a jour code steer setting");

      // for PWM High to Low interpolator
      highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
    }

    else if (udpData[3] == 251)  //251 FB - SteerConfig
    {
      uint8_t sett = udpData[5]; //setting0

      if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
      if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
      if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
      if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
      if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
      if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
      if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
      if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

      steerConfig.PulseCountMax = udpData[6];

      //was speed
      //udpData[7];

      sett = udpData[8]; //setting1 - Danfoss valve etc

      if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
      if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
      if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;
      if (bitRead(sett, 3)) steerConfig.IsUseX_Axis = 1; else steerConfig.IsUseX_Axis = 0;

      //crc
      //udpData[13];

      EEPROM.put(40, steerConfig);

      Serial.println("mise a jour code steer config");
      //reset the arduino
      resetFunc();
    }

    else if (udpData[3] == 201)
    {
      //make really sure this is the subnet pgn
      if (udpData[4] == 5 && udpData[5] == 201 && udpData[6] == 201)
      {
        networkAddress.ipOne = udpData[7];
        networkAddress.ipTwo = udpData[8];
        networkAddress.ipThree = udpData[9];

        Serial.print("\r\n Subnet Changed to: ");
        Serial.print(udpData[7]); Serial.print(" . ");
        Serial.print(udpData[8]); Serial.print(" . ");
        Serial.print(udpData[9]); Serial.println();

        delay(100);

        //save in EEPROM and restart
        EEPROM.put(60, networkAddress);
        resetFunc();
      }
    }//end FB

    //scan reply
    else if (udpData[3] == 202)
    {
      //make really sure this is the reply pgn
      if (udpData[4] == 3 && udpData[5] == 202 && udpData[6] == 202)
      {
        uint8_t scanReply[] = { 128, 129, 126, 203, 7,
                                networkAddress.ipOne, networkAddress.ipTwo, networkAddress.ipThree, 126,
                                src_ip[0], src_ip[1], src_ip[2], 23
                              };

        //checksum
        int16_t CK_A = 0;
        for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
        {
          CK_A = (CK_A + scanReply[i]);
        }
        scanReply[sizeof(scanReply) - 1] = CK_A;

        static uint8_t ipDest[] = { 255, 255, 255, 255 };
        uint16_t portDest = 9999; //AOG port that listens

        Serial.print("\r\nAdapter IP: ");
        Serial.print(src_ip[0]); Serial.print(" . ");
        Serial.print(src_ip[1]); Serial.print(" . ");
        Serial.print(src_ip[2]); Serial.print(" . ");
        Serial.print(src_ip[3]);

        //off to AOG
        ether.sendUdp(scanReply, sizeof(scanReply), portMy, ipDest, portDest);

        Serial.print("\r\n Module IP: ");
        Serial.print(src_ip[0]); Serial.print(" . ");
        Serial.print(src_ip[1]); Serial.print(" . ");
        Serial.print(src_ip[2]); Serial.print(" . ");
        Serial.print(src_ip[3]); Serial.println();

        Serial.print("CurrentSensor: ");
        Serial.println(sensorReading);
        Serial.print("Steer Counts: ");
        Serial.println(helloSteerPosition);
        Serial.print("Switch Byte: ");
        Serial.println(switchByte);
        Serial.println(" --------- ");
      }
    }

  } //end if 80 81 7F




  ////////////////



  if (udpData[0] == 0x80 && udpData[1] == 0x81 && udpData[2] == 0x7F) //Data
  {


    if (udpData[3] == 239)  //machine data
    {
      uTurn = udpData[5];
      gpsSpeed = (float)udpData[6];//actual speed times 4, single uint8_t

      hydLift = udpData[7];
      tramline = udpData[8];  //bit 0 is right bit 1 is left




      relayLo = udpData[11];          // read relay control from AgOpenGPS
      relayHi = udpData[12];

      if (aogConfig.isRelayActiveHigh)
      {
        tramline = 255 - tramline;
        relayLo = 255 - relayLo;
        relayHi = 255 - relayHi;
      }

      //Bit 13 CRC

      //reset watchdog
       //  watchdogTimer = 0;
    }

    else if (udpData[3] == 200) // Hello from AgIO
    {
      if (udpData[7] == 1)
      {
        relayLo -= 255;
        relayHi -= 255;
        //  watchdogTimer =0;
      }

      helloFromMachine[5] = relayLo;
      helloFromMachine[6] = relayHi;

      ether.sendUdp(helloFromMachine, sizeof(helloFromMachine), portMy, ipDestination, portDestination);
    }


    else if (udpData[3] == 238)
    {
      aogConfig.raiseTime = udpData[5];
      aogConfig.lowerTime = udpData[6];
      aogConfig.enableToolLift = udpData[7];

      //set1
      uint8_t sett = udpData[8];  //setting0
      if (bitRead(sett, 0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;

      aogConfig.user1 = udpData[9];
      aogConfig.user2 = udpData[10];
      aogConfig.user3 = udpData[11];
      aogConfig.user4 = udpData[12];

      //crc

      //save in EEPROM and restart
      EEPROM.put(100, aogConfig);

      Serial.println("mise a jour code tracteur relevage");

      //resetFunc();
    }

    //            else if (udpData[3] == 201)
    //            {
    //                //make really sure this is the subnet pgn
    //                if (udpData[4] == 5 && udpData[5] == 201 && udpData[6] == 201)
    //                {
    //                    networkAddress.ipOne = udpData[7];
    //                    networkAddress.ipTwo = udpData[8];
    //                    networkAddress.ipThree = udpData[9];
    //
    //                    //save in EEPROM and restart
    //                    EEPROM.put(50, networkAddress);
    //                    resetFunc();
    //                }
    //            }

    //Scan Reply
    else if (udpData[3] == 202)
    {
      //make really sure this is the subnet pgn
      if (udpData[4] == 3 && udpData[5] == 202 && udpData[6] == 202)
      {
        uint8_t scanReply[] = { 128, 129, 123, 203, 7,
                                networkAddress.ipOne, networkAddress.ipTwo, networkAddress.ipThree, 123,
                                src_ip[0], src_ip[1], src_ip[2], 23
                              };

        //checksum
        int16_t CK_A = 0;
        for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
        {
          CK_A = (CK_A + scanReply[i]);
        }
        scanReply[sizeof(scanReply) - 1] = CK_A;

        static uint8_t ipDest[] = { 255, 255, 255, 255 };
        uint16_t portDest = 9999; //AOG port that listens

        //off to AOG
        ether.sendUdp(scanReply, sizeof(scanReply), portMy, ipDest, portDest);

        Serial.println("mise a jour code  cka machine");
      }
    }

    else if (udpData[3] == 236) //EC Relay Pin Settings
    {
      for (uint8_t i = 0; i < 24; i++)
      {
        pin[i] = udpData[i + 5];
      }

      //save in EEPROM and restart
      EEPROM.put(120, pin);

      Serial.println("mise a jour code sections");
    }
  }
















  //////////////////







} //end udp callback


//ISR Steering Wheel Encoder

void EncoderFunc()
{
  if (encEnable)
  {
    pulseCount++;
    encEnable = false;
  }
}


void SetRelays(void)
{
  //pin, rate, duration  130 pp meter, 3.6 kmh = 1 m/sec or gpsSpeed * 130/3.6 or gpsSpeed * 36.1111
  //gpsSpeed is 10x actual speed so 3.61111
  gpsSpeed *= 3.61111;
  //tone(13, gpsSpeed);


  digitalWrite(Hyd_up, isLower);
  digitalWrite(Hyd_down, isRaise);


  //Load the current pgn relay state - Sections
  for (uint8_t i = 0; i < 8; i++)
  {
    relayState[i] = bitRead(relayLo, i);
  }

  for (uint8_t i = 0; i < 8; i++)
  {
    relayState[i + 8] = bitRead(relayHi, i);
  }

  // Hydraulics
  relayState[16] = isLower;
  relayState[17] = isRaise;

  //Tram
  relayState[18] = bitRead(tramline, 0); //right
  relayState[19] = bitRead(tramline, 1); //left

  //GeoStop
  relayState[20] = (geoStop == 0) ? 0 : 1;

  // if (pin[0]) digitalWrite(4, relayState[pin[0] - 1]);
  // if (pin[1]) digitalWrite(5, relayState[pin[1] - 1]);
  // if (pin[2]) digitalWrite(6, relayState[pin[2] - 1]);
  // if (pin[3]) digitalWrite(7, relayState[pin[3] - 1]);

  // if (pin[4]) digitalWrite(8, relayState[pin[4] - 1]);
  // if (pin[5]) digitalWrite(9, relayState[pin[5] - 1]);

  //if (pin[6]) digitalWrite(10, relayState[pin[6]-1]);
  //if (pin[7]) digitalWrite(11, relayState[pin[7]-1]);

  //if (pin[8]) digitalWrite(12, relayState[pin[8]-1]);
  //if (pin[9]) digitalWrite(4, relayState[pin[9]-1]);

  //if (pin[10]) digitalWrite(IO#Here, relayState[pin[10]-1]);
  //if (pin[11]) digitalWrite(IO#Here, relayState[pin[11]-1]);
  //if (pin[12]) digitalWrite(IO#Here, relayState[pin[12]-1]);
  //if (pin[13]) digitalWrite(IO#Here, relayState[pin[13]-1]);
  //if (pin[14]) digitalWrite(IO#Here, relayState[pin[14]-1]);
  //if (pin[15]) digitalWrite(IO#Here, relayState[pin[15]-1]);
  //if (pin[16]) digitalWrite(IO#Here, relayState[pin[16]-1]);
  //if (pin[17]) digitalWrite(IO#Here, relayState[pin[17]-1]);
  //if (pin[18]) digitalWrite(IO#Here, relayState[pin[18]-1]);
  //if (pin[19]) digitalWrite(IO#Here, relayState[pin[19]-1]);
}
