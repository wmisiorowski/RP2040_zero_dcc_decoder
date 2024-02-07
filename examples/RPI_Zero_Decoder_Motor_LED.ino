
// NMRA Dcc Multifunction Motor Decoder Demo
//
// Author: Alex Shepherd 2019-03-30
// 
// This example requires these Arduino Libraries:
//
// 1) The NmraDcc Library from: http://mrrwa.org/download/
//
// These libraries can be found and installed via the Arduino IDE Library Manager
//
// This is a simple demo of how to drive and motor speed and direction using PWM and a motor H-Bridge
// It uses vStart and vHigh CV values to customise the PWM values to the motor response 
// It also uses the Headling Function to drive 2 LEDs for Directional Headlights
// Apart from that there's nothing fancy like Lighting Effects or a function matrix or Speed Tables - its just the basics...
//

#include <NmraDcc.h>
// Uncomment any of the lines below to enable debug messages for different parts of the code
#define DEBUG_FUNCTIONS
#define DEBUG_SPEED
#define DEBUG_PWM
#define DEBUG_DCC_ACK
#define DEBUG_DCC_MSG

#if defined(DEBUG_FUNCTIONS) or defined(DEBUG_SPEED) or defined(DEBUG_PWM) or defined(DEBUG_DCC_ACK) or defined(DEBUG_DCC_MSG)
#define DEBUG_PRINT
#endif

// This is the default DCC Address
#define DEFAULT_DECODER_ADDRESS 14

// Define input Pin number for the DCC Signal 
#define DCC_PIN     14

#define LED_PIN_FWD 26
#define LED_PIN_REV 27
#define MOTOR_DIR_PIN 8
#define MOTOR_PWM_PIN 2

// Some global state variables
uint8_t newLedState = 0;
uint8_t lastLedState = 0;

uint8_t newLedState_1 = 0;
uint8_t lastLedState_1 = 0;

uint8_t newDirection = 0;
uint8_t lastDirection = 0;

float newSpeed = 0;
uint8_t lastSpeed = 0;
float numSpeedSteps = SPEED_STEP_128;

uint8_t lastFuncStateList[FN_LAST+1];

uint8_t vStart;
uint8_t vHigh;

// Structure for CV Values Table
struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

// CV Addresses we will be using
#define CV_VSTART  2
#define CV_VHIGH   5

// Default CV Values Table
CVPair FactoryDefaultCVs [] =
{
	// The CV Below defines the Short DCC Address
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, DEFAULT_DECODER_ADDRESS},

  // Three Step Speed Table
  {CV_VSTART, 120},
  {CV_VHIGH, 255},

  // These two CVs define the Long DCC Address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, CALC_MULTIFUNCTION_EXTENDED_ADDRESS_MSB(DEFAULT_DECODER_ADDRESS)},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, CALC_MULTIFUNCTION_EXTENDED_ADDRESS_LSB(DEFAULT_DECODER_ADDRESS)},

// ONLY uncomment 1 CV_29_CONFIG line below as approprate
//  {CV_29_CONFIG,                                      0}, // Short Address 14 Speed Steps
  {CV_29_CONFIG,                       CV29_F0_LOCATION}, // Short Address 28/128 Speed Steps
//  {CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION}, // Long  Address 28/128 Speed Steps  
};

NmraDcc  Dcc ;

uint8_t FactoryDefaultCVIndex = 0;

// This call-back function is called when a CV Value changes so we can update CVs we're using
void notifyCVChange( uint16_t CV, uint8_t Value)
{
  switch(CV)
  {
    case CV_VSTART:
      vStart = Value;
      break;
      
    case CV_VHIGH:
      vHigh = Value;
      break;
  }
}

void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

// This call-back function is called whenever we receive a DCC Speed packet for our address 
void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps )
{
  #ifdef DEBUG_SPEED
  Serial.print("notifyDccSpeed: Addr: ");
  Serial.print(Addr,DEC);
  Serial.print( (AddrType == DCC_ADDR_SHORT) ? "-S" : "-L" );
  Serial.print(" Speed: ");
  Serial.print(Speed,DEC);
  Serial.print(" Steps: ");
  Serial.print(SpeedSteps,DEC);
  Serial.print(" Dir: ");
  Serial.println( (Dir == DCC_DIR_FWD) ? "Forward" : "Reverse" );
  #endif

  newDirection = Dir;
  newSpeed = Speed;
  numSpeedSteps = SpeedSteps;
};



//------------------------------------------------------------------------------------------------------------
// This call-back function is called whenever we receive a DCC Function packet for our address 
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
  #ifdef DEBUG_FUNCTIONS
  Serial.print("notifyDccFunc: Addr: ");
  Serial.print(Addr,DEC);
  Serial.print( (AddrType == DCC_ADDR_SHORT) ? 'S' : 'L' );
  Serial.print("  Function Group: ");
  Serial.print(FuncGrp,DEC);
  Serial.println();
  #endif

  if(lastFuncStateList[FuncGrp] != FuncState)
  {
    lastFuncStateList[FuncGrp] = FuncState;
    switch(FuncGrp)
    {
    #ifdef NMRA_DCC_ENABLE_14_SPEED_STEP_MODE    
      case FN_0:
        Serial.print(" FN0: ");
        Serial.println((FuncState & FN_BIT_00) ? "1  " : "0  ");

        break;
    #endif
       
      case FN_0_4:

        
        if(Dcc.getCV(CV_29_CONFIG) & CV29_F0_LOCATION) // Only process Function 0 in this packet if we're not in Speed Step 14 Mode
        {
          Serial.print(" FN 0: ");
          Serial.print((FuncState & FN_BIT_00) ? "1  ": "0  ");
          digitalWrite(LED_BUILTIN, (FuncState & FN_BIT_00));
          
        }
       
        Serial.print(" FN 1-4: ");
        Serial.print((FuncState & FN_BIT_01) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_02) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_03) ? "1  ": "0  ");
        Serial.println((FuncState & FN_BIT_04) ? "1  ": "0  ");
        

        break;
    
      case FN_5_8:
        Serial.print(" FN 5-8: ");
        Serial.print((FuncState & FN_BIT_05) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_06) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_07) ? "1  ": "0  ");
        Serial.println((FuncState & FN_BIT_08) ? "1  ": "0  ");
        digitalWrite(LED_BUILTIN, (FuncState & FN_BIT_05));

        break;
    
      case FN_9_12:
        Serial.print(" FN 9-12: ");
        Serial.print((FuncState & FN_BIT_09) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_10) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_11) ? "1  ": "0  ");
        Serial.println((FuncState & FN_BIT_12) ? "1  ": "0  ");
        digitalWrite(LED_BUILTIN, (FuncState & FN_BIT_09));

        break;
      
      case FN_13_20:
        Serial.print(" FN 13-20: ");
        Serial.print((FuncState & FN_BIT_13) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_14) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_15) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_16) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_17) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_18) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_19) ? "1  ": "0  ");
        Serial.println((FuncState & FN_BIT_20) ? "1  ": "0  ");
        digitalWrite(LED_BUILTIN, (FuncState & FN_BIT_13));

        break;
      
      case FN_21_28:
        Serial.print(" FN 21-28: ");
        Serial.print((FuncState & FN_BIT_21) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_22) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_23) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_24) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_25) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_26) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_27) ? "1  ": "0  ");
        Serial.println((FuncState & FN_BIT_28) ? "1  ": "0  ");
        digitalWrite(LED_BUILTIN, (FuncState & FN_BIT_21));

        break;  
    }
  }
}





//-------------------------------------------------------------------------------------------------------------
// This call-back function is called whenever we receive a DCC Packet
#ifdef  DEBUG_DCC_MSG
void notifyDccMsg( DCC_MSG * Msg)
{
  Serial.print("notifyDccMsg: ") ;
  for(uint8_t i = 0; i < Msg->Size; i++)
  {
    Serial.print(Msg->Data[i], HEX);
    Serial.write(' ');
  }
  Serial.println();
}
#endif

// This call-back function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read
// So we will just turn the motor on for 8ms and then turn it off again.

void notifyCVAck(void)
{
  #ifdef DEBUG_DCC_ACK
  Serial.println("notifyCVAck") ;
  #endif

  digitalWrite(MOTOR_DIR_PIN, HIGH);
  digitalWrite(MOTOR_PWM_PIN, HIGH);

  delay( 8 );  

  digitalWrite(MOTOR_DIR_PIN, LOW);
  digitalWrite(MOTOR_PWM_PIN, LOW);
}

void setup()
{

  Serial.begin(115200);

  while (!Serial) {
    delay(1);
  }     
  
  #ifdef DEBUG_PRINT 

  Serial.println("NMRA Dcc Multifunction Motor Decoder Demo");
  #endif

  // Setup the Pins for the Fwd/Rev LED for Function 0 Headlight
  pinMode(LED_PIN_FWD, OUTPUT);
  pinMode(LED_PIN_REV, OUTPUT);

  // Setup the Pins for the Motor H-Bridge Driver
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);

  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  // Many Arduino Cores now support the digitalPinToInterrupt() function that makes it easier to figure out the
  // Interrupt Number for the Arduino Pin number, which reduces confusion. 
#ifdef digitalPinToInterrupt
  Dcc.pin(DCC_PIN, 0);
#else
  Dcc.pin(0, DCC_PIN, 1);
#endif
  
  Dcc.init( MAN_ID_DIY, 10, FLAGS_MY_ADDRESS_ONLY | FLAGS_AUTO_FACTORY_DEFAULT, 0 );

  // Uncomment to force CV Reset to Factory Defaults
//  notifyCVResetFactoryDefault();

  // Read the current CV values for vStart and vHigh
  vStart = Dcc.getCV(CV_VSTART);
  vHigh = Dcc.getCV(CV_VHIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, newLedState);
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  // Handle Speed changes
  if(lastSpeed != newSpeed)
  {
    lastSpeed = newSpeed;

    // Stop if speed = 0 or 1
    
    if(newSpeed <= 1) {
      digitalWrite(MOTOR_PWM_PIN, LOW);
      digitalWrite(MOTOR_DIR_PIN, LOW);
    }
    // Calculate PWM value in the range 1..255   
    else
    {
      uint8_t vScaleFactor;
      
      if((vHigh > 1) && (vHigh > vStart))
        vScaleFactor = vHigh - vStart;
      else
        vScaleFactor = 255 - vStart;

      uint8_t modSpeed = newSpeed - 1;
      uint8_t modSteps = numSpeedSteps - 1;
      
      uint8_t newPwm = (uint8_t) vStart + modSpeed * vScaleFactor / modSteps;

      #ifdef DEBUG_PWM
      Serial.print("New Speed: vStart: ");
      Serial.print(vStart);
      Serial.print(" vHigh: ");
      Serial.print(vHigh);
      Serial.print(" modSpeed: ");
      Serial.print(modSpeed);
      Serial.print(" vScaleFactor: ");
      Serial.print(vScaleFactor);
      Serial.print(" modSteps: ");
      Serial.print(modSteps);
      Serial.print(" newPwm: ");
      Serial.println(newPwm);
      #endif
            
      //analogWrite(MOTOR_PWM_PIN, newPwm);

      switch (newDirection) {
        case 0 :
            analogWrite(MOTOR_DIR_PIN, int((newSpeed/numSpeedSteps)*255));
            analogWrite(MOTOR_PWM_PIN, 0);
          break;

        case 1 :
            analogWrite(MOTOR_DIR_PIN, 0);
            analogWrite(MOTOR_PWM_PIN, int((newSpeed/numSpeedSteps)*255));
          break;

        default :
          break;
      }
      
    }
  }
  
  // Handle resetting CVs back to Factory Defaults
  if( FactoryDefaultCVIndex && Dcc.isSetCVReady())
  {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array 
    Dcc.setCV( FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
  }
}
