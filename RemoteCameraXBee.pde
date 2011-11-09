#include <XBee.h>
#include "types.h"

#define IS_CONTROLLER 0 //Is this the camera controller? Or the reciever?
#define DEBUG_MODE IS_CONTROLLER && 1 //Do we want debug output on the serial line?


#define AT_COMMAND_DELAY 10 //In milliseconds
#define TX_COMMAND_DELAY 20 //In milliseconds. (Ballparked by assuming )
#define PACKET_TIMEOUT 100

#define PAYLOAD_LENGTH 10


uint8_t statusLED = 11; //This LED is here just to make sure that the Arduino is on an at least running the program. Should this be a TX/RX light instead?
uint8_t errorLED = 12; //This LED (RED) will light up if there is an error. It will turn off after a while... 

boolean errorState = false;
uint32_t errorLEDMillis = 0; //This var will keep track of the last time an error was triggered. The error will be dismissed after millis() - errorLEDMillis >= errorTimeout
uint32_t errorTimeout = 500; //This is the time after the error is triggered to dismiss it.

uint8_t RTS = 2; //Arduino must pull this high if it wants the Xbee to stop sending (unused)
uint8_t CTS = 3; //If CTS is pulled high by the Xbee, Stop sending data.

//Number of steppers to control
#define STEPPER_COUNT 2

//Definition of stepper pins
uint8_t xStepperEnable = 5;
uint8_t yStepperEnable = 6;
uint8_t xStepperDir = 7;
uint8_t yStepperDir = 8;
uint8_t xStepperStep = 9;
uint8_t yStepperStep = 10;

//Vars for the X rot axis (pan)
const uint8_t xMicroStepping = 16; //Microsteps per step
const uint8_t xGearRatio = 2; //for every rotation of the big gear, the stepper gear must rotate this many times.
const uint16_t xStepperStepsPerRotation = 200;
const uint16_t xStepsPerRotation = xMicroStepping * xStepperStepsPerRotation * xGearRatio; //steps per rotation.
const uint16_t xStepRange = (xStepsPerRotation*3)/4; //This axis only has 360/4 degrees of motion (90 degrees)
const uint16_t xStepperMidpoint = xStepRange/2; //This is the midpoint of the range. The stepper should start out in this position when the controller is turned on.

//Vars for the Y rot axis (Up/Down pitch)
const uint8_t yMicroStepping = 16; //Microsteps per step
const uint8_t yGearRatio = 2; //for every rotation of the big gear, the stepper gear must rotate this many times.
const uint16_t yStepperStepsPerRotation = 200;
const uint16_t yStepsPerRotation = yMicroStepping * yStepperStepsPerRotation * yGearRatio; //steps per rotation.
const uint16_t yStepRange = (yStepsPerRotation)/4; //This axis can rotate 270 degrees 
const uint16_t yStepperMidpoint = yStepRange/2; //This is the midpoint of the range. The stepper should start out in this position when the controller is turned on.

uint16_t stepperRanges[] = {
  xStepRange, yStepRange};

const uint16_t xStepperAcceleration = 100; //Default speed vals
const uint16_t yStepperAcceleration = 100;
const uint16_t xStepperMaxSpeed = 200;
const uint16_t yStepperMaxSpeed = 200;

//Analog Inputs
uint8_t potXPin = 0;
uint8_t potYPin = 1;
uint8_t potZPin = 2;

//Array to quickly access, save and send the analog data
uint8_t potPins[] = {
  potXPin,potYPin,potZPin};

uint16_t potVals[STEPPER_COUNT]; //Potval containers
uint16_t potConvertedToSteps[STEPPER_COUNT]; //Potvals converted to steps
uint16_t oldPotVals[] = {
  0,0}; //For anti-jitter purposes
uint8_t iterations = 4; //averaging iterations

uint16_t stepperTarget[STEPPER_COUNT];

//Values used for mapping the pot rotations to big gear rotations
uint16_t analogMaxVal = 1023;

uint8_t cameraAddress[] = { //Address of the Camera
  0x10,0x00};
uint8_t controllerAddress[] = { //Address of the Controller
  0x20,0x00};

uint16_t cameraAddress16 = (cameraAddress[0] << 8) + cameraAddress[1]; //16 bit cast
uint16_t controllerAddress16 = (controllerAddress[0] << 8) + controllerAddress[1]; //16 bit cast

uint8_t channel[] = {
  0x11}; //Channel
uint8_t panID[] = {
  0x11,0x11}; //Personal Area Network

uint8_t payload[PAYLOAD_LENGTH]; //Payload buffer. Clean this promptly after using!!!!!
uint8_t recievedPayload[PAYLOAD_LENGTH]; //Buffer for the recieved data. Clean promptly after using!

//XBee AT commands. USE PROGMEM TO SAVE SPACE!
uint8_t ATMY[] = { 
  'M', 'Y' }; //my 16 bit address
uint8_t ATCH[] = { 
  'C', 'H' }; //channel
uint8_t ATID[] = { 
  'I', 'D' }; //PANID
uint8_t ATWR[] = { 
  'W', 'R' }; //write settings to xbee
uint8_t ATAC[] = { 
  'A', 'C' }; //Apply changes to xbee



//Packet command codes
const uint8_t positionCommandCode = 0x01;
const uint8_t positionResetCommandCode = 0x02;
const uint8_t positionTrimCommandCode = 0x03;




//*****Reusable Instances!!******//
XBee xbee = XBee(); //Instantiate a new xbee instance
Tx16Request txRequest = Tx16Request();
TxStatusResponse txStatus = TxStatusResponse();
AtCommandRequest atRequest = AtCommandRequest();
AtCommandResponse atResponse = AtCommandResponse();
Rx16Response rx16 = Rx16Response();


void setup() {
  pinMode(statusLED, OUTPUT);
  pinMode(errorLED, OUTPUT);

  pinMode(RTS,INPUT);
  pinMode(CTS,INPUT);

#if IS_CONTROLLER == 1 
  setupControllerPins();
  setupControllerSerial();
  delay(5000); //Wait for the XBee to initialize. 5 sec pause.
  setupXbeeControllerAddress();
#else
  setupCameraPins();
  setupCameraSerial();
  delay(5000); //Wait for the XBee to initialize. 5 sec pause.
  setupXbeeCameraAddress();
#endif

  setupXbeeGlobalSettings();

}

void loop() {
#if IS_CONTROLLER
  //Run the sampling and TX slice
  runControllerSlice();
#else
  runCameraSlice();
#endif
  handleStatusLEDs();

}

returnPacketStates readPacketBufferTimeout(uint16_t timeout = PACKET_TIMEOUT) {
  //This is a wrapper to the readPacketBuffer. It will read for a packet until the timeout expires
  uint32_t timeoutMillis = millis();

  returnPacketStates responseCode = readPacketBuffer();
  while (timeoutMillis+timeout < millis() && responseCode == PACKET_NOT_FINISHED){
    //A packet has not arrived yet and the timeout still has not expired. Read some more!
    responseCode = readPacketBuffer();

    //Handle some camera stuff:
#if IS_CONTROLLER != 1
    //Continue to step the steppers while we are stuck waiting for a packet.
    runSteppers();
#endif

  }
  //Ok, so we are done polling for some reason
  if (responseCode == PACKET_NOT_FINISHED) {
    //We did not recieve a packet. Return a fail.
    responseCode = COMM_FAIL;

#if IS_CONTROLLER == 1
    //Also report a error b/c this should not happen
    turnOnErrorLED();
#endif
  }

  return responseCode;//Well! Done!
}

returnPacketStates readPacketBuffer() { //Enumerated return vals!
  returnPacketStates returnStatus = COMM_FAIL;
  //The return status codes go like this.... 
  //COMM_FAIL is no packet was recieved Is the device unplugged?
  //PACKET_NOT_FINISHED is no packet was read. Mind waiting for a bit?
  //TX_ACK is TX ACK
  //TX_FAIL is TX FAILED
  //RX_PACKET is RX
  //AT_ACK is AT ACK
  //AT_ACK_DATA is AT ACK WITH OUTPUT
  //AT_FAIL is AT FAIL

    // after sending a TX request, we expect a status response
  //Don't wait, just poll. We will have another wrapper handling the repeated calls
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) {
    // got a response!

    if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) { //We got a response to our AT command!
      xbee.getResponse().getAtCommandResponse(atResponse);

      if (atResponse.isOk()) {
#if DEBUG_MODE
        Serial.print("Command [");
        Serial.print(atResponse.getCommand()[0]);
        Serial.print(atResponse.getCommand()[1]);
        Serial.println("] was successful!");
#endif

        if (atResponse.getValueLength() > 0) {
#if DEBUG_MODE
          Serial.print("Command value length is ");
          Serial.println(atResponse.getValueLength(), DEC);

          Serial.print("Command value: "); //Is this a masked value? Aka, is a 64 bit addresss passed as one or multiple bytes?

          for (int i = 0; i < atResponse.getValueLength(); i++) {
            Serial.print(atResponse.getValue()[i], HEX);
            Serial.print(" ");
          }

          Serial.println("");
#endif

          returnStatus = AT_ACK_DATA;
        } 
        else { //The AT command did not return any data. That's ok too! It was sucessful.
          returnStatus = AT_ACK;
        }
      } 
      else {
        turnOnErrorLED();
#if DEBUG_MODE
        Serial.print("Command return error code: ");
        Serial.println(atResponse.getStatus(), HEX);
#endif
        returnStatus = AT_FAIL;
      }
    } 
    else if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16); //put it into storage, we will check it later.  
      returnStatus = RX_PACKET;
#if DEBUG_MODE
      Serial.println("We got a RX packet!");
#endif
    }

    else if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
      //We got a response to our TX packet!
      xbee.getResponse().getTxStatusResponse(txStatus); //put it into storage, We will actually check this now though...
      if (txStatus.isSuccess()){
        //The TX was successful! WOOT!
        returnStatus = TX_ACK;
      } 
      else {
        //Awww, The TX was not successful!
        turnOnErrorLED();
        returnStatus = TX_FAIL;
      }  
      //isSuccess() 
    }

  } 
  else if (xbee.getResponse().isError()) {
    // local XBee recieved a malformed packet.
#if DEBUG_MODE == 1
    Serial.println("WTF!? A malformed packet?");
#endif
    turnOnErrorLED();
    returnStatus = COMM_FAIL;

  } 
  else {
    //The packet did not arrive yet. That's disturbing but still ok.
    //Just exit and pass the ball to the wrapper. It can decide whether to poll again.
    returnStatus = PACKET_NOT_FINISHED;

  }
  //cleanPayload();
  //cleanRXPacket();
  return returnStatus;
}

void sendAtCommand() {
  //Flush buffer
#if IS_CONTROLLER == 1
  Serial1.flush();
#else
  Serial.flush();
#endif

  //Send the command then wait for the response

  xbee.send(atRequest);

  returnPacketStates responseCode = readPacketBufferTimeout(PACKET_TIMEOUT);
  if (responseCode == AT_ACK || responseCode == AT_ACK_DATA) {
    //success! we got a packet! Now lets just double check here...
    //Nope! Good enough for me!
  } 
  else {
    //error!
    turnOnErrorLED();
#if DEBUG_MODE
    Serial.println("AT Error");
#endif
  }
  delay(AT_COMMAND_DELAY); //Delay to not overload the XBee input line
}


void cleanPayload() {
  for (int i = 0; i < PAYLOAD_LENGTH; i++) {
    //clean the packet!
    payload[i] = '\0';
  }
}

void cleanRecievedPayload() {
  for (int i = 0; i < PAYLOAD_LENGTH; i++) {
    //clean the packet!
    recievedPayload[i] = '\0';
  }
}

void turnOnErrorLED() {
  //Crap! There has been an error! Quick! Flash some red leds! That should fix the problem!
  //Log the last time an error has been triggered so that the handleStatusLEDs() function knows when to dismiss the error.
  errorLEDMillis = millis();
  errorState = true;
  digitalWrite(errorLED, HIGH);

}

void handleStatusLEDs() {
  //Check the states of the error LEDs and turn them off if the error has not been seen for a while 
  if (errorState == true) { 
    //There has been an error recently, lets see if it's time to dismiss.
    if (millis()-errorLEDMillis >= errorTimeout) {
      //The error has expired!
      errorState = false;
      digitalWrite(errorLED, LOW);
    }
  }
}

void setupXbeeGlobalSettings() {
  //Setup and save the rest of the Xbee settings (pan, channel and stuff)
  atRequest = AtCommandRequest(ATCH,channel,1); //Set our current channel
  sendAtCommand();//Send the command

    atRequest = AtCommandRequest(ATID,panID,2); //Set our current Personal Area Network
  sendAtCommand();//Send the command

    atRequest = AtCommandRequest(ATWR); //Write setting to memory
  sendAtCommand();//Send the command

    atRequest = AtCommandRequest(ATAC); //Apply changes
  sendAtCommand();//Send the command
}


