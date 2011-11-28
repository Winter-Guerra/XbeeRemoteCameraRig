//***GLOBAL INCLUDES***//
#include "WProgram.h"
#include "settings.h"
#include <XBee.h>
#include "types.h"

//****INDIVIDUAL INCLUDES FOR CAMERA AND CONTROLLER***/
#if IS_CONTROLLER == 1
#include "controller.h"
#else
#include "camera.h"
#endif

//*****ERROR AND STATUS LEDS***/// 
//Both the status and error leds were soldered to different pins on the controller and camera reciever.
#if IS_CONTROLLER == 1
uint8_t statusLED = 7; //This LED is here just to make sure that the Arduino is on an at least running the program. Should this be a TX/RX light instead?
#else
uint8_t statusLED = 13; //This LED is here just to make sure that the Arduino is on an at least running the program. Should this be a TX/RX light instead?
#endif

#if IS_CONTROLLER == 1
uint8_t errorLED = 6; //On the controller board, this led was soldered to the wrong pin.
#else
uint8_t errorLED = 12; //This LED (RED) will light up if there is an error. It will turn off after a while... 
#endif

boolean errorState = false;
uint32_t errorLEDMillis = 0; //This var will keep track of the last time an error was triggered. The error will be dismissed after millis() - errorLEDMillis >= erorTimeout
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
const uint8_t xMicroStepping = 8; //Microsteps per step
const uint8_t xGearRatio = 2; //for every rotation of the big gear, the stepper gear must rotate this many times.
const uint16_t xStepperStepsPerRotation = 200;
const uint16_t xStepsPerRotation = xMicroStepping * xStepperStepsPerRotation * xGearRatio; //steps per rotation.
const uint16_t xStepRange = (xStepsPerRotation)/2; //This axis only has 360/4 degrees of motion (90 degrees)
const uint16_t xStepperMidpoint = xStepRange/2; //This is the midpoint of the range. The stepper should start out in this position when the controller is turned on.

//Vars for the Y rot axis (Up/Down pitch)
const uint8_t yMicroStepping = 8; //Microsteps per step
const uint8_t yGearRatio = 2; //for every rotation of the big gear, the stepper gear must rotate this many times.
const uint16_t yStepperStepsPerRotation = 200;
const uint16_t yStepsPerRotation = yMicroStepping * yStepperStepsPerRotation * yGearRatio; //steps per rotation.
const uint16_t yStepRange = (yStepsPerRotation)/4; //This axis can rotate 270 degrees 
const uint16_t yStepperMidpoint = yStepRange/2; //This is the midpoint of the range. The stepper should start out in this position when the controller is turned on.

uint16_t stepperRanges[] = {
  xStepRange, yStepRange};

const uint16_t xStepperAcceleration = 800; //Default speed vals
const uint16_t yStepperAcceleration = 800;
const uint16_t xStepperMaxSpeed = 1600;
const uint16_t yStepperMaxSpeed = 1600;

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

//*****GLOBAL FUNCTION PROTOTYPES****//
returnPacketStates readPacketBufferTimeout(uint16_t timeout = PACKET_TIMEOUT);

returnPacketStates readPacketBuffer();

void sendAtCommand();

void cleanPayload();

void cleanRecievedPayload();

void turnOnErrorLED();

void handleStatusLEDs();

void setupXbeeGlobalSettings();

