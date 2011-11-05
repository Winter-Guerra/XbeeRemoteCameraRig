#include <XBee.h>
#include "types.h"

#define IS_CONTROLLER 1 //Is this the camera controller? Or the reciever?
#define DEBUG_MODE IS_CONTROLLER && 0 //Do we want debug output on the serial line?


#define AT_COMMAND_DELAY 10 //In milliseconds
#define TX_COMMAND_DELAY 20 //In milliseconds. (Ballparked by assuming )

#define PAYLOAD_LENGTH 10


uint8_t statusPin = A0;
uint8_t errorPin = A1;

uint8_t RTS = 2; //Arduino must pull this high if it wants the Xbee to stop sending (unused)
uint8_t CTS = 3; //If CTS is pulled high by the Xbee, Stop sending data.

//Number of steppers to control
uint8_t stepperCount = 2;

//Definition of stepper pins
uint8_t xStepperEnable = 5;
uint8_t yStepperEnable = 6;
uint8_t xStepperDir = 7;
uint8_t yStepperDir = 8;
uint8_t xStepperStep = 9;
uint8_t yStepperStep = 10;

//Analog Inputs
uint8_t potXPin = 0;
uint8_t potYPin = 1;
uint8_t potZPin = 2;

//Array to quickly access, save and send the analog data
uint8_t potPins[] = {
  potXPin,potYPin,potZPin};

uint16_t potVals[3]; //Potval containers
uint8_t iterations = 4; //averaging iterations


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
uint8_t recievedPayload[40]; //Buffer for the recieved data. Clean promptly after using!

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
uint8_t positionCommandCode = 0x01;
uint8_t positionResetCommandCode = 0x02;
uint8_t positionTrimCommandCode = 0x03;




//*****Reusable Instances!!******//
XBee xbee = XBee(); //Instantiate a new xbee instance
Tx16Request txRequest = Tx16Request();
TxStatusResponse txStatus = TxStatusResponse();
AtCommandRequest atRequest = AtCommandRequest();
AtCommandResponse atResponse = AtCommandResponse();
Rx16Response rx16 = Rx16Response();

void setup() {
  pinMode(statusPin, OUTPUT);
  pinMode(errorPin, OUTPUT);

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
  delay(20);
}


void sendAtCommand() {
  //Flush buffer
  Serial1.flush();

  //Send the command then wait for the response

  xbee.send(atRequest);

  returnPacketStates responseCode = readPacketBuffer();
  if (responseCode == AT_ACK || responseCode == AT_ACK_DATA) {
    //success! we got a packet! Now lets just double check here...
    //Nope! Good enough for me!
  } 
  else {
    //error!
#if DEBUG_MODE
    Serial.println("AT Error");
#endif
  }
  delay(AT_COMMAND_DELAY); //Delay to not overload the XBee input line
}

void sendTxPositionPacket(int16_t *stepperPos) {
  //Send a packet to the camera containing a stepper x and y pos
  cleanPayload();

  payload[0] = positionCommandCode;

  for (int i = 0; i < stepperCount; i++) {
    payload[i+1] = stepperPos[i] >> 8 && 0xFF; //High byte
    payload[i+2] = stepperPos[i] && 0xFF; //Low byte
  }



  txRequest = Tx16Request(cameraAddress16, payload, (stepperCount*2)+1);
  xbee.send(txRequest);

  delay(TX_COMMAND_DELAY);

  //wait for an ack.
  returnPacketStates responseStatus = readPacketBuffer();
  if (responseStatus == TX_ACK) {
    //success! 
  } 
  else {
#if DEBUG_MODE == 1
    Serial.println("Pos packet failed!");
#endif
  }


}

returnPacketStates readPacketBuffer() { //Enumerated return vals!
  returnPacketStates returnStatus = COMM_FAIL;
  //The return status codes go like this.... 
  //COMM_FAIL is no packet was recieved Is the device unplugged?
  //TX_ACK is TX ACK
  //TX_FAIL is TX FAILED
  //RX_PACKET is RX
  //AT_ACK is AT ACK
  //AT_ACK_DATA is AT ACK WITH OUTPUT
  //AT_FAIL is AT FAIL

    // after sending a TX request, we expect a status response
  // wait up to 500 mseconds for the status response
  if (xbee.readPacket(1000)) {
    // got a response!
    // should be a znet tx status
    /*if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) { //We got a transmission response packet!!
     xbee.getResponse().getZBTxStatusResponse(txStatus);
     
     // get the delivery status, the fifth byte
     if (txStatus.getStatus() == SUCCESS) {
     // success.  time to celebrate
     #if DEBUG_MODE
     Serial.println("Packet sent with ACK!");
     #endif
     returnStatus = 1;
     } 
     else {
     // the remote XBee did not receive our packet. is it powered on?
     #if DEBUG_MODE
     Serial.println("Packet transmission failed.....");
     #endif
     returnStatus = 2;
     } */

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
        returnStatus = TX_FAIL;
      }  
      //isSuccess() 
    }

  } 
  else {
    // local XBee did not provide a timely TX Status Response -- should not happen
    Serial.println("Wait, WTF? The Xbee did not respond!!!!");
    returnStatus = COMM_FAIL;

  }
  cleanPayload();
  cleanRXPacket();
  return returnStatus;
}


void cleanPayload() {
  for (int i = 0; i < PAYLOAD_LENGTH; i++) {
    //clean the packet!
    payload[i] = '\0';
  }
}

void cleanRXPacket() {
  for (int i = 0; i < 40; i++) {
    //clean the packet!
    recievedPayload[i] = '\0';
  }
}

void setupXbeeCameraAddress() {
  //Send address settup commands to XBEE 

  atRequest = AtCommandRequest(ATMY,cameraAddress,2); //Set our address
  sendAtCommand();//Send the command

}

void setupXbeeControllerAddress() {
  //Send address settup commands to Xbee

  atRequest = AtCommandRequest(ATMY,controllerAddress,2); //Set our address
  sendAtCommand();//Send the command
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

void runControllerSlice() {
  //Sample the pots, map them to the step space and then send at regular intervals.
  //Debug messages
#if DEBUG_MODE == 1
  //Serial.println("Start Cont Slice");
#endif

  readPots();

}

void runCameraSlice() {
  //check if a new packet is available. If so, log the time it has been since the last packet to try to gauge the average packet delay.
  //calculate the speed the steppers need to run at to get to their destination position based on delta and the time till the next pack

}


void setupCameraPins() {
  //Setup the stepper pins

}

void setupControllerPins() {
  //setup the analog inputs
  //No setup is necessary for the analog pins so this is just a placeholder.
}

void setupCameraSerial() {
  Serial.begin(57600); //Start talking to the Xbee
  xbee.setSerial(Serial); //set the xbee to use Serialport 1
  //No debug mode here
}

void setupControllerSerial() {
  Serial1.begin(57600); //Start talking to the Xbee
  xbee.setSerial(Serial1); //set the xbee to use Serialport 1
  Serial.begin(115200); //Start the debugging prompt.

  Serial.println("Hello World! Setting up the Xbee modems to their corresponding addresses.");//USE PROGMEM!!!!
}

void readPots() {
  //Read and average the pots
  uint32_t average[3] = {
    0,0,0        }; //temp

  for (int i = 0; i < iterations; i++ ) { //get vals from all the pins and average them all out
    for (int x = 0; x < 3; x++){

      int16_t boundsCheck = analogRead(potPins[x]);

      if (boundsCheck <= 1023 && boundsCheck >= 0) {
        average[x] += boundsCheck;
      } 
      else if (boundsCheck > 1023){
        average[x] += 1023; //No need for a third case here. adding zero will not affect anything.
      }

    }
  }

  for (int i = 0; i < 3; i++) { //average it out
    potVals[i] = average[i]/iterations;
#if DEBUG_MODE
    Serial.print("Pot");
    Serial.print(i);
    Serial.print("Val:");
    Serial.println(potVals[i]);
#endif
  }

}



