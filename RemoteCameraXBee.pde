#include <XBee.h>

#define DEBUG_MODE 1 //Do we want debug output on the serial line?
#define IS_CONTROLLER 1 //Is this the camera controller? Or the reciever?

#define AT_COMMAND_DELAY 10 //In milliseconds
#define TX_COMMAND_DELAY 20 //In milliseconds. (Ballparked by assuming )


uint8_t testLEDPin = 13;

uint8_t cameraAddress[] = { //Address of the Camera
  0x10,0x00};
uint8_t controllerAddress[] = { //Address of the Controller
  0x20,0x00};

uint8_t channel[] = {
  0x11}; //Channel
uint8_t panID[] = {
  0x11,0x11}; //Personal Area Network

uint8_t payload[40]; //Payload buffer. Clean this promptly after using!!!!!
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




//*****Reusable Instances!!******//
XBee xbee = XBee(); //Instantiate a new xbee instance
TxStatusResponse txStatus = TxStatusResponse();
AtCommandRequest atRequest = AtCommandRequest();
AtCommandResponse atResponse = AtCommandResponse();
Rx16Response rx16 = Rx16Response();

void setup() {
  pinMode(testLEDPin, OUTPUT);
  
  #if IS_CONTROLLER == 1 
  setupControllerPins();
#else
  setupCameraPins();
#endif

  Serial1.begin(57600); //Start talking to the Xbee
  xbee.setSerial(Serial1); //set the xbee to use Serialport 1
  Serial.begin(115200); //Start the debugging prompt.

  Serial.println("Hello World! Setting up the Xbee modems to their corresponding addresses.");//USE PROGMEM!!!!
  delay(5000); //Wait for the XBee to initialize. 5 sec pause.

#if IS_CONTROLLER == 1 
  setupXbeeControllerAddress();
#else
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

}


void sendAtCommand() {
  //Flush buffer
  //Serial1.flush();

  //Send the command then wait for the response

  xbee.send(atRequest);

  uint8_t responseCode = readPacketBuffer();
  if (responseCode == 30 || responseCode == 31) {
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


uint8_t readPacketBuffer() {
  uint8_t returnStatus = 0;
  //The return status codes go like this.... 
  //0 is no packet was recieved Is the device unplugged?
  //1 is TX ACK
  //2 is TX FAILED
  //20 is RX
  //30 is AT ACK
  //31 is AT ACK WITH OUTPUT
  //32 is AT FAIL

  // after sending a TX request, we expect a status response
  // wait up to 500 mseconds for the status response
  if (xbee.readPacket(1000)) {
    // got a response!
    // should be a znet tx status
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) { //We got a transmission response packet!!
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
      }
    } 
    else if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) { //We got a response to our AT command!
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

          returnStatus = 31;
        } 
        else { //The AT command did not return any data. That's ok too! It was sucessful.
          returnStatus = 30;
        }
      } 
      else {
#if DEBUG_MODE
        Serial.print("Command return error code: ");
        Serial.println(atResponse.getStatus(), HEX);
#endif
        returnStatus = 32;
      }
    } 
    else if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16); //put it into storage, we will check it later.  
      returnStatus = 20;
#if DEBUG_MODE
      Serial.println("We got a RX packet!");
#endif
    }

  } 
  else {
    // local XBee did not provide a timely TX Status Response -- should not happen
    Serial.println("Wait, WTF? The Xbee did not respond!!!!");
    returnStatus = 0;

  }
  cleanPayload();
  cleanRXPacket();
  return returnStatus;
}


void cleanPayload() {
  for (int i = 0; i < 40; i++) {
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

}

void runCameraSlice() {
  //check if a new packet is available. If so, log the time it has been since the last packet to try to gauge the average packet delay.
  //calculate the speed the steppers need to run at to get to their destination position based on delta and the time till the next pack
 

}

void setupControllerPins() {
  //setup all the pins we will need as a controller (this mainly includes the Pot pins)
  
  pinMode()
  
}

void setupCameraPins() {
  
}



