#include <XBee.h>

//Address of the Camera
//Address of the Controller
uint8_t testLEDPin = 13;

uint8_t cameraAddress[] = {0x10,0x00};
uint8_t controllerAddress[] = {0x20,0x00};

uint8_t payload[40]; //Payload buffer. Clean this promptly after using!!!!!
uint8_t recievedPayload[40]; //Buffer for the recieved data. Clean promptly after using!

//XBee AT commands. USE PROGMEM TO SAVE SPACE!
uint8_t ATMY[] = { 'M', 'Y' };
uint8_t ATWR[] = { 'W', 'R' };
uint8_t ATSH[] = { 'S', 'H' };
uint8_t ATSL[] = { 'S', 'L' };


//*****Reusable Instances!!******//
XBee xbee = XBee(); //Instantiate a new xbee instance
TxStatusResponse txStatus = TxStatusResponse();
AtCommandRequest atRequest = AtCommandRequest(ATMY);
AtCommandResponse atResponse = AtCommandResponse();

void setup() {
  pinMode(testLEDPin, OUTPUT);

  Serial1.begin(9600); //Start talking to the Xbee
  xbee.setSerial(Serial1); //set the xbee to use Serialport 1
  Serial.begin(115200); //Start the debugging prompt.

  Serial.println("Hello World! Setting up the Xbee modems to their corresponding addresses.");//USE PROGMEM!!!!
  delay(5000); //Wait for the XBee to initialize. 5 sec pause.
  
  atRequest = AtCommandRequest(ATMY,cameraAddress,2); //Set our address
	sendAtCommand();//Send the command

atRequest = AtCommandRequest(ATWR); //Set the settings
	sendAtCommand();//Send the command

  
  
}

void loop() {
  digitalWrite(testLEDPin,HIGH);
  delay(500);
  digitalWrite(testLEDPin,LOW);
  delay(500);
}


void sendAtCommand() {
  Serial.println("Sending command to the XBee"); //Debug

  xbee.send(atRequest); //Send the command packet.
  // wait up to 5 seconds for the status response
  if (xbee.readPacket(5000)) {
    // got a response!

    // should be an AT command response
    if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
      xbee.getResponse().getAtCommandResponse(atResponse);

      if (atResponse.isOk()) {
        Serial.print("Command [");
        Serial.print(atResponse.getCommand()[0]);
        Serial.print(atResponse.getCommand()[1]);
        Serial.println("] was successful!");

        if (atResponse.getValueLength() > 0) {
          Serial.print("Command value length is ");
          Serial.println(atResponse.getValueLength(), DEC);

          Serial.print("Command value: "); //Is this a masked value? Aka, is a 64 bit addresss passed as one or multiple bytes?

          for (int i = 0; i < atResponse.getValueLength(); i++) {
            Serial.print(atResponse.getValue()[i], HEX);
            Serial.print(" ");
          }

          Serial.println("");
        }
      } 
      else {
        Serial.print("Command return error code: ");
        Serial.println(atResponse.getStatus(), HEX);
      }
    } 
    else {
      Serial.print("Expected AT response but got ");
      Serial.print(xbee.getResponse().getApiId(), HEX);
    }
  } 
  else {
    // at command failed
    if (xbee.getResponse().isError()) {
      Serial.print("Error reading packet.  Error code: ");
      Serial.println(xbee.getResponse().getErrorCode());
    } 
    else {
      Serial.print("No response from radio");
    }
  }
}


uint8_t handleCommandAftermath() {
  uint8_t returnStatus = 0;
  //The return status codes go like this.... 
  //0 is no packet was recieved Is the device unplugged?
  //1 is TX ACK
  //2 is TX FAILED
  //20 is RX
  //21 is RX FAILED
  //30 is AT ACK
  //31 is AT ACK WITH OUTPUT
  //32 is AT FAIL
  
  // after sending a TX request, we expect a status response
  // wait up to 500 mseconds for the status response
  if (xbee.readPacket(100)) {
    // got a response!
    // should be a znet tx status
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) { //We got a transmission response packet!!
      xbee.getResponse().getZBTxStatusResponse(txStatus);

      // get the delivery status, the fifth byte
      if (txStatus.getStatus() == SUCCESS) {
        // success.  time to celebrate
        Serial.println("Packet sent with ACK!");
        returnStatus = 1;
      } 
      else {
        // the remote XBee did not receive our packet. is it powered on?
        Serial.println("Packet transmission failed.....");
        returnStatus = 2;
      }
    } 
    else if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) { //We got a response to our AT command!
xbee.getResponse().getAtCommandResponse(atResponse);

      if (atResponse.isOk()) {
        Serial.print("Command [");
        Serial.print(atResponse.getCommand()[0]);
        Serial.print(atResponse.getCommand()[1]);
        Serial.println("] was successful!");

        if (atResponse.getValueLength() > 0) {
          Serial.print("Command value length is ");
          Serial.println(atResponse.getValueLength(), DEC);

          Serial.print("Command value: "); //Is this a masked value? Aka, is a 64 bit addresss passed as one or multiple bytes?

          for (int i = 0; i < atResponse.getValueLength(); i++) {
            Serial.print(atResponse.getValue()[i], HEX);
            Serial.print(" ");
          }

          Serial.println("");
          returnStatus = 31;
        } else { //The AT command did not return anything. That's ok too!
        returnStatus = 30;
        }
      } 
      else {
        Serial.print("Command return error code: ");
        Serial.println(atResponse.getStatus(), HEX);
        returnStatus = 32;
      }
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


