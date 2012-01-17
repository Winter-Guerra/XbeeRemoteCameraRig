//This is a part of the source code I coded for a xbee and stepper-motor
//driven camera mount needed for a school play. More info on the current 
//status of the project can be found at its thingiverse page here: 
//http://www.thingiverse.com/thing:16092
//or at my code's Github repo here: 
//https://github.com/xtremd/Parametric-Openscad-Tripod-Mount
//and here:
//https://github.com/xtremd/XbeeRemoteCameraRig

//This code is released under the CC-BY-SA license. Read more about it here:
//http://creativecommons.org/licenses/by-sa/3.0/us/

//Go be awesome!

//-XtremD
//January 17th, 2012

#if IS_CONTROLLER == 1

void sendTxPositionPacket(uint16_t *stepperPos) {
  //Send a packet to the camera containing a stepper x and y pos
  cleanPayload();

  payload[0] = positionCommandCode;

int payloadOffset = 1;
  for (int i = 0; i < STEPPER_COUNT; i++) {
    payload[payloadOffset] = stepperPos[i] >> 8 & 0xFF; //High byte
    payload[payloadOffset+1] = stepperPos[i] & 0xFF; //Low byte
    payloadOffset += 2;
  }



  txRequest = Tx16Request(cameraAddress16, payload, (STEPPER_COUNT*2)+1);
  xbee.send(txRequest);

  //wait for an ack.
  returnPacketStates responseStatus = readPacketBufferTimeout(PACKET_TIMEOUT);
  if (responseStatus == TX_ACK) {
    //success! 
  } 
  else {
    turnOnErrorLED();
#if DEBUG_MODE == 1
    Serial.println("Pos packet failed!");
#endif
  }
  delay(TX_COMMAND_DELAY);
}

void setupXbeeControllerAddress() {
  //Send address settup commands to Xbee

  atRequest = AtCommandRequest(ATMY,controllerAddress,2); //Set our address
  sendAtCommand();//Send the command
}

void runControllerSlice() {
  //Sample the pots, map them to the step space and then send at regular intervals.
  //Should we check if the other XBEE is connected and responding?


  readPots(); //Read the pot values
  //***Good idea, but not really neccesary and a bit overkill on the "sticky"
  //eliminateJitter(); //Eliminate jittery readings by making them a bit "sticky"
  
  #if DEBUG_MODE == 1
  debugPotVals();
  #endif
  
  convertToStepPosition(); //Convert the potvals to steps for each of the steppers. Output is in the potConvertedToSteps[] array.
  sendTxPositionPacket(potConvertedToSteps); //Send out the stepperPosition!
  

}

void setupControllerPins() {
  //setup the analog inputs
  //No setup is necessary for the analog pins so this is just a placeholder.
}

void setupControllerSerial() {
  Serial1.begin(57600); //Start talking to the Xbee
  xbee.setSerial(Serial1); //set the xbee to use Serialport 1
  Serial.begin(115200); //Start the debugging prompt.

  Serial.println("Hello World! Setting up the Xbee modems to their corresponding addresses.");//USE PROGMEM!!!!
}

void readPots() {
  //Read and average the pots
  uint32_t average[STEPPER_COUNT];
  for (int i = 0; i < STEPPER_COUNT; i++) { //Zero out the array
    average[i] = 0;
  }

  for (int i = 0; i < iterations; i++ ) { //get vals from all the pins and average them all out
    for (int x = 0; x < STEPPER_COUNT; x++){

      int16_t boundsCheck = analogRead(potPins[x]);

      if (boundsCheck <= 1023 && boundsCheck >= 0) {
        average[x] += boundsCheck;
      } 
      else if (boundsCheck > 1023){
        average[x] += 1023; //No need for a third case here. adding zero will not affect anything.
      }

    }
  }

  for (int i = 0; i < STEPPER_COUNT; i++) { //average it out
    potVals[i] = average[i]/iterations;
  }

}

void eliminateJitter() {
 //remembers the old potvals and then, if the change is not big enough ignore the change and leave.
 for (int i = 0; i < STEPPER_COUNT; i++) {
   if (potVals[i] == oldPotVals[i] + 1 || potVals[i] == oldPotVals[i] - 1) {
    //Then we should just ignore this tiny change in vals;
   potVals[i] = oldPotVals[i]; 
   } else {
    //There was a significant change in vals. Let us reset our old reference vals and allow the change
   oldPotVals[i] = potVals[i]; 
   }
 }
 
}

void debugPotVals() {
  //Only to be called if DEBUG_MODE == 1
  for (int i = 0; i < STEPPER_COUNT; i++) { //average it out
    Serial.print("Pot");
    Serial.print(i);
    Serial.print("Val:");
    Serial.println(potVals[i]);

  }
}

void convertToStepPosition() {
  //Takes in some potvals and outputs the position in steps that the potval is mapped to.
  for (int i = 0; i < STEPPER_COUNT; i++){
   //map!
  potConvertedToSteps[i] = map(potVals[i], 0, 1023, 0, stepperRanges[i]); 
  }
  
}

#endif

