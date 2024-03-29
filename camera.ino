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

#if IS_CONTROLLER != 1

// Define some steppers and the pins the will use. These are reusable instances.
AccelStepper xStepper(1, xStepperStep, xStepperDir); //Step,Direction
AccelStepper yStepper(1, yStepperStep, yStepperDir); //Step, Direction


void setupXbeeCameraAddress() {
  //Send address settup commands to XBEE 

  atRequest = AtCommandRequest(ATMY,cameraAddress,2); //Set our address
  sendAtCommand();//Send the command

}

void runCameraSlice() {
  //check if a new packet is available. If so, log the time it has been since the last packet to try to gauge the average packet delay.
  //calculate the speed the steppers need to run at to get to their destination position based on delta and the time till the next pack

  //Check if we have a packet.
  returnPacketStates responseCode = readPacketBufferTimeout(PACKET_TIMEOUT);
  if (responseCode == RX_PACKET) {
    //Sweet! We got a command packet!
    //lets copy over the payload and then check what command we got.
    readRXPacketAndRunCommand();
  }

  runSteppers(); //Run the steppers.
}

void runSteppers() {
  xStepper.run();
  yStepper.run();
}

uint8_t readRXPacketAndRunCommand() {
  cleanRecievedPayload(); //Clean up workspace

  for (int i = 0; i < rx16.getDataLength(); i++) { //Copy the payload over byte by byte
    recievedPayload[i] = rx16.getData()[i];
  }

  if (recievedPayload[0] == positionCommandCode) {
    //We have a new waypoint. 
    recieveRxPositionPacket(); //Get & set the position!
  }

}

void setupCameraPins() {
  //Setup the stepper pins
  pinMode(xStepperEnable, OUTPUT);
  pinMode(yStepperEnable, OUTPUT);
  pinMode(xStepperDir, OUTPUT);
  pinMode(yStepperDir, OUTPUT);
  pinMode(xStepperStep, OUTPUT);
  pinMode(yStepperStep, OUTPUT);

  //Enable the steppers.
  digitalWrite(xStepperEnable, LOW); //They enable on low. Wierd right?
  digitalWrite(yStepperEnable, LOW);
  digitalWrite(xStepperDir, LOW); 
  digitalWrite(yStepperDir, LOW);
  digitalWrite(xStepperStep, LOW); 
  digitalWrite(yStepperStep, LOW);

  xStepper.setMaxSpeed(xStepperMaxSpeed);
  xStepper.setAcceleration(xStepperAcceleration);
  yStepper.setMaxSpeed(yStepperMaxSpeed);
  yStepper.setAcceleration(yStepperAcceleration);
  
  //Setup the AccellStepper libraries
  //The steppers should be at their midpoint
  xStepper.setCurrentPosition(xStepperMidpoint); 
  yStepper.setCurrentPosition(yStepperMidpoint); 

}

void setupCameraSerial() {
  Serial.begin(57600); //Start talking to the Xbee
  xbee.setSerial(Serial); //set the xbee to use Serialport 1
  //No debug mode here
}

//Recieve position packet

void recieveRxPositionPacket() {
  //Recieve a packet from the controller containing a stepper x and y pos

  //We are assuming that the packet was already checked for what commandcode it had
  //Ignore whatever was in the [0] slot.

  /*for (int i = 0; i < STEPPER_COUNT; i++) {
   recievedPayload[i+1] = stepperPos[i] >> 8 && 0xFF; //High byte
   recievedPayload[i+2] = stepperPos[i] && 0xFF; //Low byte
   }*/

int payloadOffset = 1;
  for (int i = 0; i < STEPPER_COUNT; i++) {
    stepperTarget[i] = recievedPayload[payloadOffset] << 8;//High byte
    stepperTarget[i] += recievedPayload[payloadOffset+1];//Add the low byte
    payloadOffset += 2;
  }

  xStepper.moveTo(stepperTarget[0]);//Move it! Move it! I haven't got all day! What is this? Sissy camp! This is bootcamp, move your ass!
  yStepper.moveTo(stepperTarget[1]);
  //delay(TX_COMMAND_DELAY); //no need for that here, we are polling as fast as we can.
}

#endif




