#if IS_CONTROLLER == 0

void setupXbeeCameraAddress() {
  //Send address settup commands to XBEE 

  atRequest = AtCommandRequest(ATMY,cameraAddress,2); //Set our address
  sendAtCommand();//Send the command

}

void runCameraSlice() {
  //check if a new packet is available. If so, log the time it has been since the last packet to try to gauge the average packet delay.
  //calculate the speed the steppers need to run at to get to their destination position based on delta and the time till the next pack

}

void setupCameraPins() {
  //Setup the stepper pins
  pinMode(xStepperEnable, INPUT);
pinMode(yStepperEnable, INPUT);
pinMode(xStepperDir, INPUT);
pinMode(yStepperDir, INPUT);
pinMode(xStepperStep, INPUT);
pinMode(yStepperStep, INPUT);

}

void setupCameraSerial() {
  Serial.begin(57600); //Start talking to the Xbee
  xbee.setSerial(Serial); //set the xbee to use Serialport 1
  //No debug mode here
}

#endif
