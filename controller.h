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

#ifndef CONTROLLER_H
#define CONTROLLER_H

#if IS_CONTROLLER == 1
void sendTxPositionPacket(uint16_t *stepperPos);

void setupXbeeControllerAddress();

void runControllerSlice();

void setupControllerPins();

void setupControllerSerial();

void readPots();

void eliminateJitter();

void debugPotVals();

void convertToStepPosition();
#endif

#endif
