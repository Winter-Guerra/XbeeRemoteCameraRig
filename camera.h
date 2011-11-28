#ifndef CAMERA_H
#define CAMERA_H

#if IS_CONTROLLER != 1
//#include </Applications/Arduino.app/Contents/Resources/Java/libraries/AccelStepper/AccelStepper.h>

void setupXbeeCameraAddress();

void runCameraSlice();

void runSteppers();

uint8_t readRXPacketAndRunCommand();

void setupCameraPins();

void setupCameraSerial();

void recieveRxPositionPacket();
#endif
#endif